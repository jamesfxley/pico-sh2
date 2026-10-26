// SPDX-License-Identifier: MIT
// Report decoders: the constexpr {report ID, wire length, decode fn} dispatch
// table, per-sensor Q-point conversions, and the input-cargo walk. All byte
// offsets, Q points, and wire lengths are from the SH-2 Reference Manual,
// section 6.5. Wire lengths are bound at compile time to the report-length seed
// (report_length_table.hpp) so a decoder and the splitter cannot disagree.
#include "pico_sh2/reports/decoders.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <span>

#include "pico_sh2/reports/q_override.hpp"
#include "pico_sh2/reports/q_point.hpp"
#include "pico_sh2/reports/report_layout.hpp"
#include "pico_sh2/reports/report_length_table.hpp"
#include "pico_sh2/reports/sensor_event.hpp"
#include "pico_sh2/reports/sensor_id.hpp"
#include "pico_sh2/reports/timestamp.hpp"
#include "pico_sh2/util/result.hpp"
#include "pico_util/bytes/byte_order.hpp"

namespace pico_sh2::reports {

namespace {

// Timestamp records embedded in an input cargo (1000-3625 §7.2).
// Both are the report ID followed by a signed 32-bit delta in 100 us ticks.
constexpr std::uint8_t BASE_TIMESTAMP_REPORT = 0xFB;
constexpr std::uint8_t TIMESTAMP_REBASE_REPORT = 0xFA;

// A 0xFB/0xFA record: the ID byte and the delta.
constexpr std::size_t TIMESTAMP_RECORD_SIZE = 5;

// The signed 32-bit delta at byte 1 of a 0xFB/0xFA record, which the caller
// has checked is TIMESTAMP_RECORD_SIZE long.
constexpr std::int32_t timestamp_delta(std::span<const std::uint8_t> report) {
    return static_cast<std::int32_t>(
        pico_util::read_le<std::uint32_t>(report, 1));
}

// A signed 16-bit fixed-point field at `offset`, scaled by its Q point.
constexpr float field(std::span<const std::uint8_t> report, std::size_t offset,
                      unsigned q_point) {
    return q_to_float(pico_util::read_le<std::int16_t>(report, offset),
                      q_point);
}

// Payload begins after the 4-byte common prefix.
constexpr std::size_t PAYLOAD = INPUT_REPORT_PREFIX_SIZE;

// Each decoder receives the report's resolved Q points (`q_points.primary` for
// the main data field(s), `q_points.secondary` for the one auxiliary field a
// few reports carry). The values default to the report's compiled Q (the
// `default_q` in the registry below) but may be overridden per part at decode
// time (DR-2); a decoder that uses no fixed-point field ignores `q_points`.

// Three-axis engineering-unit report: accelerometer (Q8), calibrated gyro (Q9),
// calibrated magnetometer (Q4), linear acceleration (Q8), gravity (Q8).
void decode_vector(std::span<const std::uint8_t> report, ReportQ q_points,
                   SensorEvent& event) {
    event.payload = Vec3{.x = field(report, PAYLOAD, q_points.primary),
                         .y = field(report, PAYLOAD + 2, q_points.primary),
                         .z = field(report, PAYLOAD + 4, q_points.primary)};
}

// Uncalibrated three-axis report: the sample (primary) plus the removed bias
// (secondary). A part's metadata may give the two different Q points (Q point
// 1 and 2 of 1000-3625 §5.1); both default to the one Q its report section
// states. Uncalibrated gyro (Q9), uncalibrated magnetometer (Q4).
void decode_uncalibrated(std::span<const std::uint8_t> report, ReportQ q_points,
                         SensorEvent& event) {
    const unsigned primary = q_points.primary;
    const unsigned bias = q_points.secondary;
    event.payload = UncalibratedSample{
        .value = Vec3{.x = field(report, PAYLOAD, primary),
                      .y = field(report, PAYLOAD + 2, primary),
                      .z = field(report, PAYLOAD + 4, primary)},
        .bias = Vec3{.x = field(report, PAYLOAD + 6, bias),
                     .y = field(report, PAYLOAD + 8, bias),
                     .z = field(report, PAYLOAD + 10, bias)}};
}

// Rotation vector with accuracy: quaternion i,j,k,real (primary, Q14) plus a
// heading accuracy in radians (secondary, Q12). Rotation vector, geomagnetic
// RV, AR/VR-stabilized RV.
void decode_rotation(std::span<const std::uint8_t> report, ReportQ q_points,
                     SensorEvent& event) {
    const unsigned primary = q_points.primary;
    event.payload = RotationSample{
        .unit = Quaternion{.i = field(report, PAYLOAD, primary),
                           .j = field(report, PAYLOAD + 2, primary),
                           .k = field(report, PAYLOAD + 4, primary),
                           .real = field(report, PAYLOAD + 6, primary)},
        .accuracy_rad = field(report, PAYLOAD + 8, q_points.secondary)};
}

// Rotation vector without accuracy: quaternion i,j,k,real (primary, Q14).
// Game rotation vector, AR/VR-stabilized game rotation vector.
void decode_game_rotation(std::span<const std::uint8_t> report,
                          ReportQ q_points, SensorEvent& event) {
    const unsigned primary = q_points.primary;
    event.payload = Quaternion{.i = field(report, PAYLOAD, primary),
                               .j = field(report, PAYLOAD + 2, primary),
                               .k = field(report, PAYLOAD + 4, primary),
                               .real = field(report, PAYLOAD + 6, primary)};
}

// Tap detector: a per-axis single/double-tap bit field (byte 4).
void decode_tap(std::span<const std::uint8_t> report, ReportQ /*q*/,
                SensorEvent& event) {
    event.payload = TapSample{report[PAYLOAD]};
}

// Step counter: detect latency (u32, bytes 4-7) and cumulative steps (u16,
// bytes 8-9). Q0 — no fixed-point scaling.
void decode_step_count(std::span<const std::uint8_t> report, ReportQ /*q*/,
                       SensorEvent& event) {
    event.payload = StepCountSample{
        .latency_us = pico_util::read_le<std::uint32_t>(report, PAYLOAD),
        .steps = pico_util::read_le<std::uint16_t>(report, PAYLOAD + 4)};
}

// Step detector: detect latency (u32, bytes 4-7).
void decode_step_detect(std::span<const std::uint8_t> report, ReportQ /*q*/,
                        SensorEvent& event) {
    event.payload =
        StepDetectSample{pico_util::read_le<std::uint32_t>(report, PAYLOAD)};
}

// Stability classifier: the classification enum value (byte 4).
void decode_stability(std::span<const std::uint8_t> report, ReportQ /*q*/,
                      SensorEvent& event) {
    event.payload = StabilitySample{report[PAYLOAD]};
}

// Raw sensor report: unscaled ADC counts (bytes 4-9) and the sensor's own
// microsecond timestamp (u32, bytes 12-15). Bytes 10-11 are reserved for raw
// accel/mag, or gyro temperature (1000-3625 Figures 82, 86, 89), not decoded.
void decode_raw(std::span<const std::uint8_t> report, ReportQ /*q*/,
                SensorEvent& event) {
    event.payload = RawSample{
        .x = pico_util::read_le<std::int16_t>(report, PAYLOAD),
        .y = pico_util::read_le<std::int16_t>(report, PAYLOAD + 2),
        .z = pico_util::read_le<std::int16_t>(report, PAYLOAD + 4),
        .timestamp = pico_util::read_le<std::uint32_t>(report, PAYLOAD + 8)};
}

using DecodeFn = void (*)(std::span<const std::uint8_t>, ReportQ, SensorEvent&);

struct DecoderEntry {
    SensorId id;
    std::uint8_t wire_length;
    ReportQ default_q;
    DecodeFn fn;
};

// The decoder registry. Wire lengths match the report-length seed (checked
// below). `default_q` is the compiled Q point(s) from the SH-2 Reference Manual
// §6.5, used unless a metadata override is installed (DR-2). Report 0x2A
// (gyro-integrated RV) is decoded separately — it is not a prefixed input
// report and is never split from a cargo.
constexpr std::array<DecoderEntry, 19> DECODERS = {{
    {.id = SensorId::ACCELEROMETER,
     .wire_length = 10,
     .default_q = {.primary = 8, .secondary = 0},
     .fn = &decode_vector},
    {.id = SensorId::GYROSCOPE,
     .wire_length = 10,
     .default_q = {.primary = 9, .secondary = 0},
     .fn = &decode_vector},
    {.id = SensorId::MAGNETIC_FIELD,
     .wire_length = 10,
     .default_q = {.primary = 4, .secondary = 0},
     .fn = &decode_vector},
    {.id = SensorId::LINEAR_ACCELERATION,
     .wire_length = 10,
     .default_q = {.primary = 8, .secondary = 0},
     .fn = &decode_vector},
    {.id = SensorId::ROTATION_VECTOR,
     .wire_length = 14,
     .default_q = {.primary = 14, .secondary = 12},
     .fn = &decode_rotation},
    {.id = SensorId::GRAVITY,
     .wire_length = 10,
     .default_q = {.primary = 8, .secondary = 0},
     .fn = &decode_vector},
    {.id = SensorId::GYROSCOPE_UNCALIBRATED,
     .wire_length = 16,
     .default_q = {.primary = 9, .secondary = 9},
     .fn = &decode_uncalibrated},
    {.id = SensorId::GAME_ROTATION_VECTOR,
     .wire_length = 12,
     .default_q = {.primary = 14, .secondary = 0},
     .fn = &decode_game_rotation},
    {.id = SensorId::GEOMAGNETIC_ROTATION_VECTOR,
     .wire_length = 14,
     .default_q = {.primary = 14, .secondary = 12},
     .fn = &decode_rotation},
    {.id = SensorId::MAGNETIC_FIELD_UNCALIBRATED,
     .wire_length = 16,
     .default_q = {.primary = 4, .secondary = 4},
     .fn = &decode_uncalibrated},
    {.id = SensorId::TAP_DETECTOR,
     .wire_length = 5,
     .default_q = {.primary = 0, .secondary = 0},
     .fn = &decode_tap},
    {.id = SensorId::STEP_COUNTER,
     .wire_length = 12,
     .default_q = {.primary = 0, .secondary = 0},
     .fn = &decode_step_count},
    {.id = SensorId::STABILITY_CLASSIFIER,
     .wire_length = 6,
     .default_q = {.primary = 0, .secondary = 0},
     .fn = &decode_stability},
    {.id = SensorId::RAW_ACCELEROMETER,
     .wire_length = 16,
     .default_q = {.primary = 0, .secondary = 0},
     .fn = &decode_raw},
    {.id = SensorId::RAW_GYROSCOPE,
     .wire_length = 16,
     .default_q = {.primary = 0, .secondary = 0},
     .fn = &decode_raw},
    {.id = SensorId::RAW_MAGNETOMETER,
     .wire_length = 16,
     .default_q = {.primary = 0, .secondary = 0},
     .fn = &decode_raw},
    {.id = SensorId::STEP_DETECTOR,
     .wire_length = 8,
     .default_q = {.primary = 0, .secondary = 0},
     .fn = &decode_step_detect},
    {.id = SensorId::ARVR_STABILIZED_ROTATION_VECTOR,
     .wire_length = 14,
     .default_q = {.primary = 14, .secondary = 12},
     .fn = &decode_rotation},
    {.id = SensorId::ARVR_STABILIZED_GAME_ROTATION_VECTOR,
     .wire_length = 12,
     .default_q = {.primary = 14, .secondary = 0},
     .fn = &decode_game_rotation},
}};

// Bind each decoder's wire length to the compiled report-length seed: a decoder
// row and the cargo splitter cannot disagree about a report's size.
consteval bool decoders_match_seed() {
    const ReportLengthTable seed = ReportLengthTable::seeded();
    for (const DecoderEntry& entry : DECODERS) {
        auto seeded = seed.length_of(static_cast<std::uint8_t>(entry.id));
        if (!seeded.ok() || seeded.value() != entry.wire_length) {
            return false;
        }
    }
    return true;
}

static_assert(decoders_match_seed(),
              "decoder wire length disagrees with the report-length seed");

// decode() takes the prefix from any report at least wire_length long.
consteval bool decoders_cover_the_prefix() {
    return std::ranges::all_of(DECODERS, [](const DecoderEntry& entry) {
        return entry.wire_length >= INPUT_REPORT_PREFIX_SIZE;
    });
}

static_assert(decoders_cover_the_prefix(),
              "a decoded report is shorter than the common prefix");

const DecoderEntry* find_decoder(std::uint8_t report_id) {
    for (const DecoderEntry& entry : DECODERS) {
        if (static_cast<std::uint8_t>(entry.id) == report_id) {
            return &entry;
        }
    }
    return nullptr;
}

}  // namespace

Result<SensorEvent> decode_report(std::span<const std::uint8_t> report,
                                  const QOverrideTable& q_overrides) {
    if (report.empty()) {
        return Sh2Error::INVALID_ARGUMENT;
    }
    const DecoderEntry* entry = find_decoder(report[0]);
    if (entry == nullptr) {
        return Sh2Error::UNSUPPORTED;
    }
    if (report.size() < entry->wire_length) {
        return Sh2Error::INVALID_ARGUMENT;
    }

    const InputReportPrefix prefix =
        decode_prefix(report.first<INPUT_REPORT_PREFIX_SIZE>());
    SensorEvent event;
    event.sensor = static_cast<SensorId>(prefix.report_id);
    event.sequence = prefix.sequence;
    event.accuracy = prefix.accuracy;
    event.delay_100us = prefix.delay_100us;
    entry->fn(report, q_overrides.resolve(report[0], entry->default_q), event);
    return event;
}

Result<SensorEvent> decode_gyro_integrated(
    std::span<const std::uint8_t> payload, const QOverrideTable& q_overrides) {
    // Quaternion i,j,k,real (Q14) then angular velocity x,y,z (Q10); no prefix.
    // The report has no bias or accuracy field, so applicability of metadata Q
    // point 2 is UNCONFIRMED (1000-3625 §5.1, §6.5.44) and the angular velocity
    // keeps the Q its report section fixes (§6.5.44); only the quaternion's Q
    // is overridable.
    constexpr std::size_t GIRV_LENGTH = 14;
    constexpr unsigned GIRV_ANGULAR_VELOCITY_Q = 10;
    constexpr auto GIRV_ID =
        static_cast<std::uint8_t>(SensorId::GYRO_INTEGRATED_ROTATION_VECTOR);
    if (payload.size() < GIRV_LENGTH) {
        return Sh2Error::INVALID_ARGUMENT;
    }
    const unsigned unit_q =
        q_overrides.resolve(GIRV_ID, ReportQ{.primary = 14, .secondary = 0})
            .primary;
    SensorEvent event;
    event.sensor = SensorId::GYRO_INTEGRATED_ROTATION_VECTOR;
    event.payload = GyroIntegratedSample{
        .unit = Quaternion{.i = field(payload, 0, unit_q),
                           .j = field(payload, 2, unit_q),
                           .k = field(payload, 4, unit_q),
                           .real = field(payload, 6, unit_q)},
        .angular_velocity =
            Vec3{.x = field(payload, 8, GIRV_ANGULAR_VELOCITY_Q),
                 .y = field(payload, 10, GIRV_ANGULAR_VELOCITY_Q),
                 .z = field(payload, 12, GIRV_ANGULAR_VELOCITY_Q)}};
    return event;
}

Result<void> walk_input_cargo(std::span<const std::uint8_t> cargo,
                              const ReportLengthTable& lengths,
                              const QOverrideTable& q_overrides,
                              std::uint64_t intn_edge_us, const EventSink& sink,
                              WalkStats& stats) {
    // Until a 0xFB record sets a base delta, events time from the HINTN edge.
    Timebase timebase = timebase_begin(intn_edge_us, 0);
    std::size_t offset = 0;
    while (offset < cargo.size()) {
        const std::uint8_t report_id = cargo[offset];
        auto length = lengths.length_of(report_id);
        if (!length.ok()) {
            // Unknown length: the walk cannot find the next report boundary.
            ++stats.unknown_reports;
            return Sh2Error::PROTOCOL;
        }
        const std::size_t report_len = length.value();
        if (offset + report_len > cargo.size()) {
            // A report that overruns the cargo cannot be decoded safely.
            ++stats.truncated_reports;
            return Sh2Error::PROTOCOL;
        }

        const std::span<const std::uint8_t> report =
            cargo.subspan(offset, report_len);
        const bool timestamp = report_id == BASE_TIMESTAMP_REPORT ||
                               report_id == TIMESTAMP_REBASE_REPORT;
        if (timestamp && report_len < TIMESTAMP_RECORD_SIZE) {
            // An advertised length too short to carry the delta: every event
            // after it would be timed from bytes that are not a delta.
            ++stats.truncated_reports;
            return Sh2Error::PROTOCOL;
        }
        if (report_id == BASE_TIMESTAMP_REPORT) {
            timebase = timebase_begin(intn_edge_us, timestamp_delta(report));
        } else if (report_id == TIMESTAMP_REBASE_REPORT) {
            timebase = timebase_rebase(timebase, timestamp_delta(report));
        } else {
            auto event = decode_report(report, q_overrides);
            if (event.ok()) {
                SensorEvent decoded = event.value();
                decoded.event_us =
                    timebase_event_us(timebase, decoded.delay_100us);
                decoded.timebase_overflow = timebase.overflow;
                ++stats.events_decoded;
                sink(decoded);
            } else {
                // Known length, no decoder (or a length shorter than the
                // decoder needs): step over it without aborting the walk.
                ++stats.reports_skipped;
            }
        }
        offset += report_len;
    }
    return Result<void>{};
}

}  // namespace pico_sh2::reports
