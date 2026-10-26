// SPDX-License-Identifier: MIT
// DR-1: golden wire vectors for every in-scope decoder, each round-tripping to
// the documented units and Q point. Byte layouts, Q points, and wire lengths
// are from the SH-2 Reference Manual, section 6.5. Each vector is hand-built so
// the expected engineering-unit values are exact powers of two, making the
// fixed-point conversion checkable with EXPECT_FLOAT_EQ.
#include "pico_sh2/reports/decoders.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cstdint>
#include <span>
#include <vector>

#include "pico_sh2/reports/q_override.hpp"
#include "pico_sh2/reports/q_point.hpp"
#include "pico_sh2/reports/report_layout.hpp"
#include "pico_sh2/reports/report_length_table.hpp"
#include "pico_sh2/reports/sensor_event.hpp"
#include "pico_sh2/reports/sensor_id.hpp"
#include "pico_sh2/util/result.hpp"

namespace pico_sh2::reports {
namespace {

// A calibrated accelerometer report (0x01, Q8): X=+1, Y=+2, Z=-1 m/s^2,
// sequence 0x2A, accuracy HIGH (status 0x03), delay 0.
constexpr std::array<std::uint8_t, 10> ACCEL = {0x01, 0x2A, 0x03, 0x00, 0x00,
                                                0x01, 0x00, 0x02, 0x00, 0xFF};

TEST(Decoders, AccelerometerVector) {
    auto ev = decode_report(ACCEL);
    ASSERT_TRUE(ev.ok());
    EXPECT_EQ(ev.value().sensor, SensorId::ACCELEROMETER);
    EXPECT_EQ(ev.value().sequence, 0x2A);
    EXPECT_EQ(ev.value().accuracy, SensorAccuracy::HIGH);
    EXPECT_EQ(ev.value().kind(), EventKind::VECTOR);
    EXPECT_FLOAT_EQ(std::get<Vec3>(ev.value().payload).x, 1.0F);
    EXPECT_FLOAT_EQ(std::get<Vec3>(ev.value().payload).y, 2.0F);
    EXPECT_FLOAT_EQ(std::get<Vec3>(ev.value().payload).z, -1.0F);
}

TEST(Decoders, GyroQ9AndMagQ4Scaling) {
    // Gyro (0x02, Q9): X = 512 / 2^9 = 1.0 rad/s.
    std::array<std::uint8_t, 10> gyro = {0x02, 0, 0x03, 0, 0x00,
                                         0x02, 0, 0,    0, 0};
    auto gyro_event = decode_report(gyro);
    ASSERT_TRUE(gyro_event.ok());
    EXPECT_FLOAT_EQ(std::get<Vec3>(gyro_event.value().payload).x, 1.0F);

    // Mag (0x03, Q4): X = 16 / 2^4 = 1.0 uT.
    std::array<std::uint8_t, 10> mag = {0x03, 0, 0x03, 0, 0x10,
                                        0x00, 0, 0,    0, 0};
    auto mag_event = decode_report(mag);
    ASSERT_TRUE(mag_event.ok());
    EXPECT_FLOAT_EQ(std::get<Vec3>(mag_event.value().payload).x, 1.0F);
}

TEST(Decoders, UncalibratedGyroValueAndBias) {
    // 0x07, Q9: value X = 512/2^9 = 1.0; bias X = 256/2^9 = 0.5.
    std::array<std::uint8_t, 16> uncal = {
        0x07, 0, 0x02, 0, 0x00, 0x02, 0, 0, 0, 0, 0x00, 0x01, 0, 0, 0, 0};
    auto ev = decode_report(uncal);
    ASSERT_TRUE(ev.ok());
    EXPECT_EQ(ev.value().kind(), EventKind::UNCALIBRATED);
    EXPECT_EQ(ev.value().accuracy, SensorAccuracy::MEDIUM);
    EXPECT_FLOAT_EQ(std::get<UncalibratedSample>(ev.value().payload).value.x,
                    1.0F);
    EXPECT_FLOAT_EQ(std::get<UncalibratedSample>(ev.value().payload).bias.x,
                    0.5F);
}

TEST(Decoders, RotationVectorWithAccuracy) {
    // 0x05: quat i = 16384/2^14 = 1.0; accuracy = 4096/2^12 = 1.0 rad.
    std::array<std::uint8_t, 14> rv = {0x05, 0, 0x03, 0,    0x00, 0x40, 0x00,
                                       0x00, 0, 0,    0x00, 0x00, 0x00, 0x10};
    auto ev = decode_report(rv);
    ASSERT_TRUE(ev.ok());
    EXPECT_EQ(ev.value().kind(), EventKind::ROTATION);
    EXPECT_FLOAT_EQ(std::get<RotationSample>(ev.value().payload).unit.i, 1.0F);
    EXPECT_FLOAT_EQ(std::get<RotationSample>(ev.value().payload).accuracy_rad,
                    1.0F);
}

TEST(Decoders, GameRotationVectorNoAccuracy) {
    // 0x08: quat real = 16384/2^14 = 1.0 (identity), i=j=k=0.
    std::array<std::uint8_t, 12> grv = {0x08, 0, 0x03, 0, 0,    0,
                                        0,    0, 0,    0, 0x00, 0x40};
    auto ev = decode_report(grv);
    ASSERT_TRUE(ev.ok());
    EXPECT_EQ(ev.value().kind(), EventKind::GAME_ROTATION);
    EXPECT_FLOAT_EQ(std::get<Quaternion>(ev.value().payload).real, 1.0F);
    EXPECT_FLOAT_EQ(std::get<Quaternion>(ev.value().payload).i, 0.0F);
}

TEST(Decoders, TapBitField) {
    std::array<std::uint8_t, 5> tap = {0x10, 0, 0x00, 0, 0x05};
    auto ev = decode_report(tap);
    ASSERT_TRUE(ev.ok());
    EXPECT_EQ(ev.value().kind(), EventKind::TAP);
    EXPECT_EQ(std::get<TapSample>(ev.value().payload).taps, 0x05);
}

TEST(Decoders, StepCounter) {
    // 0x11: latency u32 = 1000 us, steps u16 = 10.
    std::array<std::uint8_t, 12> step = {0x11, 0,    0x03, 0,    0xE8, 0x03,
                                         0x00, 0x00, 0x0A, 0x00, 0x00, 0x00};
    auto ev = decode_report(step);
    ASSERT_TRUE(ev.ok());
    EXPECT_EQ(ev.value().kind(), EventKind::STEP_COUNT);
    EXPECT_EQ(std::get<StepCountSample>(ev.value().payload).latency_us, 1000U);
    EXPECT_EQ(std::get<StepCountSample>(ev.value().payload).steps, 10U);
}

TEST(Decoders, StepDetector) {
    // 0x18: latency u32 = 500 us.
    std::array<std::uint8_t, 8> step = {0x18, 0, 0x03, 0, 0xF4, 0x01, 0, 0};
    auto ev = decode_report(step);
    ASSERT_TRUE(ev.ok());
    EXPECT_EQ(ev.value().kind(), EventKind::STEP_DETECT);
    EXPECT_EQ(std::get<StepDetectSample>(ev.value().payload).latency_us, 500U);
}

TEST(Decoders, StabilityClassifier) {
    std::array<std::uint8_t, 6> stab = {0x13, 0, 0x00, 0, 0x02, 0x00};
    auto ev = decode_report(stab);
    ASSERT_TRUE(ev.ok());
    EXPECT_EQ(ev.value().kind(), EventKind::STABILITY);
    EXPECT_EQ(std::get<StabilitySample>(ev.value().payload).classification,
              0x02);
}

TEST(Decoders, RawAccelerometerAdcAndTimestamp) {
    // 0x14: X=10, Y=20, Z=-10 ADC counts; sensor timestamp = 10000 us.
    std::array<std::uint8_t, 16> raw = {0x14, 0,    0x03, 0,    0x0A, 0x00,
                                        0x14, 0x00, 0xF6, 0xFF, 0x00, 0x00,
                                        0x10, 0x27, 0x00, 0x00};
    auto ev = decode_report(raw);
    ASSERT_TRUE(ev.ok());
    EXPECT_EQ(ev.value().kind(), EventKind::RAW);
    EXPECT_EQ(std::get<RawSample>(ev.value().payload).x, 10);
    EXPECT_EQ(std::get<RawSample>(ev.value().payload).y, 20);
    EXPECT_EQ(std::get<RawSample>(ev.value().payload).z, -10);
    EXPECT_EQ(std::get<RawSample>(ev.value().payload).timestamp, 10000U);
}

TEST(Decoders, GyroIntegratedRotationVectorHasNoPrefix) {
    // 0x2A (the gyro-RV input channel): quat i = 1.0 (Q14); angular velocity x
    // = 1024/2^10 = 1.0.
    std::array<std::uint8_t, 14> girv = {0x00, 0x40, 0x00, 0x00, 0x00,
                                         0x00, 0x00, 0x00, 0x00, 0x04,
                                         0x00, 0x00, 0x00, 0x00};
    auto ev = decode_gyro_integrated(girv);
    ASSERT_TRUE(ev.ok());
    EXPECT_EQ(ev.value().sensor, SensorId::GYRO_INTEGRATED_ROTATION_VECTOR);
    EXPECT_EQ(ev.value().kind(), EventKind::GYRO_INTEGRATED);
    EXPECT_FLOAT_EQ(std::get<GyroIntegratedSample>(ev.value().payload).unit.i,
                    1.0F);
    EXPECT_FLOAT_EQ(
        std::get<GyroIntegratedSample>(ev.value().payload).angular_velocity.x,
        1.0F);
}

// DR-1 golden vectors for the remaining decoded reports, each laid out and
// scaled as SH-2 Reference Manual 1000-3625 v1.9 §6.5 gives it. Values are
// chosen so a Q point off by one changes the result.

TEST(Decoders, LinearAccelerationQ8) {
    // §6.5.10, Q8 m/s^2: X = 256 -> 1.0, Y = -512 -> -2.0, Z = 128 -> 0.5.
    std::array<std::uint8_t, 10> report = {0x04, 0, 0x03, 0,    0x00,
                                           0x01, 0, 0xFE, 0x80, 0x00};
    auto ev = decode_report(report);
    ASSERT_TRUE(ev.ok());
    EXPECT_EQ(ev.value().sensor, SensorId::LINEAR_ACCELERATION);
    EXPECT_EQ(ev.value().kind(), EventKind::VECTOR);
    EXPECT_FLOAT_EQ(std::get<Vec3>(ev.value().payload).x, 1.0F);
    EXPECT_FLOAT_EQ(std::get<Vec3>(ev.value().payload).y, -2.0F);
    EXPECT_FLOAT_EQ(std::get<Vec3>(ev.value().payload).z, 0.5F);
}

TEST(Decoders, GravityQ8) {
    // §6.5.11, Q8 m/s^2: Z = 0x0980 = 2432 -> 9.5.
    std::array<std::uint8_t, 10> report = {0x06, 0, 0x03, 0,    0,
                                           0,    0, 0,    0x80, 0x09};
    auto ev = decode_report(report);
    ASSERT_TRUE(ev.ok());
    EXPECT_EQ(ev.value().sensor, SensorId::GRAVITY);
    EXPECT_FLOAT_EQ(std::get<Vec3>(ev.value().payload).z, 9.5F);
}

TEST(Decoders, GeomagneticRotationVectorQ14WithQ12Accuracy) {
    // §6.5.20: i = 16384 -> 1.0 at Q14; accuracy = 2048 -> 0.5 rad at Q12.
    std::array<std::uint8_t, 14> report = {
        0x09, 0, 0x03, 0, 0x00, 0x40, 0, 0, 0, 0, 0x00, 0x00, 0x00, 0x08};
    auto ev = decode_report(report);
    ASSERT_TRUE(ev.ok());
    EXPECT_EQ(ev.value().sensor, SensorId::GEOMAGNETIC_ROTATION_VECTOR);
    EXPECT_EQ(ev.value().kind(), EventKind::ROTATION);
    EXPECT_FLOAT_EQ(std::get<RotationSample>(ev.value().payload).unit.i, 1.0F);
    EXPECT_FLOAT_EQ(std::get<RotationSample>(ev.value().payload).accuracy_rad,
                    0.5F);
}

TEST(Decoders, MagneticFieldUncalibratedQ4ValueAndBias) {
    // §6.5.17, Q4 uT for both: value X = 32 -> 2.0, bias X = 8 -> 0.5.
    std::array<std::uint8_t, 16> report = {0x0F, 0, 0x02, 0, 0x20, 0, 0, 0,
                                           0,    0, 0x08, 0, 0,    0, 0, 0};
    auto ev = decode_report(report);
    ASSERT_TRUE(ev.ok());
    EXPECT_EQ(ev.value().sensor, SensorId::MAGNETIC_FIELD_UNCALIBRATED);
    EXPECT_EQ(ev.value().kind(), EventKind::UNCALIBRATED);
    EXPECT_FLOAT_EQ(std::get<UncalibratedSample>(ev.value().payload).value.x,
                    2.0F);
    EXPECT_FLOAT_EQ(std::get<UncalibratedSample>(ev.value().payload).bias.x,
                    0.5F);
}

TEST(Decoders, RawGyroscopeAdcAndTimestamp) {
    // §6.5.12: X = 7, Y = -3, Z = 100 ADC counts; the temperature at bytes
    // 10-11 is not a sample axis; timestamp = 20000 us.
    std::array<std::uint8_t, 16> report = {0x15, 0,    0x03, 0,    0x07, 0x00,
                                           0xFD, 0xFF, 0x64, 0x00, 0x55, 0x55,
                                           0x20, 0x4E, 0x00, 0x00};
    auto ev = decode_report(report);
    ASSERT_TRUE(ev.ok());
    EXPECT_EQ(ev.value().sensor, SensorId::RAW_GYROSCOPE);
    EXPECT_EQ(ev.value().kind(), EventKind::RAW);
    EXPECT_EQ(std::get<RawSample>(ev.value().payload).x, 7);
    EXPECT_EQ(std::get<RawSample>(ev.value().payload).y, -3);
    EXPECT_EQ(std::get<RawSample>(ev.value().payload).z, 100);
    EXPECT_EQ(std::get<RawSample>(ev.value().payload).timestamp, 20000U);
}

TEST(Decoders, RawMagnetometerAdcAndTimestamp) {
    // §6.5.15: X = -1, Y = 2, Z = 300 ADC counts; timestamp = 1 us.
    std::array<std::uint8_t, 16> report = {0x16, 0,    0x03, 0,    0xFF, 0xFF,
                                           0x02, 0x00, 0x2C, 0x01, 0x00, 0x00,
                                           0x01, 0x00, 0x00, 0x00};
    auto ev = decode_report(report);
    ASSERT_TRUE(ev.ok());
    EXPECT_EQ(ev.value().sensor, SensorId::RAW_MAGNETOMETER);
    EXPECT_EQ(std::get<RawSample>(ev.value().payload).x, -1);
    EXPECT_EQ(std::get<RawSample>(ev.value().payload).y, 2);
    EXPECT_EQ(std::get<RawSample>(ev.value().payload).z, 300);
    EXPECT_EQ(std::get<RawSample>(ev.value().payload).timestamp, 1U);
}

TEST(Decoders, ArvrStabilizedRotationVectorQ14WithQ12Accuracy) {
    // §6.5.42: real = 16384 -> 1.0 at Q14; accuracy = 1024 -> 0.25 rad at Q12.
    std::array<std::uint8_t, 14> report = {0x28, 0, 0x03, 0,    0,    0,   0, 0,
                                           0,    0, 0x00, 0x40, 0x00, 0x04};
    auto ev = decode_report(report);
    ASSERT_TRUE(ev.ok());
    EXPECT_EQ(ev.value().sensor, SensorId::ARVR_STABILIZED_ROTATION_VECTOR);
    EXPECT_EQ(ev.value().kind(), EventKind::ROTATION);
    EXPECT_FLOAT_EQ(std::get<RotationSample>(ev.value().payload).unit.real,
                    1.0F);
    EXPECT_FLOAT_EQ(std::get<RotationSample>(ev.value().payload).accuracy_rad,
                    0.25F);
}

TEST(Decoders, ArvrStabilizedGameRotationVectorQ14) {
    // §6.5.43: j = 8192 -> 0.5 at Q14.
    std::array<std::uint8_t, 12> report = {0x29, 0,    0x03, 0, 0, 0,
                                           0x00, 0x20, 0,    0, 0, 0};
    auto ev = decode_report(report);
    ASSERT_TRUE(ev.ok());
    EXPECT_EQ(ev.value().sensor,
              SensorId::ARVR_STABILIZED_GAME_ROTATION_VECTOR);
    EXPECT_EQ(ev.value().kind(), EventKind::GAME_ROTATION);
    EXPECT_FLOAT_EQ(std::get<Quaternion>(ev.value().payload).j, 0.5F);
}

TEST(Decoders, UnknownReportIdIsUnsupported) {
    std::array<std::uint8_t, 4> unknown = {0x99, 0, 0, 0};
    EXPECT_EQ(decode_report(unknown).error(), Sh2Error::UNSUPPORTED);
}

TEST(Decoders, ShortSpanIsInvalidArgument) {
    std::array<std::uint8_t, 5> truncated_accel = {0x01, 0, 0x03, 0, 0x00};
    EXPECT_EQ(decode_report(truncated_accel).error(),
              Sh2Error::INVALID_ARGUMENT);
}

// --- cargo walk -----------------------------------------------------------

struct Capture {
    std::vector<SensorEvent> events;

    void on_event(const SensorEvent& event) { events.push_back(event); }
};

TEST(Walk, SplitsConcatenatedReports) {
    // Accelerometer (10 bytes) followed by a tap report (5 bytes) in one cargo.
    std::array<std::uint8_t, 15> cargo = {0x01, 0x2A, 0x03, 0x00, 0x00,
                                          0x01, 0x00, 0x02, 0x00, 0xFF,
                                          0x10, 0x00, 0x00, 0x00, 0x05};
    Capture cap;
    auto sink = EventSink::bind<Capture, &Capture::on_event>(&cap);
    WalkStats stats{};
    auto walked =
        walk_input_cargo(cargo, ReportLengthTable::seeded(), QOverrideTable{},
                         /*intn_edge_us=*/0, sink, stats);
    EXPECT_TRUE(walked.ok());
    EXPECT_EQ(stats.events_decoded, 2U);
    ASSERT_EQ(cap.events.size(), 2U);
    EXPECT_EQ(cap.events[0].sensor, SensorId::ACCELEROMETER);
    EXPECT_EQ(cap.events[1].sensor, SensorId::TAP_DETECTOR);
}

// DR-3: a report with a known advertised length but no decoder is skipped and
// counted, and the walk continues.
// SR-6: Flush Completed (0xEF) is two bytes (1000-3625 §6.5.7) and the seeded
// table knows it, so one inside an input cargo is stepped over and the report
// after it still decodes.
TEST(Walk, AFlushCompletedInAnInputCargoIsSteppedOver) {
    std::array<std::uint8_t, 12> cargo = {0xEF, 0x01,  // flush, accel
                                          0x01, 0x2A, 0x03, 0x00, 0x00, 0x01,
                                          0x00, 0x02, 0x00, 0xFF};  // accel
    Capture cap;
    auto sink = EventSink::bind<Capture, &Capture::on_event>(&cap);
    WalkStats stats{};
    auto walked =
        walk_input_cargo(cargo, ReportLengthTable::seeded(), QOverrideTable{},
                         /*intn_edge_us=*/0, sink, stats);
    EXPECT_TRUE(walked.ok());
    EXPECT_EQ(stats.unknown_reports, 0U);
    ASSERT_EQ(cap.events.size(), 1U);
    EXPECT_EQ(cap.events[0].sensor, SensorId::ACCELEROMETER);
}

TEST(Walk, KnownLengthNoDecoderIsSkipped) {
    // Significant motion (0x12) is in the length seed but has no decoder.
    std::array<std::uint8_t, 6> cargo = {0x12, 0, 0x00, 0, 0x00, 0x00};
    Capture cap;
    auto sink = EventSink::bind<Capture, &Capture::on_event>(&cap);
    WalkStats stats{};
    auto walked =
        walk_input_cargo(cargo, ReportLengthTable::seeded(), QOverrideTable{},
                         /*intn_edge_us=*/0, sink, stats);
    EXPECT_TRUE(walked.ok());
    EXPECT_EQ(stats.reports_skipped, 1U);
    EXPECT_EQ(stats.events_decoded, 0U);
    EXPECT_TRUE(cap.events.empty());
}

// DR-3: a length shorter than the report's decoder needs is stepped over and
// counted like a report with no decoder, and the walk carries on from the
// length it was told.
TEST(Walk, ALengthShorterThanItsDecoderNeedsIsSkipped) {
    std::array<std::uint8_t, 15> cargo = {
        0x01, 0x00, 0x03, 0x00, 0x00,  // accelerometer, advertised as 5
        0x02, 0x00, 0x03, 0x00,        // gyroscope, the seeded 10
        0x00, 0x01, 0x00, 0x02, 0x00, 0x03};
    ReportLengthTable lengths = ReportLengthTable::seeded();
    lengths.set(0x01, ReportLength{5});
    Capture cap;
    auto sink = EventSink::bind<Capture, &Capture::on_event>(&cap);
    WalkStats stats{};
    auto walked = walk_input_cargo(cargo, lengths, QOverrideTable{},
                                   /*intn_edge_us=*/0, sink, stats);
    EXPECT_TRUE(walked.ok());
    EXPECT_EQ(stats.reports_skipped, 1U);
    EXPECT_EQ(stats.events_decoded, 1U);
    ASSERT_EQ(cap.events.size(), 1U);
    EXPECT_EQ(cap.events[0].sensor, SensorId::GYROSCOPE);
}

// DR-3: an unknown length aborts the cargo walk — it cannot resynchronize —
// and is counted.
TEST(Walk, UnknownLengthAbortsAndCounts) {
    std::array<std::uint8_t, 4> cargo = {0x99, 0, 0, 0};
    Capture cap;
    auto sink = EventSink::bind<Capture, &Capture::on_event>(&cap);
    WalkStats stats{};
    auto walked =
        walk_input_cargo(cargo, ReportLengthTable::seeded(), QOverrideTable{},
                         /*intn_edge_us=*/0, sink, stats);
    EXPECT_EQ(walked.error(), Sh2Error::PROTOCOL);
    EXPECT_EQ(stats.unknown_reports, 1U);
}

// DR-3: a report whose length overruns the cargo aborts the walk and is
// counted.
TEST(Walk, TruncatedReportAbortsAndCounts) {
    // Accelerometer needs 10 bytes but only 3 are present.
    std::array<std::uint8_t, 3> cargo = {0x01, 0x00, 0x03};
    Capture cap;
    auto sink = EventSink::bind<Capture, &Capture::on_event>(&cap);
    WalkStats stats{};
    auto walked =
        walk_input_cargo(cargo, ReportLengthTable::seeded(), QOverrideTable{},
                         /*intn_edge_us=*/0, sink, stats);
    EXPECT_EQ(walked.error(), Sh2Error::PROTOCOL);
    EXPECT_EQ(stats.truncated_reports, 1U);
}

// SR-4, BR-1: an advertised timestamp-record length too short to carry the
// delta aborts the walk. Reading the delta would take bytes from the report
// after it, or from past the end of the cargo.
TEST(Walk, AShortTimestampRecordAbortsAndCounts) {
    std::array<std::uint8_t, 11> cargo = {
        0xFB,                                // base timestamp, advertised as 1
        0x01, 0x00, 0x03, 0x00, 0x00, 0x01,  // accelerometer
        0x00, 0x00, 0x00, 0x00};
    ReportLengthTable lengths = ReportLengthTable::seeded();
    lengths.set(0xFB, ReportLength{1});
    Capture cap;
    auto sink = EventSink::bind<Capture, &Capture::on_event>(&cap);
    WalkStats stats{};
    auto walked = walk_input_cargo(cargo, lengths, QOverrideTable{},
                                   /*intn_edge_us=*/0, sink, stats);
    EXPECT_EQ(walked.error(), Sh2Error::PROTOCOL);
    EXPECT_EQ(stats.truncated_reports, 1U);
    EXPECT_EQ(stats.events_decoded, 0U);
}

TEST(Walk, ReconstructsEventTimeFromBaseTimestamp) {
    // A base timestamp record (0xFB, delta = 10 ticks) then an accelerometer
    // report with delay = 5 ticks. The base delta is subtracted and the delay
    // added: event_us = edge - 100*10 + 100*5 = edge - 500 (SR-4).
    std::array<std::uint8_t, 15> cargo = {
        0xFB, 0x0A, 0x00, 0x00, 0x00,  // base delta = 10 (100us ticks)
        0x01, 0x00, 0x00, 0x05,        // accel: seq, status(acc=0), delay_lsb=5
        0x00, 0x01, 0x00, 0x02, 0x00, 0xFF};
    Capture cap;
    auto sink = EventSink::bind<Capture, &Capture::on_event>(&cap);
    WalkStats stats{};
    auto walked =
        walk_input_cargo(cargo, ReportLengthTable::seeded(), QOverrideTable{},
                         /*intn_edge_us=*/1'000'000, sink, stats);
    EXPECT_TRUE(walked.ok());
    ASSERT_EQ(cap.events.size(), 1U);
    EXPECT_EQ(cap.events[0].sensor, SensorId::ACCELEROMETER);
    EXPECT_EQ(cap.events[0].delay_100us, 5U);
    EXPECT_EQ(cap.events[0].event_us, 1'000'000U - 500U);
    EXPECT_FALSE(cap.events[0].timebase_overflow);
}

TEST(Walk, BaseTimestampOverflowFallsBackToEdge) {
    // 0xFB with the reserved overflow delta: the accel event falls back to the
    // HINTN edge and is marked overflowed.
    std::array<std::uint8_t, 15> cargo = {
        0xFB, 0xFF, 0xFF, 0xFF, 0x7F,  // base delta = 0x7FFFFFFF (overflow)
        0x01, 0x00, 0x00, 0x05, 0x00, 0x01, 0x00, 0x02, 0x00, 0xFF};
    Capture cap;
    auto sink = EventSink::bind<Capture, &Capture::on_event>(&cap);
    WalkStats stats{};
    auto walked =
        walk_input_cargo(cargo, ReportLengthTable::seeded(), QOverrideTable{},
                         /*intn_edge_us=*/1'000'000, sink, stats);
    EXPECT_TRUE(walked.ok());
    ASSERT_EQ(cap.events.size(), 1U);
    EXPECT_EQ(cap.events[0].event_us, 1'000'000U);
    EXPECT_TRUE(cap.events[0].timebase_overflow);
}

// --- decode-time Q overrides (DR-2) ---------------------------------------

// q_to_float shifts by the Q point, so a Q above MAX_Q_POINT is undefined.
// The table is public and reachable without a Session -- a consumer decoding a
// captured cargo offline installs an override on it directly -- so the refusal
// belongs to the table rather than to the paths that happen to call it. On ARM
// an accepted Q of 40 would shift by 8 and silently mis-scale every field.
TEST(QOverride, TableRefusesAQAboveTheRepresentableMaximum) {
    QOverrideTable table;
    const auto accel = static_cast<std::uint8_t>(SensorId::ACCELEROMETER);

    EXPECT_EQ(table.set(accel, ReportQ{MAX_Q_POINT + 1, 0}).error(),
              Sh2Error::INVALID_ARGUMENT);
    EXPECT_EQ(table.set(accel, ReportQ{0, MAX_Q_POINT + 1}).error(),
              Sh2Error::INVALID_ARGUMENT);
    EXPECT_FALSE(table.has(accel));

    // The boundary itself is representable and accepted.
    EXPECT_TRUE(table.set(accel, ReportQ{MAX_Q_POINT, MAX_Q_POINT}).ok());
}

TEST(QOverride, DecodeReportUsesInstalledPrimaryQ) {
    // ACCEL decodes X=+1 at the compiled Q8. Overriding accel to Q7 halves the
    // divisor, so the same bytes decode to X=+2.
    QOverrideTable table;
    ASSERT_TRUE(table.set(static_cast<std::uint8_t>(SensorId::ACCELEROMETER),
                          ReportQ{7, 0}));

    auto ev = decode_report(ACCEL, table);
    ASSERT_TRUE(ev.ok());
    EXPECT_FLOAT_EQ(std::get<Vec3>(ev.value().payload).x, 2.0F);
    EXPECT_FLOAT_EQ(std::get<Vec3>(ev.value().payload).y, 4.0F);
    EXPECT_FLOAT_EQ(std::get<Vec3>(ev.value().payload).z, -2.0F);

    // With no override the compiled Q8 still applies.
    auto compiled = decode_report(ACCEL);
    ASSERT_TRUE(compiled.ok());
    EXPECT_FLOAT_EQ(std::get<Vec3>(compiled.value().payload).x, 1.0F);
}

TEST(QOverride, DecodeReportUsesInstalledSecondaryQ) {
    // Rotation vector (0x05): quaternion real=+1 (Q14) and heading accuracy=+1
    // (Q12). Overriding only the secondary Q to 11 doubles the accuracy while
    // leaving the quaternion untouched.
    std::array<std::uint8_t, 14> rv = {
        0x05, 0x00, 0x00, 0x00,  // prefix
        0x00, 0x00, 0x00, 0x00,  // i=0, j=0
        0x00, 0x00, 0x00, 0x40,  // k=0, real=0x4000 (Q14 -> 1.0)
        0x00, 0x10};             // accuracy=0x1000 (Q12 -> 1.0)
    QOverrideTable table;
    ASSERT_TRUE(table.set(static_cast<std::uint8_t>(SensorId::ROTATION_VECTOR),
                          ReportQ{14, 11}));

    auto ev = decode_report(rv, table);
    ASSERT_TRUE(ev.ok());
    EXPECT_FLOAT_EQ(std::get<RotationSample>(ev.value().payload).unit.real,
                    1.0F);
    EXPECT_FLOAT_EQ(std::get<RotationSample>(ev.value().payload).accuracy_rad,
                    2.0F);
}

TEST(QOverride, WalkAppliesOverride) {
    QOverrideTable table;
    ASSERT_TRUE(table.set(static_cast<std::uint8_t>(SensorId::ACCELEROMETER),
                          ReportQ{7, 0}));
    Capture cap;
    auto sink = EventSink::bind<Capture, &Capture::on_event>(&cap);
    WalkStats stats{};
    auto walked = walk_input_cargo(ACCEL, ReportLengthTable::seeded(), table,
                                   /*intn_edge_us=*/0, sink, stats);
    EXPECT_TRUE(walked.ok());
    ASSERT_EQ(cap.events.size(), 1U);
    EXPECT_FLOAT_EQ(std::get<Vec3>(cap.events[0].payload).x, 2.0F);
}

// DR-2: the gyro-integrated report has no bias or accuracy field, so an
// override moves its quaternion's Q and never its angular velocity's, which
// §6.5.44 fixes at Q10.
TEST(QOverride, GyroIntegratedOverridesOnlyTheQuaternion) {
    // Quaternion i=+1 (Q14) and angular-velocity x=+1 (Q10).
    std::array<std::uint8_t, 14> girv = {
        0x00, 0x40, 0x00, 0x00,  // i=0x4000 (Q14 -> 1.0), j=0
        0x00, 0x00, 0x00, 0x00,  // k=0, real=0
        0x00, 0x04, 0x00, 0x00,  // ang.x=0x0400 (Q10 -> 1.0), ang.y=0
        0x00, 0x00};             // ang.z=0
    QOverrideTable table;
    // Both Q points one bit down, as a metadata record carrying Q point 2 = 9
    // would install them.
    ASSERT_TRUE(table.set(
        static_cast<std::uint8_t>(SensorId::GYRO_INTEGRATED_ROTATION_VECTOR),
        ReportQ{13, 9}));

    auto ev = decode_gyro_integrated(girv, table);
    ASSERT_TRUE(ev.ok());
    EXPECT_FLOAT_EQ(std::get<GyroIntegratedSample>(ev.value().payload).unit.i,
                    2.0F);
    EXPECT_FLOAT_EQ(
        std::get<GyroIntegratedSample>(ev.value().payload).angular_velocity.x,
        1.0F);
}

// DR-2: an uncalibrated report's bias is a bias field, so metadata Q point 2
// scales it independently of the sample (1000-3625 §5.1).
TEST(QOverride, UncalibratedBiasTakesTheSecondaryQ) {
    // Uncalibrated gyro (0x07, Q9): value x=+1 (0x0200), bias x=+0.5 (0x0100).
    std::array<std::uint8_t, 16> uncal = {0x07, 0x00, 0x03, 0x00, 0x00, 0x02,
                                          0x00, 0x00, 0x00, 0x00, 0x00, 0x01,
                                          0x00, 0x00, 0x00, 0x00};
    QOverrideTable table;
    // Sample Q unchanged; bias Q one bit down, doubling it.
    ASSERT_TRUE(
        table.set(static_cast<std::uint8_t>(SensorId::GYROSCOPE_UNCALIBRATED),
                  ReportQ{9, 8}));

    auto ev = decode_report(uncal, table);
    ASSERT_TRUE(ev.ok());
    EXPECT_FLOAT_EQ(std::get<UncalibratedSample>(ev.value().payload).value.x,
                    1.0F);
    EXPECT_FLOAT_EQ(std::get<UncalibratedSample>(ev.value().payload).bias.x,
                    1.0F);
}

TEST(QOverride, ReplaceAndCapacity) {
    QOverrideTable table;
    // Replacing an existing entry keeps capacity.
    ASSERT_TRUE(table.set(0x01, ReportQ{8, 0}));
    ASSERT_TRUE(table.set(0x01, ReportQ{7, 0}));
    EXPECT_EQ(table.resolve(0x01, ReportQ{0, 0}).primary, 7U);

    // Fill the remaining slots with distinct IDs, then a new ID is refused
    // while replacing an existing one still succeeds.
    for (std::uint8_t id = 2; id <= QOverrideTable::CAPACITY; ++id) {
        ASSERT_TRUE(table.set(id, ReportQ{id, 0}));
    }
    EXPECT_FALSE(table.set(0xEE, ReportQ{1, 0}));
    EXPECT_TRUE(table.set(0x01, ReportQ{6, 0}));
    EXPECT_FALSE(table.has(0xEE));
}

}  // namespace
}  // namespace pico_sh2::reports
