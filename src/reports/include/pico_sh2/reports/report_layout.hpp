// SPDX-License-Identifier: MIT
#pragma once

/// \file report_layout.hpp
/// Common input-report prefix shared by the reports on the normal and wake
/// input channels (SH-2 Reference Manual, section 6.5). Every batched input
/// report begins with:
///   byte 0  report ID
///   byte 1  sequence number (per-sensor; a gap means a dropped report)
///   byte 2  status  — bits 1:0 accuracy, bits 7:2 high 6 bits of the delay
///   byte 3  delay   — low 8 bits of the delay
/// The 14-bit delay is in 100 microsecond units and is combined by the
/// timestamp layer with the base-timestamp reference. Payload fields follow at
/// byte 4.
///
/// The compact gyro-integrated rotation vector (0x2A, the gyro-RV input
/// channel) does NOT use this prefix: it carries no report ID, status, or delay
/// and is one-per-transfer (SH-2 SHTP Reference Manual, channel usage). It is
/// decoded separately.

#include <cstddef>
#include <cstdint>
#include <span>

namespace pico_sh2::reports {

/// Offset of the first payload byte after the common prefix.
inline constexpr std::size_t INPUT_REPORT_PREFIX_SIZE = 4;

/// Sensor status / accuracy, from status byte bits 1:0.
enum class SensorAccuracy : std::uint8_t {
    UNRELIABLE = 0,
    LOW = 1,
    MEDIUM = 2,
    HIGH = 3,
};

/// The common prefix decoded into fields. The 14-bit delay is left in raw
/// 100-microsecond units; the timestamp layer folds it into an absolute time.
struct InputReportPrefix {
    std::uint8_t report_id;     ///< Which report follows.
    std::uint8_t sequence;      ///< The sensor's own report sequence number.
    SensorAccuracy accuracy;    ///< The hub's confidence in the sample.
    std::uint16_t delay_100us;  ///< Report delay, in units of 100 us.
};

/// Decode the 4-byte common prefix, taken as it is on the wire: report ID,
/// sequence, status, delay LSB. The fixed extent is the length guarantee (the
/// cargo walk checks length against the report-length table first).
constexpr InputReportPrefix decode_prefix(
    std::span<const std::uint8_t, INPUT_REPORT_PREFIX_SIZE> prefix) {
    const std::uint8_t report_id = prefix[0];
    const std::uint8_t sequence = prefix[1];
    const std::uint8_t status = prefix[2];
    const std::uint8_t delay_lsb = prefix[3];
    /// status carries the delay's high bits above the 2-bit accuracy field.
    /// Promote through unsigned: std::uint8_t promotes to signed int, so the
    /// natural spelling shifts a signed value.
    const auto delay = static_cast<std::uint16_t>(
        ((static_cast<unsigned>(status) >> 2U) << 8U) |
        static_cast<unsigned>(delay_lsb));
    return InputReportPrefix{
        .report_id = report_id,
        .sequence = sequence,
        .accuracy = static_cast<SensorAccuracy>(status & 0x03U),
        .delay_100us = delay};
}

}  // namespace pico_sh2::reports
