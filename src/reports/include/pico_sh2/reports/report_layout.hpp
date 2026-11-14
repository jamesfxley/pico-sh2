// SPDX-License-Identifier: MIT
#pragma once

/// \file report_layout.hpp
/// Batched prefix: report ID, sequence, status and delay (1000-3625 §6.5).
/// Status bits 1:0 are accuracy; bits 7:2 and byte 3 form a 14-bit delay in
/// 100-microsecond ticks. Payload starts at byte 4.
/// Compact gyro reports have no prefix and are decoded separately (1000-3600
/// §2.2).

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

/// Decoded prefix; delay stays in 100-microsecond ticks for reconstruction.
struct InputReportPrefix {
    std::uint8_t report_id;     ///< Which report follows.
    std::uint8_t sequence;      ///< The sensor's own report sequence number.
    SensorAccuracy accuracy;    ///< The hub's confidence in the sample.
    std::uint16_t delay_100us;  ///< Report delay, in units of 100 us.
};

/// Decode a fixed-length prefix after the cargo walk has checked report length.
constexpr InputReportPrefix decode_prefix(
    std::span<const std::uint8_t, INPUT_REPORT_PREFIX_SIZE> prefix) {
    const std::uint8_t report_id = prefix[0];
    const std::uint8_t sequence = prefix[1];
    const std::uint8_t status = prefix[2];
    const std::uint8_t delay_lsb = prefix[3];
    /// Unsigned promotion avoids shifting a signed int.
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
