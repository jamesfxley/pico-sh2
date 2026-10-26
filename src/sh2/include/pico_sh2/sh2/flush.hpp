// SPDX-License-Identifier: MIT
#pragma once

/// \file flush.hpp
/// Force Sensor Flush (0xF0) and Flush Completed (0xEF): asking the hub to send
/// a sensor's outstanding batched data now, and being told when it has.
///
/// SH-2 Reference Manual §6.5.6 and §6.5.7. Both reports are two bytes, the
/// report ID followed by the sensor ID.

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>

#include "pico_sh2/reports/sensor_id.hpp"

namespace pico_sh2 {

/// Report ID asking the hub to send a sensor's batched data now (§6.5.6).
inline constexpr std::uint8_t FORCE_FLUSH_REQUEST = 0xF0;
/// Report ID telling the host the flush is done (§6.5.7).
inline constexpr std::uint8_t FLUSH_COMPLETED_REPORT = 0xEF;

/// Wire length of either report: report ID plus sensor ID (§6.5.6 Figure 80,
/// §6.5.7 Figure 81).
inline constexpr std::size_t FLUSH_REPORT_SIZE = 2;
/// Byte offset of the sensor ID within either report.
inline constexpr std::size_t FLUSH_SENSOR_ID = 1;

/// Serialize a Force Sensor Flush for `sensor`.
inline std::array<std::uint8_t, FLUSH_REPORT_SIZE> serialize_force_flush(
    reports::SensorId sensor) {
    return {FORCE_FLUSH_REQUEST, static_cast<std::uint8_t>(sensor)};
}

/// The sensor a Flush Completed names, or nothing if the report is malformed.
/// The caller has already matched report[0] == FLUSH_COMPLETED_REPORT.
[[nodiscard]] inline std::optional<reports::SensorId> parse_flush_completed(
    std::span<const std::uint8_t> report) {
    if (report.size() < FLUSH_REPORT_SIZE) {
        return std::nullopt;
    }
    return static_cast<reports::SensorId>(report[FLUSH_SENSOR_ID]);
}

}  // namespace pico_sh2
