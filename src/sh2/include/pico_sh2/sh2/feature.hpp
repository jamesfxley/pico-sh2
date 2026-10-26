// SPDX-License-Identifier: MIT
#pragma once

/// \file feature.hpp
/// Set/Get Feature reports (SH-2 Reference Manual §6.5.2-6.5.5). A
/// FeatureConfig is the host's desired configuration for one sensor and,
/// symmetrically, the hub's granted configuration reported back in a Get
/// Feature Response; the two share the on-wire feature-report layout. Report
/// and batch intervals are in microseconds; a report interval of 0 disables the
/// sensor.

#include <cstddef>
#include <cstdint>
#include <span>

#include "pico_sh2/reports/sensor_id.hpp"
#include "pico_util/bytes/byte_order.hpp"

namespace pico_sh2 {

/// Feature-flag bits (SH-2 Reference Manual §6.5.2).
namespace feature_flag {
/// Bit 0: the change sensitivity is relative, not absolute.
inline constexpr std::uint8_t CHANGE_SENSITIVITY_RELATIVE = 0x01;
/// Bit 1: apply the change sensitivity at all.
inline constexpr std::uint8_t CHANGE_SENSITIVITY_ENABLED = 0x02;
/// Bit 2: this sensor's reports wake the host.
inline constexpr std::uint8_t WAKE = 0x04;
/// Bit 3: keep the sensor running regardless of the hub's power state.
inline constexpr std::uint8_t ALWAYS_ON = 0x08;
}  // namespace feature_flag

/// Report ID asking the hub for a sensor's current configuration (§6.5.3).
inline constexpr std::uint8_t GET_FEATURE_REQUEST = 0xFE;
/// Report ID setting a sensor's configuration (§6.5.4).
inline constexpr std::uint8_t SET_FEATURE_COMMAND = 0xFD;
/// Report ID carrying the hub's granted configuration (§6.5.5).
inline constexpr std::uint8_t GET_FEATURE_RESPONSE = 0xFC;

/// The feature report body is a fixed 17 bytes: report ID + 16 payload bytes.
inline constexpr std::size_t FEATURE_REPORT_SIZE = 17;

/// One sensor's configuration, in both directions: the host's request and the
/// hub's grant share the on-wire feature-report layout.
struct FeatureConfig {
    /// The sensor this configures.
    reports::SensorId sensor{};
    /// Bitwise OR of the feature_flag values.
    std::uint8_t flags = 0;
    /// Change threshold below which the hub suppresses a report; applies only
    /// with feature_flag::CHANGE_SENSITIVITY_ENABLED set.
    std::uint16_t change_sensitivity = 0;
    /// Interval between reports, microseconds. Zero disables the sensor.
    std::uint32_t report_interval_us = 0;
    /// How long the hub may batch reports before sending, microseconds.
    std::uint32_t batch_interval_us = 0;
    /// Configuration word whose meaning is defined per sensor.
    std::uint32_t sensor_specific = 0;
};

/// Serialize `config` as a Set Feature (0xFD) command into `out`. Returns the
/// number of bytes written.
constexpr std::size_t serialize_set_feature(
    const FeatureConfig& config,
    std::span<std::uint8_t, FEATURE_REPORT_SIZE> out) {
    out[0] = SET_FEATURE_COMMAND;
    out[1] = static_cast<std::uint8_t>(config.sensor);
    out[2] = config.flags;
    pico_util::write_le<std::uint16_t>(out, config.change_sensitivity, 3);
    pico_util::write_le<std::uint32_t>(out, config.report_interval_us, 5);
    pico_util::write_le<std::uint32_t>(out, config.batch_interval_us, 9);
    pico_util::write_le<std::uint32_t>(out, config.sensor_specific, 13);
    return FEATURE_REPORT_SIZE;
}

/// Parse a Get Feature Response (0xFC) body into a FeatureConfig. The caller
/// has already checked report[0] == GET_FEATURE_RESPONSE; the extent is the
/// length check, so a short report cannot reach here.
constexpr FeatureConfig parse_feature_response(
    std::span<const std::uint8_t, FEATURE_REPORT_SIZE> report) {
    FeatureConfig config;
    config.sensor = static_cast<reports::SensorId>(report[1]);
    config.flags = report[2];
    config.change_sensitivity = pico_util::read_le<std::uint16_t>(report, 3);
    config.report_interval_us = pico_util::read_le<std::uint32_t>(report, 5);
    config.batch_interval_us = pico_util::read_le<std::uint32_t>(report, 9);
    config.sensor_specific = pico_util::read_le<std::uint32_t>(report, 13);
    return config;
}

}  // namespace pico_sh2
