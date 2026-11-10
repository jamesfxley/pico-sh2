// SPDX-License-Identifier: MIT
#pragma once

/// \file metadata_q.hpp
/// Sensor-metadata Flash Record System (FRS) Q-point overrides (DR-2).

#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>

#include "pico_sh2/reports/q_override.hpp"
#include "pico_sh2/reports/q_point.hpp"
#include "pico_sh2/reports/sensor_id.hpp"

namespace pico_sh2::reports {

/// Metadata word index for Q point 1 low, Q point 2 high (1000-3625 §5.1).
inline constexpr std::size_t METADATA_Q_WORD = 7;

/// Largest accepted signed metadata Q exponent.
inline constexpr std::int16_t METADATA_Q_MAX =
    static_cast<std::int16_t>(MAX_Q_POINT);

/// Read Q points; nullopt means a short record or invalid exponent.
inline std::optional<ReportQ> metadata_record_q(
    std::span<const std::uint32_t> words) {
    if (words.size() <= METADATA_Q_WORD) {
        return std::nullopt;
    }
    const std::uint32_t packed = words[METADATA_Q_WORD];
    const auto q1 = static_cast<std::int16_t>(packed & 0xFFFFU);
    const auto q2 = static_cast<std::int16_t>((packed >> 16U) & 0xFFFFU);
    if (q1 < 0 || q1 > METADATA_Q_MAX || q2 < 0 || q2 > METADATA_Q_MAX) {
        return std::nullopt;
    }
    return ReportQ{.primary = static_cast<std::uint8_t>(q1),
                   .secondary = static_cast<std::uint8_t>(q2)};
}

/// Decoded report ID for a metadata type, or nullopt (1000-3625 Figure 27).
inline std::optional<std::uint8_t> report_id_for_metadata_type(
    std::uint16_t record_type) {
    switch (record_type) {
        case 0xE301:
            return static_cast<std::uint8_t>(SensorId::RAW_ACCELEROMETER);
        case 0xE302:
            return static_cast<std::uint8_t>(SensorId::ACCELEROMETER);
        case 0xE303:
            return static_cast<std::uint8_t>(SensorId::LINEAR_ACCELERATION);
        case 0xE304:
            return static_cast<std::uint8_t>(SensorId::GRAVITY);
        case 0xE305:
            return static_cast<std::uint8_t>(SensorId::RAW_GYROSCOPE);
        case 0xE306:
            return static_cast<std::uint8_t>(SensorId::GYROSCOPE);
        case 0xE307:
            return static_cast<std::uint8_t>(SensorId::GYROSCOPE_UNCALIBRATED);
        case 0xE308:
            return static_cast<std::uint8_t>(SensorId::RAW_MAGNETOMETER);
        case 0xE309:
            return static_cast<std::uint8_t>(SensorId::MAGNETIC_FIELD);
        case 0xE30A:
            return static_cast<std::uint8_t>(
                SensorId::MAGNETIC_FIELD_UNCALIBRATED);
        case 0xE30B:
            return static_cast<std::uint8_t>(SensorId::ROTATION_VECTOR);
        case 0xE30C:
            return static_cast<std::uint8_t>(SensorId::GAME_ROTATION_VECTOR);
        case 0xE30D:
            return static_cast<std::uint8_t>(
                SensorId::GEOMAGNETIC_ROTATION_VECTOR);
        case 0xE313:
            return static_cast<std::uint8_t>(SensorId::TAP_DETECTOR);
        case 0xE314:
            return static_cast<std::uint8_t>(SensorId::STEP_DETECTOR);
        case 0xE315:
            return static_cast<std::uint8_t>(SensorId::STEP_COUNTER);
        case 0xE317:
            return static_cast<std::uint8_t>(SensorId::STABILITY_CLASSIFIER);
        case 0xE322:
            return static_cast<std::uint8_t>(
                SensorId::ARVR_STABILIZED_ROTATION_VECTOR);
        case 0xE323:
            return static_cast<std::uint8_t>(
                SensorId::ARVR_STABILIZED_GAME_ROTATION_VECTOR);
        case 0xE324:
            return static_cast<std::uint8_t>(
                SensorId::GYRO_INTEGRATED_ROTATION_VECTOR);
        default:
            return std::nullopt;
    }
}

}  // namespace pico_sh2::reports
