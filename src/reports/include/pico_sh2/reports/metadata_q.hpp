// SPDX-License-Identifier: MIT
#pragma once

/// \file metadata_q.hpp
/// Sensor-metadata FRS record -> decode-time Q override (DR-2). A part
/// publishes authoritative Q points for each sensor in a sensor-metadata FRS
/// record (0xE301-0xE325; 1000-3625 §5.1). A record is an array of
/// 32-bit words whose word 7 packs the two Q points (Figures 28/29):
///
///   bits 15:0  = Q point 1 (signed), the data fields -> ReportQ::primary
///   bits 31:16 = Q point 2 (signed), bias/accuracy   -> ReportQ::secondary
///
/// The record header (words 0..8) is identical for record revisions 3 and 4, so
/// word 7 is read the same way regardless of the revision in word 3. Q point 2
/// applies to bias/accuracy unless otherwise noted (1000-3625 §5.1).
/// Its applicability to GIRV angular velocity is UNCONFIRMED: §6.5.44 fixes
/// Q10 and says nothing about Q point 2. The library keeps Q10. Figure 27 fixes
/// which sensor a record type describes; the map below covers the sensors this
/// library decodes.

#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>

#include "pico_sh2/reports/q_override.hpp"
#include "pico_sh2/reports/q_point.hpp"
#include "pico_sh2/reports/sensor_id.hpp"

namespace pico_sh2::reports {

/// Word index in a sensor-metadata record that packs Q point 1 (low half) and
/// Q point 2 (high half). 1000-3625 §5.1, Figures 28/29.
inline constexpr std::size_t METADATA_Q_WORD = 7;

/// Q points a sensor-metadata record carries are signed 16-bit fields; for the
/// in-scope sensors they lie in [0, MAX_Q_POINT] (fractional scaling; ReportQ
/// stores an unsigned exponent). A word 7 whose signed halves fall outside this
/// range is treated as malformed rather than installed.
inline constexpr std::int16_t METADATA_Q_MAX =
    static_cast<std::int16_t>(MAX_Q_POINT);

/// Extract the primary/secondary Q points from a sensor-metadata record's
/// words. Returns nullopt if the record is too short to reach word 7, or either
/// Q point is out of the representable range.
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

/// The input-report ID whose Q points a sensor-metadata record type carries
/// (SH-2 Reference Manual Figure 27), restricted to the sensors this library
/// decodes. Returns nullopt for a record type outside that set.
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
