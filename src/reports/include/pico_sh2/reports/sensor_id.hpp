// SPDX-License-Identifier: MIT
#pragma once

/// \file sensor_id.hpp
/// SH-2 sensor IDs for feature requests and input reports (1000-3625 §6.5).

#include <cstdint>

namespace pico_sh2::reports {

/// Sensor IDs used in feature reports; compact gyro input omits its ID byte.
enum class SensorId : std::uint8_t {
    ACCELEROMETER = 0x01,
    GYROSCOPE = 0x02,       // calibrated
    MAGNETIC_FIELD = 0x03,  // calibrated
    LINEAR_ACCELERATION = 0x04,
    ROTATION_VECTOR = 0x05,
    GRAVITY = 0x06,
    GYROSCOPE_UNCALIBRATED = 0x07,
    GAME_ROTATION_VECTOR = 0x08,
    GEOMAGNETIC_ROTATION_VECTOR = 0x09,
    MAGNETIC_FIELD_UNCALIBRATED = 0x0F,
    TAP_DETECTOR = 0x10,
    STEP_COUNTER = 0x11,
    SIGNIFICANT_MOTION = 0x12,
    STABILITY_CLASSIFIER = 0x13,
    RAW_ACCELEROMETER = 0x14,
    RAW_GYROSCOPE = 0x15,
    RAW_MAGNETOMETER = 0x16,
    STEP_DETECTOR = 0x18,
    ARVR_STABILIZED_ROTATION_VECTOR = 0x28,
    ARVR_STABILIZED_GAME_ROTATION_VECTOR = 0x29,
    GYRO_INTEGRATED_ROTATION_VECTOR = 0x2A,  // on the gyro-RV channel only
};

}  // namespace pico_sh2::reports
