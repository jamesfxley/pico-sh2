// SPDX-License-Identifier: MIT
#include "pico_sh2/reports/sensor_id.hpp"

#include <gtest/gtest.h>

#include <cstdint>

namespace pico_sh2::reports {
namespace {

TEST(SensorId, WireValues) {
    EXPECT_EQ(static_cast<std::uint8_t>(SensorId::ACCELEROMETER), 0x01);
    EXPECT_EQ(static_cast<std::uint8_t>(SensorId::ROTATION_VECTOR), 0x05);
    EXPECT_EQ(static_cast<std::uint8_t>(SensorId::MAGNETIC_FIELD_UNCALIBRATED),
              0x0F);
    EXPECT_EQ(
        static_cast<std::uint8_t>(SensorId::GYRO_INTEGRATED_ROTATION_VECTOR),
        0x2A);
}

}  // namespace
}  // namespace pico_sh2::reports
