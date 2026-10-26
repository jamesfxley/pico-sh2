// SPDX-License-Identifier: MIT
#include "pico_sh2/reports/q_point.hpp"

#include <gtest/gtest.h>

namespace pico_sh2::reports {
namespace {

TEST(QPoint, ConvertsByPowerOfTwo) {
    // real = raw / 2^Q.
    EXPECT_FLOAT_EQ(q_to_float(256, 8), 1.0F);
    EXPECT_FLOAT_EQ(q_to_float(512, 8), 2.0F);
    EXPECT_FLOAT_EQ(q_to_float(128, 8), 0.5F);
    EXPECT_FLOAT_EQ(q_to_float(16384, 14), 1.0F);
    EXPECT_FLOAT_EQ(q_to_float(16, 4), 1.0F);
}

TEST(QPoint, HandlesNegativeAndZero) {
    EXPECT_FLOAT_EQ(q_to_float(-256, 8), -1.0F);
    EXPECT_FLOAT_EQ(q_to_float(0, 9), 0.0F);
}

TEST(QPoint, IsConstexpr) {
    static_assert(q_to_float(256, 8) == 1.0F);
    static_assert(q_to_float(-512, 9) == -1.0F);
    SUCCEED();
}

}  // namespace
}  // namespace pico_sh2::reports
