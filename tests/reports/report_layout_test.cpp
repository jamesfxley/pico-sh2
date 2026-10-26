// SPDX-License-Identifier: MIT
#include "pico_sh2/reports/report_layout.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cstdint>
#include <span>

namespace pico_sh2::reports {
namespace {

// The prefix is taken as its four bytes in wire order, so there is no argument
// order to get wrong: four loose bytes, in any order, do not compile.
template <typename... Bytes>
concept DecodesLooseBytes =
    requires(Bytes... bytes) { decode_prefix(bytes...); };
static_assert(
    !DecodesLooseBytes<std::uint8_t, std::uint8_t, std::uint8_t, std::uint8_t>);
static_assert(
    DecodesLooseBytes<std::span<const std::uint8_t, INPUT_REPORT_PREFIX_SIZE>>);

// Decode the prefix bytes given in wire order.
constexpr InputReportPrefix decode(
    std::array<std::uint8_t, INPUT_REPORT_PREFIX_SIZE> wire) {
    return decode_prefix(wire);
}

TEST(ReportPrefix, DecodesAccuracyFromLowBits) {
    EXPECT_EQ(decode({0x01, 0, 0x00, 0}).accuracy, SensorAccuracy::UNRELIABLE);
    EXPECT_EQ(decode({0x01, 0, 0x01, 0}).accuracy, SensorAccuracy::LOW);
    EXPECT_EQ(decode({0x01, 0, 0x02, 0}).accuracy, SensorAccuracy::MEDIUM);
    EXPECT_EQ(decode({0x01, 0, 0x03, 0}).accuracy, SensorAccuracy::HIGH);
}

TEST(ReportPrefix, DelayCombinesStatusHighBitsAndDelayByte) {
    // Delay = (status[7:2] << 8) | delay_lsb, in 100 us units.
    // status 0x07 -> accuracy HIGH (3), delay high bits = 0x07 >> 2 = 1.
    const InputReportPrefix prefix = decode({0x01, 0x2A, 0x07, 0x34});
    EXPECT_EQ(prefix.report_id, 0x01);
    EXPECT_EQ(prefix.sequence, 0x2A);
    EXPECT_EQ(prefix.accuracy, SensorAccuracy::HIGH);
    EXPECT_EQ(prefix.delay_100us, (1U << 8U) | 0x34U);
}

TEST(ReportPrefix, MaxDelay) {
    // status high six bits all set (0xFC) and delay 0xFF -> 0x3FFF = 16383.
    EXPECT_EQ(decode({0x01, 0, 0xFC, 0xFF}).delay_100us, 0x3FFFU);
}

}  // namespace
}  // namespace pico_sh2::reports
