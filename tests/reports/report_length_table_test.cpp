// SPDX-License-Identifier: MIT
#include "pico_sh2/reports/report_length_table.hpp"

#include <gtest/gtest.h>

#include <cstdint>

#include "pico_sh2/util/result.hpp"

namespace {

using pico_sh2::Sh2Error;
using pico_sh2::reports::ReportLength;
using pico_sh2::reports::ReportLengthTable;

TEST(ReportLengthTable, DefaultIsEmpty) {
    constexpr ReportLengthTable TABLE;
    EXPECT_FALSE(TABLE.contains(0x05));
    EXPECT_EQ(TABLE.length_of(0x05).error(), Sh2Error::UNSUPPORTED);
}

TEST(ReportLengthTable, SeededCoversCoreSensors) {
    constexpr ReportLengthTable TABLE = ReportLengthTable::seeded();
    EXPECT_EQ(TABLE.length_of(0x01).value_or(0), 10);  // Accelerometer
    EXPECT_EQ(TABLE.length_of(0x05).value_or(0), 14);  // Rotation vector
    EXPECT_EQ(TABLE.length_of(0x08).value_or(0), 12);  // Game rotation vector
    EXPECT_EQ(TABLE.length_of(0x14).value_or(0), 16);  // Raw accelerometer
}

TEST(ReportLengthTable, UnknownIdIsUnsupported) {
    constexpr ReportLengthTable TABLE = ReportLengthTable::seeded();
    // 0x2A (gyro-integrated RV) arrives alone on the gyro-RV input channel and
    // is not split by this TABLE.
    EXPECT_EQ(TABLE.length_of(0x2A).error(), Sh2Error::UNSUPPORTED);
    EXPECT_EQ(TABLE.length_of(0x77).error(), Sh2Error::UNSUPPORTED);
}

// A report ID and its length are both a byte; a call that passes them the
// wrong way round must not compile.
template <typename Id, typename Length>
concept SetsLengthWith = requires(ReportLengthTable table, Id id,
                                  Length length) { table.set(id, length); };
static_assert(SetsLengthWith<std::uint8_t, ReportLength>);
static_assert(!SetsLengthWith<ReportLength, std::uint8_t>);
static_assert(!SetsLengthWith<std::uint8_t, std::uint8_t>);

TEST(ReportLengthTable, SetOverridesSeed) {
    ReportLengthTable table = ReportLengthTable::seeded();
    table.set(0x05, ReportLength{20});
    EXPECT_EQ(table.length_of(0x05).value_or(0), 20);
    table.set(0x77, ReportLength{9});
    EXPECT_TRUE(table.contains(0x77));
    EXPECT_EQ(table.length_of(0x77).value_or(0), 9);
}

TEST(ReportLengthTable, IsConstexpr) {
    static_assert(ReportLengthTable::seeded().length_of(0x06).value_or(0) ==
                  10);
    SUCCEED();
}

}  // namespace
