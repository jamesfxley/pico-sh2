// SPDX-License-Identifier: MIT
#include "pico_sh2/reports/timestamp.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <limits>

namespace pico_sh2::reports {
namespace {

constexpr std::uint64_t EDGE = 1'000'000;

TEST(Timestamp, BaseDeltaIsSubtractedFromTheEdge) {
    // 1000-3625 section 7.2.1, worked verbatim: a HINT at t with a Base Delta
    // of 10 ticks puts the batch's base at t - 1 ms (SR-4).
    const Timebase timebase = timebase_begin(EDGE, 10);
    EXPECT_EQ(timebase_event_us(timebase, 0), EDGE - 1000);
}

TEST(Timestamp, DelayMovesForwardFromTheBase) {
    const Timebase timebase = timebase_begin(EDGE, 10);
    EXPECT_EQ(timebase_event_us(timebase, 25), EDGE - 1000 + 2500);
}

TEST(Timestamp, RebaseAccumulatesForward) {
    // 1000-3625 section 7.2.2, worked verbatim in ticks: HINT at 5.0 s with a
    // base delta of 4.0 s gives a base of 1.0 s; a rebase of 1.5 s moves it to
    // 2.5 s; a report delay of 1.0 s times the event at 3.5 s.
    constexpr std::uint64_t HINT_US = 5'000'000;
    constexpr std::int32_t BASE_DELTA_TICKS = 40'000;  // 4.0 s
    constexpr std::int32_t REBASE_TICKS = 15'000;      // 1.5 s
    constexpr std::uint16_t DELAY_TICKS = 10'000;      // 1.0 s
    Timebase timebase = timebase_begin(HINT_US, BASE_DELTA_TICKS);
    EXPECT_EQ(timebase_event_us(timebase, 0), 1'000'000U);
    timebase = timebase_rebase(timebase, REBASE_TICKS);
    EXPECT_EQ(timebase_event_us(timebase, 0), 2'500'000U);
    EXPECT_EQ(timebase_event_us(timebase, DELAY_TICKS), 3'500'000U);
}

TEST(Timestamp, NegativeBaseDeltaMovesTheBaseAfterTheEdge) {
    const Timebase timebase = timebase_begin(EDGE, -5);
    EXPECT_EQ(timebase_event_us(timebase, 0), EDGE + 500);
}

TEST(Timestamp, OverflowFallsBackToEdgeAndIgnoresRebase) {
    Timebase timebase = timebase_begin(EDGE, TIMEBASE_OVERFLOW);
    EXPECT_TRUE(timebase.overflow);
    timebase = timebase_rebase(timebase, 1000);  // ignored while overflowed
    EXPECT_EQ(timebase_event_us(timebase, 42), EDGE);
}

TEST(Timestamp, ClampsBelowZeroToZero) {
    const Timebase timebase = timebase_begin(0, 100);
    EXPECT_EQ(timebase_event_us(timebase, 0), 0U);
}

// SR-4: an edge anywhere in the 64-bit range has a defined result. A time past
// the top of the range saturates rather than wrapping, and an edge past the
// signed range is not reinterpreted as a negative time.
TEST(Timestamp, SaturatesAtTheTopOfTheRange) {
    constexpr std::uint64_t TOP = std::numeric_limits<std::uint64_t>::max();
    EXPECT_EQ(timebase_event_us(timebase_begin(TOP - 50, -1), 0), TOP);
    EXPECT_EQ(timebase_event_us(timebase_begin(TOP - 50, 1), 0), TOP - 150);

    Timebase far = timebase_begin(EDGE, 0);
    for (int rebase = 0; rebase < 8; ++rebase) {
        far = timebase_rebase(far, std::numeric_limits<std::int32_t>::max());
    }
    EXPECT_EQ(timebase_event_us(far, 16383),
              EDGE + (((8 * std::uint64_t{0x7FFFFFFF}) + 16383) * 100));
}

TEST(Timestamp, IsConstexpr) {
    static_assert(timebase_event_us(timebase_begin(1000, 5), 5) == 1000);
    SUCCEED();
}

}  // namespace
}  // namespace pico_sh2::reports
