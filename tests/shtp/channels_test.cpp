// SPDX-License-Identifier: MIT
#include "pico_sh2/shtp/channels.hpp"

#include <gtest/gtest.h>

namespace pico_sh2::shtp {
namespace {

// SR-10: the command channel is the one number SHTP fixes (1000-3535 §5.1), so
// it is usable before any advertisement; SH-2's roles are not, because their
// base is the hub's to choose (1000-3600 §2.3).
TEST(ChannelMap, OnlyTheCommandChannelIsKnownUpFront) {
    const ChannelMap map;
    EXPECT_EQ(map.number_of(ChannelRole::COMMAND).value_or(0xFF),
              COMMAND_CHANNEL);
    EXPECT_FALSE(map.number_of(ChannelRole::CONTROL).has_value());
    EXPECT_FALSE(map.number_of(ChannelRole::DEVICE).has_value());
    EXPECT_FALSE(map.complete());
}

TEST(ChannelMap, SeedingBindsARoleAndReversesIt) {
    ChannelMap map;
    map.seed(ChannelRole::DEVICE, 1);
    EXPECT_EQ(map.number_of(ChannelRole::DEVICE).value_or(0xFF), 1);
    EXPECT_EQ(map.role_of(1).value_or(ChannelRole::COMMAND),
              ChannelRole::DEVICE);
    // A number nothing claims has no role, which is what makes an unexpected
    // channel countable rather than routable.
    EXPECT_FALSE(map.role_of(9).has_value());
}

// An advertisement is authoritative over whatever the consumer guessed.
TEST(ChannelMap, LearningOverridesASeed) {
    ChannelMap map;
    map.seed(ChannelRole::CONTROL, 2);
    map.learn(ChannelRole::CONTROL, 7);
    EXPECT_EQ(map.number_of(ChannelRole::CONTROL).value_or(0xFF), 7);
    EXPECT_EQ(map.role_of(7).value_or(ChannelRole::COMMAND),
              ChannelRole::CONTROL);
    EXPECT_FALSE(map.role_of(2).has_value());
}

// A seed arriving after an advertisement must not undo what the hub said.
TEST(ChannelMap, ASeedDoesNotOverrideWhatWasLearned) {
    ChannelMap map;
    map.learn(ChannelRole::CONTROL, 7);
    map.seed(ChannelRole::CONTROL, 2);
    EXPECT_EQ(map.number_of(ChannelRole::CONTROL).value_or(0xFF), 7);
}

// SR-5/SR-7: a hub that reset may come back at a different base, so learned
// numbers are dropped — but the seed survives, which is what keeps a reset
// requestable while the session is short of ready.
TEST(ChannelMap, ForgettingLearnedFallsBackToTheSeed) {
    ChannelMap map;
    map.seed(ChannelRole::DEVICE, 1);
    map.learn(ChannelRole::DEVICE, 4);
    map.learn(ChannelRole::CONTROL, 5);
    ASSERT_EQ(map.number_of(ChannelRole::DEVICE).value_or(0xFF), 4);

    map.forget_learned();
    EXPECT_EQ(map.number_of(ChannelRole::DEVICE).value_or(0xFF), 1);
    // CONTROL was never seeded, so forgetting leaves it unbound entirely.
    EXPECT_FALSE(map.number_of(ChannelRole::CONTROL).has_value());
    EXPECT_EQ(map.number_of(ChannelRole::COMMAND).value_or(0xFF),
              COMMAND_CHANNEL);
}

TEST(ChannelMap, CompleteOnlyOnceEveryRoleHasANumber) {
    ChannelMap map;
    map.learn(ChannelRole::DEVICE, 1);
    map.learn(ChannelRole::CONTROL, 2);
    map.learn(ChannelRole::INPUT_NORMAL, 3);
    map.learn(ChannelRole::INPUT_WAKE, 4);
    EXPECT_FALSE(map.complete());
    map.learn(ChannelRole::INPUT_GYRO_RV, 5);
    EXPECT_TRUE(map.complete());
}

// The base is the hub's choice, so a map bound to a shifted block must be just
// as usable as one at the base a common part happens to use.
TEST(ChannelMap, BindsAtAnyBase) {
    ChannelMap map;
    map.learn(ChannelRole::DEVICE, 10);
    map.learn(ChannelRole::CONTROL, 11);
    map.learn(ChannelRole::INPUT_NORMAL, 12);
    map.learn(ChannelRole::INPUT_WAKE, 13);
    map.learn(ChannelRole::INPUT_GYRO_RV, 14);
    EXPECT_TRUE(map.complete());
    EXPECT_EQ(map.role_of(12).value_or(ChannelRole::COMMAND),
              ChannelRole::INPUT_NORMAL);
}

TEST(Channels, LengthFieldMasks) {
    EXPECT_EQ(CONTINUATION_MASK, 0x8000);
    EXPECT_EQ(LENGTH_MASK, 0x7FFF);
    EXPECT_EQ(RESERVED_LENGTH, 0xFFFF);
    EXPECT_EQ(HEADER_SIZE, 4U);
}

}  // namespace
}  // namespace pico_sh2::shtp
