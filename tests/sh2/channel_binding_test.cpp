// SPDX-License-Identifier: MIT
// SR-10: a role is bound only from its own application's GUID. The names SH-2
// looks for are ordinary words another application may use for a channel of
// its own, so a name matches only inside the scope that defines it.
#include "pico_sh2/sh2/channel_binding.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <optional>
#include <string_view>

#include "pico_sh2/shtp/advertisement.hpp"
#include "pico_sh2/shtp/channels.hpp"
#include "pico_util/bytes/as_bytes.hpp"

namespace {

using pico_sh2::shtp::AdvertisedChannel;
using pico_sh2::shtp::ChannelRole;

AdvertisedChannel channel(std::uint32_t guid, std::string_view name) {
    AdvertisedChannel advertised;
    advertised.guid = guid;
    advertised.number = 3;
    advertised.name = pico_util::as_bytes(name);
    return advertised;
}

TEST(ChannelBinding, EachRoleBindsUnderItsOwnApplication) {
    EXPECT_EQ(pico_sh2::role_of_advertised(channel(1, "device")),
              ChannelRole::DEVICE);
    EXPECT_EQ(pico_sh2::role_of_advertised(channel(2, "control")),
              ChannelRole::CONTROL);
    EXPECT_EQ(pico_sh2::role_of_advertised(channel(2, "inputNormal")),
              ChannelRole::INPUT_NORMAL);
    EXPECT_EQ(pico_sh2::role_of_advertised(channel(2, "inputWake")),
              ChannelRole::INPUT_WAKE);
    EXPECT_EQ(pico_sh2::role_of_advertised(channel(2, "inputGyroRv")),
              ChannelRole::INPUT_GYRO_RV);
}

TEST(ChannelBinding, ASensorhubNameUnderAnotherApplicationBindsNothing) {
    // SHTP's own command channel is named "control" too.
    EXPECT_EQ(pico_sh2::role_of_advertised(channel(0, "control")),
              std::nullopt);
    EXPECT_EQ(pico_sh2::role_of_advertised(channel(1, "control")),
              std::nullopt);
    EXPECT_EQ(pico_sh2::role_of_advertised(channel(0xEFBEADDE, "inputNormal")),
              std::nullopt);
    EXPECT_EQ(pico_sh2::role_of_advertised(channel(2, "device")), std::nullopt);
}

}  // namespace
