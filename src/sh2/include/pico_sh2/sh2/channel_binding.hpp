// SPDX-License-Identifier: MIT
#pragma once

/// \file channel_binding.hpp
/// Roles match by application GUID and channel name (SR-10).
/// Channel numbers are advertised (1000-3600 §2.3). The base may be selected
/// at compile time or run time, depending on the implementation (§2.3).
/// GUID matching avoids confusing SHTP's own "control" with SH-2 control.

#include <cstdint>
#include <optional>

#include "pico_sh2/shtp/advertisement.hpp"
#include "pico_sh2/shtp/channels.hpp"

namespace pico_sh2 {

/// The applications SH-2 advertises under (1000-3600 §2.3).
/// The executable application, which owns the device channel.
inline constexpr std::uint32_t EXECUTABLE_GUID = 1;
/// The sensorhub application, which owns the control and input channels.
inline constexpr std::uint32_t SENSORHUB_GUID = shtp::SENSORHUB_GUID;

/// SH-2 role, or nullopt for an unrecognised GUID or channel name.
constexpr std::optional<shtp::ChannelRole> role_of_advertised(
    const shtp::AdvertisedChannel& channel) {
    if (channel.guid == EXECUTABLE_GUID) {
        if (shtp::name_is(channel.name, "device")) {
            return shtp::ChannelRole::DEVICE;
        }
        return std::nullopt;
    }
    if (channel.guid != SENSORHUB_GUID) {
        return std::nullopt;
    }
    if (shtp::name_is(channel.name, "control")) {
        return shtp::ChannelRole::CONTROL;
    }
    if (shtp::name_is(channel.name, "inputNormal")) {
        return shtp::ChannelRole::INPUT_NORMAL;
    }
    if (shtp::name_is(channel.name, "inputWake")) {
        return shtp::ChannelRole::INPUT_WAKE;
    }
    if (shtp::name_is(channel.name, "inputGyroRv")) {
        return shtp::ChannelRole::INPUT_GYRO_RV;
    }
    return std::nullopt;
}

}  // namespace pico_sh2
