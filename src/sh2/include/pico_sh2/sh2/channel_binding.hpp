// SPDX-License-Identifier: MIT
#pragma once

/// \file channel_binding.hpp
/// SR-10: SH-2's channel vocabulary — which advertised channel plays which
/// role.
///
/// 1000-3600 §2.2 names SH-2's five channels device, control, inputNormal,
/// inputWake and inputGyroRv, and §2.3 shows how they are advertised — the
/// executable application (GUID 1) owning `device`, the sensorhub (GUID 2)
/// owning the rest. §2.3 also states that the base channel number "may be
/// selected at compile time or run time depending on the specific
/// implementation", which is why the numbers are read from the hub rather than
/// written down here.
///
/// Matching is scoped to the advertising GUID and not done on the name alone.
/// The SHTP application advertises a channel named "control" as well — it is
/// the command channel — so a name-only match binds SH-2's control role to
/// channel 0 and every control write goes to the wrong place.

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

/// The role an advertised channel plays, or nothing if it is not one of SH-2's.
/// Channels belonging to any other application, and SHTP's own, are not SH-2's
/// to route.
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
