// SPDX-License-Identifier: MIT
// A consumer's own sizing profile (AR-5), reached through
// PICO_SH2_PROFILE_HEADER by the declared consumer case "custom".
#pragma once
#include <cstddef>

namespace consumer_profile {
struct Profile : ::pico_sh2::DefaultProfile {
    static constexpr std::size_t MAX_CARGO_LEN = 512;
    static constexpr std::size_t SESSION_SIZE_CEILING = 2432;
};
}  // namespace consumer_profile

#define PICO_SH2_ACTIVE_PROFILE ::consumer_profile::Profile
