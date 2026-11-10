// SPDX-License-Identifier: MIT
// FRS_MAX_WORDS = 65536 exceeds the 16-bit bound; the refused consumer case
// must fail on that bound's message, without another failure (AR-5).
#pragma once
#include <cstddef>

namespace consumer_profile {
struct Profile : ::pico_sh2::DefaultProfile {
    static constexpr std::size_t FRS_MAX_WORDS = 65536;
    static constexpr std::size_t SESSION_SIZE_CEILING = 1U << 20;
};
}  // namespace consumer_profile

#define PICO_SH2_ACTIVE_PROFILE ::consumer_profile::Profile
