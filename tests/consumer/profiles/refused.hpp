// SPDX-License-Identifier: MIT
// A profile outside AR-5's range, used by the declared consumer case
// "refused": FRS_MAX_WORDS = 65536 is beyond the 16-bit FRS length, so the
// build must fail on that bound's message and not for another reason.
#pragma once
#include <cstddef>

namespace consumer_profile {
struct Profile : ::pico_sh2::DefaultProfile {
    static constexpr std::size_t FRS_MAX_WORDS = 65536;
    static constexpr std::size_t SESSION_SIZE_CEILING = 1U << 20;
};
}  // namespace consumer_profile

#define PICO_SH2_ACTIVE_PROFILE ::consumer_profile::Profile
