// SPDX-License-Identifier: MIT
#pragma once

/// \file session_timing.hpp
/// Consumer-supplied startup and transaction timing in microseconds.
/// How long the hub must stay in reset and how quickly it boots or answers are
/// part-specific; the SH-2/SHTP manuals provide no defaults.
/// SessionTiming{} zero-initialises; begin() faults with INVALID_ARGUMENT.

#include <cstdint>

namespace pico_sh2 {

/// Part-specific timing in microseconds; every field has to be non-zero.
struct SessionTiming {
    /// How long the reset line is held asserted.
    std::uint64_t reset_hold_us;
    /// How long to wait after releasing reset before the hub is expected to
    /// speak.
    std::uint64_t boot_delay_us;
    /// Deadline for the advertisement, in microseconds.
    std::uint64_t advertisement_timeout_us;
    /// Deadline for the reset-complete announcement.
    std::uint64_t reset_complete_timeout_us;
    /// Deadline for the unsolicited Initialize response.
    std::uint64_t initialize_timeout_us;
    /// Deadline for an outstanding command.
    std::uint64_t command_timeout_us;
    /// Deadline for an FRS operation.
    std::uint64_t frs_timeout_us;

    /// True when every field is non-zero; minimum usable timing is
    /// part-specific.
    [[nodiscard]] constexpr bool valid() const {
        return reset_hold_us != 0 && boot_delay_us != 0 &&
               advertisement_timeout_us != 0 &&
               reset_complete_timeout_us != 0 && initialize_timeout_us != 0 &&
               command_timeout_us != 0 && frs_timeout_us != 0;
    }
};

}  // namespace pico_sh2
