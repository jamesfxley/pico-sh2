// SPDX-License-Identifier: MIT
#pragma once

/// \file session_timing.hpp
/// Deadlines for the reset/startup sequence and for control transactions, in
/// microseconds.
///
/// None of these carry a default. How long a hub must be held in reset, how
/// long it takes to boot, and how quickly it answers a command are properties
/// of the part, not of SH-2 or SHTP — no manual in `docs/` states them — so a
/// value written here would be one device's characteristics baked into a
/// protocol library. The consumer, which knows its part, supplies them.
///
/// `SessionTiming{}` therefore zero-initialises rather than producing something
/// workable. `Session::begin()` rejects that: a zero reset hold or boot delay
/// would release the hub before it is ready and a zero deadline would expire
/// immediately, so the session faults with INVALID_ARGUMENT instead of hanging
/// in a startup phase that can never complete.

#include <cstdint>

namespace pico_sh2 {

/// The startup and transaction deadlines the session runs to. Every field is a
/// property of the part, so there are no defaults; valid() rejects a zero.
struct SessionTiming {
    /// How long the reset line is held asserted.
    std::uint64_t reset_hold_us;
    /// How long to wait after releasing reset before the hub is expected to
    /// speak.
    std::uint64_t boot_delay_us;
    /// Deadline for each of the three post-reset announcements.
    std::uint64_t advertisement_timeout_us;
    /// Deadline for the reset-complete announcement.
    std::uint64_t reset_complete_timeout_us;
    /// Deadline for the unsolicited Initialize response.
    std::uint64_t initialize_timeout_us;
    /// Deadline for an outstanding command.
    std::uint64_t command_timeout_us;
    /// Deadline for an FRS operation.
    std::uint64_t frs_timeout_us;

    /// True when every field is usable. Zero is the only value ruled out here:
    /// the library cannot know what is too short for a given part, only that
    /// nothing can work with no time at all.
    [[nodiscard]] constexpr bool valid() const {
        return reset_hold_us != 0 && boot_delay_us != 0 &&
               advertisement_timeout_us != 0 &&
               reset_complete_timeout_us != 0 && initialize_timeout_us != 0 &&
               command_timeout_us != 0 && frs_timeout_us != 0;
    }
};

}  // namespace pico_sh2
