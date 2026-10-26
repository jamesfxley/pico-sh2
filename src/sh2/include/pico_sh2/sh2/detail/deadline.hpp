// SPDX-License-Identifier: MIT
#pragma once

#include <cstdint>
#include <limits>

/// \file deadline.hpp
/// The one place an absolute deadline is formed from a start and a timeout.

namespace pico_sh2::detail {

/// The time `timeout_us` after `now_us`, saturated at the clock's end (TR-1).
///
/// A plain sum wraps for a long timeout -- SessionTiming accepts any non-zero
/// value, and the longest is the natural spelling of "never" -- into a time
/// already past, so the operation would expire at the next service().
///
/// \param now_us the current time.
/// \param timeout_us how long the operation may take.
/// \return the deadline, or the clock's last value if the sum would wrap.
[[nodiscard]] constexpr std::uint64_t deadline_after(std::uint64_t now_us,
                                                     std::uint64_t timeout_us) {
    constexpr std::uint64_t END = std::numeric_limits<std::uint64_t>::max();
    return timeout_us > END - now_us ? END : now_us + timeout_us;
}

}  // namespace pico_sh2::detail
