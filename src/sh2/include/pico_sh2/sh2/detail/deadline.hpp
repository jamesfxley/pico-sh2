// SPDX-License-Identifier: MIT
#pragma once

#include <cstdint>
#include <limits>

/// \file deadline.hpp
/// The one place an absolute deadline is formed from a start and a timeout.

namespace pico_sh2::detail {

/// Saturating microsecond deadline, so a long timeout cannot expire by wrapping
/// (TR-1).
/// \param now_us the current time in microseconds.
/// \param timeout_us how long the operation may take, in microseconds.
/// \return the deadline, or the clock's last value if the sum would wrap.
[[nodiscard]] constexpr std::uint64_t deadline_after(std::uint64_t now_us,
                                                     std::uint64_t timeout_us) {
    constexpr std::uint64_t END = std::numeric_limits<std::uint64_t>::max();
    return timeout_us > END - now_us ? END : now_us + timeout_us;
}

}  // namespace pico_sh2::detail
