// SPDX-License-Identifier: MIT
#pragma once

/// \file timestamp.hpp
/// Event-time reconstruction in microseconds (1000-3625 §7.2).

#include <algorithm>
#include <cstdint>
#include <limits>

namespace pico_sh2::reports {

/// Reserved Base Delta value: the delta could not be represented in 32 bits.
inline constexpr std::int32_t TIMEBASE_OVERFLOW = 0x7FFFFFFF;

/// Microseconds per 100 us tick, the unit of the base/rebase deltas and the
/// report delay field.
inline constexpr std::uint64_t TICK_US = 100;

/// Timebase for one cargo.
/// Its offset is the negated Base Delta plus rebases, in 100-microsecond ticks.
struct Timebase {
    /// The HINTN edge the cargo's read was triggered from.
    std::uint64_t intn_edge_us;
    /// Accumulated offset from that edge, in 100 us ticks.
    std::int64_t offset_ticks;
    /// Latches when the base delta was the reserved overflow value, so times
    /// derived from this base fall back to the edge itself.
    bool overflow;
};

/// Start from Base Delta; negation gives the signed displacement from HINTN.
constexpr Timebase timebase_begin(std::uint64_t intn_edge_us,
                                  std::int32_t base_delta) {
    if (base_delta == TIMEBASE_OVERFLOW) {
        return Timebase{
            .intn_edge_us = intn_edge_us, .offset_ticks = 0, .overflow = true};
    }
    return Timebase{.intn_edge_us = intn_edge_us,
                    .offset_ticks = -static_cast<std::int64_t>(base_delta),
                    .overflow = false};
}

/// Apply a 0xFA Rebase Delta. A rebase after an overflowed base is ignored: the
/// base is already untrustworthy.
constexpr Timebase timebase_rebase(Timebase base, std::int32_t rebase_delta) {
    if (!base.overflow) {
        base.offset_ticks += rebase_delta;
    }
    return base;
}

/// Event time for a 14-bit delay; overflow returns the HINTN edge.
/// Computed times saturate at zero and UINT64_MAX, including edges across the
/// full range.
constexpr std::uint64_t timebase_event_us(Timebase base,
                                          std::uint16_t delay_100us) {
    if (base.overflow) {
        return base.intn_edge_us;
    }
    // Bounded so the conversion to microseconds cannot overflow; a
    // displacement this large is already past either end of the range.
    constexpr std::int64_t MAX_TICKS =
        std::numeric_limits<std::int64_t>::max() /
        static_cast<std::int64_t>(TICK_US);
    const std::int64_t total_ticks =
        std::clamp(base.offset_ticks, -MAX_TICKS, MAX_TICKS - delay_100us) +
        delay_100us;
    const std::uint64_t displacement_us =
        (total_ticks < 0 ? static_cast<std::uint64_t>(-total_ticks)
                         : static_cast<std::uint64_t>(total_ticks)) *
        TICK_US;
    if (total_ticks < 0) {
        return displacement_us > base.intn_edge_us
                   ? 0
                   : base.intn_edge_us - displacement_us;
    }
    const std::uint64_t headroom =
        std::numeric_limits<std::uint64_t>::max() - base.intn_edge_us;
    return displacement_us > headroom
               ? std::numeric_limits<std::uint64_t>::max()
               : base.intn_edge_us + displacement_us;
}

}  // namespace pico_sh2::reports
