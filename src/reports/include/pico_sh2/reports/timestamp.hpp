// SPDX-License-Identifier: MIT
#pragma once

/// \file timestamp.hpp
/// Timestamp reconstruction (SH-2 Reference Manual, section 7.2). Batched input
/// reports do not carry absolute times; each carries a 14-bit delay (100 us
/// units, at most 16383 ticks). A Base Timestamp Reference record (0xFB) gives
/// a signed Base Delta in 100 us ticks relative to the transport reference
/// point (the HINTN edge that triggered the read). A Timestamp Rebase record
/// (0xFA) carries a further signed delta, used when a batch spans more than the
/// delay field can express. An event's absolute time is:
///
///   event_us = intn_edge_us - 100 * base_delta + 100 * (rebases +
///   report_delay)
///
/// The Base Delta is subtracted: it measures how far the batch's shared base
/// sits *before* the reference point. 1000-3625 section 7.2.1 states that for a
/// HINT at time t and a Base Delta of 10, the batch's timestamps are relative
/// to t - 1 ms; section 7.2.2 works the same example with a base delta of 4.0 s
/// against a HINT at 5.0 s giving a base of 1.0 s. Rebases and the per-report
/// delay both move forward from that base.
///
/// Base Delta = 0x7FFFFFFF is reserved: it means the true delta exceeded the
/// 32-bit range. Such events are marked overflowed and fall back to the HINTN
/// edge time rather than reporting a wrong absolute time.

#include <algorithm>
#include <cstdint>
#include <limits>

namespace pico_sh2::reports {

/// Reserved Base Delta value: the delta could not be represented in 32 bits.
inline constexpr std::int32_t TIMEBASE_OVERFLOW = 0x7FFFFFFF;

/// Microseconds per 100 us tick, the unit of the base/rebase deltas and the
/// report delay field.
inline constexpr std::uint64_t TICK_US = 100;

/// A running timebase for one input cargo: the HINTN edge the read was
/// triggered from, plus the accumulated signed offset in 100 us ticks — the
/// negated base delta, plus any rebases. `overflow` latches when the base delta
/// was the reserved value.
struct Timebase {
    /// The HINTN edge the cargo's read was triggered from.
    std::uint64_t intn_edge_us;
    /// Accumulated offset from that edge, in 100 us ticks.
    std::int64_t offset_ticks;
    /// Latches when the base delta was the reserved overflow value, so times
    /// derived from this base fall back to the edge itself.
    bool overflow;
};

/// Begin a timebase from a 0xFB Base Delta captured at the given HINTN edge.
/// The delta is negated on the way in, so the stored offset is already the
/// signed displacement from the edge to the batch's shared base.
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

/// Absolute time of a report with the given 14-bit delay. On an overflowed base
/// the HINTN edge time is returned. A computed time before zero is clamped to
/// 0, and one past the top of the range to its maximum: every input, including
/// an edge anywhere in the 64-bit range, has a defined result.
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
