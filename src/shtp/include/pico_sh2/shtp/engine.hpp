// SPDX-License-Identifier: MIT
#pragma once

/// \file engine.hpp
/// SHTP framing engine: the transport-layer core that turns a stream of raw
/// hub transfers into whole SH-2 cargoes and frames outbound cargoes for the
/// hub. It owns three concerns and nothing above them (no HAL, no SH-2 report
/// semantics):
///
///   * Reassembly. A cargo larger than one physical transfer is split by the
///     hub across a lead transfer (continuation flag clear) and continuation
///     transfers (flag set). The lead header's length is the total cargo plus
///     header (1000-3535 §2.2.1); each transfer's payload — its physical size
///     less the header — is appended until that many bytes have arrived, then
///     the whole cargo is delivered to the sink. Continuation length fields
///     carry the remaining count and are not consulted for completion; bytes
///     beyond the declared cargo are ignored (1000-3535 §2.3.1). An oversize
///     cargo (larger than the reassembly buffer), a stray continuation, or a
///     cargo interrupted by a new lead transfer is dropped whole and counted —
///     never truncated or delivered partial. (Device-validated at bring-up.)
///   * Sequence tracking. Each channel carries an independent 8-bit sequence
///   per
///     direction; an inbound gap is counted (reports may have been lost) but is
///     not fatal. Outbound transfers are stamped with the channel's next value.
///   * Outbound framing. The hub cannot reassemble, so every host->hub cargo
///     must fit one transfer; a cargo that does not fit the caller's transfer
///     buffer is rejected (CARGO_TOO_LARGE) rather than fragmented.
///
/// The reassembly buffer is supplied by the owner (the session sizes it from
/// the active profile), so the engine carries no profile dependency and is
/// sized by its caller.

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>

#include "pico_sh2/shtp/channels.hpp"
#include "pico_sh2/shtp/header.hpp"
#include "pico_sh2/util/result.hpp"
#include "pico_util/result/callback.hpp"

namespace pico_sh2::shtp {

/// Turns raw hub transfers into whole SH-2 cargoes and frames outbound cargoes
/// for the hub: reassembly, per-channel sequence tracking, and outbound
/// framing. Carries no HAL and no SH-2 report semantics.
class ShtpEngine {
   public:
    /// One complete cargo, delivered to the sink.
    struct Cargo {
        /// SHTP channel number the cargo arrived on.
        std::uint8_t channel;
        /// Sequence number of the cargo's lead transfer.
        std::uint8_t sequence;
        /// Data-ready edge of the lead transfer, as supplied to ingest(). A
        /// fragmented cargo carries the edge of the transfer that started it,
        /// which is what times the reports inside it (1000-3535 §2.6).
        std::uint64_t lead_edge_us;
        /// Payload with the SHTP header stripped. Valid only for the duration
        /// of the call; consumers copy what they keep.
        std::span<const std::uint8_t> payload;
    };

    /// Receives each completed cargo, from inside ingest().
    using CargoSink = pico_util::Callback<void(const Cargo&)>;

    /// Running counters for observability and tests. Each is a 32-bit count
    /// that wraps modulo 2^32, never decreases otherwise, and is cleared only
    /// by construction (OR-1).
    struct Stats {
        /// Whole cargoes handed to the sink.
        std::uint32_t cargoes_delivered = 0;
        /// Transfers carrying no payload, which an idle hub answers with.
        std::uint32_t empty_transfers = 0;
        /// Transfers whose header was reserved, short or otherwise malformed.
        std::uint32_t bad_headers = 0;
        /// Cargoes dropped whole: oversize, stray continuation, or interrupted.
        /// One per cargo, however many transfers it spanned: the continuations
        /// of a dropped cargo are discarded without counting again, until the
        /// next lead transfer on their channel. A channel at or past
        /// MAX_TRACKED_CHANNELS has no such memory, so each of its stray
        /// continuations counts.
        std::uint32_t reassembly_drops = 0;
        /// Gaps in a channel's inbound sequence, so reports may have been lost.
        std::uint32_t sequence_gaps = 0;
    };

    /// `reassembly_buffer` must outlive the engine and be large enough for the
    /// biggest cargo the hub will send (the advertisement, or the largest
    /// batched input cargo). A cargo larger than this is dropped, not
    /// truncated.
    explicit ShtpEngine(std::span<std::uint8_t> reassembly_buffer);

    /// \param sink receives each completed cargo.
    void set_cargo_sink(CargoSink sink) { sink_ = sink; }

    /// Feed one transfer read from the HAL: the 4-byte SHTP header followed by
    /// its payload. `edge_us` is the data-ready edge that announced this
    /// transfer; the engine keeps the lead transfer's value and hands it back
    /// with the completed cargo, so a caller never has to re-parse the header
    /// to learn whether a transfer started a cargo. Delivers a completed cargo
    /// to the sink as a side effect. Returns ok for a normal or benign-idle
    /// transfer; BAD_HEADER for a reserved/short header; REASSEMBLY_DROP when a
    /// cargo is discarded.
    Result<void> ingest(std::span<const std::uint8_t> transfer,
                        std::uint64_t edge_us);

    /// Frame one outbound cargo into `out` (header + payload) on `channel`,
    /// stamping the channel's next outbound sequence. Returns the number of
    /// bytes written. CARGO_TOO_LARGE if header + payload does not fit `out`.
    Result<std::size_t> frame_outbound(std::uint8_t channel,
                                       std::span<const std::uint8_t> payload,
                                       std::span<std::uint8_t> out);

    /// Drop any in-progress reassembly and clear all sequence state. Called
    /// when the hub resets so stale fragments or sequence numbers do not carry
    /// into the next session. Does not clear Stats.
    void reset();

    /// The running counters since construction, each wrapping modulo 2^32.
    [[nodiscard]] const Stats& stats() const { return stats_; }

   private:
    void track_rx_sequence(std::uint8_t channel, std::uint8_t sequence);
    void drop_in_progress();
    /// A lead transfer: deliver its cargo in place, drop it, or start
    /// reassembling it.
    Result<void> ingest_lead(const ShtpHeader& header,
                             std::span<const std::uint8_t> payload,
                             std::uint64_t edge_us);
    /// Add a transfer's payload to the cargo in progress, delivering it once
    /// the lead's declared length is reached.
    void append(std::span<const std::uint8_t> payload);
    /// Count one dropped cargo on `channel` and discard its continuations.
    void count_drop(std::uint8_t channel);
    /// Whether `channel`'s current cargo was already dropped and counted.
    [[nodiscard]] bool discarding(std::uint8_t channel) const;

    std::span<std::uint8_t> buffer_;
    std::size_t assembled_ = 0;
    std::size_t expected_ = 0;
    std::uint8_t active_channel_ = 0;
    std::uint8_t active_sequence_ = 0;
    std::uint64_t active_edge_us_ = 0;
    bool in_progress_ = false;

    /// Last inbound sequence per channel; -1 means "no packet seen yet". The
    /// constructor fills the sentinel; the in-class {} silences a member-init
    /// warning without changing behaviour.
    std::array<std::int16_t, MAX_TRACKED_CHANNELS> rx_sequence_{};
    /// Next outbound sequence per channel.
    std::array<std::uint8_t, MAX_TRACKED_CHANNELS> tx_sequence_{};
    /// Per channel: the cargo in flight was dropped, so its continuations are
    /// discarded silently until the channel's next lead transfer.
    std::array<bool, MAX_TRACKED_CHANNELS> discarding_{};

    CargoSink sink_;
    Stats stats_;
};

}  // namespace pico_sh2::shtp
