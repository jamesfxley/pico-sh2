// SPDX-License-Identifier: MIT
#pragma once

/// \file engine.hpp
/// SHTP framing with a caller-owned buffer (1000-3535 §2.2.1, §2.3.1).
/// Outbound cargo must fit one transfer under library policy.

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>

#include "pico_sh2/shtp/channels.hpp"
#include "pico_sh2/shtp/header.hpp"
#include "pico_sh2/util/result.hpp"
#include "pico_util/result/callback.hpp"

namespace pico_sh2::shtp {

/// Framing and reassembly without HAL or SH-2 semantics.
class ShtpEngine {
   public:
    /// One complete cargo, delivered to the sink.
    struct Cargo {
        /// SHTP channel number the cargo arrived on.
        std::uint8_t channel;
        /// Sequence number of the cargo's lead transfer.
        std::uint8_t sequence;
        /// Lead-transfer data-ready edge in microseconds, timing the cargo
        /// (1000-3535 §2.6).
        std::uint64_t lead_edge_us;
        /// Payload with the SHTP header stripped. Valid only for the duration
        /// of the call; consumers copy what they keep.
        std::span<const std::uint8_t> payload;
    };

    /// Receives each completed cargo, from inside ingest().
    using CargoSink = pico_util::Callback<void(const Cargo&)>;

    /// 32-bit counters, wrapping at 2^32 and cleared only at construction
    /// (OR-1).
    struct Stats {
        /// Whole cargoes handed to the sink.
        std::uint32_t cargoes_delivered = 0;
        /// Transfers carrying no payload, which an idle hub answers with.
        std::uint32_t empty_transfers = 0;
        /// Transfers whose header was reserved, short or otherwise malformed.
        std::uint32_t bad_headers = 0;
        /// Cargoes dropped whole: oversize, stray continuation or interruption.
        /// Continuations count once per dropped cargo on tracked channels; each
        /// stray continuation counts on untracked channels.
        std::uint32_t reassembly_drops = 0;
        /// Gaps in a channel's inbound sequence, so reports might have been
        /// lost.
        std::uint32_t sequence_gaps = 0;
    };

    /// reassembly_buffer must outlive the engine and hold the largest cargo the
    /// hub will send. Larger cargoes are dropped whole.
    explicit ShtpEngine(std::span<std::uint8_t> reassembly_buffer);

    /// \param sink receives each completed cargo.
    void set_cargo_sink(CargoSink sink) { sink_ = sink; }

    /// Ingest a transfer and deliver completed cargo with its lead edge.
    /// Idle is success; reserved, short or malformed headers return BAD_HEADER;
    /// discarded cargoes return REASSEMBLY_DROP.
    Result<void> ingest(std::span<const std::uint8_t> transfer,
                        std::uint64_t edge_us);

    /// Frame with the channel's next sequence and return the byte count.
    /// CARGO_TOO_LARGE means header and payload do not fit out or the length
    /// field.
    Result<std::size_t> frame_outbound(std::uint8_t channel,
                                       std::span<const std::uint8_t> payload,
                                       std::span<std::uint8_t> out);

    /// Discard reassembly and sequence state for reset; counters survive.
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

    /// Last inbound sequence per channel; -1 means none seen.
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
