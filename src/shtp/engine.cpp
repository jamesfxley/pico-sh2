// SPDX-License-Identifier: MIT
#include "pico_sh2/shtp/engine.hpp"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <span>

#include "pico_sh2/shtp/channels.hpp"
#include "pico_sh2/shtp/header.hpp"
#include "pico_sh2/util/result.hpp"

namespace pico_sh2::shtp {

ShtpEngine::ShtpEngine(std::span<std::uint8_t> reassembly_buffer)
    : buffer_(reassembly_buffer) {
    rx_sequence_.fill(-1);
}

void ShtpEngine::reset() {
    drop_in_progress();
    rx_sequence_.fill(-1);
    tx_sequence_.fill(0);
    discarding_.fill(false);
}

void ShtpEngine::count_drop(std::uint8_t channel) {
    ++stats_.reassembly_drops;
    if (channel < MAX_TRACKED_CHANNELS) {
        discarding_[channel] = true;
    }
}

bool ShtpEngine::discarding(std::uint8_t channel) const {
    return channel < MAX_TRACKED_CHANNELS && discarding_[channel];
}

void ShtpEngine::drop_in_progress() {
    in_progress_ = false;
    assembled_ = 0;
    expected_ = 0;
}

void ShtpEngine::track_rx_sequence(std::uint8_t channel,
                                   std::uint8_t sequence) {
    if (channel >= MAX_TRACKED_CHANNELS) {
        return;
    }
    const std::int16_t last = rx_sequence_[channel];
    if (last >= 0) {
        const std::uint8_t next = static_cast<std::uint8_t>(last) + 1;
        if (sequence != next) {
            ++stats_.sequence_gaps;
        }
    }
    rx_sequence_[channel] = sequence;
}

Result<void> ShtpEngine::ingest(std::span<const std::uint8_t> transfer,
                                std::uint64_t edge_us) {
    auto header = parse_header(transfer);
    if (!header.ok()) {
        ++stats_.bad_headers;
        return header.error();
    }
    if (header.value().is_empty()) {
        // Zero-length idle transfer: polled with no data pending.
        ++stats_.empty_transfers;
        return Result<void>{};
    }

    const std::uint8_t channel = header.value().channel;
    track_rx_sequence(channel, header.value().sequence);

    // Payload physically present in this transfer, independent of the length
    // field (a lead fragment reports the whole cargo but carries only part).
    const std::span<const std::uint8_t> payload =
        transfer.size() > HEADER_SIZE ? transfer.subspan(HEADER_SIZE)
                                      : std::span<const std::uint8_t>{};

    if (!header.value().continuation) {
        return ingest_lead(header.value(), payload, edge_us);
    }
    // Continuation transfer: must extend an in-progress cargo on the same
    // channel. A stray or cross-channel continuation is a protocol error.
    if (!in_progress_ || channel != active_channel_) {
        if (in_progress_) {
            count_drop(active_channel_);
            drop_in_progress();
        }
        // The rest of a cargo already dropped and counted.
        if (!discarding(channel)) {
            count_drop(channel);
        }
        return Sh2Error::REASSEMBLY_DROP;
    }
    append(payload);
    return Result<void>{};
}

Result<void> ShtpEngine::ingest_lead(const ShtpHeader& header,
                                     std::span<const std::uint8_t> payload,
                                     std::uint64_t edge_us) {
    // A new cargo. Any in-progress cargo was interrupted.
    if (header.channel < MAX_TRACKED_CHANNELS) {
        discarding_[header.channel] = false;
    }
    if (in_progress_) {
        count_drop(active_channel_);
        drop_in_progress();
    }
    // Completion uses the lead's total length; continuation length fields do
    // not set the completion boundary.
    const std::size_t expected = header.cargo_length();
    if (expected > buffer_.size()) {
        // An oversize cargo is dropped whole.
        count_drop(header.channel);
        return Sh2Error::REASSEMBLY_DROP;
    }
    if (payload.size() >= expected) {
        // Deliver single-transfer cargo in place; only fragmented cargo is
        // copied (AR-2).
        ++stats_.cargoes_delivered;
        // Preserve the wire channel so the session can count unrouted cargoes.
        sink_(Cargo{.channel = header.channel,
                    .sequence = header.sequence,
                    .lead_edge_us = edge_us,
                    .payload = payload.first(expected)});
        return Result<void>{};
    }
    // The lead edge times the whole fragmented cargo (1000-3535 §2.6).
    expected_ = expected;
    in_progress_ = true;
    assembled_ = 0;
    active_channel_ = header.channel;
    active_sequence_ = header.sequence;
    active_edge_us_ = edge_us;
    append(payload);
    return Result<void>{};
}

void ShtpEngine::append(std::span<const std::uint8_t> payload) {
    // Ignore bytes beyond the lead's declared cargo length.
    const std::size_t take = std::min(payload.size(), expected_ - assembled_);
    std::copy_n(payload.data(), take, buffer_.data() + assembled_);
    assembled_ += take;

    if (assembled_ >= expected_) {
        const std::span<const std::uint8_t> cargo(buffer_.data(), expected_);
        const Cargo complete{.channel = active_channel_,
                             .sequence = active_sequence_,
                             .lead_edge_us = active_edge_us_,
                             .payload = cargo};
        drop_in_progress();
        ++stats_.cargoes_delivered;
        sink_(complete);
    }
}

Result<std::size_t> ShtpEngine::frame_outbound(
    std::uint8_t channel, std::span<const std::uint8_t> payload,
    std::span<std::uint8_t> out) {
    const std::size_t transfer_length = payload.size() + HEADER_SIZE;
    if (transfer_length > out.size()) {
        // Writes are unfragmented.
        return Sh2Error::CARGO_TOO_LARGE;
    }
    const std::uint8_t index = channel;
    const std::uint8_t sequence =
        index < MAX_TRACKED_CHANNELS ? tx_sequence_[index] : std::uint8_t{0};

    auto framed = serialize_header(out, channel, SequenceNumber{sequence},
                                   payload.size());
    if (!framed.ok()) {
        return framed.error();
    }
    std::copy_n(payload.data(), payload.size(), out.data() + HEADER_SIZE);

    if (index < MAX_TRACKED_CHANNELS) {
        tx_sequence_[index] = static_cast<std::uint8_t>(sequence + 1);
    }
    return transfer_length;
}

}  // namespace pico_sh2::shtp
