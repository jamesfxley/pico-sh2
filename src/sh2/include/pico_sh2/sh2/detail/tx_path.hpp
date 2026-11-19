// SPDX-License-Identifier: MIT
#pragma once

/// \file tx_path.hpp
/// Internal outbound queue; bus access waits for service() (AR-3).
/// Profile-sized storage allocates nothing (AR-1).
/// Requests that do not fit return an error (BR-2).

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>

#include "pico_sh2/sh2/hal_transport.hpp"
#include "pico_sh2/sh2/profile.hpp"
#include "pico_sh2/shtp/channels.hpp"
#include "pico_sh2/shtp/engine.hpp"
#include "pico_sh2/util/result.hpp"
#include "pico_util/containers/ring_buffer.hpp"

namespace pico_sh2::detail {

/// Queued payload with its raw wire channel number.
struct TxCargo {
    std::uint8_t channel = 0;  ///< Raw SHTP channel number.
    std::size_t length = 0;    ///< Payload bytes in use within `bytes`.
    /// Payload storage; only the first `length` bytes are meaningful.
    std::array<std::uint8_t, ActiveProfile::MAX_TX_CARGO> bytes{};
};

class TxPath {
   public:
    /// Borrow the session's role-to-channel map; it outlives this path.
    void bind_channels(const shtp::ChannelMap& channels) {
        channels_ = &channels;
    }

    /// Queue by role; NOT_READY means no map or neither a learned nor seeded
    /// number.
    Result<void> enqueue(shtp::ChannelRole role,
                         std::span<const std::uint8_t> payload) {
        if (channels_ == nullptr) {
            return Sh2Error::NOT_READY;
        }
        const std::optional<std::uint8_t> channel = channels_->number_of(role);
        if (!channel.has_value()) {
            return Sh2Error::NOT_READY;
        }
        return enqueue(*channel, payload);
    }

    /// Stage one cargo for an explicit channel number. QUEUE_FULL if the ring
    /// has no free slot or the payload exceeds a slot; the queue is unchanged
    /// either way.
    Result<void> enqueue(std::uint8_t channel,
                         std::span<const std::uint8_t> payload) {
        if (payload.size() > ActiveProfile::MAX_TX_CARGO) {
            return Sh2Error::QUEUE_FULL;
        }
        // Copying directly into the ring avoids a maximum-sized temporary.
        TxCargo* slot = queue_.acquire();
        if (slot == nullptr) {
            return Sh2Error::QUEUE_FULL;
        }
        slot->channel = channel;
        slot->length = payload.size();
        std::copy_n(payload.data(), payload.size(), slot->bytes.data());
        queue_.commit();
        return Result<void>{};
    }

    /// What one flush did.
    struct Flushed {
        std::uint32_t written = 0;  ///< Cargoes that reached the hub.
        bool write_refused =
            false;  ///< The HAL reported a write error; flush stopped.
    };

    /// Write until input is ready or a write fails. A refused transfer stays
    /// framed in scratch and is written again without re-framing (NR-7).
    /// Unframeable cargoes are dropped so they cannot block the queue.
    /// Pending input yields without error to the next receive pass.
    Flushed flush(shtp::ShtpEngine& engine, IHalTransport& hal) {
        Flushed flushed;
        while (const TxCargo* slot = queue_.front()) {
            if (held_length_ == 0) {
                auto framed =
                    engine.frame_outbound(slot->channel,
                                          std::span<const std::uint8_t>(
                                              slot->bytes.data(), slot->length),
                                          scratch_);
                if (!framed.ok()) {
                    queue_.pop();
                    continue;
                }
                held_length_ = framed.value();
            }
            auto written = hal.write_transfer(
                std::span<const std::uint8_t>(scratch_.data(), held_length_));
            if (!written.ok()) {
                flushed.write_refused = true;
                return flushed;
            }
            ++flushed.written;
            queue_.pop();
            held_length_ = 0;
            // A duplex write can fill the HAL's receive staging (AR-3).
            if (hal.data_ready()) {
                break;
            }
        }
        return flushed;
    }

    [[nodiscard]] bool empty() const { return queue_.empty(); }

    /// Drop a held frame without touching the queue; the queued cargo is
    /// framed again on the next flush. Call when SHTP framing state restarts,
    /// before the next flush.
    void discard_framed() { held_length_ = 0; }

    /// Discard prior-session requests; clear() is safe because producer and
    /// consumer share one context.
    void reset() {
        queue_.clear();
        held_length_ = 0;
    }

   private:
    const shtp::ChannelMap* channels_ = nullptr;
    // Requests and service() share one context; this ring is not synchronised.
    pico_util::RingBuffer<TxCargo, ActiveProfile::TX_QUEUE_DEPTH,
                          pico_util::SingleContextCounter>
        queue_;
    std::array<std::uint8_t, ActiveProfile::MAX_TX_CARGO + shtp::HEADER_SIZE>
        scratch_{};
    std::size_t held_length_ = 0;  ///< Framed bytes in scratch_; 0 = none.
};

}  // namespace pico_sh2::detail
