// SPDX-License-Identifier: MIT
#pragma once

/// \file tx_path.hpp
/// Outbound path: the queue a consumer's requests are staged in and the flush
/// that frames them and hands them to the HAL.
///
/// Requests arrive from consumer calls at any time, but every bus access
/// happens inside service() (AR-3), so a request is queued when it is made and
/// written on the next service() pass. The queue and its framing scratch are
/// sized from the active profile and allocate nothing (AR-1); a request that
/// does not fit is refused so the caller sees the failure (BR-2).
///
/// This is an internal collaborator of Session, not public API.

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

/// One staged cargo: the channel to send it on and its payload bytes.
///
/// Layer-independent by construction -- the channel is its raw wire number, so
/// nothing here depends on the SHTP or SH-2 layers.
struct TxCargo {
    std::uint8_t channel = 0;  ///< Raw SHTP channel number.
    std::size_t length = 0;    ///< Payload bytes in use within `bytes`.
    /// Payload storage; only the first `length` bytes are meaningful.
    std::array<std::uint8_t, ActiveProfile::MAX_TX_CARGO> bytes{};
};

class TxPath {
   public:
    /// The map that turns a role into a wire number. Bound once by the session
    /// that owns both; until then, and for any role the hub has not advertised
    /// and no consumer seeded, a role-addressed enqueue has nowhere to go.
    void bind_channels(const shtp::ChannelMap& channels) {
        channels_ = &channels;
    }

    /// Stage one cargo for the channel playing `role`. NOT_READY when that role
    /// has no number yet — the hub has not advertised it and the consumer did
    /// not seed it — which is a different failure from a full queue and has a
    /// different remedy.
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
        // acquire()/commit() rather than push(): the payload is copied straight
        // into the ring's slot, so a cargo is never staged through a temporary
        // the size of the profile's largest.
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
            false;  ///< The HAL refused a write; flush stopped.
    };

    /// Frame and write queued cargoes until input is ready or a write fails. A
    /// cargo the engine cannot frame is dropped rather than left to wedge the
    /// queue behind it; a write the HAL refuses stops the flush and leaves its
    /// cargo queued to be retried on the next pass, and says so in the result.
    /// Pending input after a successful write yields to the next pass's receive
    /// drain without reporting an error.
    Flushed flush(shtp::ShtpEngine& engine, IHalTransport& hal) {
        Flushed flushed;
        while (const TxCargo* slot = queue_.front()) {
            auto framed = engine.frame_outbound(
                slot->channel,
                std::span<const std::uint8_t>(slot->bytes.data(), slot->length),
                scratch_);
            if (!framed.ok()) {
                queue_.pop();
                continue;
            }
            auto written = hal.write_transfer(
                std::span<const std::uint8_t>(scratch_.data(), framed.value()));
            if (!written.ok()) {
                flushed.write_refused = true;
                return flushed;
            }
            ++flushed.written;
            queue_.pop();
            // AR-3: a duplex write may have filled the HAL's receive staging.
            if (hal.data_ready()) {
                break;
            }
        }
        return flushed;
    }

    [[nodiscard]] bool empty() const { return queue_.empty(); }

    /// Drop everything staged. Called when the session restarts, so requests
    /// made against the previous hub incarnation are not delivered to the next.
    ///
    /// clear() is safe here for the reason SingleContextCounter is: producer
    /// and consumer are the same context, so there is no concurrent side for it
    /// to race.
    void reset() { queue_.clear(); }

   private:
    const shtp::ChannelMap* channels_ = nullptr;
    // The counter policy has no default in pico_util, deliberately, and this
    // is the choice it forces: requests are staged from consumer calls and
    // drained in service(), both mainline, so no synchronisation is owed. A
    // ring shared across contexts passes every single-threaded host test it is
    // given and fails only on the target under preemption, which is why the
    // assumption is named here rather than inferred.
    pico_util::RingBuffer<TxCargo, ActiveProfile::TX_QUEUE_DEPTH,
                          pico_util::SingleContextCounter>
        queue_;
    std::array<std::uint8_t, ActiveProfile::MAX_TX_CARGO + shtp::HEADER_SIZE>
        scratch_{};
};

}  // namespace pico_sh2::detail
