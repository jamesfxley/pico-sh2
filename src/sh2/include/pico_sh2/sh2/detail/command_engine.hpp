// SPDX-License-Identifier: MIT
#pragma once

/// \file command_engine.hpp
/// Command transaction state (SR-2): exactly one command outstanding at a time,
/// correlated to its response by the sequence number the request carried, and
/// abandoned if no response arrives inside the deadline (TR-1).
///
/// The engine owns only the transaction. Counters, the consumer's command sink,
/// and the diagnostics hook stay with Session, which reacts to what the engine
/// reports; that keeps this testable on its own and stops the transaction rules
/// from being spread across the routing code.
///
/// This is an internal collaborator of Session, not public API.

#include <array>
#include <cstdint>
#include <span>

#include "pico_sh2/sh2/command.hpp"
#include "pico_sh2/sh2/detail/deadline.hpp"
#include "pico_sh2/sh2/detail/tx_path.hpp"
#include "pico_sh2/shtp/channels.hpp"
#include "pico_sh2/util/result.hpp"

namespace pico_sh2::detail {

class CommandEngine {
   public:
    /// Highest valid command identifier: the range is 1-127, with 0 reserved
    /// and 128-255 reserved (1000-3625 §6.3.8). The unsolicited flag
    /// uses the MSB in responses (§6.3.9).
    static constexpr std::uint8_t MAX_COMMAND = 127;

    /// Queue a command request and start its deadline. BUSY if one is already
    /// outstanding (SR-2), INVALID_ARGUMENT for an out-of-range identifier or
    /// more parameter bytes than a request carries, and
    /// whatever the queue refused with if the request could not be staged — in
    /// which case no command becomes outstanding.
    Result<void> send(std::uint8_t command,
                      std::span<const std::uint8_t> params,
                      std::uint64_t now_us, std::uint64_t timeout_us,
                      TxPath& tx) {
        if (command == 0 || command > MAX_COMMAND ||
            params.size() > COMMAND_MAX_PARAMS) {
            return Sh2Error::INVALID_ARGUMENT;
        }
        if (pending_) {
            return Sh2Error::BUSY;
        }
        std::array<std::uint8_t, COMMAND_REQUEST_SIZE> body{};
        const std::uint8_t sequence = next_sequence_;
        serialize_command_request(CommandSequence{sequence}, command, params,
                                  body);
        auto queued = tx.enqueue(shtp::ChannelRole::CONTROL, body);
        if (!queued.ok()) {
            return queued;
        }
        pending_ = true;
        pending_sequence_ = sequence;
        next_sequence_ = static_cast<std::uint8_t>(sequence + 1);
        deadline_us_ = deadline_after(now_us, timeout_us);
        return Result<void>{};
    }

    /// Offer a decoded response. True if it completed the outstanding command,
    /// which requires it to be solicited and to echo the pending sequence
    /// number; an unsolicited response never completes one (SR-2).
    bool on_response(const CommandResponse& response, bool unsolicited) {
        if (unsolicited || !pending_ ||
            response.command_sequence != pending_sequence_) {
            return false;
        }
        pending_ = false;
        return true;
    }

    /// True if an outstanding command has passed its deadline, in which case it
    /// is no longer outstanding and the slot is free again.
    bool check_deadline(std::uint64_t now_us) {
        if (!pending_ || now_us < deadline_us_) {
            return false;
        }
        pending_ = false;
        return true;
    }

    [[nodiscard]] bool pending() const { return pending_; }

    /// Abandon any outstanding command. The sequence counter is deliberately
    /// not rewound: it only has to be unambiguous across the responses still in
    /// flight, and restarting it could alias a stale response onto a new
    /// command.
    void reset() {
        pending_ = false;
        deadline_us_ = 0;
    }

   private:
    bool pending_ = false;
    std::uint8_t next_sequence_ = 0;
    std::uint8_t pending_sequence_ = 0;
    std::uint64_t deadline_us_ = 0;
};

}  // namespace pico_sh2::detail
