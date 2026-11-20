// SPDX-License-Identifier: MIT
#pragma once

/// \file command_engine.hpp
/// Internal single-command correlation and deadline state (SR-2, TR-1).

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
    /// Highest command ID (1000-3625 §6.3.8).
    static constexpr std::uint8_t MAX_COMMAND = 127;

    /// Queue and start the deadline; BUSY if a command is outstanding (SR-2).
    /// INVALID_ARGUMENT means bad ID or parameter count; queue errors leave no
    /// pending command.
    Result<void> send(std::uint8_t command,
                      std::span<const std::uint8_t> params,
                      std::uint64_t now_us, std::uint64_t timeout_us,
                      TxPath& tx) {
        auto sequence = enqueue_request(command, params, tx);
        if (!sequence.ok()) {
            return sequence.error();
        }
        pending_ = true;
        pending_sequence_ = sequence.value();
        deadline_us_ = deadline_after(now_us, timeout_us);
        return Result<void>{};
    }

    /// Queue a command the hub sends no response to (SR-2). BUSY while a
    /// command is outstanding; no deadline is armed and the request never
    /// becomes pending. A later response never completes it.
    Result<void> send_without_response(std::uint8_t command,
                                       std::span<const std::uint8_t> params,
                                       TxPath& tx) {
        auto sequence = enqueue_request(command, params, tx);
        if (!sequence.ok()) {
            return sequence.error();
        }
        return Result<void>{};
    }

    /// Complete only a solicited response with the pending sequence (SR-2).
    bool on_response(const CommandResponse& response, bool unsolicited) {
        if (unsolicited || !pending_ ||
            response.command_sequence != pending_sequence_) {
            return false;
        }
        pending_ = false;
        return true;
    }

    /// End an expired command and return true; otherwise return false.
    bool check_deadline(std::uint64_t now_us) {
        if (!pending_ || now_us < deadline_us_) {
            return false;
        }
        pending_ = false;
        return true;
    }

    [[nodiscard]] bool pending() const { return pending_; }

    /// Abandon the command; preserve sequence to avoid matching stale
    /// responses.
    void reset() {
        pending_ = false;
        deadline_us_ = 0;
    }

   private:
    /// Validate, serialise and enqueue; returns the sequence on success (SR-2).
    Result<std::uint8_t> enqueue_request(std::uint8_t command,
                                         std::span<const std::uint8_t> params,
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
            return queued.error();
        }
        next_sequence_ = static_cast<std::uint8_t>(sequence + 1);
        return sequence;
    }

    bool pending_ = false;
    std::uint8_t next_sequence_ = 0;
    std::uint8_t pending_sequence_ = 0;
    std::uint64_t deadline_us_ = 0;
};

}  // namespace pico_sh2::detail
