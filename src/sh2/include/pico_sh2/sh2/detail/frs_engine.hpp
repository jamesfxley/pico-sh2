// SPDX-License-Identifier: MIT
#pragma once

/// \file frs_engine.hpp
/// FRS operation state over a bounded record buffer; only one FRS operation may
/// run at a time (1000-3625 §6.3.7).
/// Completed reads must cover every word below their highest (SR-3).
/// Non-dense hub reads are UNCONFIRMED (1000-3625 §6.3.7, FW-5).

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>

#include "pico_sh2/sh2/detail/deadline.hpp"
#include "pico_sh2/sh2/detail/tx_path.hpp"
#include "pico_sh2/sh2/frs.hpp"
#include "pico_sh2/sh2/profile.hpp"
#include "pico_sh2/shtp/channels.hpp"
#include "pico_sh2/util/result.hpp"
#include "pico_util/bytes/byte_order.hpp"

namespace pico_sh2::detail {

class FrsEngine {
   public:
    enum class Phase : std::uint8_t { IDLE, READING, WRITING };

    /// Completion with borrowed words, empty for writes and valid during
    /// handling. When finished is false, the other fields carry no result.
    struct Completion {
        bool finished = false;
        Sh2Error status = Sh2Error::NONE;
        std::uint16_t type = 0;
        std::span<const std::uint32_t> words;
        /// The hub's status byte when the hub's own response ended the
        /// operation; empty for a deadline, an abandon or a full buffer.
        std::optional<std::uint8_t> device_status;
    };

    [[nodiscard]] Phase phase() const { return phase_; }

    [[nodiscard]] bool busy() const { return phase_ != Phase::IDLE; }

    /// True while an operation the engine ended is still running on the hub.
    [[nodiscard]] bool holding() const { return held_ != Phase::IDLE; }

    /// Request a record. BUSY if an operation is already running.
    Result<void> begin_read(std::uint16_t type, std::uint64_t now_us,
                            std::uint64_t timeout_us, TxPath& tx) {
        if (busy() || holding()) {
            return Sh2Error::BUSY;
        }
        std::array<std::uint8_t, FRS_READ_REQUEST_SIZE> body{};
        serialize_frs_read_request(type, body);
        auto queued = tx.enqueue(shtp::ChannelRole::CONTROL, body);
        if (!queued.ok()) {
            return queued;
        }
        /// Prior data is cleared; uncovered words end the read with PROTOCOL.
        buffer_.fill(0);
        covered_.fill(0);
        phase_ = Phase::READING;
        type_ = type;
        word_count_ = 0;
        timeout_us_ = timeout_us;
        deadline_us_ = deadline_after(now_us, timeout_us);
        return Result<void>{};
    }

    /// Write a record; an empty `words` erases it. BUSY if an operation is
    /// already running, INVALID_ARGUMENT if the record exceeds the buffer.
    Result<void> begin_write(std::uint16_t type,
                             std::span<const std::uint32_t> words,
                             std::uint64_t now_us, std::uint64_t timeout_us,
                             TxPath& tx) {
        if (busy() || holding()) {
            return Sh2Error::BUSY;
        }
        if (words.size() > buffer_.size()) {
            return Sh2Error::INVALID_ARGUMENT;
        }
        std::copy_n(words.data(), words.size(), buffer_.data());
        word_count_ = words.size();
        write_cursor_ = 0;
        std::array<std::uint8_t, FRS_WRITE_REQUEST_SIZE> body{};
        serialize_frs_write_request(
            FrsWriteRequest{
                .type = type,
                .length_words = static_cast<std::uint16_t>(words.size())},
            body);
        auto queued = tx.enqueue(shtp::ChannelRole::CONTROL, body);
        if (!queued.ok()) {
            return queued;
        }
        phase_ = Phase::WRITING;
        type_ = type;
        timeout_us_ = timeout_us;
        deadline_us_ = deadline_after(now_us, timeout_us);
        return Result<void>{};
    }

    /// How a response was handled, for the caller's counters.
    enum class Outcome : std::uint8_t {
        /// Belonged to the running operation.
        APPLIED,
        /// No operation of its kind is running.
        UNROUTED,
        /// Shorter than its report.
        MALFORMED,
        /// A response to no running operation: another record's, or one the
        /// engine ended and holds.
        STRAY,
    };

    /// Apply a read response; unfinished means more are expected.
    /// outcome lets the caller count unused responses.
    Completion on_read_response(std::span<const std::uint8_t> payload,
                                std::uint64_t now_us, Outcome& outcome) {
        if (phase_ != Phase::READING) {
            outcome = held_ == Phase::READING ? release_read(payload)
                                              : Outcome::UNROUTED;
            return {};
        }
        if (payload.size() < FRS_READ_RESPONSE_SIZE) {
            outcome = Outcome::MALFORMED;
            return {};
        }
        if (pico_util::read_le<std::uint16_t>(payload, FRS_READ_RESP_TYPE) !=
            type_) {
            outcome = Outcome::STRAY;
            return {};
        }
        outcome = Outcome::APPLIED;
        const std::uint8_t length_status = payload[FRS_READ_RESP_LENGTH_STATUS];
        const std::uint8_t status = length_status & FRS_READ_STATUS_MASK;
        const std::size_t data_words =
            (static_cast<unsigned>(length_status) >> FRS_READ_DATA_LEN_SHIFT) &
            FRS_READ_STATUS_MASK;
        const auto word_offset = pico_util::read_le<std::uint16_t>(
            payload, FRS_READ_RESP_WORD_OFFSET);

        if (status == frs_read_status::UNRECOGNIZED_TYPE) {
            return finish(Sh2Error::UNSUPPORTED, 0, status);
        }
        if (status == frs_read_status::DEVICE_ERROR) {
            return finish(Sh2Error::PROTOCOL, 0, status);
        }
        if (status == frs_read_status::RECORD_EMPTY) {
            return finish(Sh2Error::NONE, 0, status);
        }
        // Busy answers a read made while another FRS operation runs, and the
        // hub then completes that one, not this (1000-3625 §6.3.6).
        if (status == frs_read_status::BUSY) {
            return finish(Sh2Error::BUSY, 0, status);
        }
        // Only NO_ERROR and RECORD_COMPLETED advance a read (SR-3).
        // Other codes are deprecated or reserved (1000-3625 §6.3.7).
        if (status != frs_read_status::NO_ERROR &&
            status != frs_read_status::RECORD_COMPLETED) {
            return finish(Sh2Error::PROTOCOL, 0, status);
        }

        for (std::size_t word = 0;
             word < data_words && word < FRS_WORDS_PER_REPORT; ++word) {
            const std::size_t index =
                static_cast<std::size_t>(word_offset) + word;
            if (index >= buffer_.size()) {
                // Unless this was the record's last response, the hub is still
                // sending the rest of it.
                if (status == frs_read_status::RECORD_COMPLETED) {
                    return finish(Sh2Error::TABLE_FULL, 0);
                }
                return finish_and_hold(Sh2Error::TABLE_FULL, now_us);
            }
            buffer_[index] = pico_util::read_le<std::uint32_t>(
                payload, FRS_READ_RESP_DATA + (sizeof(std::uint32_t) * word));
            covered_[index / COVER_BITS] |= 1U << (index % COVER_BITS);
            word_count_ = std::max(word_count_, index + 1);
        }

        if (status == frs_read_status::RECORD_COMPLETED) {
            // The hub has finished either way, so a gap ends the read without
            // a hold.
            if (!dense()) {
                return finish(Sh2Error::PROTOCOL, 0, status);
            }
            return finish(Sh2Error::NONE, word_count_, status);
        }
        return {};
    }

    /// Handle an FRS Write Response (0xF5), pacing the next data report when
    /// the hub asks for one.
    Completion on_write_response(std::span<const std::uint8_t> payload,
                                 std::uint64_t now_us, Outcome& outcome,
                                 TxPath& tx) {
        if (phase_ != Phase::WRITING) {
            // A held write's tail is stray, as a held read's is.
            outcome = held_ == Phase::WRITING ? release_write(payload)
                                              : Outcome::UNROUTED;
            return {};
        }
        if (payload.size() < FRS_WRITE_RESPONSE_SIZE) {
            outcome = Outcome::MALFORMED;
            return {};
        }
        outcome = Outcome::APPLIED;
        const std::uint8_t status = payload[FRS_WRITE_RESP_STATUS];
        switch (status) {
            case frs_write_status::READY:
            case frs_write_status::WORDS_RECEIVED:
                return send_next_chunk(tx, now_us);
            case frs_write_status::RECORD_VALID:
                // The record passed validation; write completed (3) follows
                // once it is written.
                return {};
            case frs_write_status::WRITE_COMPLETED:
                return finish(Sh2Error::NONE, 0, status);
            case frs_write_status::UNRECOGNIZED_TYPE:
                return finish(Sh2Error::UNSUPPORTED, 0, status);
            case frs_write_status::BUSY:
                // Further responses after BUSY are UNCONFIRMED (1000-3625
                // §6.3.5), so end the write here (FW-5).
                return finish(Sh2Error::BUSY, 0, status);
            default:
                /// The remaining statuses indicate failure; retain the hub's
                /// status byte.
                return finish(Sh2Error::PROTOCOL, 0, status);
        }
    }

    /// End a running operation that has passed its deadline, and a hold that
    /// has passed its own.
    Completion check_deadline(std::uint64_t now_us) {
        if (holding() && now_us >= hold_until_us_) {
            held_ = Phase::IDLE;
        }
        if (!busy() || now_us < deadline_us_) {
            return {};
        }
        return finish_and_hold(Sh2Error::TIMEOUT, now_us);
    }

    /// Abandon and become idle; unfinished means no operation was running.
    Completion abandon(Sh2Error status) {
        if (!busy()) {
            reset();
            return {};
        }
        Completion ended = finish(status, 0);
        reset();
        return ended;
    }

    /// Drop any running operation without reporting a completion.
    void reset() {
        phase_ = Phase::IDLE;
        held_ = Phase::IDLE;
        type_ = 0;
        word_count_ = 0;
        write_cursor_ = 0;
        deadline_us_ = 0;
    }

   private:
    /// Bits per word of the coverage map.
    static constexpr std::size_t COVER_BITS = 32;

    /// End the operation the engine is cutting short, and hold until the hub
    /// ends its side of it.
    Completion finish_and_hold(Sh2Error status, std::uint64_t now_us) {
        held_ = phase_;
        held_type_ = type_;
        hold_until_us_ = deadline_after(now_us, timeout_us_);
        return finish(status, 0);
    }

    /// Held-read replies are stray; a terminal reply for its record releases
    /// the hold.
    Outcome release_read(std::span<const std::uint8_t> payload) {
        if (payload.size() < FRS_READ_RESPONSE_SIZE) {
            return Outcome::MALFORMED;
        }
        if (pico_util::read_le<std::uint16_t>(payload, FRS_READ_RESP_TYPE) ==
                held_type_ &&
            (payload[FRS_READ_RESP_LENGTH_STATUS] & FRS_READ_STATUS_MASK) !=
                frs_read_status::NO_ERROR) {
            held_ = Phase::IDLE;
        }
        return Outcome::STRAY;
    }

    /// Held-write replies are stray; a terminal status releases the hold.
    Outcome release_write(std::span<const std::uint8_t> payload) {
        if (payload.size() < FRS_WRITE_RESPONSE_SIZE) {
            return Outcome::MALFORMED;
        }
        const std::uint8_t status = payload[FRS_WRITE_RESP_STATUS];
        if (status != frs_write_status::READY &&
            status != frs_write_status::WORDS_RECEIVED &&
            status != frs_write_status::RECORD_VALID) {
            held_ = Phase::IDLE;
        }
        return Outcome::STRAY;
    }

    /// True when every word below the highest one read was covered.
    [[nodiscard]] bool dense() const {
        for (std::size_t index = 0; index < word_count_; ++index) {
            if ((covered_[index / COVER_BITS] & (1U << (index % COVER_BITS))) ==
                0) {
                return false;
            }
        }
        return true;
    }

    Completion finish(
        Sh2Error status, std::size_t word_count,
        std::optional<std::uint8_t> device_status = std::nullopt) {
        phase_ = Phase::IDLE;
        return Completion{
            .finished = true,
            .status = status,
            .type = type_,
            .words = std::span<const std::uint32_t>(buffer_.data(), word_count),
            .device_status = device_status};
    }

    Completion send_next_chunk(TxPath& tx, std::uint64_t now_us) {
        if (write_cursor_ >= word_count_) {
            return {};  // all data sent; await the completion status
        }
        const std::array<std::uint32_t, FRS_WORDS_PER_REPORT> words{
            buffer_[write_cursor_],
            write_cursor_ + 1 < word_count_ ? buffer_[write_cursor_ + 1] : 0};
        std::array<std::uint8_t, FRS_WRITE_DATA_SIZE> body{};
        serialize_frs_write_data(static_cast<std::uint16_t>(write_cursor_),
                                 words, body);
        auto queued = tx.enqueue(shtp::ChannelRole::CONTROL, body);
        if (!queued.ok()) {
            // The hub is waiting for data that will not come.
            return finish_and_hold(queued.error(), now_us);
        }
        write_cursor_ += FRS_WORDS_PER_REPORT;
        return {};
    }

    std::array<std::uint32_t, ActiveProfile::FRS_MAX_WORDS> buffer_{};
    /// One bit per buffer word: set when a response covered it.
    std::array<std::uint32_t,
               (ActiveProfile::FRS_MAX_WORDS + COVER_BITS - 1) / COVER_BITS>
        covered_{};
    Phase phase_ = Phase::IDLE;
    /// The kind of operation held after the engine ended it; IDLE when none.
    Phase held_ = Phase::IDLE;
    std::uint16_t held_type_ = 0;
    std::uint64_t hold_until_us_ = 0;
    std::uint64_t timeout_us_ = 0;
    std::uint16_t type_ = 0;
    std::size_t word_count_ = 0;    // words read so far / words to write
    std::size_t write_cursor_ = 0;  // next word index to send in a write
    std::uint64_t deadline_us_ = 0;
};

}  // namespace pico_sh2::detail
