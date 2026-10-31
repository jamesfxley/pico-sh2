// SPDX-License-Identifier: MIT
#pragma once

/// \file frs_engine.hpp
/// FRS read/write transaction state (SR-3): one operation at a time, over a
/// record buffer sized by the active profile.
///
/// A read streams 0xF3 responses, each carrying up to two words at their own
/// record offset, until a status says the record is complete or empty. A write
/// sends the 0xF7 request then paces 0xF6 data reports, one per 0xF5 status,
/// until the hub reports completion. Either way the operation ends exactly
/// once, through a Completion the caller forwards to the consumer -- including
/// when the session abandons it because the hub reset.
///
/// A read response names the record it carries (1000-3625 §6.3.7, Figure 39).
/// One naming another record is the tail of an earlier read that ended here
/// first, typically by timing out; it is reported as stray and never written
/// into the running read's buffer. A record longer than the buffer ends the
/// read with TABLE_FULL rather than completing it short.
///
/// Only one FRS operation may be in progress at a time (§6.3.7), and one the
/// engine ended itself -- past its deadline, for a full buffer, or because its
/// next data report could not be queued -- may still be running on the hub.
/// So the engine then holds: a new operation is refused BUSY, and a response
/// to the ended one is stray, until the hub ends it with a status of its own,
/// or for one more FRS timeout if it never does. Without the hold, the tail of
/// a timed-out read would complete a retry of the same record.
///
/// A completed read must cover every word below its highest. Whether the hub
/// may answer a read non-densely is UNCONFIRMED (FW-5; §6.3.7 does not say),
/// and a gap is what a dropped response leaves, so the read ends PROTOCOL
/// rather than delivering a word nobody sent.
///
/// The engine owns the transaction and the buffer; counters, the consumer's FRS
/// sink, and the diagnostics hook stay with Session.
///
/// This is an internal collaborator of Session, not public API.

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

    /// The end of an operation. `finished` is false while the operation is
    /// still running, in which case the other fields carry nothing. `words` is
    /// empty for a write and valid only for the duration of the caller's
    /// handling.
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
        /// No word of an earlier read survives into this one; one this read
        /// leaves uncovered ends it PROTOCOL rather than being delivered.
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

    /// What became of one response, besides any Completion it produced.
    enum class Outcome : std::uint8_t {
        APPLIED,    ///< Belonged to the running operation.
        UNROUTED,   ///< No operation of its kind is running.
        MALFORMED,  ///< Shorter than its report.
        STRAY,      ///< A response to no running operation: another record's,
                    ///< or one the engine ended and holds.
    };

    /// Handle an FRS Read Response (0xF3). Returns an unfinished Completion
    /// while more responses are expected; `outcome` says whether the report
    /// was used, so the caller can count the ones that were not.
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
        // SR-3: only no-error and completed carry a read forward. The
        // deprecated (4, 6, 7) and reserved (9-15) codes are not defined for a
        // read in progress (1000-3625 §6.3.7), so the read ends rather than
        // storing the report and waiting out the deadline.
        if (status != frs_read_status::NO_ERROR &&
            status != frs_read_status::RECORD_COMPLETED) {
            return finish(Sh2Error::PROTOCOL, 0, status);
        }

        /// Store this report's words at their offset within the record.
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
        /// NO_ERROR: more responses follow.
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
                // UNCONFIRMED (FW-5) what follows a busy write status
                // (1000-3625 §6.3.5 lists the code only), so the write ends
                // here rather than waiting on a response that may never come.
                return finish(Sh2Error::BUSY, 0, status);
            default:
                /// WRITE_FAILED / INVALID_LENGTH / RECORD_INVALID /
                /// NOT_IN_WRITE_MODE / READ_ONLY / MEMORY_FULL: the write
                /// cannot complete. The status says which.
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

    /// End a running operation with `status`, for a caller that is abandoning
    /// it; an unfinished Completion when nothing was running. The engine is
    /// idle afterwards either way.
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

    /// A read response while a read is held: stray, and the end of the hold
    /// when it is the held record's last.
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

    /// A write status while a write is held: stray, and the end of the hold
    /// unless it says more follows (asks for data, or reports the record
    /// valid ahead of its write completing).
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
