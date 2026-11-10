// SPDX-License-Identifier: MIT
#pragma once

/// \file fake_hub.hpp
/// Host-only scripted bus, clock and faults for exercising the stack without
/// hardware. Test support uses the heap; on-device allocation rules do not
/// apply.

#include <cstddef>
#include <cstdint>
#include <deque>
#include <span>
#include <vector>

#include "pico_sh2/sh2/hal_transport.hpp"
#include "pico_sh2/util/result.hpp"

namespace pico_sh2::testing {

/// Scripted transport: queue the transfers it should return, drive time and
/// inject errors.
class FakeHub final : public IHalTransport {
   public:
    /// Full SHTP frame and the HINTN timestamp the driver should observe with
    /// its read.
    struct ScriptedTransfer {
        std::vector<std::uint8_t> frame;  ///< Full SHTP frame, header first.
        std::uint64_t intn_timestamp_us;  ///< HINTN edge reported with it.
    };

    /// A host->hub write captured for assertions.
    struct CapturedWrite {
        std::vector<std::uint8_t> frame;  ///< The bytes the host sent.
    };

    /// Copy a frame and edge timestamp into the inbound script.
    void queue_transfer(std::span<const std::uint8_t> frame,
                        std::uint64_t intn_timestamp_us);

    /// \param delta_us microseconds to move the fake clock forward by.
    void advance_time(std::uint64_t delta_us) { now_us_ += delta_us; }

    /// Make the next write_transfer() fail with `error`. Consumed once.
    void fail_next_write(Sh2Error error) { next_write_error_ = error; }

    /// Make the next read_transfer() (with data pending) fail with `error`,
    /// modelling a bus fault mid-drain. Consumed once.
    void fail_next_read(Sh2Error error) { fail_read_after(0, error); }

    /// Fail once after successful_reads more reads with pending input have
    /// succeeded.
    void fail_read_after(std::size_t successful_reads, Sh2Error error) {
        reads_until_error_ = successful_reads;
        next_read_error_ = error;
    }

    /// Queue a successful null-header read with the supplied edge timestamp.
    void queue_idle_transfer(std::uint64_t intn_timestamp_us = 0);

    /// Queue advertisement, reset-complete and Initialize with one sequence
    /// value. The session accepts either announcement order; this scripts
    /// reset-complete first.
    void queue_startup(std::uint64_t intn_timestamp_us = 0,
                       std::uint8_t sequence = 0);

    /// Captured host writes in arrival order.
    [[nodiscard]] const std::vector<CapturedWrite>& writes() const {
        return writes_;
    }

    /// Whether the host currently holds the hub in reset.
    [[nodiscard]] bool reset_asserted() const { return reset_asserted_; }

    /// True while scripted input remains.
    bool data_ready() override { return !inbound_.empty(); }

    /// Read scripted input; with nothing queued, the fake must return a
    /// successful null header with the last edge. A destination smaller than
    /// the header returns BUS_ERROR when idle.
    Result<ReceivedTransfer> read_transfer(
        std::span<std::uint8_t> dst) override;
    Result<void> write_transfer(std::span<const std::uint8_t> frame) override;

    std::uint64_t now_us() override { return now_us_; }

    void set_reset(bool asserted) override { reset_asserted_ = asserted; }

   private:
    std::deque<ScriptedTransfer> inbound_;
    std::vector<CapturedWrite> writes_;
    std::uint64_t now_us_ = 0;
    std::uint64_t last_edge_us_ = 0;  // the edge of the last transfer read
    Sh2Error next_write_error_ = Sh2Error::NONE;
    Sh2Error next_read_error_ = Sh2Error::NONE;
    std::size_t reads_until_error_ = 0;
    bool reset_asserted_ = false;
};

}  // namespace pico_sh2::testing
