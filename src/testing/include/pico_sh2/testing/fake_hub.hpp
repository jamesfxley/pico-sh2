// SPDX-License-Identifier: MIT
#pragma once

/// \file fake_hub.hpp
/// Host-only SH-2 hub emulator implementing IHalTransport at the bus level.
/// Tests script inbound transfers (with controllable HINTN timestamps), drive
/// time, capture host writes, and inject failures, so the SHTP/SH-2 stack is
/// exercised end-to-end without hardware. This is test-support code: it uses
/// the heap freely (the zero-post-init-heap rule governs the library on-device,
/// not the fakes).

#include <cstddef>
#include <cstdint>
#include <deque>
#include <span>
#include <vector>

#include "pico_sh2/sh2/hal_transport.hpp"
#include "pico_sh2/util/result.hpp"

namespace pico_sh2::testing {

/// A scripted hub on the far side of IHalTransport: queue the transfers it
/// should hand back, drive its clock, capture what the host wrote, and inject
/// bus faults.
class FakeHub final : public IHalTransport {
   public:
    /// A raw inbound transfer (full SHTP frame: 4-byte header + payload) the
    /// hub will hand back on the next read, tagged with the HINTN edge
    /// timestamp the driver should observe for it.
    struct ScriptedTransfer {
        std::vector<std::uint8_t> frame;  ///< Full SHTP frame, header first.
        std::uint64_t intn_timestamp_us;  ///< HINTN edge reported with it.
    };

    /// A host->hub write captured for assertions.
    struct CapturedWrite {
        std::vector<std::uint8_t> frame;  ///< The bytes the host sent.
    };

    /// --- scripting ---
    void queue_transfer(std::span<const std::uint8_t> frame,
                        std::uint64_t intn_timestamp_us);

    /// \param delta_us microseconds to move the fake clock forward by.
    void advance_time(std::uint64_t delta_us) { now_us_ += delta_us; }

    /// Make the next write_transfer() fail with `error`. Consumed once.
    void fail_next_write(Sh2Error error) { next_write_error_ = error; }

    /// Make the next read_transfer() (with data pending) fail with `error`,
    /// modelling a bus fault mid-drain. Consumed once.
    void fail_next_read(Sh2Error error) { fail_read_after(0, error); }

    /// Make read_transfer() fail with `error` only after `successful_reads`
    /// more reads have succeeded, so a test can put a fault partway through a
    /// multi-transfer drain rather than at its head. Consumed once.
    void fail_read_after(std::size_t successful_reads, Sh2Error error) {
        reads_until_error_ = successful_reads;
        next_read_error_ = error;
    }

    /// Queue an idle poll: the hub asserts HINTN and then hands back a
    /// header-only transfer with a zero length, which is what a BNO086 does
    /// when polled with nothing to say, and what a dead or absent bus reads as.
    ///
    /// Scripted rather than left to a hand-rolled `queue_transfer({0,0,0,0})`
    /// because this is the case the seam's contract turns on: a read of it
    /// **succeeds**, and a transport returning an error here would fault an
    /// otherwise healthy session.
    void queue_idle_transfer(std::uint64_t intn_timestamp_us = 0);

    /// Queue the three cargoes a hub sends on its way up: the command-channel
    /// advertisement, the device channel's reset-complete, and the unsolicited
    /// Initialize response. `sequence` stamps all three, so a second call after
    /// a hub reset can present a fresh sequence rather than repeating the
    /// first. The two announcements are issued by independent applications and
    /// the session does not depend on their order; this queues one of the two.
    void queue_startup(std::uint64_t intn_timestamp_us = 0,
                       std::uint8_t sequence = 0);

    /// --- capture / inspection ---
    [[nodiscard]] const std::vector<CapturedWrite>& writes() const {
        return writes_;
    }

    /// Whether the host currently holds the hub in reset.
    [[nodiscard]] bool reset_asserted() const { return reset_asserted_; }

    /// --- IHalTransport ---
    bool data_ready() override { return !inbound_.empty(); }

    /// Reads the next scripted transfer, or -- with nothing queued -- reports a
    /// header-only transfer of zero length, exactly as a real transport does
    /// for an absent hub. **Not an error:** the session's drain loop faults on
    /// any read error, and permanently so during startup, which is why both
    /// shipped transports report an absent hub as success and why this fake
    /// must do the same. A fake that returned an error here would teach the
    /// opposite of the contract to anyone reading it for one. The edge it
    /// reports is the transfer's scripted one, or with nothing queued the last
    /// one read, as the interface requires of a read with no fresh edge.
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
