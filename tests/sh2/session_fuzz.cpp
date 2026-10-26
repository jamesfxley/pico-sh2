// SPDX-License-Identifier: MIT
// NR-6. The framing and decoder layers each have their own target; this one
// drives the state machine that composes them, where a hang is reachable that
// neither layer alone can produce.
#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <span>
#include <utility>
#include <vector>

#include "pico_sh2/sh2/command.hpp"
#include "pico_sh2/sh2/frs.hpp"
#include "pico_sh2/sh2/hal_transport.hpp"
#include "pico_sh2/sh2/session.hpp"
#include "pico_sh2/sh2/session_timing.hpp"
#include "pico_sh2/shtp/channels.hpp"
#include "pico_sh2/testing/frame_builder.hpp"
#include "pico_sh2/util/result.hpp"

// libFuzzer entry point for the full SH-2 session driven over a fuzz-backed
// HAL. Built only when PICO_SH2_ENABLE_FUZZERS is set and the compiler supports
// -fsanitize=fuzzer (Clang). The always-on session_test pins the behavior; this
// target drives begin()/service() against arbitrary hub traffic and clock
// advances, and must never read outside a buffer or fail to terminate
// (ASan/UBSan guard this).

namespace {

// A HAL whose inbound transfers and clock come from the fuzz input. Each
// read_transfer consumes a length byte followed by that many payload bytes;
// each service tick advances the clock so the startup timers fire.
// final: the base declares its destructor protected and non-virtual on purpose
// (a transport is owned by whoever supplies it, never deleted through this
// pointer). A derived class with virtual functions and a public non-virtual
// destructor reopens exactly that hazard unless it cannot be derived from.
class FuzzHal final : public pico_sh2::IHalTransport {
   public:
    FuzzHal(const std::uint8_t* data, std::size_t size)
        : data_(data), size_(size) {}

    /// Serve `prelude` ahead of the fuzz data once the hub would have booted,
    /// so an input can start from a ready session.
    void set_prelude(std::vector<std::vector<std::uint8_t>> prelude) {
        prelude_ = std::move(prelude);
    }

    [[nodiscard]] bool has_data() const {
        return prelude_pos_ < prelude_.size() || pos_ < size_;
    }

    void advance() { now_ += CLOCK_STEP_US; }

    // With a prelude, the fuzz data waits behind it: offered earlier it would
    // be drained and dropped while the session boots.
    bool data_ready() override {
        if (prelude_pos_ < prelude_.size()) {
            return prelude_pending();
        }
        return pos_ < size_;
    }

    pico_sh2::Result<pico_sh2::ReceivedTransfer> read_transfer(
        std::span<std::uint8_t> dst) override {
        if (prelude_pos_ < prelude_.size()) {
            if (!prelude_pending()) {
                return pico_sh2::Sh2Error::NOT_READY;
            }
            const std::vector<std::uint8_t>& next = prelude_[prelude_pos_++];
            const std::size_t length = std::min(next.size(), dst.size());
            std::copy_n(next.data(), length, dst.data());
            return pico_sh2::ReceivedTransfer{.length = length,
                                              .intn_edge_us = now_};
        }
        if (pos_ >= size_) {
            return pico_sh2::Sh2Error::NOT_READY;
        }
        std::size_t want = data_[pos_++];
        want = std::min(want, dst.size());
        want = std::min(want, size_ - pos_);
        std::copy_n(data_ + pos_, want, dst.data());
        pos_ += want;
        return pico_sh2::ReceivedTransfer{.length = want, .intn_edge_us = now_};
    }

    pico_sh2::Result<void> write_transfer(
        std::span<const std::uint8_t> /*frame*/) override {
        return pico_sh2::Result<void>{};
    }

    std::uint64_t now_us() override { return now_; }

    void set_reset(bool /*asserted*/) override {}

   private:
    static constexpr std::uint64_t CLOCK_STEP_US = 30'000;
    // Past the reset hold and boot interval below, so the prelude is not
    // drained and dropped while the session is still booting.
    static constexpr std::uint64_t PRELUDE_AFTER_US = 150'000;

    [[nodiscard]] bool prelude_pending() const {
        return prelude_pos_ < prelude_.size() && now_ >= PRELUDE_AFTER_US;
    }

    const std::uint8_t* data_;
    std::size_t size_;
    std::size_t pos_ = 0;
    std::uint64_t now_ = 0;
    std::vector<std::vector<std::uint8_t>> prelude_;
    std::size_t prelude_pos_ = 0;
};

}  // namespace

// The entry point's name is libFuzzer's, not ours.
// NOLINTNEXTLINE(readability-identifier-naming)
extern "C" int LLVMFuzzerTestOneInput(const std::uint8_t* data,
                                      std::size_t size) {
    // The first byte's low bit chooses whether a valid startup precedes the
    // fuzz data. Without it every input would have to find an advertisement,
    // a reset-complete and an Initialize before reaching the transaction
    // engines; with it the fuzz data lands on a ready session.
    if (size == 0) {
        return 0;
    }
    const bool start_ready = (data[0] & 1U) != 0;
    FuzzHal hal(data + 1, size - 1);
    if (start_ready) {
        hal.set_prelude(
            {pico_sh2::testing::frame(0, 0, pico_sh2::testing::advertisement()),
             pico_sh2::testing::frame(1, 0,
                                      pico_sh2::testing::reset_complete()),
             pico_sh2::testing::frame(
                 2, 0, pico_sh2::testing::initialize_response(0))});
    }
    // Timing has no defaults; the fuzzer supplies its own, as a consumer must.
    // A seeded map lets the pre-advertisement paths be reached.
    constexpr pico_sh2::SessionTiming TIMING{
        .reset_hold_us = 2'000,
        .boot_delay_us = 100'000,
        .advertisement_timeout_us = 200'000,
        .reset_complete_timeout_us = 200'000,
        .initialize_timeout_us = 200'000,
        .command_timeout_us = 500'000,
        .frs_timeout_us = 500'000,
    };
    pico_sh2::shtp::ChannelMap channels;
    channels.seed(pico_sh2::shtp::ChannelRole::DEVICE, 1);
    channels.seed(pico_sh2::shtp::ChannelRole::CONTROL, 2);
    channels.seed(pico_sh2::shtp::ChannelRole::INPUT_NORMAL, 3);
    channels.seed(pico_sh2::shtp::ChannelRole::INPUT_WAKE, 4);
    channels.seed(pico_sh2::shtp::ChannelRole::INPUT_GYRO_RV, 5);
    pico_sh2::Session session(hal, TIMING, channels);
    session.begin();

    // Bounded pump: drain fuzz-driven transfers and advance the clock so timed
    // transitions and deadlines fire. On a fault, restart the handshake to keep
    // exercising the startup path against the remaining input.
    // Once ready, keep a command and an FRS operation outstanding, alternating
    // read and write, so fuzzed control traffic reaches the transaction
    // engines -- the FRS read places words at a hub-supplied offset.
    static constexpr std::array<std::uint32_t, 3> RECORD{1, 2, 3};
    unsigned frs_operations = 0;
    for (int i = 0; i < 512; ++i) {
        if (session.ready()) {
            if (!session.frs_busy()) {
                if ((frs_operations++ & 1U) == 0) {
                    (void)session.frs_read(
                        pico_sh2::frs_type::SYSTEM_ORIENTATION);
                } else {
                    (void)session.frs_write(
                        pico_sh2::frs_type::SYSTEM_ORIENTATION, RECORD);
                }
            }
            if (!session.command_pending()) {
                (void)session.send_command(pico_sh2::command::COUNTER);
            }
        }
        session.service();
        hal.advance();
        if (session.faulted()) {
            session.begin();
        }
        if (!hal.has_data()) {
            break;
        }
    }
    return 0;
}
