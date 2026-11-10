// SPDX-License-Identifier: MIT
// Fuzzes termination and memory safety of the composed session state machine
// (NR-6).
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

// Arbitrary hub transfers driven with a ticking clock must terminate without
// out-of-bounds access (checked by ASan/UBSan).

namespace {

// Reads length-prefixed fuzz transfers; each pump tick advances the clock.
// final: the public non-virtual destructor is safe only while nothing derives
// from this class.
class FuzzHal final : public pico_sh2::IHalTransport {
   public:
    FuzzHal(const std::uint8_t* data, std::size_t size)
        : data_(data), size_(size) {}

    /// Supply startup before fuzz data so an input can reach READY.
    void set_prelude(std::vector<std::vector<std::uint8_t>> prelude) {
        prelude_ = std::move(prelude);
    }

    [[nodiscard]] bool has_data() const {
        return prelude_pos_ < prelude_.size() || pos_ < size_;
    }

    void advance() { now_ += CLOCK_STEP_US; }

    // With a prelude, fuzz data waits until boot completes to avoid being
    // dropped.
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
    // Later than reset hold and boot, keeping startup data out of the boot
    // drain.
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

// readability-identifier-naming: libFuzzer requires this entry point name.
// NOLINTNEXTLINE(readability-identifier-naming)
extern "C" int LLVMFuzzerTestOneInput(const std::uint8_t* data,
                                      std::size_t size) {
    // The low bit of the first byte selects valid startup before fuzz data,
    // allowing transaction coverage without finding all three announcements.
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
    // A consumer must supply timing; seeds also expose pre-advertisement paths.
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

    // Pumping advances the clock so deadlines fire; faults restart startup.
    // Commands and alternating FRS reads and writes exercise the transaction
    // engines, including hub-supplied FRS word offsets.
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
