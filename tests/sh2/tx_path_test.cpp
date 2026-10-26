// SPDX-License-Identifier: MIT
#include "pico_sh2/sh2/detail/tx_path.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>
#include <vector>

#include "pico_sh2/sh2/hal_transport.hpp"
#include "pico_sh2/sh2/profile.hpp"
#include "pico_sh2/shtp/channels.hpp"
#include "pico_sh2/shtp/engine.hpp"
#include "pico_sh2/testing/fake_hub.hpp"
#include "pico_sh2/util/result.hpp"

namespace {

using pico_sh2::ActiveProfile;
using pico_sh2::Sh2Error;
using pico_sh2::detail::TxPath;

// AR-3: a one-buffer duplex HAL cannot accept another write until RX drains.
class DuplexHub final : public pico_sh2::IHalTransport {
   public:
    bool data_ready() override { return hub_.data_ready(); }

    pico_sh2::Result<pico_sh2::ReceivedTransfer> read_transfer(
        std::span<std::uint8_t> dst) override {
        return hub_.read_transfer(dst);
    }

    pico_sh2::Result<void> write_transfer(
        std::span<const std::uint8_t> frame) override {
        if (hub_.data_ready()) {
            return Sh2Error::BUSY;
        }
        auto result = hub_.write_transfer(frame);
        if (result.ok()) {
            const std::array<std::uint8_t, 5> incoming{5, 0, 3, 0, 0x44};
            hub_.queue_transfer(incoming, 0);
        }
        return result;
    }

    std::uint64_t now_us() override { return hub_.now_us(); }

    void set_reset(bool asserted) override { hub_.set_reset(asserted); }

    [[nodiscard]] const auto& writes() const { return hub_.writes(); }

   private:
    pico_sh2::testing::FakeHub hub_;
};

TEST(TxPath, YieldsForDuplexInputBeforeWritingAgain) {
    TxPath path;
    DuplexHub hub;
    std::array<std::uint8_t, 64> buffer{};
    pico_sh2::shtp::ShtpEngine engine(buffer);
    const std::array<std::uint8_t, 1> payload{0x11};
    ASSERT_TRUE(path.enqueue(2, payload).ok());
    ASSERT_TRUE(path.enqueue(2, payload).ok());
    const auto first = path.flush(engine, hub);
    EXPECT_EQ(first.written, 1U);
    EXPECT_FALSE(first.write_refused);
    EXPECT_FALSE(path.empty());
    ASSERT_TRUE(hub.read_transfer(buffer).ok());
    const auto second = path.flush(engine, hub);
    EXPECT_EQ(second.written, 1U);
    EXPECT_FALSE(second.write_refused);
    EXPECT_TRUE(path.empty());
    ASSERT_EQ(hub.writes().size(), 2U);
    EXPECT_EQ(hub.writes().at(0).frame.at(3), 0);
    EXPECT_EQ(hub.writes().at(1).frame.at(3), 1);
}

// BR-2: the outbound queue refuses rather than dropping, so the caller sees the
// failure. Held here because the queue is a TxPath member since the ring moved
// to pico_util::RingBuffer -- pico-util owns the container's own behaviour, and
// what is this library's is the policy around it.

TEST(TxPath, StagesUpToTheProfileDepthThenRefuses) {
    TxPath path;
    const std::array<std::uint8_t, 1> payload{0x11};

    // The depth is the element count outright: the ring counts rather than
    // reserving a slot to tell full from empty.
    for (std::size_t i = 0; i < ActiveProfile::TX_QUEUE_DEPTH; ++i) {
        EXPECT_TRUE(path.enqueue(0, payload).ok())
            << "cargo " << i << " of " << ActiveProfile::TX_QUEUE_DEPTH;
    }

    const auto refused = path.enqueue(0, payload);
    EXPECT_FALSE(refused.ok());
    EXPECT_EQ(refused.error(), Sh2Error::QUEUE_FULL);
}

TEST(TxPath, APayloadLargerThanASlotIsRefused) {
    TxPath path;
    const std::vector<std::uint8_t> too_big(ActiveProfile::MAX_TX_CARGO + 1,
                                            0xAB);

    const auto refused = path.enqueue(0, too_big);
    EXPECT_FALSE(refused.ok());
    EXPECT_EQ(refused.error(), Sh2Error::QUEUE_FULL);
    // Refused, not partially staged.
    EXPECT_TRUE(path.empty());
}

TEST(TxPath, ACargoExactlyFillingASlotIsAccepted) {
    TxPath path;
    const std::vector<std::uint8_t> exact(ActiveProfile::MAX_TX_CARGO, 0xCD);

    EXPECT_TRUE(path.enqueue(0, exact).ok());
    EXPECT_FALSE(path.empty());
}

TEST(TxPath, ResetDropsEverythingStaged) {
    TxPath path;
    const std::array<std::uint8_t, 1> payload{0x22};
    ASSERT_TRUE(path.enqueue(0, payload).ok());
    ASSERT_TRUE(path.enqueue(1, payload).ok());
    ASSERT_FALSE(path.empty());

    path.reset();

    EXPECT_TRUE(path.empty());
    // Room again, so reset released the slots rather than only hiding them.
    for (std::size_t i = 0; i < ActiveProfile::TX_QUEUE_DEPTH; ++i) {
        EXPECT_TRUE(path.enqueue(0, payload).ok());
    }
}

TEST(TxPath, AnUnboundRoleHasNowhereToGo) {
    TxPath path;
    const std::array<std::uint8_t, 1> payload{0x33};

    const auto result =
        path.enqueue(pico_sh2::shtp::ChannelRole::CONTROL, payload);

    EXPECT_FALSE(result.ok());
    // A different failure from a full queue, and a different remedy.
    EXPECT_EQ(result.error(), Sh2Error::NOT_READY);
}

}  // namespace
