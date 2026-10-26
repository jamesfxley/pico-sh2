// SPDX-License-Identifier: MIT
#include "pico_sh2/testing/fake_hub.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cstdint>
#include <span>

#include "pico_sh2/shtp/channels.hpp"
#include "pico_sh2/shtp/header.hpp"
#include "pico_sh2/util/result.hpp"

namespace pico_sh2::testing {
namespace {

TEST(FakeHub, DeliversQueuedTransferWithTimestamp) {
    FakeHub hub;
    const std::array<std::uint8_t, 5> frame{0x05, 0x00, 0x02, 0x00, 0xAA};
    hub.queue_transfer(frame, 1234);

    EXPECT_TRUE(hub.data_ready());

    std::array<std::uint8_t, 32> dst{};
    auto n = hub.read_transfer(dst);
    ASSERT_TRUE(n.ok());
    EXPECT_EQ(n.value().length, frame.size());
    EXPECT_EQ(n.value().intn_edge_us, 1234U);
    EXPECT_EQ(dst[4], 0xAA);
    EXPECT_FALSE(hub.data_ready());
}

TEST(FakeHub, CapturesWritesAndInjectsFailure) {
    FakeHub hub;
    const std::array<std::uint8_t, 4> frame{0x04, 0x00, 0x01, 0x00};

    hub.fail_next_write(Sh2Error::BUS_ERROR);
    EXPECT_EQ(hub.write_transfer(frame).error(), Sh2Error::BUS_ERROR);
    EXPECT_TRUE(hub.writes().empty());

    ASSERT_TRUE(hub.write_transfer(frame).ok());
    ASSERT_EQ(hub.writes().size(), 1U);
    EXPECT_EQ(hub.writes()[0].frame[2], 0x01);
}

// The contract both shipped transports keep: an absent hub reads as SUCCESS
// with a zero-length header, never as an error. Session::drain_transfers()
// faults on any read error -- permanently so during startup -- so a transport
// that reported an idle bus as a fault would kill a healthy session. This test
// previously asserted the opposite, which is how the fake came to teach the
// wrong rule to anyone reading it for one.
TEST(FakeHub, ReadWithNoDataSucceedsWithAZeroLengthHeader) {
    FakeHub hub;
    std::array<std::uint8_t, 8> dst{0xFF, 0xFF, 0xFF, 0xFF,
                                    0xFF, 0xFF, 0xFF, 0xFF};
    const auto read = hub.read_transfer(dst);
    ASSERT_TRUE(read.ok());
    EXPECT_EQ(read.value().length, pico_sh2::shtp::HEADER_SIZE);
    // The header is zeroed, so parse_header() reads it as a null transfer.
    EXPECT_EQ(dst[0], 0U);
    EXPECT_EQ(dst[1], 0U);
    const auto parsed = pico_sh2::shtp::parse_header(
        std::span<const std::uint8_t>(dst.data(), pico_sh2::shtp::HEADER_SIZE));
    ASSERT_TRUE(parsed.ok());
    EXPECT_TRUE(parsed.value().is_empty());
}

// A read with nothing behind it reports the most recent earlier edge, as the
// interface requires of every transport, not the time the read ran.
TEST(FakeHub, AReadWithNothingQueuedReportsTheLastEdge) {
    FakeHub hub;
    const std::array<std::uint8_t, 5> frame{0x05, 0x00, 0x02, 0x00, 0xAA};
    hub.queue_transfer(frame, 1234);
    std::array<std::uint8_t, 32> dst{};
    ASSERT_TRUE(hub.read_transfer(dst).ok());

    hub.advance_time(10'000);
    const auto empty = hub.read_transfer(dst);
    ASSERT_TRUE(empty.ok());
    EXPECT_EQ(empty.value().intn_edge_us, 1234U);
}

// A buffer too small to hold a header is a caller bug, not an idle bus.
TEST(FakeHub, ReadIntoAnUndersizedBufferIsABusError) {
    FakeHub hub;
    std::array<std::uint8_t, 2> dst{};
    EXPECT_EQ(hub.read_transfer(dst).error(), Sh2Error::BUS_ERROR);
}

TEST(FakeHub, TracksResetAndTime) {
    FakeHub hub;
    EXPECT_FALSE(hub.reset_asserted());
    hub.set_reset(true);
    EXPECT_TRUE(hub.reset_asserted());
    hub.advance_time(500);
    EXPECT_EQ(hub.now_us(), 500U);
}

}  // namespace
}  // namespace pico_sh2::testing
