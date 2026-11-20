// SPDX-License-Identifier: MIT
#include "pico_sh2/shtp/header.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>

#include "pico_sh2/util/result.hpp"

namespace {

using pico_sh2::Sh2Error;

using pico_sh2::shtp::parse_header;
using pico_sh2::shtp::SequenceNumber;
using pico_sh2::shtp::serialize_header;

// The channel and the sequence number are both a byte; a call that passes
// them the wrong way round must not compile.
template <typename Channel, typename Sequence>
concept FramesWith =
    requires(std::span<std::uint8_t> out, Channel channel, Sequence sequence) {
        serialize_header(out, channel, sequence, std::size_t{0});
    };
static_assert(FramesWith<std::uint8_t, SequenceNumber>);
static_assert(!FramesWith<SequenceNumber, std::uint8_t>);
static_assert(!FramesWith<std::uint8_t, std::uint8_t>);

TEST(ShtpHeader, ParsesLengthChannelSequence) {
    // length 0x0104 = 260 bytes total, channel 2, sequence 0x11.
    constexpr std::array<std::uint8_t, 4> RAW{0x04, 0x01, 0x02, 0x11};
    const auto header = parse_header(RAW);
    ASSERT_TRUE(header.ok());
    EXPECT_EQ(header.value().transfer_length, 260);
    EXPECT_EQ(header.value().channel, 2);
    EXPECT_EQ(header.value().sequence, 0x11);
    EXPECT_FALSE(header.value().continuation);
    EXPECT_EQ(header.value().cargo_length(), 256U);
}

TEST(ShtpHeader, DecodesContinuationFlag) {
    // bit 15 set -> continuation; masked length is 0x0005 = 5.
    constexpr std::array<std::uint8_t, 4> RAW{0x05, 0x80, 0x03, 0x00};
    const auto header = parse_header(RAW);
    ASSERT_TRUE(header.ok());
    EXPECT_TRUE(header.value().continuation);
    EXPECT_EQ(header.value().transfer_length, 5);
    EXPECT_EQ(header.value().cargo_length(), 1U);
}

// Reserved length 0xFFFF is rejected (NR-1).
TEST(ShtpHeader, RejectsReservedLength) {
    constexpr std::array<std::uint8_t, 4> RAW{0xFF, 0xFF, 0x02, 0x00};
    EXPECT_EQ(parse_header(RAW).error(), Sh2Error::BAD_HEADER);
}

// Transfers shorter than four bytes are rejected (NR-1).
TEST(ShtpHeader, RejectsShortBuffer) {
    constexpr std::array<std::uint8_t, 3> RAW{0x04, 0x00, 0x02};
    EXPECT_EQ(parse_header(RAW).error(), Sh2Error::BAD_HEADER);
}

TEST(ShtpHeader, EmptyTransferIsIdle) {
    constexpr std::array<std::uint8_t, 4> RAW{0x00, 0x00, 0x03, 0x07};
    const auto header = parse_header(RAW);
    ASSERT_TRUE(header.ok());
    EXPECT_TRUE(header.value().is_empty());
    EXPECT_EQ(header.value().cargo_length(), 0U);
}

// A nonzero length must count at least the header bytes (NR-1).
TEST(ShtpHeader, RejectsNonzeroLengthBelowHeaderSize) {
    // Below the header size, only length zero is valid (1000-3535 §2.2.1).
    for (const std::uint8_t len :
         {std::uint8_t{1}, std::uint8_t{2}, std::uint8_t{3}}) {
        const std::array<std::uint8_t, 4> raw{len, 0x00, 0x02, 0x00};
        EXPECT_EQ(parse_header(raw).error(), Sh2Error::BAD_HEADER);
        // Same masked length under the continuation flag is equally invalid.
        const std::array<std::uint8_t, 4> cont{len, 0x80, 0x02, 0x00};
        EXPECT_EQ(parse_header(cont).error(), Sh2Error::BAD_HEADER);
    }
}

// No transfer is longer than 32766 bytes (NR-1, 1000-3535 §2.2.1).
TEST(ShtpHeader, RejectsALengthAboveTheLargestTransfer) {
    constexpr std::array<std::uint8_t, 4> TOO_LONG{0xFF, 0x7F, 0x02, 0x00};
    EXPECT_EQ(parse_header(TOO_LONG).error(), Sh2Error::BAD_HEADER);

    constexpr std::array<std::uint8_t, 4> LARGEST{0xFE, 0x7F, 0x02, 0x00};
    const auto header = parse_header(LARGEST);
    ASSERT_TRUE(header.ok());
    EXPECT_EQ(header.value().transfer_length, 32766);
}

TEST(ShtpHeader, SerializeAddsHeaderToCargoLength) {
    std::array<std::uint8_t, 4> out{};
    ASSERT_TRUE(serialize_header(out, 2, SequenceNumber{0x09}, 17).ok());
    // 17 payload + 4 header = 21, channel 2, sequence 9, continuation clear.
    EXPECT_EQ(out[0], 21);
    EXPECT_EQ(out[1], 0x00);
    EXPECT_EQ(out[2], 2);
    EXPECT_EQ(out[3], 0x09);

    const auto round = parse_header(out);
    ASSERT_TRUE(round.ok());
    EXPECT_EQ(round.value().cargo_length(), 17U);
    EXPECT_FALSE(round.value().continuation);
}

// serialize_header accepts at most 0x7FFB cargo bytes (length 0x7FFF);
// 1000-3535 §2.2.1 gives the largest cargo as 32766 minus the header.
TEST(ShtpHeader, SerializeRejectsOverlongCargo) {
    std::array<std::uint8_t, 4> out{};
    EXPECT_TRUE(serialize_header(out, 2, SequenceNumber{0}, 0x7FFB).ok());
    EXPECT_EQ(serialize_header(out, 2, SequenceNumber{0}, 0x7FFC).error(),
              Sh2Error::CARGO_TOO_LARGE);
}

TEST(ShtpHeader, SerializeRejectsShortBuffer) {
    std::array<std::uint8_t, 3> out{};
    EXPECT_EQ(serialize_header(out, 2, SequenceNumber{0}, 1).error(),
              Sh2Error::INVALID_ARGUMENT);
}

}  // namespace
