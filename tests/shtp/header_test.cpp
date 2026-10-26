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

// NR-1: 0xFFFF is the reserved dead-bus signature and is never parsed.
TEST(ShtpHeader, RejectsReservedLength) {
    constexpr std::array<std::uint8_t, 4> RAW{0xFF, 0xFF, 0x02, 0x00};
    EXPECT_EQ(parse_header(RAW).error(), Sh2Error::BAD_HEADER);
}

// NR-1: a transfer shorter than the 4-byte minimum is rejected.
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

// NR-1: a nonzero length field below the header size it must count.
TEST(ShtpHeader, RejectsNonzeroLengthBelowHeaderSize) {
    // The length field counts the 4 header bytes (1000-3535 §2.2.1); a nonzero
    // length below HEADER_SIZE claims a transfer shorter than its own header
    // and is rejected. Only length 0 (null header) is a valid sub-header
    // length.
    for (const std::uint8_t len :
         {std::uint8_t{1}, std::uint8_t{2}, std::uint8_t{3}}) {
        const std::array<std::uint8_t, 4> raw{len, 0x00, 0x02, 0x00};
        EXPECT_EQ(parse_header(raw).error(), Sh2Error::BAD_HEADER);
        // Same masked length under the continuation flag is equally invalid.
        const std::array<std::uint8_t, 4> cont{len, 0x80, 0x02, 0x00};
        EXPECT_EQ(parse_header(cont).error(), Sh2Error::BAD_HEADER);
    }
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

// The length field is 15 bits and counts the header too, so the largest cargo
// is 0x7FFF - 4 = 0x7FFB. Both sides of that edge: one cargo past it would set
// the continuation bit if the check were made against 0x8000.
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
