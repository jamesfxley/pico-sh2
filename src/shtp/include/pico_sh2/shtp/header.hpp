// SPDX-License-Identifier: MIT
#pragma once

/// \file header.hpp
/// Four-byte SHTP header: little-endian length, channel and sequence.
/// Length includes the header; bit 15 marks continuation (1000-3535 §2.2.1).
/// Reserved 0xFFFF can indicate a failed peripheral and returns BAD_HEADER.

#include <cstddef>
#include <cstdint>
#include <span>

#include "pico_sh2/shtp/channels.hpp"
#include "pico_sh2/util/result.hpp"
#include "pico_util/bytes/byte_order.hpp"

namespace pico_sh2::shtp {

/// Parsed header; transfer_length includes the four header bytes.
struct ShtpHeader {
    /// Bytes including one header: total cargo for a lead, remaining cargo for
    /// a continuation (1000-3535 §2.2.1, §2.3.1).
    std::uint16_t transfer_length = 0;
    std::uint8_t channel = 0;   ///< Channel number the transfer belongs to.
    std::uint8_t sequence = 0;  ///< The channel's sequence number.
    /// Whether this transfer continues a cargo rather than starting one.
    bool continuation = false;

    /// Payload length in bytes; lengths below HEADER_SIZE saturate to zero.
    [[nodiscard]] constexpr std::size_t cargo_length() const {
        return transfer_length >= HEADER_SIZE
                   ? static_cast<std::size_t>(transfer_length) - HEADER_SIZE
                   : 0;
    }

    /// True for a null header, indicating no cargo.
    [[nodiscard]] constexpr bool is_empty() const {
        return transfer_length == 0;
    }
};

/// Parse a header; short input, reserved length and non-zero lengths below four
/// return BAD_HEADER.
constexpr Result<ShtpHeader> parse_header(
    std::span<const std::uint8_t> transfer) {
    if (transfer.size() < HEADER_SIZE) {
        return Sh2Error::BAD_HEADER;
    }
    const auto raw = pico_util::read_le<std::uint16_t>(transfer);
    if (raw == RESERVED_LENGTH) {
        return Sh2Error::BAD_HEADER;
    }
    const auto length = static_cast<std::uint16_t>(raw & LENGTH_MASK);
    /// Length includes the header (1000-3535 §2.2.1); zero means no cargo
    /// (§2.3.1).
    if (length != 0 && length < HEADER_SIZE) {
        return Sh2Error::BAD_HEADER;
    }
    ShtpHeader header;
    header.transfer_length = length;
    header.continuation = (raw & CONTINUATION_MASK) != 0;
    header.channel = transfer[2];
    header.sequence = transfer[3];
    return header;
}

/// Distinct sequence type to prevent swapping it with the byte-sized channel.
struct SequenceNumber {
    std::uint8_t value;  ///< The sequence number itself.
};

/// Serialise an unfragmented header; length includes its four bytes.
/// SHTP permits fragmentation but hubs may reject it (1000-3535 §2.3.1, §2.4).
/// BNO08X parts reject fragmented writes (1000-3927 §1.3.1).
constexpr Result<void> serialize_header(std::span<std::uint8_t> out,
                                        std::uint8_t channel,
                                        SequenceNumber sequence,
                                        std::size_t cargo_length) {
    if (out.size() < HEADER_SIZE) {
        return Sh2Error::INVALID_ARGUMENT;
    }
    const std::size_t transfer_length = cargo_length + HEADER_SIZE;
    /// The length field is 15 bits; a cargo that overruns it cannot be framed.
    if (transfer_length > LENGTH_MASK) {
        return Sh2Error::CARGO_TOO_LARGE;
    }
    pico_util::write_le<std::uint16_t>(
        out, static_cast<std::uint16_t>(transfer_length));
    out[2] = channel;
    out[3] = sequence.value;
    return Result<void>{};
}

}  // namespace pico_sh2::shtp
