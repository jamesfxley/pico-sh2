// SPDX-License-Identifier: MIT
#pragma once

/// \file header.hpp
/// SHTP transfer header: the 4-byte prefix on every read from and write to the
/// sensor hub. Layout (little-endian):
///
///   byte 0  length LSB
///   byte 1  length MSB; bit 15 (this byte's bit 7) is the continuation flag
///   byte 2  channel
///   byte 3  sequence number (per channel, per direction)
///
/// The 15-bit length counts the whole transfer including these 4 header bytes.
/// A raw length field of 0xFFFF is reserved and signals a dead/failed bus read
/// (1000-3535 §2.2.1: a failed peripheral can produce 0xFFFF); it is
/// never a valid length and is rejected as BAD_HEADER.

#include <cstddef>
#include <cstdint>
#include <span>

#include "pico_sh2/shtp/channels.hpp"
#include "pico_sh2/util/result.hpp"
#include "pico_util/bytes/byte_order.hpp"

namespace pico_sh2::shtp {

/// A parsed SHTP header. `transfer_length` is the full on-wire length including
/// the 4 header bytes; `cargo_length()` gives the payload that follows.
struct ShtpHeader {
    /// Full on-wire transfer length, the 4 header bytes included.
    std::uint16_t transfer_length = 0;
    std::uint8_t channel = 0;   ///< Channel number the transfer belongs to.
    std::uint8_t sequence = 0;  ///< The channel's sequence number.
    /// Whether this transfer continues a cargo rather than starting one.
    bool continuation = false;

    /// Payload bytes after the 4-byte header. A length field below HEADER_SIZE
    /// is degenerate (an empty transfer reports length 0); saturate to zero
    /// rather than underflow so callers never see a bogus huge cargo.
    [[nodiscard]] constexpr std::size_t cargo_length() const {
        return transfer_length >= HEADER_SIZE
                   ? static_cast<std::size_t>(transfer_length) - HEADER_SIZE
                   : 0;
    }

    /// True for the zero-length keep-alive/idle transfer a hub emits when
    /// polled with no data pending. Skipped as a no-op rather than an error.
    [[nodiscard]] constexpr bool is_empty() const {
        return transfer_length == 0;
    }
};

/// Parse the first HEADER_SIZE bytes of `transfer`. Rejects a reserved (0xFFFF)
/// length field. Requires at least HEADER_SIZE bytes; shorter input is a bad
/// read, not a valid short header.
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
    /// The length field counts the 4 header bytes (1000-3535 §2.2.1), so a
    /// nonzero length below HEADER_SIZE describes a transfer shorter than its
    /// own header — impossible, and rejected. Length 0 is a valid null header
    /// (no cargo, 1000-3535 §2.3.1).
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

/// An SHTP sequence number. Its own type so that serialize_header() cannot
/// take it and the channel number the wrong way round: both are a byte.
struct SequenceNumber {
    std::uint8_t value;  ///< The sequence number itself.
};

/// Serialize a header into the first HEADER_SIZE bytes of `out`. `cargo_length`
/// is the payload size; the on-wire length adds the 4 header bytes. The
/// continuation flag is always clear: this library does not fragment writes.
/// SHTP allows segmentation (1000-3535 §2.3.1); hubs may refuse it (§2.4).
/// BNO08X parts refuse fragmented writes (1000-3927 §1.3.1).
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
