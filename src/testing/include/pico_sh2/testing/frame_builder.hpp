// SPDX-License-Identifier: MIT
#pragma once

/// \file frame_builder.hpp
/// Wire-format builders for tests: raw SHTP frames and the control-channel
/// reports the startup handshake is made of.
///
/// Every suite that drives the stack needs these, and hand-rolling the header
/// arithmetic in each one is how a test ends up asserting against its own
/// mistake rather than against the protocol. They live here so there is one
/// definition of what a frame looks like on the wire.
///
/// Test-support code: it uses the heap freely (the zero-post-init-heap rule
/// governs the library on-device, not the fakes).

#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <initializer_list>
#include <span>
#include <string_view>
#include <vector>

#include "pico_sh2/sh2/command.hpp"
#include "pico_sh2/sh2/feature.hpp"
#include "pico_sh2/sh2/flush.hpp"
#include "pico_sh2/sh2/frs.hpp"
#include "pico_sh2/shtp/advertisement.hpp"
#include "pico_sh2/shtp/channels.hpp"
#include "pico_util/bytes/byte_order.hpp"

namespace pico_sh2::testing {

/// One raw SHTP transfer: the 4-byte header (length includes the header, bit 15
/// of the length is the continuation flag) followed by `payload`.
///
/// Precondition: `payload.size() + HEADER_SIZE <= LENGTH_MASK`, the most the
/// header's 15-bit length can say. A longer payload aborts rather than build a
/// frame whose header disagrees with its body.
inline std::vector<std::uint8_t> frame(std::uint8_t channel, std::uint8_t seq,
                                       std::span<const std::uint8_t> payload,
                                       bool continuation = false) {
    const std::size_t total = payload.size() + shtp::HEADER_SIZE;
    // The header's length field is 15 bits; a longer frame would encode a
    // length its body disagrees with, so it is refused rather than built.
    if (total > shtp::LENGTH_MASK) {
        std::abort();
    }
    auto high = static_cast<std::uint8_t>((total >> 8U) & 0x7FU);
    if (continuation) {
        high = static_cast<std::uint8_t>(high | 0x80U);
    }
    // Built from the header bytes, rather than reserved and appended to: GCC
    // 15 at -O3 reports a false -Wfree-nonheap-object from push_back's
    // reallocation path even after reserve().
    std::vector<std::uint8_t> out{static_cast<std::uint8_t>(total & 0xFFU),
                                  high, channel, seq};
    out.insert(out.end(), payload.begin(), payload.end());
    return out;
}

/// A braced payload does not deduce to a span, and spelling out a vector at
/// every call site is what pushed tests into hand-rolling the header instead.
inline std::vector<std::uint8_t> frame(
    std::uint8_t channel, std::uint8_t seq,
    std::initializer_list<std::uint8_t> payload, bool continuation = false) {
    return frame(channel, seq,
                 std::span<const std::uint8_t>(payload.begin(), payload.size()),
                 continuation);
}

/// Append one TLV entry: tag, length, value.
inline void append_tlv(std::vector<std::uint8_t>& out, std::uint8_t tag,
                       std::initializer_list<std::uint8_t> value) {
    out.push_back(tag);
    out.push_back(static_cast<std::uint8_t>(value.size()));
    out.insert(out.end(), value.begin(), value.end());
}

/// Append a TLV whose value is a null-terminated string, as ChannelName and
/// AppName are (1000-3535 §5.2).
inline void append_string_tlv(std::vector<std::uint8_t>& out, std::uint8_t tag,
                              std::string_view text) {
    out.push_back(tag);
    out.push_back(static_cast<std::uint8_t>(text.size() + 1));
    for (const char character : text) {
        out.push_back(static_cast<std::uint8_t>(character));
    }
    out.push_back(0);
}

/// Append a GUID TLV; the value is four bytes, LSB first.
inline void append_guid(std::vector<std::uint8_t>& out, std::uint8_t guid) {
    append_tlv(out, shtp::GUID_TAG, {guid, 0x00, 0x00, 0x00});
}

/// A channel advertisement in the shape 1000-3600 §2.3 defines, as the
/// Response-0 cargo it arrives in on channel 0: the SHTP
/// application (GUID 0) naming the command channel, the executable application
/// (GUID 1) naming `device`, and the sensorhub (GUID 2) naming control,
/// inputNormal, inputWake and inputGyroRv. SH-2's five sit at N..N+4 for a
/// hub-chosen N, so `base` is that N — a test can move the whole block and the
/// session must follow it. One tag-0x81 report-length pair closes the sensorhub
/// scope so a test can observe that lengths were applied too.
inline std::vector<std::uint8_t> advertisement(std::uint8_t report_id = 0x05,
                                               std::uint8_t length = 0x18,
                                               std::uint8_t base = 1) {
    // Byte 0 of a channel-0 read cargo is the SHTP response code; an
    // advertisement is Response 0 (1000-3535 §5.1.1.1).
    std::vector<std::uint8_t> out{0x00};
    append_guid(out, 0);
    append_string_tlv(out, shtp::APP_NAME_TAG, "SHTP");
    append_tlv(out, shtp::NORMAL_CHANNEL_TAG, {shtp::COMMAND_CHANNEL});
    append_string_tlv(out, shtp::CHANNEL_NAME_TAG, "control");

    append_guid(out, 1);
    append_string_tlv(out, shtp::APP_NAME_TAG, "executable");
    append_tlv(out, shtp::NORMAL_CHANNEL_TAG, {base});
    append_string_tlv(out, shtp::CHANNEL_NAME_TAG, "device");

    append_guid(out, 2);
    append_string_tlv(out, shtp::APP_NAME_TAG, "sensorhub");
    append_tlv(out, shtp::NORMAL_CHANNEL_TAG,
               {static_cast<std::uint8_t>(base + 1)});
    append_string_tlv(out, shtp::CHANNEL_NAME_TAG, "control");
    append_tlv(out, shtp::NORMAL_CHANNEL_TAG,
               {static_cast<std::uint8_t>(base + 2)});
    append_string_tlv(out, shtp::CHANNEL_NAME_TAG, "inputNormal");
    append_tlv(out, shtp::WAKE_CHANNEL_TAG,
               {static_cast<std::uint8_t>(base + 3)});
    append_string_tlv(out, shtp::CHANNEL_NAME_TAG, "inputWake");
    append_tlv(out, shtp::NORMAL_CHANNEL_TAG,
               {static_cast<std::uint8_t>(base + 4)});
    append_string_tlv(out, shtp::CHANNEL_NAME_TAG, "inputGyroRv");

    append_tlv(out, shtp::REPORT_LENGTHS_TAG, {report_id, length});
    return out;
}

/// The channel numbers `advertisement(.., base)` assigns, for a test that has
/// to address one directly.
constexpr std::uint8_t device_channel(std::uint8_t base = 1) { return base; }

/// \param base the first channel number the advertisement assigns.
/// \return the control channel number.
constexpr std::uint8_t control_channel(std::uint8_t base = 1) {
    return static_cast<std::uint8_t>(base + 1);
}

/// \param base the first channel number the advertisement assigns.
/// \return the normal input channel number.
constexpr std::uint8_t input_normal_channel(std::uint8_t base = 1) {
    return static_cast<std::uint8_t>(base + 2);
}

/// \param base the first channel number the advertisement assigns.
/// \return the wake input channel number.
constexpr std::uint8_t input_wake_channel(std::uint8_t base = 1) {
    return static_cast<std::uint8_t>(base + 3);
}

/// \param base the first channel number the advertisement assigns.
/// \return the gyro-integrated rotation vector channel number.
constexpr std::uint8_t input_gyro_rv_channel(std::uint8_t base = 1) {
    return static_cast<std::uint8_t>(base + 4);
}

/// The channel map a consumer seeds for a hub laid out at `base`, so a reset
/// can be requested before the hub has advertised (SR-7).
inline shtp::ChannelMap seeded_channels(std::uint8_t base = 1) {
    shtp::ChannelMap map;
    map.seed(shtp::ChannelRole::DEVICE, device_channel(base));
    map.seed(shtp::ChannelRole::CONTROL, control_channel(base));
    map.seed(shtp::ChannelRole::INPUT_NORMAL, input_normal_channel(base));
    map.seed(shtp::ChannelRole::INPUT_WAKE, input_wake_channel(base));
    map.seed(shtp::ChannelRole::INPUT_GYRO_RV, input_gyro_rv_channel(base));
    return map;
}

/// The device channel's reset-complete cargo (1000-3600 §2.2).
inline std::vector<std::uint8_t> reset_complete() { return {0x01}; }

/// A command response (0xF1). `command_sequence` echoes the request's sequence;
/// an unsolicited response sets the MSB of the command byte.
inline std::vector<std::uint8_t> command_response(std::uint8_t command,
                                                  std::uint8_t command_sequence,
                                                  bool unsolicited) {
    std::vector<std::uint8_t> frame(COMMAND_RESPONSE_SIZE, 0);
    frame[0] = COMMAND_RESPONSE_REPORT;
    frame[COMMAND_RESP_COMMAND] =
        unsolicited
            ? static_cast<std::uint8_t>(command | COMMAND_UNSOLICITED_FLAG)
            : command;
    frame[COMMAND_RESP_CMD_SEQUENCE] = command_sequence;
    return frame;
}

/// The unsolicited Initialize response that ends the handshake: 0xF1 with
/// command byte 0x84 and R0 carrying the status (SH-2 Reference Manual §6.4.4).
inline std::vector<std::uint8_t> initialize_response(std::uint8_t status = 0) {
    std::vector<std::uint8_t> frame =
        command_response(command::INITIALIZE, 0, true);
    frame[COMMAND_RESP_RESULTS] = status;
    return frame;
}

/// A Get Feature Response (0xFC) granting `report_interval_us` for `sensor`.
inline std::vector<std::uint8_t> feature_response(
    std::uint8_t sensor, std::uint32_t report_interval_us) {
    std::vector<std::uint8_t> frame(FEATURE_REPORT_SIZE, 0);
    frame[0] = GET_FEATURE_RESPONSE;
    frame[1] = sensor;
    pico_util::write_le<std::uint32_t>(frame, report_interval_us, 5);
    return frame;
}

/// An FRS Read Response (0xF3) for record `frs_type`, carrying `data_words`
/// words (0-2) from record word `first_word`, with the 4-bit `status` sharing
/// byte 1 with the word count.
inline std::vector<std::uint8_t> frs_read_response(
    std::uint16_t frs_type, std::uint8_t status, std::uint8_t data_words,
    std::uint16_t first_word, std::uint32_t word0, std::uint32_t word1) {
    std::vector<std::uint8_t> frame(FRS_READ_RESPONSE_SIZE, 0);
    frame[0] = FRS_READ_RESPONSE_REPORT;
    frame[FRS_READ_RESP_LENGTH_STATUS] = static_cast<std::uint8_t>(
        (static_cast<unsigned>(data_words) << FRS_READ_DATA_LEN_SHIFT) |
        (static_cast<unsigned>(status) & FRS_READ_STATUS_MASK));
    pico_util::write_le<std::uint16_t>(frame, first_word,
                                       FRS_READ_RESP_WORD_OFFSET);
    pico_util::write_le<std::uint32_t>(frame, word0, FRS_READ_RESP_DATA);
    pico_util::write_le<std::uint32_t>(
        frame, word1, FRS_READ_RESP_DATA + sizeof(std::uint32_t));
    pico_util::write_le<std::uint16_t>(frame, frs_type, FRS_READ_RESP_TYPE);
    return frame;
}

/// A Flush Completed report (0xEF) naming the flushed sensor (§6.5.7).
inline std::vector<std::uint8_t> flush_completed(std::uint8_t sensor) {
    return {FLUSH_COMPLETED_REPORT, sensor};
}

/// An FRS Write Response (0xF5) with the given status.
inline std::vector<std::uint8_t> frs_write_response(std::uint8_t status) {
    std::vector<std::uint8_t> frame(FRS_WRITE_RESPONSE_SIZE, 0);
    frame[0] = FRS_WRITE_RESPONSE_REPORT;
    frame[FRS_WRITE_RESP_STATUS] = status;
    return frame;
}

}  // namespace pico_sh2::testing
