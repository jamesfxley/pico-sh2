// SPDX-License-Identifier: MIT
#pragma once

/// \file frame_builder.hpp
/// Shared wire-format builders for host tests; heap allocation is allowed here.

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

/// SHTP header and payload; bit 15 marks continuation.
/// A total length beyond LENGTH_MASK aborts.
inline std::vector<std::uint8_t> frame(std::uint8_t channel, std::uint8_t seq,
                                       std::span<const std::uint8_t> payload,
                                       bool continuation = false) {
    const std::size_t total = payload.size() + shtp::HEADER_SIZE;
    if (total > shtp::LENGTH_MASK) {
        std::abort();
    }
    auto high = static_cast<std::uint8_t>((total >> 8U) & 0x7FU);
    if (continuation) {
        high = static_cast<std::uint8_t>(high | 0x80U);
    }
    // Constructing the header directly avoids GCC 15 -O3's false
    // -Wfree-nonheap-object in push_back reallocation after reserve().
    std::vector<std::uint8_t> out{static_cast<std::uint8_t>(total & 0xFFU),
                                  high, channel, seq};
    out.insert(out.end(), payload.begin(), payload.end());
    return out;
}

/// Braced-payload overload; a braced list does not deduce to a span.
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

/// Response-0 advertisement with channels at base (1000-3600 §2.3).
/// The session must follow a shifted base; one report-length pair checks
/// application.
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

/// Device channel assigned by advertisement(..., base).
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

/// Channel seeds at base for reset before advertisement (SR-7).
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

/// Response echoing command_sequence; unsolicited sets the command MSB.
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

/// Unsolicited Initialize with R0 status (1000-3625 §6.4.4).
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

/// Read response with 0–2 words at first_word and status in byte 1.
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
