// SPDX-License-Identifier: MIT
#pragma once

/// \file frs.hpp
/// Flash Record System (FRS) read/write reports for configuration and
/// calibration records (1000-3625 §6.3.3–6.3.7). Only one operation may run at
/// a time.

#include <cstddef>
#include <cstdint>
#include <span>

#include "pico_util/bytes/byte_order.hpp"

namespace pico_sh2 {

/// Report ID of an FRS Write Request, opening a write (§6.3.3).
inline constexpr std::uint8_t FRS_WRITE_REQUEST_REPORT = 0xF7;
/// Report ID of an FRS Write Data report, carrying up to two words (§6.3.4).
inline constexpr std::uint8_t FRS_WRITE_DATA_REPORT = 0xF6;
/// Report ID of an FRS Write Response, one per request or data report (§6.3.5).
inline constexpr std::uint8_t FRS_WRITE_RESPONSE_REPORT = 0xF5;
/// Report ID of an FRS Read Request, opening a read (§6.3.6).
inline constexpr std::uint8_t FRS_READ_REQUEST_REPORT = 0xF4;
/// Report ID of an FRS Read Response, streamed until the record is complete
/// (§6.3.7).
inline constexpr std::uint8_t FRS_READ_RESPONSE_REPORT = 0xF3;

/// Wire length of an FRS Read Request (§6.3.6).
inline constexpr std::size_t FRS_READ_REQUEST_SIZE = 8;
/// Wire length of an FRS Read Response (§6.3.7).
inline constexpr std::size_t FRS_READ_RESPONSE_SIZE = 16;
/// Wire length of an FRS Write Request (§6.3.3).
inline constexpr std::size_t FRS_WRITE_REQUEST_SIZE = 6;
/// Wire length of an FRS Write Data report (§6.3.4).
inline constexpr std::size_t FRS_WRITE_DATA_SIZE = 12;
/// Wire length of an FRS Write Response (§6.3.5).
inline constexpr std::size_t FRS_WRITE_RESPONSE_SIZE = 4;

/// Words carried per FRS read response / write data report.
inline constexpr std::size_t FRS_WORDS_PER_REPORT = 2;

/// Byte offset of packed status (bits 3:0) and word count (7:4) (§6.3.7).
inline constexpr std::size_t FRS_READ_RESP_LENGTH_STATUS = 1;
/// Byte offset of the u16le word offset within an FRS Read Response.
inline constexpr std::size_t FRS_READ_RESP_WORD_OFFSET = 2;
/// Byte offset of the first u32le data word within an FRS Read Response.
inline constexpr std::size_t FRS_READ_RESP_DATA = 4;
/// Byte offset of the record type (1000-3625 §6.3.7, Figure 39).
inline constexpr std::size_t FRS_READ_RESP_TYPE = 12;
/// Mask selecting the status nibble of FRS_READ_RESP_LENGTH_STATUS.
inline constexpr std::uint8_t FRS_READ_STATUS_MASK = 0x0F;
/// Right shift from the word-count nibble of FRS_READ_RESP_LENGTH_STATUS to
/// the low bits.
inline constexpr unsigned FRS_READ_DATA_LEN_SHIFT = 4;

/// Byte offset of the status in an FRS Write Response (0xF5, §6.3.5).
inline constexpr std::size_t FRS_WRITE_RESP_STATUS = 1;

/// Read status in byte 1, bits 3:0; other codes are deprecated or reserved
/// (§6.3.7).
namespace frs_read_status {
/// The report carries record data and the read continues.
inline constexpr std::uint8_t NO_ERROR = 0;
/// The requested record type is not one the hub recognises.
inline constexpr std::uint8_t UNRECOGNIZED_TYPE = 1;
/// Another FRS operation is in progress; that one runs to completion.
inline constexpr std::uint8_t BUSY = 2;
/// The record has been returned in full; this is the last response.
inline constexpr std::uint8_t RECORD_COMPLETED = 3;
/// The record type is recognised but holds no data.
inline constexpr std::uint8_t RECORD_EMPTY = 5;
/// The DFU flash memory device is unavailable.
inline constexpr std::uint8_t DEVICE_ERROR = 8;
}  // namespace frs_read_status

/// FRS write-response status (byte 1). §6.3.5.
namespace frs_write_status {
/// The words in the last data report were accepted; send the next.
inline constexpr std::uint8_t WORDS_RECEIVED = 0;
/// The requested record type is not one the hub recognises.
inline constexpr std::uint8_t UNRECOGNIZED_TYPE = 1;
/// Another FRS operation is in progress.
inline constexpr std::uint8_t BUSY = 2;
/// Every word of the record has been written.
inline constexpr std::uint8_t WRITE_COMPLETED = 3;
/// Write mode entered; the hub is ready for data reports.
inline constexpr std::uint8_t READY = 4;
/// The write did not take effect.
inline constexpr std::uint8_t WRITE_FAILED = 5;
/// A data report arrived without a write request opening the write.
inline constexpr std::uint8_t NOT_IN_WRITE_MODE = 6;
/// The declared record length is not valid for the record type.
inline constexpr std::uint8_t INVALID_LENGTH = 7;
/// The complete record passed the hub's internal validation; write completed
/// follows once it is written.
inline constexpr std::uint8_t RECORD_VALID = 8;
/// The complete record failed the hub's internal validation.
inline constexpr std::uint8_t RECORD_INVALID = 9;
/// The record type cannot be written.
inline constexpr std::uint8_t READ_ONLY = 11;
/// FRS memory is full, so the record cannot be written.
inline constexpr std::uint8_t MEMORY_FULL = 12;
}  // namespace frs_write_status

/// Well-known FRS record types (§4.3, "Figure 26"). u16 on the wire.
namespace frs_type {
/// Rotation applied to every sensor output, correcting the mounting.
inline constexpr std::uint16_t SYSTEM_ORIENTATION = 0x2D3E;
/// Configuration of the Gyro-Integrated Rotation Vector.
inline constexpr std::uint16_t GYRO_INTEGRATED_RV_CONFIG = 0xA1A2;
/// Dynamic calibration data (DCD), the hub's saved calibration state.
inline constexpr std::uint16_t DYNAMIC_CALIBRATION = 0x1F1F;
}  // namespace frs_type

/// Record type and length in 32-bit words; zero erases it.
/// Named fields prevent swapping the wire's length-then-type pair.
struct FrsWriteRequest {
    std::uint16_t type;          ///< FRS record type (see frs_type).
    std::uint16_t length_words;  ///< Record length in words; 0 erases it.
};

/// Serialise an FRS Read Request (0xF4) for `type` into `out`. Read offset and
/// block size are left zero: the whole record from the start.
constexpr std::size_t serialize_frs_read_request(
    std::uint16_t type, std::span<std::uint8_t, FRS_READ_REQUEST_SIZE> out) {
    for (std::size_t i = 0; i < FRS_READ_REQUEST_SIZE; ++i) {
        out[i] = 0;
    }
    out[0] = FRS_READ_REQUEST_REPORT;
    pico_util::write_le<std::uint16_t>(out, type, 4);
    return FRS_READ_REQUEST_SIZE;
}

/// Serialise an FRS Write Request (0xF7) opening `request` into `out`.
constexpr std::size_t serialize_frs_write_request(
    FrsWriteRequest request,
    std::span<std::uint8_t, FRS_WRITE_REQUEST_SIZE> out) {
    out[0] = FRS_WRITE_REQUEST_REPORT;
    out[1] = 0;
    pico_util::write_le<std::uint16_t>(out, request.length_words, 2);
    pico_util::write_le<std::uint16_t>(out, request.type, 4);
    return FRS_WRITE_REQUEST_SIZE;
}

/// Serialise two words at offset_words; the span prevents swapping adjacent
/// words.
constexpr std::size_t serialize_frs_write_data(
    std::uint16_t offset_words,
    std::span<const std::uint32_t, FRS_WORDS_PER_REPORT> words,
    std::span<std::uint8_t, FRS_WRITE_DATA_SIZE> out) {
    out[0] = FRS_WRITE_DATA_REPORT;
    out[1] = 0;
    pico_util::write_le<std::uint16_t>(out, offset_words, 2);
    pico_util::write_le<std::uint32_t>(out, words[0], 4);
    pico_util::write_le<std::uint32_t>(out, words[1], 8);
    return FRS_WRITE_DATA_SIZE;
}

}  // namespace pico_sh2
