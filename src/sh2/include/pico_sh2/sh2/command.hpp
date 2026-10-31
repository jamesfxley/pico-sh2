// SPDX-License-Identifier: MIT
#pragma once

/// \file command.hpp
/// SH-2 command request/response (1000-3625 §6.3.8-6.3.9, §6.4). A
/// command (0xF2) is a special operation the host asks the hub to perform; the
/// hub answers with one or more command responses (0xF1). Exactly one command
/// is outstanding at a time; responses are correlated by the request's sequence
/// number, echoed in the response's command-sequence-number field. A response
/// whose command byte has its MSB set is unsolicited (e.g. the post-reset
/// Initialize response) and never completes a pending command (SR-2).

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>

namespace pico_sh2 {

/// Report ID of a command request (§6.3.8).
inline constexpr std::uint8_t COMMAND_REQUEST_REPORT = 0xF2;
/// Report ID of a command response (§6.3.9).
inline constexpr std::uint8_t COMMAND_RESPONSE_REPORT = 0xF1;

/// Set in a response's command byte when the response is unsolicited.
inline constexpr std::uint8_t COMMAND_UNSOLICITED_FLAG = 0x80;

/// Wire length of a command request: 0xF2, sequence, command, P0-P8.
inline constexpr std::size_t COMMAND_REQUEST_SIZE = 12;
/// Wire length of a command response: 0xF1 and fifteen further bytes.
inline constexpr std::size_t COMMAND_RESPONSE_SIZE = 16;
/// Parameter bytes a command request carries, P0-P8.
// 1000-3625 §6.3.8 says P0-P9 in prose, but Figure 40 and Figures 51/62
// lay out P0-P8. COMMAND_MAX_PARAMS follows the figures.
inline constexpr std::size_t COMMAND_MAX_PARAMS = 9;
/// Result bytes a command response carries, R0-R10.
inline constexpr std::size_t COMMAND_MAX_RESULTS = 11;

/// Byte offset of the command byte within a command response (0xF1).
inline constexpr std::size_t COMMAND_RESP_COMMAND = 2;
/// Byte offset of the echoed request sequence number.
inline constexpr std::size_t COMMAND_RESP_CMD_SEQUENCE = 3;
/// Byte offset of the response-group counter.
inline constexpr std::size_t COMMAND_RESP_RESP_SEQUENCE = 4;
/// Byte offset of the first result byte; R0 to R10 follow.
inline constexpr std::size_t COMMAND_RESP_RESULTS = 5;

/// SH-2 command identifiers (§6.4). Range 1-127; the MSB is the unsolicited
/// flag.
namespace command {
/// Access the hub's error queue (§6.4.1).
inline constexpr std::uint8_t REPORT_ERRORS = 0x01;
/// Read or clear the per-sensor counters (§6.4.2).
inline constexpr std::uint8_t COUNTER = 0x02;
/// Tare the current orientation, persist a tare, or set reorientation
/// (§6.4.3).
inline constexpr std::uint8_t TARE = 0x03;
/// Reinitialize the sensor hub components (§6.4.4). Its response is also sent
/// unsolicited after a reset.
inline constexpr std::uint8_t INITIALIZE = 0x04;
/// Save the dynamic calibration data to flash (§6.4.5).
inline constexpr std::uint8_t SAVE_DCD = 0x06;
/// Configure which motion-engine calibrations run (§6.4.6).
inline constexpr std::uint8_t ME_CALIBRATION = 0x07;
/// Configure periodic saving of the dynamic calibration data (§6.4.7).
inline constexpr std::uint8_t PERIODIC_DCD = 0x09;
/// Report which oscillator the hub is running from (§6.4.8).
inline constexpr std::uint8_t OSCILLATOR_TYPE = 0x0A;
/// Erase the dynamic calibration data and reset the hub (§6.4.9).
inline constexpr std::uint8_t CLEAR_DCD_AND_RESET = 0x0B;
// Simple and turntable calibration, interactive calibration and the wheel
// request (§6.4.10, §6.4.12, §6.4.13) have no constant here (FW-1).
}  // namespace command

/// A completed command response handed to the consumer.
struct CommandResponse {
    /// The command this answers, with COMMAND_UNSOLICITED_FLAG cleared.
    std::uint8_t command = 0;
    /// Echoes the sequence number of the request that asked for it.
    std::uint8_t command_sequence = 0;
    /// Counts the responses within one response group.
    std::uint8_t response_sequence = 0;
    /// Result bytes R0 to R10, interpreted per command.
    std::array<std::uint8_t, COMMAND_MAX_RESULTS> results{};
};

/// A command request's sequence number, echoed by its response. Its own type
/// so that serialize_command_request() cannot take it and the command the
/// wrong way round: both are a byte.
struct CommandSequence {
    std::uint8_t value;  ///< The sequence number itself.
};

/// Serialize a command request into `out`. The first COMMAND_MAX_PARAMS
/// parameter bytes are copied and the rest of the field is zero-filled; a
/// caller refuses a longer `params` before reaching here.
constexpr std::size_t serialize_command_request(
    CommandSequence sequence, std::uint8_t command,
    std::span<const std::uint8_t> params,
    std::span<std::uint8_t, COMMAND_REQUEST_SIZE> out) {
    out[0] = COMMAND_REQUEST_REPORT;
    out[1] = sequence.value;
    out[2] = command;
    for (std::size_t i = 0; i < COMMAND_MAX_PARAMS; ++i) {
        out[3 + i] = i < params.size() ? params[i] : std::uint8_t{0};
    }
    return COMMAND_REQUEST_SIZE;
}

}  // namespace pico_sh2
