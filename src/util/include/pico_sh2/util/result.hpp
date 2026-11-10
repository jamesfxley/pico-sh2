// SPDX-License-Identifier: MIT
#pragma once

/// \file result.hpp
/// Library error vocabulary carried by pico_util::Result.

#include <cstdint>

#include "pico_util/result/result.hpp"

namespace pico_sh2 {

/// Library-wide error codes. Grouped by layer; NONE is success. New codes are
/// appended (never renumbered) so diagnostics captured on-device stay stable.
enum class Sh2Error : std::uint8_t {
    NONE = 0,

    /// Transport / HAL layer.
    BUS_ERROR,  // underlying I2C/SPI transaction failed
    TIMEOUT,    // deadline expired waiting on the hub
    NOT_READY,  // operation attempted before the session reached ready

    /// SHTP framing.
    BAD_HEADER,       // reserved length (0xFFFF) or otherwise malformed header
    CARGO_TOO_LARGE,  // outbound cargo exceeds one transfer (no fragmentation)
    REASSEMBLY_DROP,  // inbound fragmented cargo dropped: overflow, interleave

    /// SH-2 session.
    PROTOCOL,     // hub response violated the expected protocol sequence
    BUSY,         // a single-in-flight resource (command / FRS) is occupied
    QUEUE_FULL,   // TX cargo queue has no free slot
    TABLE_FULL,   // fixed-capacity table has no room for a new entry
    UNSUPPORTED,  // request not supported in this build/profile
    INVALID_ARGUMENT,  // caller passed an out-of-range or malformed argument
    HUB_RESET,         // the hub reset while the operation was in progress
};

/// A value or Sh2Error, never both; Result<void> carries no value.
/// No exceptions or heap; trivially copyable when T is. [[nodiscard]] makes an
/// ignored result a diagnostic, failing builds with -Werror.
template <typename T>
using Result = pico_util::Result<T, Sh2Error>;

}  // namespace pico_sh2
