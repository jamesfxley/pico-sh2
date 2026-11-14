// SPDX-License-Identifier: MIT
#pragma once

/// \file advertisement.hpp
/// Channel-0 advertisement parser for tag-length-value (TLV) entries.
/// Tag and length are one byte each; length counts value bytes (1000-3535
/// §5.2). Tag 0x81 means report lengths only under the sensorhub GUID
/// (1000-3600 §2.3); under SHTP it means UART timeout (1000-3535 §5.3). Pairs
/// reach a sink because SH-2 report semantics belong to reports/.

#include <cstddef>
#include <cstdint>
#include <span>
#include <string_view>

#include "pico_sh2/util/result.hpp"
#include "pico_util/result/callback.hpp"

namespace pico_sh2::shtp {

/// Global advertisement tag introducing an application; its value is the
/// application's GUID. GUID 0 is the SHTP application (1000-3535 §5.2).
inline constexpr std::uint8_t GUID_TAG = 0x01;

/// Sensorhub report-length pairs; application-scoped tag (1000-3600 §2.3).
inline constexpr std::uint8_t REPORT_LENGTHS_TAG = 0x81;

/// Sensorhub GUID, scoping application-specific tags (1000-3535 §5.2).
inline constexpr std::uint32_t SENSORHUB_GUID = 2;

/// Advisory NUL-terminated application name; the GUID identifies the
/// application.
inline constexpr std::uint8_t APP_NAME_TAG = 0x08;

/// Normal channel-number tag (1000-3535 §5.2).
inline constexpr std::uint8_t NORMAL_CHANNEL_TAG = 0x06;
/// Assigns a channel that wakes the host.
inline constexpr std::uint8_t WAKE_CHANNEL_TAG = 0x07;
/// NUL-terminated name for the immediately preceding channel tag.
inline constexpr std::uint8_t CHANNEL_NAME_TAG = 0x09;

/// Receives each advertised (report ID, on-wire length) pair in wire order.
using ReportLengthSink = pico_util::Callback<void(std::uint8_t, std::uint8_t)>;

/// Advertised channel and borrowed name bytes; name_is() compares without a
/// cast.
struct AdvertisedChannel {
    std::uint32_t guid = 0;   ///< GUID of the application that owns it.
    std::uint8_t number = 0;  ///< Its wire channel number.
    bool wake = false;        ///< Whether traffic on it wakes the host.
    /// Its advertised name with any trailing NUL removed; empty when unnamed.
    /// Points into the caller's cargo and does not outlive the parse call.
    std::span<const std::uint8_t> name;
};

/// True when an advertised channel name equals `expected`.
constexpr bool name_is(std::span<const std::uint8_t> name,
                       std::string_view expected) {
    if (name.size() != expected.size()) {
        return false;
    }
    for (std::size_t i = 0; i < name.size(); ++i) {
        if (static_cast<char>(name[i]) != expected[i]) {
            return false;
        }
    }
    return true;
}

/// Receives each advertised channel in wire order.
using ChannelSink = pico_util::Callback<void(const AdvertisedChannel&)>;

/// Parse a Response-0 body, emitting sensorhub lengths and channels (1000-3535
/// §5.1). Either sink may be unset; return the pair count or PROTOCOL for
/// malformed TLV. Channel emission waits for its optional following name
/// (§5.2). Entries can reach sinks before a later error; a caller treating it
/// as fatal must discard what it applied.
Result<std::size_t> parse_advertisement(std::span<const std::uint8_t> cargo,
                                        const ReportLengthSink& lengths,
                                        const ChannelSink& channels = {});

}  // namespace pico_sh2::shtp
