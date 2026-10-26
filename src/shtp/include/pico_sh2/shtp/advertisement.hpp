// SPDX-License-Identifier: MIT
#pragma once

/// \file advertisement.hpp
/// Channel-0 advertisement parser. After reset the hub sends, on channel 0, a
/// list of TLV entries describing itself: GUIDs, channel assignments, app and
/// channel names, a version, and — the field the decoder depends on — tag 0x81
/// "Report Lengths", an array of (report ID, length) byte pairs.
///
/// Each TLV entry is: tag (1 byte), length (1 byte), value (`length` bytes).
/// The walk is fully bounds-checked: a length that runs past the end of the
/// cargo is a malformed advertisement and is rejected rather than read out of
/// bounds.
///
/// Tags 0x80–0xFF are application-specific and scoped to the nearest preceding
/// GUID tag (1000-3535 §5.2). GUID 0 is the SHTP application itself, whose
/// tag 0x81 is "SHTP over UART Timeout" (1000-3535 §5.3) — a different meaning
/// from the SH-2 sensorhub's tag 0x81 "Report Lengths" (1000-3600 §2.3). The
/// parser therefore tracks GUID scope and applies 0x81 as report lengths only
/// under the sensorhub (SENSORHUB_GUID); under any other application it is that
/// application's own tag. Every other tag is skipped by its length, so
/// unknown/vendor tags never break the parse.
///
/// Channel assignments are also carried here, as a NormalChannel (tag 6) or
/// WakeChannel (tag 7) giving the number, optionally followed immediately by a
/// ChannelName (tag 9) naming it (1000-3535 §5.2). They are emitted with the
/// GUID that scopes them, because the name alone does not identify a channel:
/// the SHTP application and the SH-2 sensorhub both name one "control".
///
/// The pairs are emitted through a sink rather than written into a
/// report-length table: this layer owns the advertisement's wire format, not
/// the meaning of an SH-2 report ID, which belongs to reports/.

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

/// SHTP advertisement tag carrying the (report ID, length) pairs. Application-
/// scoped: the pairs only under the sensorhub application (SENSORHUB_GUID); the
/// SHTP application uses 0x81 for its UART timeout, and any other application
/// for whatever it defines.
inline constexpr std::uint8_t REPORT_LENGTHS_TAG = 0x81;

/// The sensorhub application's GUID, the one scope in which REPORT_LENGTHS_TAG
/// is the report-length list: tags 0x80-0xFF are each application's own
/// (1000-3535 §5.2).
inline constexpr std::uint32_t SENSORHUB_GUID = 2;

/// Names the application introduced by the preceding GUID; a null-terminated
/// string, and advisory — the GUID identifies the application, not the name.
inline constexpr std::uint8_t APP_NAME_TAG = 0x08;

/// Channel-assignment tags (1000-3535 §5.2). The value of either is the channel
/// number; a wake channel wakes the host, a normal one does not.
/// Assigns a channel that does not wake the host.
inline constexpr std::uint8_t NORMAL_CHANNEL_TAG = 0x06;
/// Assigns a channel that wakes the host.
inline constexpr std::uint8_t WAKE_CHANNEL_TAG = 0x07;
/// Names the channel introduced by the immediately preceding channel tag. The
/// value is a null-terminated string.
inline constexpr std::uint8_t CHANNEL_NAME_TAG = 0x09;

/// Receives each advertised (report ID, on-wire length) pair in wire order.
using ReportLengthSink = pico_util::Callback<void(std::uint8_t, std::uint8_t)>;

/// One advertised channel: the GUID of the application that owns it, its wire
/// number, whether it is a wake channel, and its name with any trailing NUL
/// removed. `name` is empty when the hub advertised no ChannelName for it, and
/// points into the caller's cargo, so it does not outlive the parse call.
///
/// The name stays bytes rather than becoming a string_view: this layer reads
/// the wire, and viewing those bytes as characters needs a cast that buys
/// nothing here. `name_is` below does the comparison a caller actually wants.
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

/// Walk the advertisement `cargo` -- the body of a channel-0 Response 0, after
/// the SHTP header and the response byte (1000-3535 §5.1) -- emitting every
/// sensorhub-scoped tag-0x81 (report ID, length) pair to `lengths` and every
/// advertised channel to `channels`. Either sink may be left unset. Returns the
/// number of pairs emitted, or PROTOCOL if any TLV entry is truncated.
///
/// A channel is emitted once its name is known — that is, on the following tag,
/// or at the end of the cargo — because 1000-3535 §5.2 puts the ChannelName
/// after the channel tag it names. A truncated entry is rejected after earlier
/// pairs and channels have already reached their sinks, so a caller that treats
/// a malformed advertisement as fatal must discard what it applied.
Result<std::size_t> parse_advertisement(std::span<const std::uint8_t> cargo,
                                        const ReportLengthSink& lengths,
                                        const ChannelSink& channels = {});

}  // namespace pico_sh2::shtp
