// SPDX-License-Identifier: MIT
#include "pico_sh2/shtp/advertisement.hpp"

#include <cstddef>
#include <cstdint>
#include <span>

#include "pico_sh2/util/result.hpp"

namespace pico_sh2::shtp {
namespace {

// The GUID of the SHTP application itself (1000-3535 §5.2). Its tag 0x81 is a
// UART timeout, not report lengths, and its "control" channel is the command
// channel rather than SH-2's control channel.
constexpr std::uint32_t SHTP_GUID = 0;

// A ChannelName is NUL-terminated; the NUL is not part of its name.
// Accepting an unterminated name as its whole value is a library choice.
// 1000-3535 §5.2 requires ignoring unrecognized tags, not malformed names.
std::span<const std::uint8_t> trim_name(std::span<const std::uint8_t> value) {
    std::size_t length = 0;
    while (length < value.size() && value[length] != 0) {
        ++length;
    }
    return value.first(length);
}

// A GUID value is up to four bytes, LSB first. The LSB-first order is 1000-3535
// §5.2's rule for multi-byte values; the four-byte width is this library's
// limit, UNCONFIRMED (NR-4, FW-5).
constexpr std::size_t GUID_MAX_BYTES = sizeof(std::uint32_t);

// The caller has refused a value longer than GUID_MAX_BYTES.
std::uint32_t parse_guid(std::span<const std::uint8_t> value) {
    std::uint32_t guid = 0;
    for (std::size_t i = 0; i < value.size(); ++i) {
        guid |= static_cast<std::uint32_t>(value[i])
                << (8U * static_cast<unsigned>(i));
    }
    return guid;
}

// Emit each (report ID, length) pair. An odd value length would leave a
// dangling ID with no length, which is malformed.
Result<std::size_t> emit_report_lengths(std::span<const std::uint8_t> value,
                                        const ReportLengthSink& sink) {
    if ((value.size() & 1U) != 0) {
        return Sh2Error::PROTOCOL;
    }
    std::size_t pairs = 0;
    for (std::size_t i = 0; i + 1 < value.size(); i += 2) {
        sink(value[i], value[i + 1]);
        ++pairs;
    }
    return pairs;
}

}  // namespace

Result<std::size_t> parse_advertisement(std::span<const std::uint8_t> cargo,
                                        const ReportLengthSink& lengths,
                                        const ChannelSink& channels) {
    std::size_t pairs_applied = 0;
    std::size_t offset = 0;
    std::uint32_t guid = SHTP_GUID;
    // Tag 0x81 means "Report Lengths" only under the sensorhub; the SHTP
    // application uses it for a UART timeout and any other application for its
    // own purpose. Suppressed until the sensorhub's GUID opens scope, so a
    // stray pre-GUID 0x81 is never misread.
    bool in_sensorhub_scope = false;

    // A channel tag names the channel that follows it, so the emit is deferred
    // until the next tag is known to be (or not to be) its ChannelName.
    AdvertisedChannel pending;
    bool has_pending = false;
    auto flush_pending = [&]() {
        if (has_pending) {
            channels(pending);
            has_pending = false;
        }
    };

    while (offset < cargo.size()) {
        const std::uint8_t tag = cargo[offset];
        // A tag byte with no length byte after it is a truncated entry.
        if (offset + 1 >= cargo.size()) {
            return Sh2Error::PROTOCOL;
        }
        const std::size_t value_len = cargo[offset + 1];
        const std::size_t value_begin = offset + 2;
        // The value must lie entirely within the cargo.
        if (value_begin + value_len > cargo.size()) {
            return Sh2Error::PROTOCOL;
        }
        const std::span<const std::uint8_t> value =
            cargo.subspan(value_begin, value_len);

        if (tag == CHANNEL_NAME_TAG && has_pending) {
            pending.name = trim_name(value);
            flush_pending();
            offset = value_begin + value_len;
            continue;
        }

        // Anything else ends the previous channel's naming opportunity.
        flush_pending();

        if (tag == GUID_TAG) {
            // A longer GUID names an application this parse cannot tell from
            // another, and the roles bound to it would be guesses (SR-10).
            if (value_len > GUID_MAX_BYTES) {
                return Sh2Error::PROTOCOL;
            }
            guid = parse_guid(value);
            in_sensorhub_scope = value_len > 0 && guid == SENSORHUB_GUID;
        } else if ((tag == NORMAL_CHANNEL_TAG || tag == WAKE_CHANNEL_TAG) &&
                   value_len > 0) {
            // A channel tag with no value names no channel.
            pending = AdvertisedChannel{.guid = guid,
                                        .number = value[0],
                                        .wake = tag == WAKE_CHANNEL_TAG,
                                        .name = {}};
            has_pending = true;
        } else if (tag == REPORT_LENGTHS_TAG && in_sensorhub_scope) {
            const Result<std::size_t> emitted =
                emit_report_lengths(value, lengths);
            if (!emitted.ok()) {
                return emitted.error();
            }
            pairs_applied += emitted.value();
        }

        offset = value_begin + value_len;
    }

    // A channel advertised last in the cargo has nothing following it.
    flush_pending();
    return pairs_applied;
}

}  // namespace pico_sh2::shtp
