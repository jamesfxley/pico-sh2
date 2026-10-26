// SPDX-License-Identifier: MIT
#include "pico_sh2/testing/fake_hub.hpp"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <span>
#include <vector>

#include "pico_sh2/sh2/hal_transport.hpp"
#include "pico_sh2/shtp/channels.hpp"
#include "pico_sh2/testing/frame_builder.hpp"
#include "pico_sh2/util/result.hpp"

namespace pico_sh2::testing {

void FakeHub::queue_idle_transfer(std::uint64_t intn_timestamp_us) {
    // A zero-length header: four bytes, all zero. parse_header() accepts it as
    // a valid null transfer (length 0), which is the empty keep-alive a hub
    // emits when polled with nothing pending.
    const std::vector<std::uint8_t> null_header(shtp::HEADER_SIZE, 0);
    queue_transfer(null_header, intn_timestamp_us);
}

void FakeHub::queue_startup(std::uint64_t intn_timestamp_us,
                            std::uint8_t sequence) {
    queue_transfer(frame(shtp::COMMAND_CHANNEL, sequence, advertisement()),
                   intn_timestamp_us);
    queue_transfer(frame(device_channel(), sequence, reset_complete()),
                   intn_timestamp_us);
    queue_transfer(frame(control_channel(), sequence, initialize_response()),
                   intn_timestamp_us);
}

void FakeHub::queue_transfer(std::span<const std::uint8_t> frame,
                             std::uint64_t intn_timestamp_us) {
    inbound_.push_back(ScriptedTransfer{
        .frame = std::vector<std::uint8_t>(frame.begin(), frame.end()),
        .intn_timestamp_us = intn_timestamp_us});
}

Result<ReceivedTransfer> FakeHub::read_transfer(std::span<std::uint8_t> dst) {
    if (inbound_.empty()) {
        // An absent hub, reported the way the real transports report it: a
        // zeroed header and success. Returning an error would fault an
        // otherwise healthy session, and permanently so during startup.
        if (dst.size() < shtp::HEADER_SIZE) {
            return Sh2Error::BUS_ERROR;
        }
        std::fill_n(dst.begin(), shtp::HEADER_SIZE, std::uint8_t{0});
        return ReceivedTransfer{.length = shtp::HEADER_SIZE,
                                .intn_edge_us = last_edge_us_};
    }
    if (next_read_error_ != Sh2Error::NONE) {
        if (reads_until_error_ > 0) {
            --reads_until_error_;
        } else {
            const Sh2Error error = next_read_error_;
            next_read_error_ = Sh2Error::NONE;
            return error;
        }
    }
    const ScriptedTransfer& transfer = inbound_.front();
    const std::size_t n = std::min(dst.size(), transfer.frame.size());
    std::copy_n(transfer.frame.begin(), n, dst.begin());
    last_edge_us_ = transfer.intn_timestamp_us;
    const ReceivedTransfer received{.length = n, .intn_edge_us = last_edge_us_};
    // A short read leaves the remainder to be re-offered as a continuation; the
    // full-frame case (the common one) consumes the transfer.
    if (n == transfer.frame.size()) {
        inbound_.pop_front();
    }
    return received;
}

Result<void> FakeHub::write_transfer(std::span<const std::uint8_t> frame) {
    if (next_write_error_ != Sh2Error::NONE) {
        const Sh2Error error = next_write_error_;
        next_write_error_ = Sh2Error::NONE;
        return error;
    }
    writes_.push_back(
        CapturedWrite{std::vector<std::uint8_t>(frame.begin(), frame.end())});
    return Result<void>{};
}

}  // namespace pico_sh2::testing
