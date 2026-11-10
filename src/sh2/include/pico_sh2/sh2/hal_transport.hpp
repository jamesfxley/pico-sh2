// SPDX-License-Identifier: MIT
#pragma once

/// \file hal_transport.hpp
/// Hardware boundary for SH-2/SHTP; no SDK or OS dependency (AR-4).
/// Bus transactions run in begin()/service(); requests can read now_us().
/// An interrupt handler needs only to latch the HINTN flag and timestamp.

#include <cstddef>
#include <cstdint>
#include <span>

#include "pico_sh2/util/result.hpp"

namespace pico_sh2 {

/// One transfer read from the hub, and the HINTN edge it answers.
struct ReceivedTransfer {
    /// Bytes written to the read's buffer, header included.
    std::size_t length = 0;
    /// HINTN falling-edge time in monotonic microseconds; MUST be latched in
    /// the interrupt, since read-time sampling adds scheduling and bus delays.
    /// Return the edge consumed with the read; data_ready() can precede edge
    /// capture. Without a fresh edge, return the most recent earlier edge.
    std::uint64_t intn_edge_us = 0;
};

/// Synchronous bus, clock and reset interface implemented by the consumer.
class IHalTransport {
   public:
    IHalTransport() = default;

    /// Consumer-owned; copying slices implementations, and deletion through
    /// this interface is unavailable.
    IHalTransport(const IHalTransport&) = delete;
    IHalTransport& operator=(const IHalTransport&) = delete;
    IHalTransport(IHalTransport&&) = delete;
    IHalTransport& operator=(IHalTransport&&) = delete;

    /// True when a transfer is available: the hub has asserted HINTN, or the
    /// adapter has retained an inbound transfer from a duplex exchange.
    /// Reading consumes that pending transfer; returning a retained transfer
    /// requires no new bus activity.
    virtual bool data_ready() = 0;

    /// Read one transfer, bounded by dst.size(); success returns at least four
    /// bytes and the consumed edge. A partial payload is legal: the hub
    /// re-offers the rest as continuation. dst is borrowed only for this call;
    /// copy anything that must outlive it.
    virtual Result<ReceivedTransfer> read_transfer(
        std::span<std::uint8_t> dst) = 0;

    /// Write one complete unfragmented transfer, including its header (NR-2).
    /// A hub may reject fragments; the implementation owns any wake
    /// handshake. frame is borrowed only for this call; copy anything that must
    /// outlive it.
    virtual Result<void> write_transfer(
        std::span<const std::uint8_t> frame) = 0;

    /// Monotonic microsecond clock used for command/startup/FRS deadlines.
    virtual std::uint64_t now_us() = 0;

    /// Drive the hub's NRST line. The session sequences assert -> settle ->
    /// release and then waits for the advertisement.
    virtual void set_reset(bool asserted) = 0;

   protected:
    ~IHalTransport() = default;
};

}  // namespace pico_sh2
