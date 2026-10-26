// SPDX-License-Identifier: MIT
#pragma once

/// \file hal_transport.hpp
/// The one seam between platform-independent SH-2/SHTP logic and the hardware.
/// A consumer implements this over whatever bus its part uses. Every method
/// is called only from Session::service()/begin(); the library never touches
/// the bus from any other context, so an implementation's interrupt handler may
/// do nothing but latch the HINTN flag and timestamp (see ReceivedTransfer).
///
/// AR-4: this interface is the whole of the library's platform dependency —
/// time and bus I/O reach it through nothing else, and no Pico SDK or OS header
/// is included anywhere beneath it. The bare-metal cross build compiles the
/// tree with no SDK and no OS, which is what holds that true.

#include <cstddef>
#include <cstdint>
#include <span>

#include "pico_sh2/util/result.hpp"

namespace pico_sh2 {

/// One transfer read from the hub, and the HINTN edge it answers.
struct ReceivedTransfer {
    /// Bytes written to the read's buffer, header included.
    std::size_t length = 0;
    /// Monotonic microsecond time of the HINTN falling edge this read answers.
    /// The SH-2 base-timestamp (0xFB) delta is relative to it, so it MUST be
    /// latched in the HINTN interrupt, not sampled when the read runs: bus
    /// arbitration would otherwise inject millisecond-scale error into every
    /// timestamp.
    ///
    /// It comes back with the read, not from a query before it, because only
    /// the transport knows which edge the read consumed. data_ready() may see
    /// the line asserted before the interrupt has latched the edge behind it,
    /// and a time taken then is the previous edge's. A read served with no
    /// fresh edge behind it reports the most recent earlier edge.
    std::uint64_t intn_edge_us = 0;
};

/// The bus beneath the session: whether a transfer is pending, how to move
/// bytes in either direction and when the edge behind a read arrived, the
/// clock, and the reset line. One implementation per bus.
class IHalTransport {
   public:
    IHalTransport() = default;

    /// An interface is never copied: assigning one implementation to another
    /// through this type would slice it. The destructor below is protected and
    /// non-virtual on purpose — a transport is owned by the consumer that
    /// supplies it and is never deleted through this pointer.
    IHalTransport(const IHalTransport&) = delete;
    IHalTransport& operator=(const IHalTransport&) = delete;
    IHalTransport(IHalTransport&&) = delete;
    IHalTransport& operator=(IHalTransport&&) = delete;

    /// True when a transfer is available: the hub has asserted HINTN, or the
    /// adapter has retained an inbound transfer from a duplex exchange.
    /// Reading consumes that pending transfer; returning a retained transfer
    /// requires no new bus activity.
    virtual bool data_ready() = 0;

    /// Read exactly one SHTP transfer (4-byte header + payload) into dst,
    /// bounded by dst.size(). Returns the number of bytes written (>= header
    /// size) and the HINTN edge the read answers, or an error. Reading fewer
    /// payload bytes than the cargo holds is legal: the hub re-offers the
    /// remainder as a continuation transfer.
    ///
    /// `dst` is borrowed for the duration of the call: copy anything that must
    /// outlive it. The session reuses one staging buffer for every read, so a
    /// transport that retained `dst.data()` — the natural shape for a DMA
    /// completion that finishes after the call returns — would be writing into
    /// a buffer the session has already decoded and handed on.
    virtual Result<ReceivedTransfer> read_transfer(
        std::span<std::uint8_t> dst) = 0;

    /// Write one complete transfer (header included). The frame always fits a
    /// single transfer by this library's policy (NR-2); a hub may refuse
    /// fragments. Implementations own any wake handshake (SPI: assert WAKE,
    /// await HINTN, assert CSN, clock the bytes).
    ///
    /// `frame` is borrowed for the duration of the call: copy anything that
    /// must outlive it. It points into the one framing scratch buffer the
    /// transmit path reuses for every queued cargo, so a retained pointer is
    /// overwritten by the very next cargo in the same flush loop.
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
