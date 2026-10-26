// SPDX-License-Identifier: MIT
// AR-4's cross-compiled probe: the public surface a consumer uses, compiled and
// linked for the target with no SDK library behind it.
//
// Nothing runs this. It exists so the linker has to resolve everything the
// umbrella header pulls in, which is what turns "it compiles" into "it also
// links freestanding". A header that reached for a hosted facility -- an
// exception path, <iostream>, a heap-assuming container -- or for an SDK or OS
// symbol fails here rather than in the first firmware consumer's build. It is
// also the image the footprint gate measures.
#include <algorithm>
#include <cstdint>
#include <span>

// The umbrella is what this probes.
#include "pico_sh2/pico_sh2.hpp"  // IWYU pragma: keep
#include "pico_sh2/sh2/hal_transport.hpp"
#include "pico_sh2/sh2/session.hpp"
#include "pico_sh2/sh2/session_timing.hpp"
#include "pico_sh2/shtp/channels.hpp"
#include "pico_sh2/util/result.hpp"

namespace {

/// The seam, implemented as trivially as the interface allows. read_transfer
/// reports an idle bus as a zeroed header, length 0 -- a transfer with no
/// cargo -- which is what an idle read must return: an error there wedges
/// Session::drain_transfers().
class NullTransport final : public pico_sh2::IHalTransport {
   public:
    bool data_ready() override { return false; }

    pico_sh2::Result<pico_sh2::ReceivedTransfer> read_transfer(
        std::span<std::uint8_t> dst) override {
        if (dst.size() < pico_sh2::shtp::HEADER_SIZE) {
            return pico_sh2::Sh2Error::INVALID_ARGUMENT;
        }
        std::fill_n(dst.begin(), pico_sh2::shtp::HEADER_SIZE, std::uint8_t{0});
        return pico_sh2::ReceivedTransfer{.length =
                                              pico_sh2::shtp::HEADER_SIZE};
    }

    pico_sh2::Result<void> write_transfer(
        std::span<const std::uint8_t> /*frame*/) override {
        return pico_sh2::Result<void>{};
    }

    std::uint64_t now_us() override { return 0; }

    void set_reset(bool /*asserted*/) override {}
};

}  // namespace

int main() {
    // Static storage, not automatic: the sizing profile's reassembly buffer and
    // TX queue are Session members, so on the stack they never reach .bss and
    // the footprint gate would measure newlib alone. Function-local so this is
    // still not a global variable.
    static NullTransport transport;
    static pico_sh2::Session session(transport, pico_sh2::SessionTiming{});
    session.begin();
    session.service();
    return 0;
}
