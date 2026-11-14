// SPDX-License-Identifier: MIT
// Cross-compiles and links the public surface without an SDK library;
// the footprint gate measures this image. It is not run (AR-4).
#include <algorithm>
#include <cstdint>
#include <span>

// The probe includes the consumer umbrella.
#include "pico_sh2/pico_sh2.hpp"  // IWYU pragma: keep
#include "pico_sh2/sh2/hal_transport.hpp"
#include "pico_sh2/sh2/session.hpp"
#include "pico_sh2/sh2/session_timing.hpp"
#include "pico_sh2/shtp/channels.hpp"
#include "pico_sh2/util/result.hpp"

namespace {

// final: the public non-virtual destructor is safe only while nothing derives
// from this class.
/// Idle reads must succeed (docs/usage-guide.md, Supplying a transport).
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
    // Static storage is explained in tests/firmware/README.md.
    // Function-local storage avoids a global variable.
    static NullTransport transport;
    static pico_sh2::Session session(transport, pico_sh2::SessionTiming{});
    session.begin();
    session.service();
    return 0;
}
