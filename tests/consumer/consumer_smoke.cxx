// SPDX-License-Identifier: MIT
// Installed headers and symbols must reach this consumer through find_package
// and its imported targets. In-tree aliases cannot exercise missing exports.
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <span>

#include "pico_sh2/pico_sh2.hpp"
// Construction below checks both the installed testing header and its archive.
#include "pico_sh2/testing/fake_hub.hpp"

static_assert(pico_sh2::VERSION_MAJOR == 0 && pico_sh2::VERSION_MINOR == 1);

// PICO_SH2_ACTIVE_PROFILE must propagate through installed compile definitions.
// EXPECT_DEFAULT, EXPECT_LARGE and EXPECT_CUSTOM select assertions.
// Without one, this accepts any profile, including large.
using Profile = PICO_SH2_ACTIVE_PROFILE;
#if defined(EXPECT_LARGE)
static_assert(Profile::MAX_CARGO_LEN == 1024,
              "the large profile did not reach this translation unit");
static_assert(Profile::MAX_ENABLED_SENSORS == 16,
              "the large profile did not reach this translation unit");
#elif defined(EXPECT_CUSTOM)
static_assert(Profile::MAX_CARGO_LEN == 512,
              "the consumer's profile header did not reach this translation "
              "unit");
#elif defined(EXPECT_DEFAULT)
static_assert(Profile::MAX_CARGO_LEN == 384,
              "the default profile did not reach this translation unit");
#endif

namespace {

// A consumer-owned transport exercises construction without FakeHub.
// final: the public non-virtual destructor is safe only while nothing derives
// from this class.
class SilentTransport final : public pico_sh2::IHalTransport {
   public:
    bool data_ready() override { return false; }

    pico_sh2::Result<pico_sh2::ReceivedTransfer> read_transfer(
        std::span<std::uint8_t> /*dst*/) override {
        return pico_sh2::ReceivedTransfer{.intn_edge_us = now_us_};
    }

    pico_sh2::Result<void> write_transfer(
        std::span<const std::uint8_t> /*frame*/) override {
        return pico_sh2::Result<void>{};
    }

    std::uint64_t now_us() override { return now_us_; }

    void set_reset(bool asserted) override { reset_asserted_ = asserted; }

    [[nodiscard]] bool reset_asserted() const { return reset_asserted_; }

   private:
    std::uint64_t now_us_ = 0;
    bool reset_asserted_ = false;
};

}  // namespace

int main() {
    std::printf("pico_sh2 %d.%d.%d\n", pico_sh2::VERSION_MAJOR,
                pico_sh2::VERSION_MINOR, pico_sh2::VERSION_PATCH);

    // Construction requires the installed archive as well as the header.
    pico_sh2::testing::FakeHub hub;
    hub.queue_startup();
    if (!hub.data_ready()) {
        std::puts("the shipped hub emulator did not come through the package");
        return 1;
    }

    SilentTransport transport;
    // Host timing follows tests/sh2/session_fixture.hpp.
    const pico_sh2::SessionTiming timing{
        .reset_hold_us = 2'000,
        .boot_delay_us = 100'000,
        .advertisement_timeout_us = 200'000,
        .reset_complete_timeout_us = 200'000,
        .initialize_timeout_us = 200'000,
        .command_timeout_us = 500'000,
        .frs_timeout_us = 500'000,
    };
    pico_sh2::Session session(transport, timing);

    if (session.state() != pico_sh2::SessionState::IDLE) {
        std::puts("a fresh session was not IDLE");
        return 1;
    }

    session.begin();
    if (!transport.reset_asserted()) {
        std::puts("begin() did not assert reset through the transport seam");
        return 1;
    }

    // The length seed also needs the installed reports component.
    if (session.report_lengths()
            .length_of(static_cast<std::uint8_t>(
                pico_sh2::reports::SensorId::ROTATION_VECTOR))
            .value_or(0) != 14) {
        std::puts("the report-length table did not come through the package");
        return 1;
    }
    return 0;
}
