// SPDX-License-Identifier: MIT
// Consumer-side view of the installed package. Nothing here reaches into the
// source tree: every symbol below must arrive through the installed
// pico_sh2::core target, resolved by find_package, so a header the package
// fails to ship is a compile error in this test rather than a discovery made by
// the first downstream project.
//
// pico_sh2::core is an INTERFACE aggregate over four component targets. Its
// export is the thing most likely to break silently: a component dropped from
// the export set still links in-tree, because the in-tree alias resolves
// regardless, and fails only once a consumer resolves it through the installed
// config file.
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <span>

#include "pico_sh2/pico_sh2.hpp"
// The hub emulator the package ships for downstream tests; pico-bno085's driver
// tests link it. Included so a pico_sh2::testing that left the export set, or
// whose headers did not install, is a compile error here rather than a
// downstream discovery.
#include "pico_sh2/testing/fake_hub.hpp"

static_assert(pico_sh2::VERSION_MAJOR == 0 && pico_sh2::VERSION_MINOR == 1);

// Consumer-side view of the sizing profile. Nothing here defines a profile
// macro; PICO_SH2_ACTIVE_PROFILE must arrive through the installed target's
// compile definitions. The declared consumer cases in house.toml pin the
// profile they install and name it with EXPECT_DEFAULT, EXPECT_LARGE or
// EXPECT_CUSTOM, so a profile lost at the install boundary is a compile error
// here rather than a silently default-sized consumer. With none of them the
// consumer takes whatever profile the build selected (a CI leg may select
// large), and asserts only that it compiles and runs against it.
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

// Enough of a transport to construct and pump a session without reaching for
// the shipped emulator: a consumer's own transport is the case this stands in
// for, and FakeHub is exercised separately below.
//
// final: the base declares its destructor protected and non-virtual on purpose
// (a transport is owned by whoever supplies it, never deleted through this
// pointer). A derived class with virtual functions and a public non-virtual
// destructor reopens exactly that hazard unless it cannot be derived from.
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

    // The shipped emulator, from the installed package alone. Constructing it
    // is what proves the archive linked, not just that the header was found.
    pico_sh2::testing::FakeHub hub;
    hub.queue_startup();
    if (!hub.data_ready()) {
        std::puts("the shipped hub emulator did not come through the package");
        return 1;
    }

    SilentTransport transport;
    // Timing is the consumer's to supply: the library ships no defaults,
    // because a reset hold and a boot delay describe a part rather than the
    // protocol. These stand in for a datasheet's numbers.
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

    // The report-length seed reaches a consumer through the aggregate too --
    // it comes from pico_sh2::reports, a different component target than the
    // session, so this fails if that component left the export set.
    if (session.report_lengths()
            .length_of(static_cast<std::uint8_t>(
                pico_sh2::reports::SensorId::ROTATION_VECTOR))
            .value_or(0) != 14) {
        std::puts("the report-length table did not come through the package");
        return 1;
    }
    return 0;
}
