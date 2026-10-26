// SPDX-License-Identifier: MIT
// Compiles the umbrella header as a translation unit so the full public surface
// is exercised by the analysis toolchain, and pins the advertised version to
// the values the build system carries.
//
// This file deliberately includes *only* the umbrella: it is the one place that
// proves a consumer can drive the library from that single include, so a public
// header dropped from it is a build failure here rather than a surprise
// downstream. The umbrella marks its includes `IWYU pragma: export`, so a
// symbol reached through it counts as provided here, as it does for a consumer.
#include "pico_sh2/pico_sh2.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <span>

#include "pico_sh2/sh2/startup_phase.hpp"

namespace {

// A transport that never has data and never fails: enough to construct and pump
// a session without the testing library, which is not part of the public
// surface.
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

    void advance(std::uint64_t delta_us) { now_us_ += delta_us; }

    [[nodiscard]] bool reset_asserted() const { return reset_asserted_; }

   private:
    std::uint64_t now_us_ = 0;
    bool reset_asserted_ = false;
};

// The advertised version and project(pico_sh2 VERSION ...) are two records of
// one fact, compared here against the value CMake holds rather than against a
// literal copied out of the header -- which would only ever compare the header
// with itself.
TEST(PicoSh2, VersionMatchesTheBuildSystem) {
    EXPECT_EQ(pico_sh2::VERSION_MAJOR, PICO_SH2_CMAKE_VERSION_MAJOR);
    EXPECT_EQ(pico_sh2::VERSION_MINOR, PICO_SH2_CMAKE_VERSION_MINOR);
    EXPECT_EQ(pico_sh2::VERSION_PATCH, PICO_SH2_CMAKE_VERSION_PATCH);
}

// The umbrella alone must be enough to build and drive a session: construct it,
// start it, pump it, and reach the public request surface and counters.
TEST(PicoSh2, UmbrellaHeaderAloneCanDriveASession) {
    SilentTransport transport;
    // Timing is the consumer's to supply: the library ships no defaults,
    // because a reset hold and a boot delay describe a part rather than the
    // protocol. These stand in for a datasheet's numbers.
    constexpr std::uint64_t RESET_HOLD_US = 2'000;
    const pico_sh2::SessionTiming timing{
        .reset_hold_us = RESET_HOLD_US,
        .boot_delay_us = 100'000,
        .advertisement_timeout_us = 200'000,
        .reset_complete_timeout_us = 200'000,
        .initialize_timeout_us = 200'000,
        .command_timeout_us = 500'000,
        .frs_timeout_us = 500'000,
    };
    pico_sh2::Session session(transport, timing);

    EXPECT_EQ(session.state(), pico_sh2::SessionState::IDLE);
    session.begin();
    EXPECT_TRUE(transport.reset_asserted());
    EXPECT_EQ(session.state(), pico_sh2::SessionState::STARTING);
    EXPECT_EQ(session.startup_phase(), pico_sh2::StartupPhase::RESET_ASSERTED);

    transport.advance(RESET_HOLD_US);
    session.service();
    EXPECT_FALSE(transport.reset_asserted());

    // Requests are reachable and correctly refused before the handshake ends.
    EXPECT_EQ(session.set_feature(pico_sh2::FeatureConfig{}).error(),
              pico_sh2::Sh2Error::NOT_READY);
    EXPECT_EQ(session.send_command(pico_sh2::command::COUNTER).error(),
              pico_sh2::Sh2Error::NOT_READY);
    EXPECT_EQ(session.frs_read(pico_sh2::frs_type::SYSTEM_ORIENTATION).error(),
              pico_sh2::Sh2Error::NOT_READY);
    EXPECT_FALSE(session.command_pending());
    EXPECT_FALSE(session.frs_busy());
    EXPECT_EQ(session.counters().cargoes_in, 0U);

    // The report-length seed and the Q-override surface come through too.
    EXPECT_EQ(session.report_lengths()
                  .length_of(static_cast<std::uint8_t>(
                      pico_sh2::reports::SensorId::ROTATION_VECTOR))
                  .value_or(0),
              14);
    EXPECT_TRUE(
        session.set_report_q(0x05, pico_sh2::reports::ReportQ{14, 12}).ok());

    // The headers a consumer reaches for a field of every decoded event, a
    // flush and the channel binding: each one the umbrella must export, so
    // the include check fails here if one is dropped from it.
    EXPECT_NE(pico_sh2::reports::SensorAccuracy::HIGH,
              pico_sh2::reports::SensorAccuracy::LOW);
    EXPECT_EQ(pico_sh2::FLUSH_COMPLETED_REPORT, 0xEF);
    EXPECT_EQ(pico_sh2::EXECUTABLE_GUID, 1U);
}

}  // namespace
