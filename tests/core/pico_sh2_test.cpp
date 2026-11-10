// SPDX-License-Identifier: MIT
// Checks umbrella inclusion and compares version constants with the build.
// Missing umbrella exports fail here rather than in downstream builds. The
// umbrella marks its includes `IWYU pragma: export`, so a
// symbol reached through it counts as provided here, as it does for a consumer.
#include "pico_sh2/pico_sh2.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <span>

#include "pico_sh2/sh2/startup_phase.hpp"

namespace {

// Constructs and pumps sessions without the testing library.
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

    void advance(std::uint64_t delta_us) { now_us_ += delta_us; }

    [[nodiscard]] bool reset_asserted() const { return reset_asserted_; }

   private:
    std::uint64_t now_us_ = 0;
    bool reset_asserted_ = false;
};

// CMake supplies the independent version value, exposing header drift.
TEST(PicoSh2, VersionMatchesTheBuildSystem) {
    EXPECT_EQ(pico_sh2::VERSION_MAJOR, PICO_SH2_CMAKE_VERSION_MAJOR);
    EXPECT_EQ(pico_sh2::VERSION_MINOR, PICO_SH2_CMAKE_VERSION_MINOR);
    EXPECT_EQ(pico_sh2::VERSION_PATCH, PICO_SH2_CMAKE_VERSION_PATCH);
}

// The umbrella must expose session construction, startup, requests and
// counters.
TEST(PicoSh2, UmbrellaHeaderAloneCanDriveASession) {
    SilentTransport transport;
    // Host timing follows tests/sh2/session_fixture.hpp.
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

    // Requests return NOT_READY before the handshake completes.
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

    // The umbrella must also export event fields, flush and channel binding.
    EXPECT_NE(pico_sh2::reports::SensorAccuracy::HIGH,
              pico_sh2::reports::SensorAccuracy::LOW);
    EXPECT_EQ(pico_sh2::FLUSH_COMPLETED_REPORT, 0xEF);
    EXPECT_EQ(pico_sh2::EXECUTABLE_GUID, 1U);
}

}  // namespace
