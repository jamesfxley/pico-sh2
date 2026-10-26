// SPDX-License-Identifier: MIT
#pragma once

// Shared setup for the session suites: the captures the sinks deliver into and
// the two helpers that drive a session to a known point.
//
// The session tests are split by area (startup, reset recovery, input, control
// transactions, FRS, heap), and every one of them needs to get a session to
// READY first. Keeping that in one place means a change to the handshake is a
// change to one function rather than to six suites.

#include <gtest/gtest.h>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <string>
#include <vector>

#include "pico_sh2/reports/sensor_event.hpp"
#include "pico_sh2/sh2/command.hpp"
#include "pico_sh2/sh2/feature.hpp"
#include "pico_sh2/sh2/session.hpp"
#include "pico_sh2/sh2/session_timing.hpp"
#include "pico_sh2/sh2/startup_phase.hpp"
#include "pico_sh2/testing/fake_hub.hpp"
#include "pico_sh2/testing/frame_builder.hpp"
#include "pico_sh2/util/result.hpp"

namespace pico_sh2::sh2_test {

// Only what this header's own body uses. The rest of pico_sh2::testing is
// named directly by the suite that wants it, so a helper moving out of that
// namespace is a compile error where it is used rather than a silent
// re-resolution through a re-export here.
using pico_sh2::testing::FakeHub;
using pico_sh2::testing::seeded_channels;

struct EventCapture {
    std::vector<reports::SensorEvent> events;

    void on_event(const reports::SensorEvent& event) {
        events.push_back(event);
    }
};

struct ResetObserver {
    int count = 0;

    void on_reset() { ++count; }
};

struct FeatureCapture {
    std::vector<FeatureConfig> configs;

    void on_feature(const FeatureConfig& config) { configs.push_back(config); }
};

struct CommandCapture {
    std::vector<CommandResponse> responses;

    void on_command(const CommandResponse& response) {
        responses.push_back(response);
    }
};

struct DiagObserver {
    std::vector<DiagEvent> events;

    void on_diag(DiagEvent diag_event) { events.push_back(diag_event); }
};

struct FrsCapture {
    Sh2Error status = Sh2Error::NONE;
    std::uint16_t type = 0;
    std::vector<std::uint32_t> words;
    std::optional<std::uint8_t> device_status;
    int calls = 0;

    void on_frs(Sh2Error status_in, std::uint16_t type_in,
                std::span<const std::uint32_t> words_in,
                std::optional<std::uint8_t> device_status_in) {
        status = status_in;
        type = type_in;
        words.assign(words_in.begin(), words_in.end());
        device_status = device_status_in;
        ++calls;
    }
};

// Timing for the host suites. The library ships no defaults — a reset hold and
// a boot delay are properties of a part, not of the protocol — so every test
// supplies its own. These only have to be non-zero and consistent with how the
// helpers below advance time.
inline constexpr std::uint64_t TEST_RESET_HOLD_US = 2'000;
inline constexpr std::uint64_t TEST_BOOT_DELAY_US = 100'000;
inline constexpr std::uint64_t TEST_PHASE_TIMEOUT_US = 200'000;
inline constexpr std::uint64_t TEST_TRANSACTION_TIMEOUT_US = 500'000;

constexpr SessionTiming test_timing() {
    return {
        .reset_hold_us = TEST_RESET_HOLD_US,
        .boot_delay_us = TEST_BOOT_DELAY_US,
        .advertisement_timeout_us = TEST_PHASE_TIMEOUT_US,
        .reset_complete_timeout_us = TEST_PHASE_TIMEOUT_US,
        .initialize_timeout_us = TEST_PHASE_TIMEOUT_US,
        .command_timeout_us = TEST_TRANSACTION_TIMEOUT_US,
        .frs_timeout_us = TEST_TRANSACTION_TIMEOUT_US,
    };
}

// A session wired to `hub` with the test timing and a channel map seeded for a
// hub advertising at the default base, which is what a consumer supplies.
inline Session make_session(FakeHub& hub, std::uint8_t base = 1) {
    return {hub, test_timing(), seeded_channels(base)};
}

// Drive the timed reset/boot transitions so the session is waiting for the
// advertisement.
inline void advance_to_wait_advertisement(FakeHub& hub, Session& session) {
    session.begin();
    EXPECT_EQ(session.startup_phase(), StartupPhase::RESET_ASSERTED);
    hub.advance_time(TEST_RESET_HOLD_US);
    session.service();
    EXPECT_EQ(session.startup_phase(), StartupPhase::BOOTING);
    hub.advance_time(TEST_BOOT_DELAY_US);
    session.service();
    EXPECT_EQ(session.startup_phase(), StartupPhase::WAIT_ADVERTISEMENT);
}

// Complete the full startup handshake so the session is READY.
inline void drive_to_ready(FakeHub& hub, Session& session) {
    advance_to_wait_advertisement(hub, session);
    hub.queue_startup();
    session.service();
    ASSERT_EQ(session.state(), SessionState::READY);
}

// A counter as for_each_counter reports it.
struct Visited {
    std::string name;
    std::string description;
    std::uint32_t value;
};

inline std::vector<Visited> visit_all(const SessionCounters& counters) {
    std::vector<Visited> seen;
    pico_sh2::for_each_counter(
        counters, [&seen](const char* name, const char* description,
                          std::uint32_t value) {
            seen.push_back({name, description, value});
        });
    return seen;
}

// Names of the counters whose value differs between two snapshots, sorted so a
// case can compare against a literal set.
inline std::vector<std::string> moved(const SessionCounters& before,
                                      const SessionCounters& after) {
    const std::vector<Visited> was = visit_all(before);
    const std::vector<Visited> now = visit_all(after);
    std::vector<std::string> names;
    for (std::size_t i = 0; i < now.size(); ++i) {
        if (now[i].value != was[i].value) {
            names.push_back(now[i].name);
        }
    }
    std::ranges::sort(names);
    return names;
}

}  // namespace pico_sh2::sh2_test
