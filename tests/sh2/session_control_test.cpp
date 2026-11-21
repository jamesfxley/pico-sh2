// SPDX-License-Identifier: MIT
// Control requests, command correlation, queue errors and diagnostics
// (SR-2, BR-2, OR-2).
#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>
#include <initializer_list>
#include <limits>
#include <type_traits>
#include <vector>

#include "pico_sh2/reports/sensor_id.hpp"
#include "pico_sh2/sh2/command.hpp"
#include "pico_sh2/sh2/feature.hpp"
#include "pico_sh2/sh2/frs.hpp"
#include "pico_sh2/sh2/session.hpp"
#include "pico_sh2/sh2/session_timing.hpp"
#include "pico_sh2/sh2/startup_phase.hpp"
#include "pico_sh2/shtp/channels.hpp"
#include "pico_sh2/testing/fake_hub.hpp"
#include "pico_sh2/testing/frame_builder.hpp"
#include "pico_sh2/util/result.hpp"
#include "session_fixture.hpp"

namespace {

using pico_sh2::sh2_test::advance_to_wait_advertisement;
using pico_sh2::sh2_test::CommandCapture;
using pico_sh2::sh2_test::DiagObserver;
using pico_sh2::sh2_test::drive_to_ready;
using pico_sh2::sh2_test::FeatureCapture;
using pico_sh2::sh2_test::make_session;
using pico_sh2::sh2_test::test_timing;
using pico_sh2::testing::advertisement;
using pico_sh2::testing::command_response;
using pico_sh2::testing::control_channel;
using pico_sh2::testing::device_channel;
using pico_sh2::testing::FakeHub;
using pico_sh2::testing::feature_response;
using pico_sh2::testing::flush_completed;
using pico_sh2::testing::frame;
using pico_sh2::testing::initialize_response;
using pico_sh2::testing::reset_complete;

using pico_sh2::Session;
using pico_sh2::Sh2Error;

// Copying or moving would leave internal buffer views and callbacks
// pointing into the original session.
TEST(Session, IsNeitherCopyableNorMovable) {
    static_assert(!std::is_copy_constructible_v<Session>);
    static_assert(!std::is_copy_assignable_v<Session>);
    static_assert(!std::is_move_constructible_v<Session>);
    static_assert(!std::is_move_assignable_v<Session>);
    SUCCEED();
}

// Set Feature is queued on the control channel (SR-8).
TEST(Session, SetFeatureEmitsCargo) {
    FakeHub hub;
    Session session = make_session(hub);
    drive_to_ready(hub, session);

    pico_sh2::FeatureConfig config;
    config.sensor = pico_sh2::reports::SensorId::ROTATION_VECTOR;
    config.report_interval_us = 10'000;
    ASSERT_TRUE(session.set_feature(config).ok());
    session.service();  // flush the queue

    ASSERT_EQ(hub.writes().size(), 1U);
    const auto& sent = hub.writes()[0].frame;
    EXPECT_EQ(sent[2], 2);     // channel 2 (control)
    EXPECT_EQ(sent[4], 0xFD);  // Set Feature report ID
    EXPECT_EQ(sent[5], static_cast<std::uint8_t>(
                           pico_sh2::reports::SensorId::ROTATION_VECTOR));
    EXPECT_EQ(session.counters().cargoes_out, 1U);
}

// Get Feature sends 0xFE and the sensor ID on the control channel (SR-8).
TEST(Session, GetFeatureEmitsCargo) {
    FakeHub hub;
    Session session = make_session(hub);
    drive_to_ready(hub, session);

    ASSERT_TRUE(session.get_feature(pico_sh2::reports::SensorId::GRAVITY).ok());
    session.service();

    ASSERT_EQ(hub.writes().size(), 1U);
    const auto& sent = hub.writes()[0].frame;
    EXPECT_EQ(sent[2], 2);     // channel 2 (control)
    EXPECT_EQ(sent[4], 0xFE);  // Get Feature request
    EXPECT_EQ(sent[5],
              static_cast<std::uint8_t>(pico_sh2::reports::SensorId::GRAVITY));
}

// A short Get Feature Response is a protocol error and reaches no sink (SR-8).
TEST(Session, AShortGetFeatureResponseIsNotDelivered) {
    FakeHub hub;
    Session session = make_session(hub);
    FeatureCapture cap;
    session.set_feature_sink(
        pico_sh2::Session::FeatureSink::bind<FeatureCapture,
                                             &FeatureCapture::on_feature>(
            &cap));
    drive_to_ready(hub, session);

    std::vector<std::uint8_t> response = feature_response(0x05, 5'000);
    response.pop_back();
    hub.queue_transfer(frame(2, 1, response), 0);
    session.service();
    EXPECT_TRUE(cap.configs.empty());
    EXPECT_EQ(session.counters().feature_responses, 0U);
    EXPECT_EQ(session.counters().protocol_errors, 1U);
}

// Control requests return NOT_READY before the handshake completes (SR-8).
TEST(Session, RequestsBeforeReadyAreNotReady) {
    FakeHub hub;
    Session session = make_session(hub);
    const pico_sh2::FeatureConfig config;
    EXPECT_EQ(session.set_feature(config).error(), Sh2Error::NOT_READY);
    EXPECT_EQ(session.get_feature(pico_sh2::reports::SensorId::GRAVITY).error(),
              Sh2Error::NOT_READY);
    EXPECT_EQ(session.send_command(pico_sh2::command::COUNTER).error(),
              Sh2Error::NOT_READY);
}

// A Get Feature Response reaches the feature sink (SR-8).
TEST(Session, GetFeatureResponseRoutedToSink) {
    FakeHub hub;
    Session session = make_session(hub);
    FeatureCapture cap;
    session.set_feature_sink(
        pico_sh2::Session::FeatureSink::bind<FeatureCapture,
                                             &FeatureCapture::on_feature>(
            &cap));
    drive_to_ready(hub, session);

    hub.queue_transfer(frame(2, 1, feature_response(0x05, 5'000)), 0);
    session.service();
    ASSERT_EQ(cap.configs.size(), 1U);
    EXPECT_EQ(cap.configs[0].sensor,
              pico_sh2::reports::SensorId::ROTATION_VECTOR);
    EXPECT_EQ(cap.configs[0].report_interval_us, 5'000U);
    EXPECT_EQ(session.counters().feature_responses, 1U);
}

TEST(Session, CommandRoundTripCompletes) {
    FakeHub hub;
    Session session = make_session(hub);
    CommandCapture cap;
    session.set_command_sink(
        pico_sh2::Session::CommandSink::bind<CommandCapture,
                                             &CommandCapture::on_command>(
            &cap));
    drive_to_ready(hub, session);

    ASSERT_TRUE(session.send_command(pico_sh2::command::COUNTER).ok());
    EXPECT_TRUE(session.command_pending());
    session.service();  // flush the 0xF2 request

    ASSERT_EQ(hub.writes().size(), 1U);
    EXPECT_EQ(hub.writes()[0].frame[4], 0xF2);  // Command Request
    EXPECT_EQ(hub.writes()[0].frame[6], pico_sh2::command::COUNTER);

    // Response with matching command sequence (0) completes the command.
    hub.queue_transfer(
        frame(2, 1, command_response(pico_sh2::command::COUNTER, 0, false)), 0);
    session.service();
    EXPECT_FALSE(session.command_pending());
    EXPECT_EQ(session.counters().command_completions, 1U);
    ASSERT_EQ(cap.responses.size(), 1U);
    EXPECT_EQ(cap.responses[0].command, pico_sh2::command::COUNTER);
}

TEST(Session, OnlyOneCommandInFlight) {
    FakeHub hub;
    Session session = make_session(hub);
    drive_to_ready(hub, session);
    ASSERT_TRUE(session.send_command(pico_sh2::command::COUNTER).ok());
    EXPECT_EQ(session.send_command(pico_sh2::command::SAVE_DCD).error(),
              Sh2Error::BUSY);
}

TEST(Session, UnsolicitedResponseDoesNotCompleteCommand) {
    FakeHub hub;
    Session session = make_session(hub);
    CommandCapture cap;
    session.set_command_sink(
        pico_sh2::Session::CommandSink::bind<CommandCapture,
                                             &CommandCapture::on_command>(
            &cap));
    drive_to_ready(hub, session);
    ASSERT_TRUE(session.send_command(pico_sh2::command::COUNTER).ok());
    session.service();

    // An unsolicited response must not complete the pending command,
    // even with a matching sequence; it still reaches the sink (SR-2).
    hub.queue_transfer(
        frame(2, 1, command_response(pico_sh2::command::COUNTER, 0, true)), 0);
    session.service();
    EXPECT_TRUE(session.command_pending());
    EXPECT_EQ(session.counters().command_completions, 0U);
    ASSERT_EQ(cap.responses.size(), 1U);
}

// A response echoing another request sequence reaches the sink
// without completing the pending command (SR-2).
TEST(Session, AResponseToAnotherSequenceDoesNotCompleteTheCommand) {
    FakeHub hub;
    Session session = make_session(hub);
    CommandCapture cap;
    session.set_command_sink(
        pico_sh2::Session::CommandSink::bind<CommandCapture,
                                             &CommandCapture::on_command>(
            &cap));
    drive_to_ready(hub, session);
    ASSERT_TRUE(session.send_command(pico_sh2::command::COUNTER).ok());
    session.service();

    hub.queue_transfer(
        frame(2, 1, command_response(pico_sh2::command::COUNTER, 1, false)), 0);
    session.service();
    EXPECT_TRUE(session.command_pending());
    EXPECT_EQ(session.counters().command_completions, 0U);
    EXPECT_EQ(cap.responses.size(), 1U);
}

TEST(Session, CommandTimesOut) {
    FakeHub hub;
    Session session = make_session(hub);
    drive_to_ready(hub, session);
    ASSERT_TRUE(session.send_command(pico_sh2::command::COUNTER).ok());

    hub.advance_time(500'001);  // past command_timeout_us
    session.service();
    EXPECT_FALSE(session.command_pending());
    EXPECT_EQ(session.counters().command_timeouts, 1U);
    session.service();
    EXPECT_EQ(session.counters().command_timeouts, 1U);
}

// The deadline for the longest timeout saturates at the clock limit
// and does not expire on the next service() call (TR-1).
TEST(Session, TheLongestStartupTimeoutDoesNotWrap) {
    FakeHub hub;
    pico_sh2::SessionTiming timing = test_timing();
    timing.advertisement_timeout_us = std::numeric_limits<std::uint64_t>::max();
    Session session{hub, timing, pico_sh2::testing::seeded_channels()};
    advance_to_wait_advertisement(hub, session);
    hub.advance_time(1);
    session.service();
    EXPECT_EQ(session.startup_phase(),
              pico_sh2::StartupPhase::WAIT_ADVERTISEMENT);
    EXPECT_EQ(session.counters().startup_timeouts, 0U);
}

TEST(Session, TheLongestCommandTimeoutDoesNotWrap) {
    FakeHub hub;
    pico_sh2::SessionTiming timing = test_timing();
    timing.command_timeout_us = std::numeric_limits<std::uint64_t>::max();
    Session session{hub, timing, pico_sh2::testing::seeded_channels()};
    drive_to_ready(hub, session);
    ASSERT_TRUE(session.send_command(pico_sh2::command::COUNTER).ok());
    hub.advance_time(1);
    session.service();
    EXPECT_TRUE(session.command_pending());
    EXPECT_EQ(session.counters().command_timeouts, 0U);
}

TEST(Session, InvalidCommandRejected) {
    FakeHub hub;
    Session session = make_session(hub);
    drive_to_ready(hub, session);
    EXPECT_EQ(session.send_command(0).error(), Sh2Error::INVALID_ARGUMENT);
    EXPECT_EQ(session.send_command(128).error(), Sh2Error::INVALID_ARGUMENT);
}

TEST(Session, TxQueueOverflowSurfaced) {
    FakeHub hub;
    Session session = make_session(hub);
    drive_to_ready(hub, session);
    // Enqueue without servicing until QUEUE_FULL; the depth depends
    // on the profile (BR-2).
    bool saw_full = false;
    std::size_t accepted = 0;
    for (int i = 0; i < 1000; ++i) {
        auto result = session.get_feature(pico_sh2::reports::SensorId::GRAVITY);
        if (result.ok()) {
            ++accepted;
        } else {
            EXPECT_EQ(result.error(), Sh2Error::QUEUE_FULL);
            saw_full = true;
            break;
        }
    }
    EXPECT_TRUE(saw_full);
    EXPECT_GT(accepted, 0U);
}

TEST(Session, DiagnosticsHookReportsEvents) {
    FakeHub hub;
    Session session = make_session(hub);
    DiagObserver observer;
    session.set_diag_sink(
        pico_sh2::Session::DiagSink::bind<DiagObserver, &DiagObserver::on_diag>(
            &observer));

    // A startup deadline fires a STARTUP_FAULT event (OR-2).
    advance_to_wait_advertisement(hub, session);
    hub.advance_time(200'001);
    session.service();
    ASSERT_EQ(observer.events.size(), 1U);
    EXPECT_EQ(observer.events[0], pico_sh2::DiagEvent::STARTUP_FAULT);
}

TEST(Session, DiagnosticsHookReportsCommandTimeout) {
    FakeHub hub;
    Session session = make_session(hub);
    DiagObserver observer;
    session.set_diag_sink(
        pico_sh2::Session::DiagSink::bind<DiagObserver, &DiagObserver::on_diag>(
            &observer));
    drive_to_ready(hub, session);
    ASSERT_TRUE(session.send_command(pico_sh2::command::COUNTER).ok());
    hub.advance_time(500'001);
    session.service();
    ASSERT_EQ(observer.events.size(), 1U);
    EXPECT_EQ(observer.events[0], pico_sh2::DiagEvent::COMMAND_TIMEOUT);
}

// A hub reset while ready reaches the diagnostics hook (OR-2).
TEST(Session, DiagnosticsHookReportsAHubReset) {
    FakeHub hub;
    Session session = make_session(hub);
    DiagObserver observer;
    session.set_diag_sink(
        pico_sh2::Session::DiagSink::bind<DiagObserver, &DiagObserver::on_diag>(
            &observer));
    drive_to_ready(hub, session);
    hub.queue_transfer(frame(0, 1, pico_sh2::testing::advertisement()), 0);
    session.service();
    ASSERT_EQ(observer.events.size(), 1U);
    EXPECT_EQ(observer.events[0], pico_sh2::DiagEvent::HUB_RESET);
}

// An FRS timeout reaches the diagnostics hook (OR-2).
TEST(Session, DiagnosticsHookReportsAnFrsTimeout) {
    FakeHub hub;
    Session session = make_session(hub);
    DiagObserver observer;
    session.set_diag_sink(
        pico_sh2::Session::DiagSink::bind<DiagObserver, &DiagObserver::on_diag>(
            &observer));
    drive_to_ready(hub, session);
    ASSERT_TRUE(session.frs_read(pico_sh2::frs_type::SYSTEM_ORIENTATION).ok());
    session.service();
    hub.advance_time(500'001);
    session.service();
    ASSERT_EQ(observer.events.size(), 1U);
    EXPECT_EQ(observer.events[0], pico_sh2::DiagEvent::FRS_TIMEOUT);
}

// Force flush (1000-3625 §6.5.6 / §6.5.7).

struct FlushCapture {
    std::vector<pico_sh2::reports::SensorId> sensors;

    void on_flush(pico_sh2::reports::SensorId sensor) {
        sensors.push_back(sensor);
    }
};

// Force Sensor Flush is sent on the control channel (SR-6).
TEST(Session, ForceFlushEmitsTheRequest) {
    FakeHub hub;
    Session session = make_session(hub);
    drive_to_ready(hub, session);

    ASSERT_TRUE(
        session.force_flush(pico_sh2::reports::SensorId::ROTATION_VECTOR).ok());
    session.service();  // the frame is flushed on the next service()

    ASSERT_FALSE(hub.writes().empty());
    const auto& frame_bytes = hub.writes().back().frame;
    // Header, then the two-byte report: 0xF0 and the sensor ID (Figure 80).
    ASSERT_EQ(frame_bytes.size(), pico_sh2::shtp::HEADER_SIZE + 2);
    EXPECT_EQ(frame_bytes[2], control_channel());
    EXPECT_EQ(frame_bytes[pico_sh2::shtp::HEADER_SIZE], 0xF0);
    EXPECT_EQ(frame_bytes[pico_sh2::shtp::HEADER_SIZE + 1],
              static_cast<std::uint8_t>(
                  pico_sh2::reports::SensorId::ROTATION_VECTOR));
    EXPECT_EQ(session.counters().flushes_requested, 1U);
}

TEST(Session, ForceFlushBeforeReadyIsNotReady) {
    FakeHub hub;
    Session session = make_session(hub);
    auto result =
        session.force_flush(pico_sh2::reports::SensorId::ACCELEROMETER);
    EXPECT_FALSE(result.ok());
    EXPECT_EQ(result.error(), Sh2Error::NOT_READY);
}

// Flush Completed reaches the flush sink (SR-6).
TEST(Session, FlushCompletedReachesTheSink) {
    FakeHub hub;
    Session session = make_session(hub);
    FlushCapture cap;
    session.set_flush_sink(
        Session::FlushSink::bind<FlushCapture, &FlushCapture::on_flush>(&cap));
    drive_to_ready(hub, session);

    hub.queue_transfer(
        frame(2, 1,
              flush_completed(static_cast<std::uint8_t>(
                  pico_sh2::reports::SensorId::ROTATION_VECTOR))),
        0);
    session.service();

    ASSERT_EQ(cap.sensors.size(), 1U);
    EXPECT_EQ(cap.sensors[0], pico_sh2::reports::SensorId::ROTATION_VECTOR);
    EXPECT_EQ(session.counters().flushes_completed, 1U);
}

// The hub may flush other sensors too; every Flush Completed
// reaches the sink (1000-3625 §6.5.7; SR-6).
TEST(Session, EveryFlushCompletedIsDelivered) {
    FakeHub hub;
    Session session = make_session(hub);
    FlushCapture cap;
    session.set_flush_sink(
        Session::FlushSink::bind<FlushCapture, &FlushCapture::on_flush>(&cap));
    drive_to_ready(hub, session);

    hub.queue_transfer(frame(2, 1, flush_completed(0x05)), 0);
    hub.queue_transfer(frame(2, 2, flush_completed(0x01)), 0);
    session.service();

    ASSERT_EQ(cap.sensors.size(), 2U);
    EXPECT_EQ(cap.sensors[1], pico_sh2::reports::SensorId::ACCELEROMETER);
    EXPECT_EQ(session.counters().flushes_completed, 2U);
}

// Concatenate reports into one control cargo.
std::vector<std::uint8_t> cargo_of(
    std::initializer_list<std::vector<std::uint8_t>> reports) {
    std::vector<std::uint8_t> cargo;
    for (const std::vector<std::uint8_t>& report : reports) {
        cargo.insert(cargo.end(), report.begin(), report.end());
    }
    return cargo;
}

// Each report in a control cargo is split at its table length
// and routed (1000-3600 §2.3; NR-4).
TEST(Session, EveryReportInAControlCargoIsRouted) {
    FakeHub hub;
    Session session = make_session(hub);
    FlushCapture cap;
    session.set_flush_sink(
        Session::FlushSink::bind<FlushCapture, &FlushCapture::on_flush>(&cap));
    FeatureCapture features;
    session.set_feature_sink(
        Session::FeatureSink::bind<FeatureCapture, &FeatureCapture::on_feature>(
            &features));
    drive_to_ready(hub, session);

    hub.queue_transfer(
        frame(2, 1,
              cargo_of({flush_completed(0x05), feature_response(0x01, 5'000),
                        flush_completed(0x01)})),
        0);
    session.service();

    ASSERT_EQ(cap.sensors.size(), 2U);
    EXPECT_EQ(cap.sensors[1], pico_sh2::reports::SensorId::ACCELEROMETER);
    ASSERT_EQ(features.configs.size(), 1U);
    EXPECT_EQ(features.configs[0].report_interval_us, 5'000U);
    EXPECT_EQ(session.counters().unrouted_cargoes, 0U);
}

// An unknown report length ends the walk and is counted;
// earlier reports remain delivered (NR-4).
TEST(Session, AnUnsizedReportEndsTheControlCargo) {
    FakeHub hub;
    Session session = make_session(hub);
    FlushCapture cap;
    session.set_flush_sink(
        Session::FlushSink::bind<FlushCapture, &FlushCapture::on_flush>(&cap));
    drive_to_ready(hub, session);

    hub.queue_transfer(frame(2, 1,
                             cargo_of({flush_completed(0x05),
                                       {0x77, 0x00},
                                       flush_completed(0x01)})),
                       0);
    session.service();

    ASSERT_EQ(cap.sensors.size(), 1U);
    EXPECT_EQ(session.counters().unrouted_cargoes, 1U);
}

// A report after Initialize in the same cargo is routed as ready (NR-4, SR-1).
TEST(Session, AReportBatchedAfterTheInitializeResponseIsRouted) {
    FakeHub hub;
    Session session = make_session(hub);
    FlushCapture cap;
    session.set_flush_sink(
        Session::FlushSink::bind<FlushCapture, &FlushCapture::on_flush>(&cap));
    advance_to_wait_advertisement(hub, session);

    hub.queue_transfer(frame(0, 0, advertisement()), 0);
    hub.queue_transfer(frame(1, 0, reset_complete()), 0);
    hub.queue_transfer(
        frame(2, 0, cargo_of({initialize_response(0), flush_completed(0x05)})),
        0);
    session.service();

    EXPECT_TRUE(session.ready());
    ASSERT_EQ(cap.sensors.size(), 1U);
    EXPECT_EQ(session.counters().unrouted_cargoes, 0U);
}

// A flush sink that restarts the session it was called from.
struct FlushRestarter {
    Session* session = nullptr;
    int calls = 0;

    void on_flush(pico_sh2::reports::SensorId /*sensor*/) {
        ++calls;
        session->begin();
    }
};

// A sink calling begin() ends delivery from its control cargo (NR-4).
TEST(Session, BeginFromASinkEndsTheControlCargo) {
    FakeHub hub;
    Session session = make_session(hub);
    FlushRestarter restarter{.session = &session, .calls = 0};
    session.set_flush_sink(
        Session::FlushSink::bind<FlushRestarter, &FlushRestarter::on_flush>(
            &restarter));
    drive_to_ready(hub, session);

    hub.queue_transfer(
        frame(2, 1, cargo_of({flush_completed(0x05), flush_completed(0x01)})),
        0);
    session.service();

    EXPECT_EQ(restarter.calls, 1);
    EXPECT_EQ(session.counters().flushes_completed, 1U);
    EXPECT_EQ(session.counters().unrouted_cargoes, 0U);
}

// A failed Initialize ends delivery from its cargo (NR-4, SR-1).
TEST(Session, AFailedInitializeEndsTheControlCargo) {
    FakeHub hub;
    Session session = make_session(hub);
    advance_to_wait_advertisement(hub, session);

    hub.queue_transfer(frame(0, 0, advertisement()), 0);
    hub.queue_transfer(frame(1, 0, reset_complete()), 0);
    hub.queue_transfer(
        frame(2, 0, cargo_of({initialize_response(1), {0x77, 0x00}})), 0);
    session.service();

    EXPECT_EQ(session.state(), pico_sh2::SessionState::FAULTED);
    EXPECT_EQ(session.counters().unrouted_cargoes, 0U);
}

// Transfers later in the same pass must not advance startup
// past a fault and hide it from the consumer (SR-1).
TEST(Session, AFaultEndsTheServicePass) {
    FakeHub hub;
    Session session = make_session(hub);
    advance_to_wait_advertisement(hub, session);

    hub.queue_transfer(frame(0, 0, advertisement()), 0);
    hub.queue_transfer(frame(1, 0, reset_complete()), 0);
    hub.queue_transfer(frame(2, 0, initialize_response(1)), 0);
    hub.queue_transfer(frame(2, 1, initialize_response(0)), 0);
    session.service();

    EXPECT_EQ(session.state(), pico_sh2::SessionState::FAULTED);
}

// A Flush Completed without a sensor ID reaches no sink (SR-6).
TEST(Session, AShortFlushCompletedIsAProtocolError) {
    FakeHub hub;
    Session session = make_session(hub);
    FlushCapture cap;
    session.set_flush_sink(
        Session::FlushSink::bind<FlushCapture, &FlushCapture::on_flush>(&cap));
    drive_to_ready(hub, session);

    // The sensor ID is missing.
    hub.queue_transfer(frame(2, 1, {0xEF}), 0);
    session.service();

    EXPECT_TRUE(cap.sensors.empty());
    EXPECT_EQ(session.counters().protocol_errors, 1U);
    EXPECT_EQ(session.counters().flushes_completed, 0U);
}

// Construction aborts when the caller supplies a mismatched profile binding
// (docs/architecture.md, Allocation).
TEST(Session, MismatchedProfileBindingAborts) {
    FakeHub hub;
    pico_sh2::SessionBinding wrong = Session::caller_binding();
    wrong.max_cargo_len += 1;
    EXPECT_DEATH(
        {
            const Session session(hub, test_timing(),
                                  pico_sh2::shtp::ChannelMap{}, wrong);
        },
        "");
}

// A profile may change layout beyond its knobs; size and alignment mismatches
// still abort (docs/architecture.md, Allocation).
TEST(Session, ProfileBindingDetectsLayoutDrift) {
    FakeHub hub;
    pico_sh2::SessionBinding wrong = Session::caller_binding();
    wrong.session_size += 1;
    EXPECT_DEATH(
        {
            const Session session(hub, test_timing(),
                                  pico_sh2::shtp::ChannelMap{}, wrong);
        },
        "");

    pico_sh2::SessionBinding misaligned = Session::caller_binding();
    misaligned.session_align *= 2;
    EXPECT_DEATH(
        {
            const Session session(hub, test_timing(),
                                  pico_sh2::shtp::ChannelMap{}, misaligned);
        },
        "");
}

// MAX_ENABLED_SENSORS sizes no Session member, so size alone cannot
// detect its mismatch (docs/architecture.md, Allocation).
TEST(Session, ProfileBindingCatchesAKnobThatSizesNothingHere) {
    FakeHub hub;
    pico_sh2::SessionBinding wrong = Session::caller_binding();
    wrong.max_enabled_sensors += 1;
    EXPECT_EQ(wrong.session_size, Session::caller_binding().session_size);
    EXPECT_DEATH(
        {
            const Session session(hub, test_timing(),
                                  pico_sh2::shtp::ChannelMap{}, wrong);
        },
        "");
}

TEST(Session, MatchingProfileBindingConstructs) {
    FakeHub hub;
    const Session session(hub, test_timing(), pico_sh2::shtp::ChannelMap{},
                          Session::caller_binding());
    EXPECT_EQ(session.state(), pico_sh2::SessionState::IDLE);
}

// More than COMMAND_MAX_PARAMS returns INVALID_ARGUMENT and sends nothing
// (SR-2).
TEST(Session, SendCommandRefusesMoreParametersThanARequestCarries) {
    FakeHub hub;
    Session session = make_session(hub);
    drive_to_ready(hub, session);
    const std::size_t writes_before = hub.writes().size();

    const std::vector<std::uint8_t> too_many(pico_sh2::COMMAND_MAX_PARAMS + 1,
                                             0x01);
    EXPECT_EQ(session.send_command(pico_sh2::command::ME_CALIBRATION, too_many)
                  .error(),
              Sh2Error::INVALID_ARGUMENT);
    EXPECT_FALSE(session.command_pending());
    session.service();
    EXPECT_EQ(hub.writes().size(), writes_before);

    const std::vector<std::uint8_t> exactly(pico_sh2::COMMAND_MAX_PARAMS, 0x01);
    EXPECT_TRUE(
        session.send_command(pico_sh2::command::ME_CALIBRATION, exactly).ok());
}

// A transport write error is counted and reported; the frame
// stays queued for retry on the next pass (OR-1, OR-2).
TEST(Session, ARefusedWriteIsCountedReportedAndRetried) {
    FakeHub hub;
    Session session = make_session(hub);
    DiagObserver observer;
    session.set_diag_sink(
        pico_sh2::Session::DiagSink::bind<DiagObserver, &DiagObserver::on_diag>(
            &observer));
    drive_to_ready(hub, session);
    const std::size_t writes_before = hub.writes().size();

    ASSERT_TRUE(
        session.get_feature(pico_sh2::reports::SensorId::ACCELEROMETER).ok());
    hub.fail_next_write(Sh2Error::BUS_ERROR);
    session.service();
    EXPECT_EQ(session.counters().write_errors, 1U);
    ASSERT_EQ(observer.events.size(), 1U);
    EXPECT_EQ(observer.events[0], pico_sh2::DiagEvent::WRITE_ERROR);
    EXPECT_EQ(hub.writes().size(), writes_before);

    session.service();
    EXPECT_EQ(session.counters().write_errors, 1U);
    EXPECT_EQ(hub.writes().size(), writes_before + 1);
}

// A refusal must not consume the number of the next cargo (NR-7).
TEST(Session, ARefusedWriteKeepsTheNextSequenceNumber) {
    FakeHub hub;
    Session session = make_session(hub);
    drive_to_ready(hub, session);
    const std::size_t writes_before = hub.writes().size();

    ASSERT_TRUE(
        session.get_feature(pico_sh2::reports::SensorId::ACCELEROMETER).ok());
    session.service();
    ASSERT_EQ(hub.writes().size(), writes_before + 1);
    const std::uint8_t previous = hub.writes().at(writes_before).frame.at(3);

    ASSERT_TRUE(
        session.get_feature(pico_sh2::reports::SensorId::ACCELEROMETER).ok());
    hub.fail_next_write(Sh2Error::BUS_ERROR);
    session.service();
    ASSERT_EQ(hub.writes().size(), writes_before + 1);
    session.service();
    ASSERT_EQ(hub.writes().size(), writes_before + 2);

    EXPECT_EQ(hub.writes().at(writes_before + 1).frame.at(3),
              static_cast<std::uint8_t>(previous + 1))
        << "a refused write consumed a second sequence number";
}

// A held frame surviving an out-of-phase advertisement is reframed (NR-7).
TEST(Session, AnOutOfPhaseAdvertisementReframesTheHeldCargo) {
    FakeHub hub;
    Session session = make_session(hub);
    advance_to_wait_advertisement(hub, session);

    // The first advertisement moves startup past WAIT_ADVERTISEMENT.
    hub.queue_transfer(frame(0, 0, advertisement()), 0);
    session.service();
    ASSERT_EQ(session.startup_phase(),
              pico_sh2::StartupPhase::WAIT_RESET_COMPLETE);

    // One accepted device write leaves the engine's number at one.
    ASSERT_TRUE(session.request_reset().ok());
    session.service();
    const std::size_t writes_before = hub.writes().size();

    // The next device cargo is held with that number after a refusal.
    ASSERT_TRUE(session.request_reset().ok());
    hub.fail_next_write(Sh2Error::BUS_ERROR);
    session.service();
    ASSERT_EQ(hub.writes().size(), writes_before);

    // A second advertisement arrives out of phase and resets SHTP framing.
    hub.queue_transfer(frame(0, 0, advertisement()), 0);
    session.service();

    ASSERT_EQ(hub.writes().size(), writes_before + 1);
    const auto& written = hub.writes().back().frame;
    EXPECT_EQ(written.at(2), device_channel());
    EXPECT_EQ(written.at(3), 0);
}

}  // namespace
