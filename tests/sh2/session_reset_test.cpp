// SPDX-License-Identifier: MIT
// Hub reset recovery and the state it preserves or ends (SR-5).
#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "pico_sh2/reports/decoders.hpp"
#include "pico_sh2/reports/sensor_event.hpp"
#include "pico_sh2/reports/sensor_id.hpp"
#include "pico_sh2/sh2/command.hpp"
#include "pico_sh2/sh2/frs.hpp"
#include "pico_sh2/sh2/profile.hpp"
#include "pico_sh2/sh2/session.hpp"
#include "pico_sh2/sh2/startup_phase.hpp"
#include "pico_sh2/shtp/channels.hpp"
#include "pico_sh2/testing/fake_hub.hpp"
#include "pico_sh2/testing/frame_builder.hpp"
#include "pico_sh2/util/result.hpp"
#include "session_fixture.hpp"

namespace {

using pico_sh2::sh2_test::advance_to_wait_advertisement;
using pico_sh2::sh2_test::CommandCapture;
using pico_sh2::sh2_test::drive_to_ready;
using pico_sh2::sh2_test::make_session;
using pico_sh2::sh2_test::moved;
using pico_sh2::sh2_test::ResetObserver;
using pico_sh2::sh2_test::TEST_PHASE_TIMEOUT_US;
using pico_sh2::sh2_test::test_timing;
using pico_sh2::testing::advertisement;
using pico_sh2::testing::control_channel;
using pico_sh2::testing::device_channel;
using pico_sh2::testing::FakeHub;
using pico_sh2::testing::frame;
using pico_sh2::testing::initialize_response;
using pico_sh2::testing::reset_complete;

using pico_sh2::Session;
using pico_sh2::SessionState;
using pico_sh2::Sh2Error;
using pico_sh2::StartupPhase;

TEST(Session, UnexpectedAdvertisementRestartsHandshake) {
    FakeHub hub;
    Session session = make_session(hub);
    ResetObserver observer;
    session.set_reset_sink(pico_sh2::Session::ResetSink::bind<
                           ResetObserver, &ResetObserver::on_reset>(&observer));
    advance_to_wait_advertisement(hub, session);
    hub.queue_transfer(frame(0, 0, advertisement()), 0);
    hub.queue_transfer(frame(1, 0, {0x01}), 0);
    hub.queue_transfer(frame(2, 0, initialize_response(0)), 0);
    session.service();
    ASSERT_EQ(session.state(), SessionState::READY);

    // The hub resets: it re-advertises. The session restarts its handshake and
    // notifies the consumer (SR-5).
    hub.queue_transfer(frame(0, 1, advertisement()), 0);
    session.service();
    EXPECT_EQ(session.startup_phase(), StartupPhase::WAIT_RESET_COMPLETE);
    EXPECT_EQ(session.counters().resets, 1U);
    EXPECT_EQ(observer.count, 1);

    // And it can complete the handshake again.
    hub.queue_transfer(frame(1, 1, {0x01}), 0);
    hub.queue_transfer(frame(2, 1, initialize_response(0)), 0);
    session.service();
    EXPECT_EQ(session.state(), SessionState::READY);
}

// An Error List leaves a ready session ready; only Response 0
// is an advertisement (1000-3535 §5.1, §5.1.2.1; SR-5, NR-4).
TEST(Session, AnErrorListWhileReadyIsNotAReset) {
    FakeHub hub;
    Session session = make_session(hub);
    ResetObserver observer;
    session.set_reset_sink(
        Session::ResetSink::bind<ResetObserver, &ResetObserver::on_reset>(
            &observer));
    drive_to_ready(hub, session);

    // Error 9: host wrote to an unrecognised channel.
    hub.queue_transfer(frame(0, 1, {0x01, 0x09}), 0);
    session.service();
    EXPECT_TRUE(session.ready());
    EXPECT_EQ(observer.count, 0);
    EXPECT_EQ(session.counters().resets, 0U);
    EXPECT_EQ(session.counters().protocol_errors, 0U);
}

// Parsing starts after the response byte (NR-4). From byte 0, the parser
// consumes the GUID tag as a value, then expects 0x81 bytes this fixture lacks.
TEST(Session, AnAdvertisementIsReadAfterItsResponseByte) {
    FakeHub hub;
    Session session = make_session(hub);
    drive_to_ready(hub, session);

    hub.queue_transfer(
        frame(0, 1, {0x00, 0x01, 0x01, 0x02, 0x81, 0x02, 0x05, 0x0E}), 0);
    session.service();
    EXPECT_EQ(session.report_lengths().length_of(0x05).value_or(0), 0x0E);
    EXPECT_EQ(session.counters().protocol_errors, 0U);
}

// Response 0 without a body does not reset a ready session (NR-4, SR-5).
TEST(Session, ABodilessResponseZeroWhileReadyIsNotAReset) {
    FakeHub hub;
    Session session = make_session(hub);
    drive_to_ready(hub, session);

    hub.queue_transfer(frame(0, 1, {0x00}), 0);
    session.service();
    EXPECT_TRUE(session.ready());
    EXPECT_EQ(session.counters().resets, 0U);
    EXPECT_EQ(session.counters().protocol_errors, 1U);
}

// Reserved responses 2-255 are protocol errors, without a reset
// (1000-3535 §5.1.3).
TEST(Session, AReservedChannelZeroResponseIsAProtocolError) {
    FakeHub hub;
    Session session = make_session(hub);
    drive_to_ready(hub, session);

    hub.queue_transfer(frame(0, 1, {0x02, 0x00}), 0);
    session.service();
    EXPECT_TRUE(session.ready());
    EXPECT_EQ(session.counters().resets, 0U);
    EXPECT_EQ(session.counters().protocol_errors, 1U);
}

TEST(Session, EmptyChannelZeroCargoWhileReadyIsNotAReset) {
    FakeHub hub;
    Session session = make_session(hub);
    drive_to_ready(hub, session);
    // An empty channel-0 cargo must not be mistaken for a re-advertisement and
    // trigger a spurious hub-reset recovery (SR-5).
    hub.queue_transfer(frame(0, 1, {}), 0);
    session.service();
    EXPECT_TRUE(session.ready());
    EXPECT_EQ(session.counters().resets, 0U);
    EXPECT_EQ(session.counters().protocol_errors, 1U);
}

TEST(Session, BusFaultDuringDrainDoesNotDoubleFault) {
    FakeHub hub;
    Session session = make_session(hub);
    advance_to_wait_advertisement(hub, session);
    // A phase deadline must not overwrite the read error that faulted
    // the session in this drain pass.
    hub.queue_transfer(frame(0, 0, advertisement()), 0);
    hub.fail_next_read(Sh2Error::BUS_ERROR);
    hub.advance_time(10'000'000);  // push well past the advertisement deadline
    session.service();
    EXPECT_TRUE(session.faulted());
    EXPECT_EQ(session.fault(), Sh2Error::BUS_ERROR);
    EXPECT_EQ(session.startup_phase(), StartupPhase::WAIT_ADVERTISEMENT);
    EXPECT_EQ(session.counters().startup_timeouts, 0U);
}

TEST(Session, TransportErrorWhileReadyRestartsTheHandshake) {
    FakeHub hub;
    Session session = make_session(hub);
    ResetObserver observer;
    session.set_reset_sink(pico_sh2::Session::ResetSink::bind<
                           ResetObserver, &ResetObserver::on_reset>(&observer));
    drive_to_ready(hub, session);
    // A read error while READY restarts the handshake and notifies
    // the consumer without faulting (SR-5).
    hub.queue_transfer(frame(3, 0, {}), 0);
    hub.fail_next_read(Sh2Error::BUS_ERROR);
    session.service();
    EXPECT_FALSE(session.faulted());
    EXPECT_EQ(session.startup_phase(), StartupPhase::WAIT_ADVERTISEMENT);
    EXPECT_EQ(session.counters().transport_restarts, 1U);
    EXPECT_EQ(session.counters().resets, 1U);
    EXPECT_EQ(observer.count, 1);

    // The hub returns and the handshake completes from the re-advertisement.
    hub.queue_transfer(frame(0, 0, advertisement()), 0);
    hub.queue_transfer(frame(1, 0, {0x01}), 0);
    hub.queue_transfer(frame(2, 0, initialize_response(0)), 0);
    session.service();
    EXPECT_EQ(session.state(), SessionState::READY);
}

// Earlier cargoes must not wait for a clean pass to be counted, even when
// a transport error ends the pass (OR-1).
TEST(Session, MirroredCountersSurviveATransportError) {
    FakeHub hub;
    Session session = make_session(hub);
    drive_to_ready(hub, session);
    const pico_sh2::SessionCounters before = session.counters();

    // One good cargo is delivered, then the next read fails in the same pass.
    hub.queue_transfer(frame(2, 1, {0x00}), 0);
    hub.queue_transfer(frame(3, 0, {}), 0);
    hub.fail_read_after(1, Sh2Error::BUS_ERROR);
    session.service();

    // Only cargoes_in, resets, transport_restarts and unrouted_cargoes move.
    EXPECT_EQ(
        moved(before, session.counters()),
        (std::vector<std::string>{"cargoes_in", "resets", "transport_restarts",
                                  "unrouted_cargoes"}));
    EXPECT_EQ(session.counters().cargoes_in, before.cargoes_in + 1);
    EXPECT_EQ(session.counters().transport_restarts, 1U);
}

// begin() preserves counters; only construction clears them (OR-1).
// No transfer is read during reset hold and boot, so nothing should move.
TEST(Session, CountersSurviveBegin) {
    FakeHub hub;
    Session session = make_session(hub);
    drive_to_ready(hub, session);
    // Both timeout counters are made non-zero so clearing them is observable.
    ASSERT_TRUE(session.send_command(pico_sh2::command::COUNTER).ok());
    ASSERT_TRUE(session.frs_read(pico_sh2::frs_type::SYSTEM_ORIENTATION).ok());
    hub.advance_time(pico_sh2::sh2_test::TEST_TRANSACTION_TIMEOUT_US + 1);
    session.service();
    ASSERT_GT(session.counters().command_timeouts, 0U);
    ASSERT_GT(session.counters().frs_timeouts, 0U);
    std::vector<std::uint32_t> before;
    pico_sh2::for_each_counter(
        session.counters(),
        [&before](const char*, const char*, std::uint32_t value) {
            before.push_back(value);
        });
    ASSERT_GT(session.counters().cargoes_in, 0U);

    // A draining pass precedes comparison: reset and boot passes
    // do not mirror the engine counts, so an engine reset is hidden until then.
    advance_to_wait_advertisement(hub, session);
    session.service();

    std::vector<std::uint32_t> after;
    pico_sh2::for_each_counter(
        session.counters(),
        [&after](const char*, const char*, std::uint32_t value) {
            after.push_back(value);
        });
    EXPECT_EQ(after, before);
}

TEST(Session, TransportThatNeverReturnsTimesOutInsteadOfRestartingForever) {
    FakeHub hub;
    Session session = make_session(hub);
    drive_to_ready(hub, session);
    // Without re-advertisement, recovery ends TIMEOUT in WAIT_ADVERTISEMENT
    // rather than retaining the original bus error (SR-5).
    hub.queue_transfer(frame(3, 0, {}), 0);
    hub.fail_next_read(Sh2Error::BUS_ERROR);
    session.service();
    ASSERT_EQ(session.startup_phase(), StartupPhase::WAIT_ADVERTISEMENT);

    hub.advance_time(200'001);
    session.service();
    EXPECT_TRUE(session.faulted());
    EXPECT_EQ(session.fault(), Sh2Error::TIMEOUT);
    EXPECT_EQ(session.startup_phase(), StartupPhase::WAIT_ADVERTISEMENT);
    EXPECT_EQ(session.counters().startup_timeouts, 1U);
    EXPECT_EQ(session.counters().transport_restarts, 1U);
}

// A service() pass drains at most the profile's transfer bound (AR-3).
TEST(Session, DrainIsBoundedPerServiceCall) {
    FakeHub hub;
    Session session = make_session(hub);
    advance_to_wait_advertisement(hub, session);
    // More idle transfers than one pass can drain.
    constexpr std::size_t FLOOD = 500;
    for (std::size_t i = 0; i < FLOOD; ++i) {
        hub.queue_transfer(frame(0, static_cast<std::uint8_t>(i), {}), 0);
    }
    session.service();
    // Comparing with the profile bound catches an overlong pass; the flood size
    // would accept a pass leaving just one transfer (AR-3).
    EXPECT_LE(session.counters().protocol_errors,
              pico_sh2::ActiveProfile::MAX_TRANSFERS_PER_SERVICE);
    std::uint32_t guard = 0;
    while (session.counters().protocol_errors < FLOOD && guard < 100) {
        session.service();
        ++guard;
    }
    EXPECT_EQ(session.counters().protocol_errors, FLOOD);
}

TEST(Session, BeginDropsCargoesQueuedBeforeIt) {
    FakeHub hub;
    Session session = make_session(hub);
    drive_to_ready(hub, session);

    // Queue a request but do not service, so it is still staged.
    ASSERT_TRUE(
        session.get_feature(pico_sh2::reports::SensorId::ACCELEROMETER).ok());
    const std::size_t writes_before = hub.writes().size();

    // Restarting must discard the staged request before the new handshake.
    session.begin();
    drive_to_ready(hub, session);
    session.service();
    EXPECT_EQ(hub.writes().size(), writes_before);
}

TEST(Session, BeginClearsAnOutstandingCommand) {
    FakeHub hub;
    Session session = make_session(hub);
    drive_to_ready(hub, session);
    ASSERT_TRUE(session.send_command(pico_sh2::command::COUNTER).ok());
    EXPECT_TRUE(session.command_pending());

    session.begin();
    EXPECT_FALSE(session.command_pending());

    // The command slot is genuinely free again, not merely reported so.
    drive_to_ready(hub, session);
    EXPECT_TRUE(session.send_command(pico_sh2::command::COUNTER).ok());
}

TEST(Session, BeginClearsAnInProgressFrsOperation) {
    FakeHub hub;
    Session session = make_session(hub);
    drive_to_ready(hub, session);
    ASSERT_TRUE(session.frs_read(0x2D3E).ok());
    EXPECT_TRUE(session.frs_busy());

    session.begin();
    EXPECT_FALSE(session.frs_busy());

    drive_to_ready(hub, session);
    EXPECT_TRUE(session.frs_read(0x2D3E).ok());
}

// A hub reset must free the command slot (SR-5).
TEST(Session, HubResetClearsAnOutstandingCommand) {
    FakeHub hub;
    Session session = make_session(hub);
    drive_to_ready(hub, session);
    ASSERT_TRUE(session.send_command(pico_sh2::command::COUNTER).ok());
    EXPECT_TRUE(session.command_pending());

    // The hub re-advertises: it reset underneath us.
    hub.queue_transfer(frame(0, 0, advertisement()), 0);
    session.service();
    EXPECT_EQ(session.counters().resets, 1U);
    EXPECT_FALSE(session.command_pending());
}

TEST(Session, HubResetClearsAnInProgressFrsOperation) {
    FakeHub hub;
    Session session = make_session(hub);
    drive_to_ready(hub, session);
    ASSERT_TRUE(session.frs_read(0x2D3E).ok());
    EXPECT_TRUE(session.frs_busy());

    hub.queue_transfer(frame(0, 0, advertisement()), 0);
    session.service();
    EXPECT_EQ(session.counters().resets, 1U);
    EXPECT_FALSE(session.frs_busy());
}

// Even a malformed re-advertisement starts recovery. The session must not
// remain READY after dropping learned lengths back to the seed (SR-5).
TEST(Session, MalformedReadvertisementLeavesTheSessionWaiting) {
    FakeHub hub;
    Session session = make_session(hub);
    drive_to_ready(hub, session);
    ASSERT_EQ(session.report_lengths().length_of(0x05).value_or(0), 0x18);

    // Response 0, then a truncated TLV: the tag claims more value bytes than
    // the cargo holds.
    hub.queue_transfer(frame(0, 0, {0x00, 0x01, 0x01, 0x02, 0x81, 0x06, 0x05}),
                       0);
    session.service();

    EXPECT_FALSE(session.ready());
    EXPECT_FALSE(session.faulted());
    EXPECT_EQ(session.startup_phase(), StartupPhase::WAIT_ADVERTISEMENT);
    EXPECT_EQ(session.counters().protocol_errors, 1U);
    // The advertised override is gone with the hub that advertised it.
    EXPECT_EQ(session.report_lengths().length_of(0x05).value_or(0), 14);

    // A clean advertisement afterwards completes the handshake normally.
    hub.queue_transfer(frame(0, 1, advertisement()), 0);
    hub.queue_transfer(frame(1, 0, {0x01}), 0);
    hub.queue_transfer(frame(2, 0, initialize_response(0)), 0);
    session.service();
    EXPECT_EQ(session.state(), SessionState::READY);
}

// Host-requested reset (1000-3600 §2.2).

// request_reset() sends one byte on the device channel (SR-7).
TEST(Session, RequestResetWritesTheExecutableCommand) {
    FakeHub hub;
    Session session = make_session(hub);
    drive_to_ready(hub, session);

    ASSERT_TRUE(session.request_reset().ok());
    session.service();

    ASSERT_FALSE(hub.writes().empty());
    const auto& frame = hub.writes().back().frame;
    // A one-byte cargo on the executable channel, value 1.
    ASSERT_EQ(frame.size(), pico_sh2::shtp::HEADER_SIZE + 1);
    EXPECT_EQ(frame[2], device_channel());
    EXPECT_EQ(frame[pico_sh2::shtp::HEADER_SIZE], 0x01);
}

// A seeded device channel allows reset before ready (SR-7).
TEST(Session, RequestResetDoesNotNeedReady) {
    FakeHub hub;
    Session session = make_session(hub);
    advance_to_wait_advertisement(hub, session);

    ASSERT_TRUE(session.request_reset().ok());

    // The reset has to reach the HAL from a pass that never enters READY;
    // accepting it alone does not demonstrate recovery (SR-7).
    session.service();

    ASSERT_FALSE(hub.writes().empty());
    const auto& frame = hub.writes().back().frame;
    ASSERT_EQ(frame.size(), pico_sh2::shtp::HEADER_SIZE + 1);
    EXPECT_EQ(frame[2], device_channel());
    EXPECT_EQ(frame[pico_sh2::shtp::HEADER_SIZE], 0x01);
}

// Without a seed or advertisement, reset returns NOT_READY
// (docs/usage-guide.md, Bringing a session up).
TEST(Session, RequestResetBeforeAnyAdvertisementNeedsASeed) {
    FakeHub hub;
    Session session(hub, test_timing());  // no seeded channel map
    advance_to_wait_advertisement(hub, session);

    EXPECT_EQ(session.request_reset().error(), Sh2Error::NOT_READY);

    // Once the hub has advertised, the role is bound and the request lands.
    hub.queue_startup();
    session.service();
    ASSERT_EQ(session.state(), SessionState::READY);
    EXPECT_TRUE(session.request_reset().ok());
}

// The session must use the advertised device channel even when seeded
// otherwise.
TEST(Session, AnAdvertisementOverridesAWrongSeed) {
    constexpr std::uint8_t ACTUAL = 10;
    FakeHub hub;
    Session session = make_session(hub);  // seeded for base 1
    advance_to_wait_advertisement(hub, session);

    hub.queue_transfer(frame(pico_sh2::shtp::COMMAND_CHANNEL, 0,
                             advertisement(0x05, 0x18, ACTUAL)),
                       0);
    hub.queue_transfer(frame(device_channel(ACTUAL), 0, reset_complete()), 0);
    hub.queue_transfer(
        frame(control_channel(ACTUAL), 0, initialize_response(0)), 0);
    session.service();
    ASSERT_EQ(session.state(), SessionState::READY);

    ASSERT_TRUE(session.request_reset().ok());
    session.service();
    ASSERT_FALSE(hub.writes().empty());
    // Sent to the advertised device channel, not the seeded one.
    EXPECT_EQ(hub.writes().back().frame[2], device_channel(ACTUAL));
}

// A reset-complete while ready starts recovery without an advertisement (SR-5),
// including after a requested reset (SR-5, SR-7), with no consumer action
// (SR-5).
TEST(Session, AResetCompleteWhileReadyIsAHubReset) {
    FakeHub hub;
    Session session = make_session(hub);
    ResetObserver observer;
    session.set_reset_sink(
        Session::ResetSink::bind<ResetObserver, &ResetObserver::on_reset>(
            &observer));
    drive_to_ready(hub, session);

    hub.queue_transfer(frame(1, 0, reset_complete()), 0);
    session.service();
    EXPECT_EQ(observer.count, 1);
    EXPECT_EQ(session.counters().resets, 1U);
    EXPECT_FALSE(session.ready());
    EXPECT_EQ(session.counters().unrouted_cargoes, 0U);
    // The advertisement that should precede reset-complete was missed;
    // recovery waits for one and requests it.
    EXPECT_EQ(session.startup_phase(), StartupPhase::WAIT_ADVERTISEMENT);
}

// Recovery requests the whole advertisement with SHTP Command 0, P0 = 1
// on channel 0 (1000-3535 §5.1.1; SR-5).
TEST(Session, AResetCompleteWithoutAnAdvertisementRequestsOne) {
    FakeHub hub;
    Session session = make_session(hub);
    drive_to_ready(hub, session);
    const std::size_t before = hub.writes().size();

    hub.queue_transfer(frame(1, 0, reset_complete()), 0);
    session.service();
    ASSERT_EQ(hub.writes().size(), before + 1);
    const std::vector<std::uint8_t>& sent = hub.writes().back().frame;
    ASSERT_EQ(sent.size(), 6U);
    EXPECT_EQ(sent[0], 6U);  // length LSB
    EXPECT_EQ(sent[1], 0U);  // length MSB
    EXPECT_EQ(sent[2], pico_sh2::shtp::COMMAND_CHANNEL);
    EXPECT_EQ(sent[4], 0x00U);  // Command 0: Get Advertisement
    EXPECT_EQ(sent[5], 0x01U);  // P0: the entire hub
}

// Held announcements complete recovery when the requested advertisement
// arrives, without a second announcement (SR-5).
TEST(Session, RecoveryFromAMissedAdvertisementCompletesOnTheRequestedOne) {
    FakeHub hub;
    Session session = make_session(hub);
    drive_to_ready(hub, session);

    hub.queue_transfer(frame(1, 0, reset_complete()), 0);
    hub.queue_transfer(frame(2, 0, initialize_response(0)), 0);
    session.service();
    EXPECT_EQ(session.startup_phase(), StartupPhase::WAIT_ADVERTISEMENT);
    EXPECT_FALSE(session.ready());

    hub.queue_transfer(frame(0, 0, advertisement()), 0);
    session.service();
    EXPECT_TRUE(session.ready());
    EXPECT_EQ(session.counters().resets, 1U);
    EXPECT_EQ(session.counters().unrouted_cargoes, 0U);
}

// begin() abandons a requested advertisement and held announcements;
// an Initialize before the next advertisement is out of order (SR-5).
TEST(Session, BeginAbandonsARequestedAdvertisement) {
    FakeHub hub;
    Session session = make_session(hub);
    drive_to_ready(hub, session);
    hub.queue_transfer(frame(1, 0, reset_complete()), 0);
    session.service();
    ASSERT_EQ(session.startup_phase(), StartupPhase::WAIT_ADVERTISEMENT);

    advance_to_wait_advertisement(hub, session);
    const auto unrouted = session.counters().unrouted_cargoes;
    hub.queue_transfer(frame(2, 0, initialize_response(0)), 0);
    session.service();
    EXPECT_EQ(session.counters().unrouted_cargoes, unrouted + 1);
}

// Recovery without seeds keeps the learned map long enough to route
// Initialize after reset-complete (SR-5).
TEST(Session, RecoveryFromAMissedAdvertisementNeedsNoSeededControlChannel) {
    FakeHub hub;
    Session session{hub, test_timing()};
    advance_to_wait_advertisement(hub, session);
    hub.queue_startup();
    session.service();
    ASSERT_EQ(session.state(), SessionState::READY);

    hub.queue_transfer(frame(1, 0, reset_complete()), 0);
    hub.queue_transfer(frame(2, 0, initialize_response(0)), 0);
    session.service();
    hub.queue_transfer(frame(0, 0, advertisement()), 0);
    session.service();
    EXPECT_TRUE(session.ready());
    EXPECT_EQ(session.counters().unrouted_cargoes, 0U);
}

// A transport-error restart forgets learned numbers, leaving an unseeded
// device channel unbound until re-advertisement (1000-3600 §2.3; SR-5).
TEST(Session, ATransportRestartForgetsTheLearnedChannelNumbers) {
    FakeHub hub;
    Session session{hub, test_timing()};
    advance_to_wait_advertisement(hub, session);
    hub.queue_startup();
    session.service();
    ASSERT_EQ(session.state(), SessionState::READY);
    ASSERT_TRUE(session.request_reset().ok());
    session.service();

    hub.queue_transfer(frame(3, 0, {}), 0);
    hub.fail_next_read(Sh2Error::BUS_ERROR);
    session.service();
    ASSERT_EQ(session.counters().transport_restarts, 1U);
    EXPECT_EQ(session.request_reset().error(), Sh2Error::NOT_READY);
}

// The requested advertisement replaces the temporarily kept channel numbers
// (SR-5).
TEST(Session, TheRequestedAdvertisementReplacesTheKeptChannelNumbers) {
    constexpr std::uint8_t MOVED = 10;
    FakeHub hub;
    Session session{hub, test_timing()};
    advance_to_wait_advertisement(hub, session);
    hub.queue_startup();
    session.service();
    ASSERT_EQ(session.state(), SessionState::READY);

    hub.queue_transfer(frame(1, 0, reset_complete()), 0);
    hub.queue_transfer(frame(2, 0, initialize_response(0)), 0);
    session.service();
    hub.queue_transfer(frame(0, 0, advertisement(0x05, 0x18, MOVED)), 0);
    session.service();
    ASSERT_TRUE(session.ready());

    ASSERT_TRUE(session.request_reset().ok());
    session.service();
    ASSERT_FALSE(hub.writes().empty());
    EXPECT_EQ(hub.writes().back().frame[2], device_channel(MOVED));
}

// Recovery still waits for Initialize if it follows the requested advertisement
// (SR-5).
TEST(Session, RecoveryFromAMissedAdvertisementStillWaitsForInitialize) {
    FakeHub hub;
    Session session = make_session(hub);
    drive_to_ready(hub, session);

    hub.queue_transfer(frame(1, 0, reset_complete()), 0);
    session.service();
    hub.queue_transfer(frame(0, 0, advertisement()), 0);
    session.service();
    EXPECT_EQ(session.startup_phase(), StartupPhase::WAIT_INITIALIZE);

    hub.queue_transfer(frame(2, 0, initialize_response(0)), 0);
    session.service();
    EXPECT_TRUE(session.ready());
}

// Recovery accepts either announcement order, remembering an Initialize
// received while READY before reset-complete (SR-1, SR-5).
TEST(Session, RecoveryTakesAnInitializeThatArrivedBeforeTheResetComplete) {
    FakeHub hub;
    Session session = make_session(hub);
    drive_to_ready(hub, session);

    hub.queue_transfer(frame(2, 0, initialize_response(0)), 0);
    hub.queue_transfer(frame(1, 0, reset_complete()), 0);
    session.service();
    hub.queue_transfer(frame(0, 0, advertisement()), 0);
    session.service();
    EXPECT_TRUE(session.ready());
}

// A remembered failed Initialize faults the recovery that takes it (SR-5).
TEST(Session, ARememberedFailedInitializeFaults) {
    FakeHub hub;
    Session session = make_session(hub);
    drive_to_ready(hub, session);

    hub.queue_transfer(frame(2, 0, initialize_response(1)), 0);
    hub.queue_transfer(frame(1, 0, reset_complete()), 0);
    session.service();
    EXPECT_EQ(session.state(), pico_sh2::SessionState::FAULTED);
    EXPECT_EQ(session.fault(), pico_sh2::Sh2Error::PROTOCOL);
}

// An intervening restart forgets a remembered Initialize, so a second reset
// waits for its own (SR-5).
TEST(Session, ARestartForgetsAnInitializeItDidNotTake) {
    FakeHub hub;
    Session session = make_session(hub);
    drive_to_ready(hub, session);

    hub.queue_transfer(frame(2, 0, initialize_response(0)), 0);
    session.service();
    // The hub leaves the bus; the handshake restarts and completes in full.
    hub.queue_transfer(frame(3, 0, {}), 0);
    hub.fail_next_read(Sh2Error::BUS_ERROR);
    session.service();
    hub.queue_transfer(frame(0, 0, advertisement()), 0);
    hub.queue_transfer(frame(1, 0, reset_complete()), 0);
    hub.queue_transfer(frame(2, 1, initialize_response(0)), 0);
    session.service();
    ASSERT_TRUE(session.ready());

    // A second reset, its advertisement missed, inside the first one's window.
    hub.queue_transfer(frame(1, 1, reset_complete()), 0);
    session.service();
    hub.queue_transfer(frame(0, 1, advertisement()), 0);
    session.service();
    EXPECT_EQ(session.startup_phase(), StartupPhase::WAIT_INITIALIZE);
}

// An Initialize older than its phase timeout is not reused for recovery (SR-5).
TEST(Session, AnInitializeTooOldForTheResetIsNotTaken) {
    FakeHub hub;
    Session session = make_session(hub);
    drive_to_ready(hub, session);

    hub.queue_transfer(frame(2, 0, initialize_response(0)), 0);
    session.service();
    hub.advance_time(TEST_PHASE_TIMEOUT_US + 1);
    hub.queue_transfer(frame(1, 0, reset_complete()), 0);
    session.service();
    hub.queue_transfer(frame(0, 0, advertisement()), 0);
    session.service();
    EXPECT_EQ(session.startup_phase(), StartupPhase::WAIT_INITIALIZE);
}

// begin() forgets a remembered Initialize (SR-5).
TEST(Session, BeginForgetsARememberedInitialize) {
    FakeHub hub;
    Session session = make_session(hub);
    drive_to_ready(hub, session);

    hub.queue_transfer(frame(2, 0, initialize_response(0)), 0);
    session.service();
    drive_to_ready(hub, session);

    hub.queue_transfer(frame(1, 1, reset_complete()), 0);
    session.service();
    hub.queue_transfer(frame(0, 1, advertisement()), 0);
    session.service();
    EXPECT_EQ(session.startup_phase(), StartupPhase::WAIT_INITIALIZE);
}

// A remembered Initialize still reaches the command sink as unsolicited (SR-5).
TEST(Session, ARememberedInitializeStillReachesTheCommandSink) {
    FakeHub hub;
    Session session = make_session(hub);
    CommandCapture cap;
    session.set_command_sink(
        Session::CommandSink::bind<CommandCapture, &CommandCapture::on_command>(
            &cap));
    drive_to_ready(hub, session);
    cap.responses.clear();

    hub.queue_transfer(frame(2, 0, initialize_response(0)), 0);
    session.service();
    ASSERT_EQ(cap.responses.size(), 1U);
    EXPECT_EQ(cap.responses[0].command, pico_sh2::command::INITIALIZE);
}

TEST(Session, TheHubsAnswerToARequestedResetIsRecognised) {
    FakeHub hub;
    Session session = make_session(hub);
    ResetObserver observer;
    session.set_reset_sink(
        Session::ResetSink::bind<ResetObserver, &ResetObserver::on_reset>(
            &observer));
    drive_to_ready(hub, session);

    ASSERT_TRUE(session.request_reset().ok());
    session.service();

    hub.queue_transfer(frame(0, 0, advertisement()), 0);
    session.service();
    EXPECT_EQ(observer.count, 1);
    EXPECT_FALSE(session.ready());

    hub.queue_transfer(frame(1, 0, reset_complete()), 0);
    hub.queue_transfer(frame(2, 0, initialize_response(0)), 0);
    session.service();
    EXPECT_TRUE(session.ready());
}

// Reset ends an FRS operation exactly once through its sink (SR-3, SR-5).
TEST(Session, HubResetEndsAnInProgressFrsOperationThroughItsSink) {
    FakeHub hub;
    Session session = make_session(hub);
    pico_sh2::sh2_test::FrsCapture cap;
    session.set_frs_sink(pico_sh2::Session::FrsSink::bind<
                         pico_sh2::sh2_test::FrsCapture,
                         &pico_sh2::sh2_test::FrsCapture::on_frs>(&cap));
    drive_to_ready(hub, session);
    ASSERT_TRUE(session.frs_read(0x2D3E).ok());

    hub.queue_transfer(frame(0, 0, advertisement()), 0);
    session.service();
    EXPECT_EQ(cap.calls, 1);
    EXPECT_EQ(cap.status, Sh2Error::HUB_RESET);
    EXPECT_FALSE(cap.device_status.has_value());  // the session ended it
    EXPECT_EQ(cap.type, 0x2D3E);
    EXPECT_TRUE(cap.words.empty());

    // Past the deadline the abandoned read would have had: nothing more.
    hub.advance_time(pico_sh2::sh2_test::TEST_TRANSACTION_TIMEOUT_US);
    session.service();
    EXPECT_EQ(cap.calls, 1);
}

// A sink that restarts the session from inside service().
struct Restarter {
    Session* session = nullptr;
    int calls = 0;

    void restart() {
        ++calls;
        session->begin();
    }

    void on_event(const pico_sh2::reports::SensorEvent& /*event*/) {
        restart();
    }
};

// begin() from the reset sink holds reset for the full interval and completes
// a fresh handshake, discarding the advertisement that triggered recovery.
TEST(Session, BeginFromTheResetSinkRestartsCleanly) {
    FakeHub hub;
    Session session = make_session(hub);
    Restarter restarter{.session = &session, .calls = 0};
    session.set_reset_sink(
        pico_sh2::Session::ResetSink::bind<Restarter, &Restarter::restart>(
            &restarter));
    drive_to_ready(hub, session);

    hub.queue_transfer(frame(0, 1, advertisement()), 0);
    session.service();
    EXPECT_EQ(restarter.calls, 1);
    EXPECT_EQ(session.startup_phase(), StartupPhase::RESET_ASSERTED);
    EXPECT_TRUE(hub.reset_asserted());

    advance_to_wait_advertisement(hub, session);
    EXPECT_FALSE(hub.reset_asserted());
    hub.queue_startup(0, 2);
    session.service();
    EXPECT_EQ(session.state(), SessionState::READY);
}

// begin() from the event sink ends delivery from the current cargo.
TEST(Session, BeginFromTheEventSinkDropsTheRestOfTheCargo) {
    FakeHub hub;
    Session session = make_session(hub);
    Restarter restarter{.session = &session, .calls = 0};
    session.set_event_sink(
        pico_sh2::reports::EventSink::bind<Restarter, &Restarter::on_event>(
            &restarter));
    drive_to_ready(hub, session);

    // Two accelerometer reports in one cargo on the normal input channel.
    const std::vector<std::uint8_t> accel = {0x01, 0x00, 0x03, 0x00, 0x00,
                                             0x01, 0x00, 0x00, 0x00, 0x00};
    std::vector<std::uint8_t> cargo = accel;
    cargo.insert(cargo.end(), accel.begin(), accel.end());
    hub.queue_transfer(frame(3, 0, cargo), 0);
    session.service();

    EXPECT_EQ(restarter.calls, 1);
    EXPECT_EQ(session.startup_phase(), StartupPhase::RESET_ASSERTED);
    EXPECT_TRUE(hub.reset_asserted());
}

// A sink that pumps the session it was called from.
struct Pumper {
    Session* session = nullptr;
    std::vector<std::uint16_t> delays;

    void on_event(const pico_sh2::reports::SensorEvent& event) {
        delays.push_back(event.delay_100us);
        session->service();
    }
};

// service() from a sink does nothing, preserving the staging buffer (AR-3).
// Draining here would deliver the next cargo before this one's last events.
TEST(Session, ServiceFromASinkIsANoOp) {
    FakeHub hub;
    Session session = make_session(hub);
    Pumper pumper{.session = &session, .delays = {}};
    session.set_event_sink(
        pico_sh2::reports::EventSink::bind<Pumper, &Pumper::on_event>(&pumper));
    drive_to_ready(hub, session);

    // Accelerometer reports told apart by their delay byte.
    auto accel = [](std::uint8_t delay) {
        return std::vector<std::uint8_t>{0x01, 0x00, 0x03, delay, 0x00,
                                         0x01, 0x00, 0x00, 0x00,  0x00};
    };
    std::vector<std::uint8_t> first = accel(1);
    const std::vector<std::uint8_t> second = accel(2);
    first.insert(first.end(), second.begin(), second.end());
    hub.queue_transfer(frame(3, 0, first), 0);
    hub.queue_transfer(frame(3, 1, accel(3)), 0);
    session.service();

    EXPECT_EQ(pumper.delays, (std::vector<std::uint16_t>{1, 2, 3}));
}

}  // namespace
