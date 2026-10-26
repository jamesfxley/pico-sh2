// SPDX-License-Identifier: MIT
// Session tests: recognising a hub that reset underneath the session and
// re-running the handshake without losing or stranding state (SR-5).
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

// Named one at a time rather than with a using-directive: this suite
// reaches for a handful of helpers, and a budget line it does not need
// is budget spent for nothing. Each is named in the namespace that
// defines it.
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

// SR-5, NR-4: byte 0 of a channel-0 read cargo is the SHTP response code
// (1000-3535 §5.1). Only Response 0 is an advertisement. The hub sends
// Response 1, its Error List, on its own whenever it meets a recoverable SHTP
// error (§5.1.2.1); that is a report, not a re-advertisement, so a ready
// session stays ready.
TEST(Session, AnErrorListWhileReadyIsNotAReset) {
    FakeHub hub;
    Session session = make_session(hub);
    ResetObserver observer;
    session.set_reset_sink(
        Session::ResetSink::bind<ResetObserver, &ResetObserver::on_reset>(
            &observer));
    drive_to_ready(hub, session);

    // Error 9: host wrote to an unrecognized channel.
    hub.queue_transfer(frame(0, 1, {0x01, 0x09}), 0);
    session.service();
    EXPECT_TRUE(session.ready());
    EXPECT_EQ(observer.count, 0);
    EXPECT_EQ(session.counters().resets, 0U);
    EXPECT_EQ(session.counters().protocol_errors, 0U);
}

// NR-4: the advertisement is the body after the response byte. This one
// parses only with the byte removed -- read from byte 0, the 0x00 is a tag
// whose one-byte value swallows the GUID tag, and 0x02 then claims 0x81 bytes
// the cargo does not hold.
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

// NR-4, SR-5: a Response 0 with no body is not an advertisement, and a ready
// session is not reset by one.
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

// Responses 2-255 are reserved (1000-3535 §5.1.3): a protocol error, not a
// reset.
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
    // A read fails mid-drain: the session faults with the bus error and records
    // the faulting phase. A coincidentally-elapsed phase deadline must not
    // re-fault it as a startup timeout (which would overwrite fault()).
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
    // A read the hub does not answer while READY is the hub leaving the bus,
    // which is how a reset presents on a transport that can report one at all
    // (SR-5). The handshake restarts and the consumer is notified; the session
    // does not fault.
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

// OR-1: the counters mirrored from the framing engine are published on every
// service() pass, including one that stopped draining early on a transport
// error. Cargoes delivered before the error must not go unreported until some
// later clean pass.
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

    // The whole moved set, so the same pass moving an unrelated counter fails:
    // the cargo, which no handler takes while READY, and the restart, which
    // `resets` counts and `transport_restarts` breaks down.
    EXPECT_EQ(
        moved(before, session.counters()),
        (std::vector<std::string>{"cargoes_in", "resets", "transport_restarts",
                                  "unrouted_cargoes"}));
    EXPECT_EQ(session.counters().cargoes_in, before.cargoes_in + 1);
    EXPECT_EQ(session.counters().transport_restarts, 1U);
}

// OR-1: only constructing a new session clears its counters. begin() restarts
// the hub, not the record of what the session has seen. No transfer is read
// during the reset hold and boot, so nothing should have moved.
TEST(Session, CountersSurviveBegin) {
    FakeHub hub;
    Session session = make_session(hub);
    drive_to_ready(hub, session);
    // Counters at zero would survive a begin() that zeroed them, so the two
    // timeout counters are moved first: a command and an FRS read, each left
    // to expire.
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

    // The engine's counts reach counters() only from a pass that drains, and
    // the reset and boot passes do not, so one runs past them before the
    // comparison: a begin() that reset the engine would show there, not
    // straight after the call.
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
    // A transport that is dead rather than resetting never re-advertises, so
    // the restart above resolves to the advertisement deadline: a TIMEOUT
    // attributed to WAIT_ADVERTISEMENT rather than the bus error that is only
    // the symptom (SR-5).
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

// AR-3: a single service() call drains a bounded number of transfers, so a HAL
// reporting data_ready() indefinitely cannot spin the drain loop.
TEST(Session, DrainIsBoundedPerServiceCall) {
    FakeHub hub;
    Session session = make_session(hub);
    advance_to_wait_advertisement(hub, session);
    // Flood the hub with empty idle transfers. A single service() drains a
    // bounded number of them and always terminates (no unbounded loop, even if
    // a HAL reported data_ready() indefinitely).
    constexpr std::size_t FLOOD = 500;
    for (std::size_t i = 0; i < FLOOD; ++i) {
        hub.queue_transfer(frame(0, static_cast<std::uint8_t>(i), {}), 0);
    }
    session.service();
    // AR-3: the cap is the profile's, so the assertion is against the knob and
    // not against the flood size. "fewer than 500" passes at 499 and says
    // nothing about the bound the requirement states.
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

    // Restarting from scratch must discard it rather than flush it at the new
    // hub the moment the handshake completes.
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

// SR-5: a hub reset in the middle of a command must free the command slot. The
// hub restarting has abandoned the request, so waiting for its response would
// hold the slot until the deadline for no reason.
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

// SR-5: a re-advertisement that turns out to be malformed still means the hub
// reset. The session must not keep claiming to be READY over a report-length
// table that has already been dropped back to the seed — it has to wait for a
// clean advertisement.
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

// --- host-requested reset (1000-3600 §2.2) ----------------------------------

// SR-7: request_reset() writes the one-byte reset on the executable channel.
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

// SR-7: unlike the control-channel requests, a reset does not require `ready`.
// The device channel's number comes from the consumer's seed, which is the
// whole reason a seed exists: it is what keeps a session stuck short of the
// handshake recoverable.
TEST(Session, RequestResetDoesNotNeedReady) {
    FakeHub hub;
    Session session = make_session(hub);
    advance_to_wait_advertisement(hub, session);

    ASSERT_TRUE(session.request_reset().ok());

    // Accepting the request is not the requirement: SR-7 is a way *out* of a
    // session stuck short of ready, so the cargo has to reach the hub from a
    // service() pass that never enters the READY branch.
    session.service();

    ASSERT_FALSE(hub.writes().empty());
    const auto& frame = hub.writes().back().frame;
    ASSERT_EQ(frame.size(), pico_sh2::shtp::HEADER_SIZE + 1);
    EXPECT_EQ(frame[2], device_channel());
    EXPECT_EQ(frame[pico_sh2::shtp::HEADER_SIZE], 0x01);
}

// Without a seed there is no device channel until the hub advertises one, so
// the request is refused rather than sent to a guessed number. This is the cost
// of the library holding no channel numbers of its own.
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

// A seed is a starting guess, not a claim: the advertisement decides. A session
// seeded for one base must still follow a hub that advertises another.
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

// The hub answers by restarting and re-announcing itself, which is the same
// path an unexpected reset takes (SR-5).
// SR-7: the hub's re-announcement after a requested reset is recognised by the
// SR-5 recovery path, so recovery needs nothing further.
// SR-5: a reset-complete with no advertisement before it, arriving while the
// session is ready, is a hub reset as surely as an advertisement is.
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
    // The advertisement that should have preceded it was missed, so the
    // session waits for one and asks the hub for it.
    EXPECT_EQ(session.startup_phase(), StartupPhase::WAIT_ADVERTISEMENT);
}

// SR-5: the advertisement a reset-complete arrived without is requested --
// SHTP Command 0 with P0 = 1, the whole hub's advertisement (1000-3535
// §5.1.1) -- on the command channel.
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

// SR-5: the announcements that follow a missed advertisement are held until
// the requested one arrives, so recovery completes without the hub announcing
// itself twice.
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

// SR-5: begin() while a requested advertisement is outstanding starts over,
// so the next startup holds nothing for it: an Initialize response ahead of
// the advertisement is out of order again.
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

// SR-5: the same recovery for a consumer that seeded nothing, whose map knows
// the control channel only from the advertisement before the reset. The
// reset-complete arrived on the learned device channel, so the hub's numbering
// is unchanged and the learned map still routes the Initialize response.
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

// SR-5: every other restart forgets what the last advertisement taught, since
// a hub that reset may come back numbered differently (1000-3600 §2.3). With
// nothing seeded, a transport-error restart leaves no device channel to send a
// reset on until the hub advertises again.
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

// SR-5: the numbers kept through that recovery are the previous
// advertisement's, and the requested advertisement still replaces them.
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

// SR-5: where the Initialize response has not arrived by the time the
// requested advertisement does, the session waits for it and nothing else.
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

// SR-1, SR-5: recovery from a missed advertisement does not depend on which of
// the two announcements arrives first. An Initialize response that comes ahead
// of the reset-complete, while the session is still READY, is remembered, and
// the handshake the reset-complete restarts takes it rather than waiting for
// one that has already gone by.
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

// SR-5: a remembered Initialize that reported failure faults the handshake it
// is taken into, as one arriving in the usual order does.
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

// SR-5: the remembered Initialize belongs to the reset that follows it, and to
// no later one: a restart for any other reason forgets it, so a second reset
// inside the window waits for its own Initialize rather than reusing the first.
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

// SR-5: the remembered Initialize belongs to the reset only while it is recent:
// one older than the Initialize phase's own timeout is not taken, and the
// handshake waits for a fresh one as it always has.
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

// SR-5: begin() is a restart too, so it forgets a remembered Initialize; the
// reset that follows a fresh handshake waits for its own.
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

// SR-5: an Initialize remembered for a reset is still an unsolicited response,
// and reaches the command sink as one.
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

// SR-3, SR-5: an FRS operation a hub reset cuts off still ends exactly once,
// through the FRS sink, so a consumer waiting on it is not stranded.
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

// begin() from the reset sink starts a clean run: the hub is held in reset for
// the hold interval, released, and the handshake completes. The advertisement
// that announced the reset is not carried into the new run.
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

// begin() from the event sink ends the cargo being walked: its remaining
// reports belong to the hub incarnation the call restarted.
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

// service() from a sink returns at once. Draining again from inside the walk
// would deliver the next cargo's events ahead of this one's remainder, from a
// staging buffer the outer pass is still reading.
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
