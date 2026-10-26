// SPDX-License-Identifier: MIT
// Session tests: the reset pulse, the boot wait, and the three announcements
// the hub makes on its way up (SR-1), including each phase's deadline.
#include <gtest/gtest.h>

#include <cstdint>
#include <vector>

#include "pico_sh2/reports/report_length_table.hpp"
#include "pico_sh2/sh2/session.hpp"
#include "pico_sh2/sh2/session_timing.hpp"
#include "pico_sh2/sh2/startup_phase.hpp"
#include "pico_sh2/shtp/advertisement.hpp"
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
using pico_sh2::sh2_test::drive_to_ready;
using pico_sh2::sh2_test::make_session;
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

// A malformed advertisement is applied entry by entry and rejected only when
// the truncation is reached, so the entries before it have already run. The
// session has to discard them: a wrong report length that outlives the cargo
// that carried it mis-splits every later input cargo for the life of the
// session, and no counter shows it.
TEST(Session, MalformedAdvertisementLeavesNothingApplied) {
    FakeHub hub;
    Session session = make_session(hub);
    advance_to_wait_advertisement(hub, session);

    // A wrong length for the rotation vector, then a tag byte with no length
    // byte after it -- a truncated entry the parse rejects.
    std::vector<std::uint8_t> malformed = advertisement(0x05, 3);
    malformed.push_back(pico_sh2::shtp::REPORT_LENGTHS_TAG);

    hub.queue_transfer(frame(0, 0, malformed), 1'000);
    session.service();

    // The handshake does not advance, and the wrong length is gone.
    EXPECT_EQ(session.startup_phase(), StartupPhase::WAIT_ADVERTISEMENT);
    EXPECT_EQ(session.counters().protocol_errors, 1U);
    const auto seeded = pico_sh2::reports::ReportLengthTable::seeded();
    EXPECT_EQ(session.report_lengths().length_of(0x05).value(),
              seeded.length_of(0x05).value());

    // A clean advertisement afterwards still brings the session up.
    hub.queue_startup();
    session.service();
    EXPECT_EQ(session.state(), SessionState::READY);
}

// SR-1: reset stays asserted for the whole hold interval, and not a
// microsecond past it.
TEST(Session, ResetIsHeldForTheWholeHoldInterval) {
    FakeHub hub;
    Session session = make_session(hub);
    session.begin();
    ASSERT_TRUE(hub.reset_asserted());

    hub.advance_time(pico_sh2::sh2_test::TEST_RESET_HOLD_US - 1);
    session.service();
    EXPECT_EQ(session.startup_phase(), StartupPhase::RESET_ASSERTED);
    EXPECT_TRUE(hub.reset_asserted());

    hub.advance_time(1);
    session.service();
    EXPECT_EQ(session.startup_phase(), StartupPhase::BOOTING);
    EXPECT_FALSE(hub.reset_asserted());
}

// SR-1: the boot interval is waited out in full before the session listens for
// the advertisement, and hub traffic inside it is not consumed.
TEST(Session, TheBootIntervalIsWaitedOutInFull) {
    FakeHub hub;
    Session session = make_session(hub);
    session.begin();
    hub.advance_time(pico_sh2::sh2_test::TEST_RESET_HOLD_US);
    session.service();
    ASSERT_EQ(session.startup_phase(), StartupPhase::BOOTING);

    hub.queue_startup();
    hub.advance_time(pico_sh2::sh2_test::TEST_BOOT_DELAY_US - 1);
    session.service();
    EXPECT_EQ(session.startup_phase(), StartupPhase::BOOTING);
    EXPECT_EQ(session.counters().cargoes_in, 0U);

    hub.advance_time(1);
    session.service();
    EXPECT_EQ(session.startup_phase(), StartupPhase::WAIT_ADVERTISEMENT);
    session.service();
    EXPECT_EQ(session.state(), SessionState::READY);
}

TEST(Session, StartupReachesReady) {
    FakeHub hub;
    Session session = make_session(hub);
    advance_to_wait_advertisement(hub, session);
    EXPECT_TRUE(hub.reset_asserted() == false);

    hub.queue_transfer(frame(0, 0, advertisement()), 1'000);
    session.service();
    EXPECT_EQ(session.startup_phase(), StartupPhase::WAIT_RESET_COMPLETE);

    hub.queue_transfer(frame(1, 0, {0x01}), 1'000);
    session.service();
    EXPECT_EQ(session.startup_phase(), StartupPhase::WAIT_INITIALIZE);

    hub.queue_transfer(frame(2, 0, initialize_response(0)), 1'000);
    session.service();
    EXPECT_EQ(session.state(), SessionState::READY);
    EXPECT_TRUE(session.ready());

    // The advertisement's report-length override took effect.
    EXPECT_EQ(session.report_lengths().length_of(0x05).value_or(0), 0x18);
    EXPECT_EQ(session.counters().cargoes_in, 3U);
}

// SR-1: the executable and SH-2 applications announce leaving reset
// independently, and the session does not depend on which arrives first.
TEST(Session, StartupReachesReadyWithInitializeBeforeResetComplete) {
    FakeHub hub;
    Session session = make_session(hub);
    advance_to_wait_advertisement(hub, session);

    hub.queue_transfer(frame(0, 0, advertisement()), 1'000);
    session.service();
    EXPECT_EQ(session.startup_phase(), StartupPhase::WAIT_RESET_COMPLETE);

    hub.queue_transfer(frame(2, 0, initialize_response(0)), 1'000);
    session.service();
    // Still waiting on the executable's announcement, not yet ready.
    EXPECT_EQ(session.startup_phase(), StartupPhase::WAIT_RESET_COMPLETE);
    EXPECT_FALSE(session.ready());

    hub.queue_transfer(frame(1, 0, {0x01}), 1'000);
    session.service();
    EXPECT_EQ(session.state(), SessionState::READY);
    EXPECT_TRUE(session.ready());
    EXPECT_EQ(session.counters().unrouted_cargoes, 0U);
}

TEST(Session, StartupHandlesReversedCargoesInOnePump) {
    FakeHub hub;
    Session session = make_session(hub);
    advance_to_wait_advertisement(hub, session);

    hub.queue_transfer(frame(0, 0, advertisement()), 0);
    hub.queue_transfer(frame(2, 0, initialize_response(0)), 0);
    hub.queue_transfer(frame(1, 0, {0x01}), 0);
    session.service();
    EXPECT_EQ(session.state(), SessionState::READY);
}

TEST(Session, StartupHandlesAllCargoesInOnePump) {
    FakeHub hub;
    Session session = make_session(hub);
    advance_to_wait_advertisement(hub, session);

    hub.queue_transfer(frame(0, 0, advertisement()), 0);
    hub.queue_transfer(frame(1, 0, {0x01}), 0);
    hub.queue_transfer(frame(2, 0, initialize_response(0)), 0);
    session.service();
    EXPECT_EQ(session.state(), SessionState::READY);
}

TEST(Session, AdvertisementTimeoutFaults) {
    FakeHub hub;
    Session session = make_session(hub);
    advance_to_wait_advertisement(hub, session);

    hub.advance_time(200'001);  // past advertisement_timeout_us
    session.service();
    EXPECT_EQ(session.state(), SessionState::FAULTED);
    EXPECT_EQ(session.fault(), Sh2Error::TIMEOUT);
    EXPECT_EQ(session.startup_phase(), StartupPhase::WAIT_ADVERTISEMENT);
    EXPECT_EQ(session.counters().startup_timeouts, 1U);
}

TEST(Session, ResetCompleteTimeoutFaults) {
    FakeHub hub;
    Session session = make_session(hub);
    advance_to_wait_advertisement(hub, session);
    hub.queue_transfer(frame(0, 0, advertisement()), 0);
    session.service();
    ASSERT_EQ(session.startup_phase(), StartupPhase::WAIT_RESET_COMPLETE);

    hub.advance_time(200'001);
    session.service();
    EXPECT_EQ(session.state(), SessionState::FAULTED);
    EXPECT_EQ(session.fault(), Sh2Error::TIMEOUT);
    EXPECT_EQ(session.startup_phase(), StartupPhase::WAIT_RESET_COMPLETE);
}

TEST(Session, InitializeTimeoutFaults) {
    FakeHub hub;
    Session session = make_session(hub);
    advance_to_wait_advertisement(hub, session);
    hub.queue_transfer(frame(0, 0, advertisement()), 0);
    hub.queue_transfer(frame(1, 0, {0x01}), 0);
    session.service();
    ASSERT_EQ(session.startup_phase(), StartupPhase::WAIT_INITIALIZE);

    hub.advance_time(200'001);
    session.service();
    EXPECT_EQ(session.state(), SessionState::FAULTED);
    EXPECT_EQ(session.fault(), Sh2Error::TIMEOUT);
    EXPECT_EQ(session.startup_phase(), StartupPhase::WAIT_INITIALIZE);
}

TEST(Session, InitializeFailureStatusFaults) {
    FakeHub hub;
    Session session = make_session(hub);
    advance_to_wait_advertisement(hub, session);
    hub.queue_transfer(frame(0, 0, advertisement()), 0);
    hub.queue_transfer(frame(1, 0, {0x01}), 0);
    hub.queue_transfer(frame(2, 0, initialize_response(1)), 0);  // R0 = failure
    session.service();
    EXPECT_EQ(session.state(), SessionState::FAULTED);
    EXPECT_EQ(session.fault(), Sh2Error::PROTOCOL);
}

TEST(Session, RejectsMalformedResetComplete) {
    FakeHub hub;
    Session session = make_session(hub);
    advance_to_wait_advertisement(hub, session);
    hub.queue_transfer(frame(0, 0, advertisement()), 0);
    session.service();
    ASSERT_EQ(session.startup_phase(), StartupPhase::WAIT_RESET_COMPLETE);

    // A reset-complete carrying a reserved value is a protocol error and does
    // not advance the handshake.
    hub.queue_transfer(frame(1, 0, {0x02}), 0);
    session.service();
    EXPECT_EQ(session.startup_phase(), StartupPhase::WAIT_RESET_COMPLETE);
    EXPECT_EQ(session.counters().protocol_errors, 1U);
}

TEST(Session, EmptyChannelZeroCargoDoesNotAdvanceHandshake) {
    FakeHub hub;
    Session session = make_session(hub);
    advance_to_wait_advertisement(hub, session);
    // A header-only (length == HEADER_SIZE) channel-0 transfer carries an empty
    // cargo. It is not an advertisement and must not advance the handshake.
    hub.queue_transfer(frame(0, 0, {}), 0);
    session.service();
    EXPECT_EQ(session.startup_phase(), StartupPhase::WAIT_ADVERTISEMENT);
    EXPECT_EQ(session.counters().protocol_errors, 1U);

    // A real advertisement still advances it.
    hub.queue_transfer(frame(0, 1, advertisement()), 0);
    session.service();
    EXPECT_EQ(session.startup_phase(), StartupPhase::WAIT_RESET_COMPLETE);
}

// SR-10: the hub chooses its own channel base (1000-3600 §2.3), so the session
// must route from what was advertised rather than from any number compiled in.
// The same handshake, with every SH-2 channel moved by nine.
TEST(Session, ReachesReadyWithChannelsAtAnUnusualBase) {
    constexpr std::uint8_t BASE = 10;
    FakeHub hub;
    Session session = make_session(hub, BASE);
    advance_to_wait_advertisement(hub, session);

    hub.queue_transfer(frame(pico_sh2::shtp::COMMAND_CHANNEL, 0,
                             advertisement(0x05, 0x18, BASE)),
                       0);
    session.service();
    EXPECT_EQ(session.startup_phase(), StartupPhase::WAIT_RESET_COMPLETE);

    hub.queue_transfer(frame(device_channel(BASE), 0, reset_complete()), 0);
    hub.queue_transfer(frame(control_channel(BASE), 0, initialize_response(0)),
                       0);
    session.service();
    EXPECT_EQ(session.state(), SessionState::READY);
    EXPECT_EQ(session.counters().unrouted_cargoes, 0U);
}

// The numbers a hub used before a reset say nothing about the ones it comes
// back with, so what the previous advertisement taught must not be reused.
TEST(Session, FollowsTheHubToADifferentBaseAfterAReset) {
    FakeHub hub;
    Session session = make_session(hub);
    drive_to_ready(hub, session);

    constexpr std::uint8_t MOVED = 10;
    hub.queue_transfer(frame(pico_sh2::shtp::COMMAND_CHANNEL, 1,
                             advertisement(0x05, 0x18, MOVED)),
                       0);
    session.service();
    ASSERT_EQ(session.startup_phase(), StartupPhase::WAIT_RESET_COMPLETE);

    hub.queue_transfer(frame(device_channel(MOVED), 1, reset_complete()), 0);
    hub.queue_transfer(frame(control_channel(MOVED), 1, initialize_response(0)),
                       0);
    session.service();
    EXPECT_EQ(session.state(), SessionState::READY);

    // Traffic on the *old* control channel is now nobody's, and is counted
    // rather than misrouted.
    const std::uint32_t before = session.counters().unrouted_cargoes;
    hub.queue_transfer(frame(control_channel(), 2, {0x00}), 0);
    session.service();
    EXPECT_GT(session.counters().unrouted_cargoes, before);
}

// An advertisement naming fewer channels than SH-2 defines leaves an unseeded
// session unable to carry control or input, so it must not advance the
// handshake. A seeded session is a different case: it already has numbers, and
// proceeds on the consumer's claim about its part.
TEST(Session, AnIncompleteAdvertisementDoesNotAdvanceAnUnseededSession) {
    FakeHub hub;
    Session session(hub, test_timing());  // no seed to fall back on
    advance_to_wait_advertisement(hub, session);

    // Response 0 carrying only the SHTP application, so no SH-2 role is bound.
    std::vector<std::uint8_t> partial{0x00};
    pico_sh2::testing::append_tlv(partial, pico_sh2::shtp::GUID_TAG,
                                  {0x00, 0x00, 0x00, 0x00});
    pico_sh2::testing::append_tlv(partial, pico_sh2::shtp::NORMAL_CHANNEL_TAG,
                                  {pico_sh2::shtp::COMMAND_CHANNEL});
    pico_sh2::testing::append_string_tlv(
        partial, pico_sh2::shtp::CHANNEL_NAME_TAG, "control");

    hub.queue_transfer(frame(pico_sh2::shtp::COMMAND_CHANNEL, 0, partial), 0);
    session.service();
    EXPECT_EQ(session.startup_phase(), StartupPhase::WAIT_ADVERTISEMENT);
    EXPECT_EQ(session.counters().protocol_errors, 1U);
}

// SR-10: the seeded half of the case above. The same incomplete advertisement
// advances a session the consumer seeded, whose seed supplies the roles the
// advertisement leaves out.
TEST(Session, AnIncompleteAdvertisementAdvancesASeededSession) {
    FakeHub hub;
    Session session = make_session(hub);
    advance_to_wait_advertisement(hub, session);

    std::vector<std::uint8_t> partial{0x00};
    pico_sh2::testing::append_tlv(partial, pico_sh2::shtp::GUID_TAG,
                                  {0x00, 0x00, 0x00, 0x00});
    pico_sh2::testing::append_tlv(partial, pico_sh2::shtp::NORMAL_CHANNEL_TAG,
                                  {pico_sh2::shtp::COMMAND_CHANNEL});
    pico_sh2::testing::append_string_tlv(
        partial, pico_sh2::shtp::CHANNEL_NAME_TAG, "control");

    hub.queue_transfer(frame(pico_sh2::shtp::COMMAND_CHANNEL, 0, partial), 0);
    session.service();
    EXPECT_NE(session.startup_phase(), StartupPhase::WAIT_ADVERTISEMENT);
    EXPECT_EQ(session.counters().protocol_errors, 0U);
}

// Timing has no defaults, so a consumer that supplies none hands the session a
// zeroed struct. Releasing reset immediately and arming deadlines that expire
// on the same tick would look like a hub that never answers, so begin() names
// the real fault instead.
TEST(Session, BeginRejectsUnsetTiming) {
    FakeHub hub;
    Session session(hub, pico_sh2::SessionTiming{});
    session.begin();
    EXPECT_TRUE(session.faulted());
    EXPECT_EQ(session.fault(), Sh2Error::INVALID_ARGUMENT);
}

TEST(Session, BeginRejectsTimingWithAnyZeroField) {
    // Each field zeroed on its own, so a field valid() stops checking fails
    // here rather than passing behind one that is still checked.
    using Field = std::uint64_t pico_sh2::SessionTiming::*;
    for (const Field field :
         {&pico_sh2::SessionTiming::reset_hold_us,
          &pico_sh2::SessionTiming::boot_delay_us,
          &pico_sh2::SessionTiming::advertisement_timeout_us,
          &pico_sh2::SessionTiming::reset_complete_timeout_us,
          &pico_sh2::SessionTiming::initialize_timeout_us,
          &pico_sh2::SessionTiming::command_timeout_us,
          &pico_sh2::SessionTiming::frs_timeout_us}) {
        FakeHub hub;
        pico_sh2::SessionTiming timing = test_timing();
        timing.*field = 0;
        Session session(hub, timing);
        session.begin();
        EXPECT_TRUE(session.faulted());
        EXPECT_EQ(session.fault(), Sh2Error::INVALID_ARGUMENT);
    }
}

TEST(Session, BadHeaderIsCountedNotFatal) {
    FakeHub hub;
    Session session = make_session(hub);
    advance_to_wait_advertisement(hub, session);
    // Reserved length 0xFFFF: a dead-bus header. Counted, session keeps
    // waiting.
    std::vector<std::uint8_t> dead_bus{0xFF, 0xFF, 0x00, 0x00};
    hub.queue_transfer(dead_bus, 0);
    session.service();
    EXPECT_EQ(session.startup_phase(), StartupPhase::WAIT_ADVERTISEMENT);
    EXPECT_EQ(session.counters().bad_headers, 1U);
}

}  // namespace
