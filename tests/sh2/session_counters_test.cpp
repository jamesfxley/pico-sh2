// SPDX-License-Identifier: MIT
// OR-1: the counter set is generated from session_counters.def, so the struct
// and the visitor cannot disagree about what counters exist, and each counter
// moves only on the event it names.
#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "pico_sh2/reports/decoders.hpp"
#include "pico_sh2/reports/sensor_id.hpp"
#include "pico_sh2/sh2/command.hpp"
#include "pico_sh2/sh2/frs.hpp"
#include "pico_sh2/sh2/profile.hpp"
#include "pico_sh2/sh2/session.hpp"
#include "pico_sh2/shtp/channels.hpp"
#include "pico_sh2/testing/fake_hub.hpp"
#include "pico_sh2/testing/frame_builder.hpp"
#include "pico_sh2/util/result.hpp"
#include "session_fixture.hpp"

namespace {

// Named one at a time rather than with a using-directive: this suite reaches
// for only a handful of helpers, and the inline suppression a directive would
// need is budget spent for nothing. Each is named in the namespace that defines
// it rather than the fixture that re-exports it.
using pico_sh2::sh2_test::drive_to_ready;
using pico_sh2::sh2_test::EventCapture;
using pico_sh2::sh2_test::make_session;
using pico_sh2::sh2_test::moved;
using pico_sh2::sh2_test::visit_all;
using pico_sh2::testing::advertisement;
using pico_sh2::testing::control_channel;
using pico_sh2::testing::FakeHub;
using pico_sh2::testing::feature_response;
using pico_sh2::testing::flush_completed;
using pico_sh2::testing::frame;
using pico_sh2::testing::input_normal_channel;

using pico_sh2::COUNTER_COUNT;
using pico_sh2::Session;
using pico_sh2::SessionCounters;

TEST(SessionCounters, DefaultConstructsToZero) {
    const SessionCounters counters;
    for (const auto& entry : visit_all(counters)) {
        EXPECT_EQ(entry.value, 0U) << entry.name;
    }
}

TEST(SessionCounters, VisitorCoversTheWholeSet) {
    const SessionCounters counters;
    EXPECT_EQ(visit_all(counters).size(), COUNTER_COUNT);
    EXPECT_GT(COUNTER_COUNT, 0U);
}

TEST(SessionCounters, VisitorReportsEachCountersOwnValue) {
    SessionCounters counters;
    counters.cargoes_in = 7;
    counters.frs_timeouts = 3;

    bool saw_cargoes_in = false;
    bool saw_frs_timeouts = false;
    for (const auto& entry : visit_all(counters)) {
        if (entry.name == "cargoes_in") {
            EXPECT_EQ(entry.value, 7U);
            saw_cargoes_in = true;
        } else if (entry.name == "frs_timeouts") {
            EXPECT_EQ(entry.value, 3U);
            saw_frs_timeouts = true;
        } else {
            EXPECT_EQ(entry.value, 0U) << entry.name;
        }
    }
    EXPECT_TRUE(saw_cargoes_in);
    EXPECT_TRUE(saw_frs_timeouts);
}

// Names and descriptions are what a consumer prints, so both must be present
// and each name unique.
TEST(SessionCounters, EveryCounterHasAUniqueNameAndADescription) {
    const SessionCounters counters;
    std::vector<std::string> names;
    for (const auto& entry : visit_all(counters)) {
        EXPECT_FALSE(entry.name.empty());
        EXPECT_FALSE(entry.description.empty());
        names.push_back(entry.name);
    }
    std::vector<std::string> unique = names;
    std::ranges::sort(unique);
    // std::ranges::unique returns the subrange of duplicates to drop, where
    // std::unique returns a single iterator; erase takes both ends of it.
    const auto duplicates = std::ranges::unique(unique);
    unique.erase(duplicates.begin(), duplicates.end());
    EXPECT_EQ(unique.size(), names.size());
}

// --- OR-1: each counter moves only on the event it names ---
//
// The cases below assert on the *whole* moved set rather than on the one
// counter each is about. Asserting `flushes_completed == 1` would pass just as
// well if the same event also moved three unrelated counters, which is the half
// of OR-1 that says a counter does not move on an event it does not name.
// Driving the set from for_each_counter means a counter added to
// session_counters.def is covered here without touching this helper.

// A calibrated accelerometer report (0x01), the same 10-byte layout the decoder
// suite uses. Decodes to one event. An array rather than a vector so the
// namespace-scope initialisation cannot throw.
constexpr std::array<std::uint8_t, 10> ACCEL_REPORT = {
    0x01, 0x2A, 0x03, 0x00, 0x00, 0x01, 0x00, 0x02, 0x00, 0xFF};

TEST(SessionCounters, AnIdleTransferMovesOnlyEmptyTransfers) {
    FakeHub hub;
    Session session = make_session(hub);
    drive_to_ready(hub, session);

    const SessionCounters before = session.counters();
    // A null header: length field 0, no cargo.
    hub.queue_transfer(std::vector<std::uint8_t>{0x00, 0x00, 0x03, 0x00}, 0);
    session.service();

    EXPECT_EQ(moved(before, session.counters()),
              std::vector<std::string>{"empty_transfers"});
}

TEST(SessionCounters, AReservedLengthMovesOnlyBadHeaders) {
    FakeHub hub;
    Session session = make_session(hub);
    drive_to_ready(hub, session);

    const SessionCounters before = session.counters();
    // 0xFFFF is the reserved dead-bus signature (NR-1).
    hub.queue_transfer(std::vector<std::uint8_t>{0xFF, 0xFF, 0x02, 0x00}, 0);
    session.service();

    EXPECT_EQ(moved(before, session.counters()),
              std::vector<std::string>{"bad_headers"});
}

TEST(SessionCounters, AnOversizeCargoMovesOnlyReassemblyDrops) {
    FakeHub hub;
    Session session = make_session(hub);
    drive_to_ready(hub, session);

    const SessionCounters before = session.counters();
    // A lead header declaring one byte more cargo than the active profile's
    // reassembly buffer holds, so it is dropped whole rather than truncated.
    // Derived from the profile rather than written as a literal: the two
    // profiles size this buffer differently, and any fixed length is oversize
    // for only one of them.
    const std::size_t declared = pico_sh2::ActiveProfile::MAX_CARGO_LEN +
                                 pico_sh2::shtp::HEADER_SIZE + 1;
    hub.queue_transfer(
        std::vector<std::uint8_t>{
            static_cast<std::uint8_t>(declared & 0xFFU),
            static_cast<std::uint8_t>((declared >> 8U) & 0x7FU),
            input_normal_channel(), 0x00, 0xAA, 0xBB},
        0);
    session.service();

    EXPECT_EQ(moved(before, session.counters()),
              std::vector<std::string>{"reassembly_drops"});
}

TEST(SessionCounters, ADecodedInputReportMovesCargoesInAndEventsDecoded) {
    FakeHub hub;
    Session session = make_session(hub);
    EventCapture capture;
    session.set_event_sink(
        pico_sh2::reports::EventSink::bind<EventCapture,
                                           &EventCapture::on_event>(&capture));
    drive_to_ready(hub, session);

    const SessionCounters before = session.counters();
    hub.queue_transfer(frame(input_normal_channel(), 0, ACCEL_REPORT), 0);
    session.service();

    ASSERT_EQ(capture.events.size(), 1U);
    EXPECT_EQ(moved(before, session.counters()),
              (std::vector<std::string>{"cargoes_in", "events_decoded"}));
}

TEST(SessionCounters, ASkippedSequenceNumberAlsoMovesSequenceGaps) {
    FakeHub hub;
    Session session = make_session(hub);
    drive_to_ready(hub, session);
    hub.queue_transfer(frame(input_normal_channel(), 0, ACCEL_REPORT), 0);
    session.service();

    // Snapshot after the first cargo so only the gap-carrying one is measured.
    const SessionCounters before = session.counters();
    hub.queue_transfer(frame(input_normal_channel(), 2, ACCEL_REPORT), 0);
    session.service();

    EXPECT_EQ(moved(before, session.counters()),
              (std::vector<std::string>{"cargoes_in", "events_decoded",
                                        "sequence_gaps"}));
}

TEST(SessionCounters, AnUnknownReportMovesCargoesInAndUnknownReports) {
    FakeHub hub;
    Session session = make_session(hub);
    drive_to_ready(hub, session);

    const SessionCounters before = session.counters();
    // 0x99 is in neither the length seed nor the advertisement, so the walk
    // cannot step over it and aborts (DR-3).
    hub.queue_transfer(frame(input_normal_channel(), 0, {0x99, 0x00, 0x00}), 0);
    session.service();

    EXPECT_EQ(moved(before, session.counters()),
              (std::vector<std::string>{"cargoes_in", "unknown_reports"}));
}

TEST(SessionCounters, AForceFlushMovesFlushesRequestedAndCargoesOut) {
    FakeHub hub;
    Session session = make_session(hub);
    drive_to_ready(hub, session);

    const SessionCounters before = session.counters();
    ASSERT_TRUE(
        session.force_flush(pico_sh2::reports::SensorId::ACCELEROMETER).ok());
    session.service();

    EXPECT_EQ(moved(before, session.counters()),
              (std::vector<std::string>{"cargoes_out", "flushes_requested"}));
}

TEST(SessionCounters, AFlushCompletedMovesCargoesInAndFlushesCompleted) {
    FakeHub hub;
    Session session = make_session(hub);
    drive_to_ready(hub, session);

    const SessionCounters before = session.counters();
    hub.queue_transfer(frame(control_channel(), 1,
                             flush_completed(static_cast<std::uint8_t>(
                                 pico_sh2::reports::SensorId::ACCELEROMETER))),
                       0);
    session.service();

    EXPECT_EQ(moved(before, session.counters()),
              (std::vector<std::string>{"cargoes_in", "flushes_completed"}));
}

TEST(SessionCounters, AGetFeatureResponseMovesCargoesInAndFeatureResponses) {
    FakeHub hub;
    Session session = make_session(hub);
    drive_to_ready(hub, session);

    const SessionCounters before = session.counters();
    hub.queue_transfer(
        frame(control_channel(), 1,
              feature_response(static_cast<std::uint8_t>(
                                   pico_sh2::reports::SensorId::ACCELEROMETER),
                               10000)),
        0);
    session.service();

    EXPECT_EQ(moved(before, session.counters()),
              (std::vector<std::string>{"cargoes_in", "feature_responses"}));
}

TEST(SessionCounters, AnUnexpectedReadvertisementMovesCargoesInAndResets) {
    FakeHub hub;
    Session session = make_session(hub);
    drive_to_ready(hub, session);

    const SessionCounters before = session.counters();
    hub.queue_transfer(
        frame(pico_sh2::shtp::COMMAND_CHANNEL, 1, advertisement()), 0);
    session.service();

    EXPECT_EQ(moved(before, session.counters()),
              (std::vector<std::string>{"cargoes_in", "resets"}));
}

TEST(SessionCounters, ARefusedWriteMovesOnlyWriteErrors) {
    FakeHub hub;
    Session session = make_session(hub);
    drive_to_ready(hub, session);
    ASSERT_TRUE(
        session.get_feature(pico_sh2::reports::SensorId::ACCELEROMETER).ok());

    const SessionCounters before = session.counters();
    hub.fail_next_write(pico_sh2::Sh2Error::BUS_ERROR);
    session.service();

    EXPECT_EQ(moved(before, session.counters()),
              std::vector<std::string>{"write_errors"});
}

TEST(SessionCounters, AnFrsResponseForAnotherRecordMovesCargoesInAndStray) {
    FakeHub hub;
    Session session = make_session(hub);
    drive_to_ready(hub, session);
    ASSERT_TRUE(session.frs_read(pico_sh2::frs_type::SYSTEM_ORIENTATION).ok());
    session.service();

    const SessionCounters before = session.counters();
    hub.queue_transfer(
        frame(control_channel(), 1,
              pico_sh2::testing::frs_read_response(
                  pico_sh2::frs_type::DYNAMIC_CALIBRATION, 3, 1, 0, 1, 0)),
        0);
    session.service();

    EXPECT_EQ(moved(before, session.counters()),
              (std::vector<std::string>{"cargoes_in", "frs_stray_responses"}));
}

// The counters no case above reaches, each by the event it names.

TEST(SessionCounters, AStartupPhaseTimeoutMovesOnlyStartupTimeouts) {
    FakeHub hub;
    Session session = make_session(hub);
    pico_sh2::sh2_test::advance_to_wait_advertisement(hub, session);

    const SessionCounters before = session.counters();
    hub.advance_time(pico_sh2::sh2_test::TEST_PHASE_TIMEOUT_US + 1);
    session.service();

    EXPECT_EQ(moved(before, session.counters()),
              std::vector<std::string>{"startup_timeouts"});
}

TEST(SessionCounters, AShortControlReportMovesCargoesInAndProtocolErrors) {
    FakeHub hub;
    Session session = make_session(hub);
    drive_to_ready(hub, session);

    const SessionCounters before = session.counters();
    std::vector<std::uint8_t> response = feature_response(
        static_cast<std::uint8_t>(pico_sh2::reports::SensorId::ACCELEROMETER),
        10000);
    response.pop_back();
    hub.queue_transfer(frame(control_channel(), 1, response), 0);
    session.service();

    EXPECT_EQ(moved(before, session.counters()),
              (std::vector<std::string>{"cargoes_in", "protocol_errors"}));
}

TEST(SessionCounters,
     ATransportErrorWhileReadyMovesTransportRestartsAndResets) {
    FakeHub hub;
    Session session = make_session(hub);
    drive_to_ready(hub, session);

    const SessionCounters before = session.counters();
    // The failure applies to a read with data pending, so one is queued.
    hub.queue_transfer(frame(input_normal_channel(), 0, {}), 0);
    hub.fail_next_read(pico_sh2::Sh2Error::BUS_ERROR);
    session.service();

    EXPECT_EQ(moved(before, session.counters()),
              (std::vector<std::string>{"resets", "transport_restarts"}));
}

TEST(SessionCounters, ACargoOnAnUnclaimedChannelMovesCargoesInAndUnrouted) {
    FakeHub hub;
    Session session = make_session(hub);
    drive_to_ready(hub, session);

    const SessionCounters before = session.counters();
    // Channel 7 is past the five the advertisement assigns from base 1.
    hub.queue_transfer(frame(7, 0, {0x01, 0x02}), 0);
    session.service();

    EXPECT_EQ(moved(before, session.counters()),
              (std::vector<std::string>{"cargoes_in", "unrouted_cargoes"}));
}

TEST(SessionCounters, AKnownReportWithNoDecoderMovesCargoesInAndSkipped) {
    FakeHub hub;
    Session session = make_session(hub);
    drive_to_ready(hub, session);

    const SessionCounters before = session.counters();
    // 0x12 has a seeded length and no compiled decoder.
    hub.queue_transfer(
        frame(input_normal_channel(), 1, {0x12, 0x00, 0x00, 0x00, 0x00, 0x00}),
        0);
    session.service();

    EXPECT_EQ(moved(before, session.counters()),
              (std::vector<std::string>{"cargoes_in", "reports_skipped"}));
}

TEST(SessionCounters, AReportOverrunningItsCargoMovesCargoesInAndTruncated) {
    FakeHub hub;
    Session session = make_session(hub);
    drive_to_ready(hub, session);

    const SessionCounters before = session.counters();
    hub.queue_transfer(frame(input_normal_channel(), 1, {0x01, 0x00, 0x03}), 0);
    session.service();

    EXPECT_EQ(moved(before, session.counters()),
              (std::vector<std::string>{"cargoes_in", "truncated_reports"}));
}

TEST(SessionCounters, ACommandSentMovesCommandsSentAndCargoesOut) {
    FakeHub hub;
    Session session = make_session(hub);
    drive_to_ready(hub, session);

    const SessionCounters before = session.counters();
    ASSERT_TRUE(session.send_command(pico_sh2::command::COUNTER).ok());
    session.service();

    EXPECT_EQ(moved(before, session.counters()),
              (std::vector<std::string>{"cargoes_out", "commands_sent"}));
}

TEST(SessionCounters, AMatchedResponseMovesCargoesInAndCommandCompletions) {
    FakeHub hub;
    Session session = make_session(hub);
    drive_to_ready(hub, session);
    ASSERT_TRUE(session.send_command(pico_sh2::command::COUNTER).ok());
    session.service();

    const SessionCounters before = session.counters();
    hub.queue_transfer(frame(control_channel(), 1,
                             pico_sh2::testing::command_response(
                                 pico_sh2::command::COUNTER, 0, false)),
                       0);
    session.service();

    EXPECT_EQ(moved(before, session.counters()),
              (std::vector<std::string>{"cargoes_in", "command_completions"}));
}

TEST(SessionCounters, AnUnansweredCommandMovesOnlyCommandTimeouts) {
    FakeHub hub;
    Session session = make_session(hub);
    drive_to_ready(hub, session);
    ASSERT_TRUE(session.send_command(pico_sh2::command::COUNTER).ok());
    session.service();

    const SessionCounters before = session.counters();
    hub.advance_time(pico_sh2::sh2_test::TEST_TRANSACTION_TIMEOUT_US + 1);
    session.service();

    EXPECT_EQ(moved(before, session.counters()),
              std::vector<std::string>{"command_timeouts"});
}

TEST(SessionCounters, AFinishedFrsReadMovesCargoesInAndFrsCompletions) {
    FakeHub hub;
    Session session = make_session(hub);
    drive_to_ready(hub, session);
    ASSERT_TRUE(session.frs_read(pico_sh2::frs_type::SYSTEM_ORIENTATION).ok());
    session.service();

    const SessionCounters before = session.counters();
    // Status 3: record completed.
    hub.queue_transfer(
        frame(control_channel(), 1,
              pico_sh2::testing::frs_read_response(
                  pico_sh2::frs_type::SYSTEM_ORIENTATION, 3, 1, 0, 1, 0)),
        0);
    session.service();

    EXPECT_EQ(moved(before, session.counters()),
              (std::vector<std::string>{"cargoes_in", "frs_completions"}));
}

TEST(SessionCounters, AnUnansweredFrsReadMovesOnlyFrsTimeouts) {
    FakeHub hub;
    Session session = make_session(hub);
    drive_to_ready(hub, session);
    ASSERT_TRUE(session.frs_read(pico_sh2::frs_type::SYSTEM_ORIENTATION).ok());
    session.service();

    const SessionCounters before = session.counters();
    hub.advance_time(pico_sh2::sh2_test::TEST_TRANSACTION_TIMEOUT_US + 1);
    session.service();

    EXPECT_EQ(moved(before, session.counters()),
              std::vector<std::string>{"frs_timeouts"});
}

TEST(SessionCounters, AnErrorListMovesCargoesInAndHubErrorLists) {
    FakeHub hub;
    Session session = make_session(hub);
    drive_to_ready(hub, session);

    const SessionCounters before = session.counters();
    hub.queue_transfer(frame(pico_sh2::shtp::COMMAND_CHANNEL, 1, {0x01, 0x09}),
                       0);
    session.service();

    EXPECT_EQ(moved(before, session.counters()),
              (std::vector<std::string>{"cargoes_in", "hub_error_lists"}));
}

}  // namespace

// OR-1: every visited counter is unsigned 32-bit and wraps modulo 2^32.
TEST(SessionCounters, EveryEnumeratedCounterIsUnsigned32Bit) {
    const pico_sh2::SessionCounters counters;
    std::size_t visited = 0;
    pico_sh2::for_each_counter(
        counters,
        [&visited]([[maybe_unused]] const char* name,
                   [[maybe_unused]] const char* description, auto value) {
            static_assert(std::same_as<decltype(value), std::uint32_t>);
            ++visited;
        });
    EXPECT_EQ(visited, pico_sh2::COUNTER_COUNT);
}
