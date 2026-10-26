// SPDX-License-Identifier: MIT
// Session tests: FRS record reads and writes: one operation at a time, streamed
// responses, paced write data, and the error and deadline paths (SR-3).
#include <gtest/gtest.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <span>
#include <vector>

#include "pico_sh2/reports/sensor_id.hpp"
#include "pico_sh2/sh2/feature.hpp"
#include "pico_sh2/sh2/frs.hpp"
#include "pico_sh2/sh2/profile.hpp"
#include "pico_sh2/sh2/session.hpp"
#include "pico_sh2/sh2/session_timing.hpp"
#include "pico_sh2/testing/fake_hub.hpp"
#include "pico_sh2/testing/frame_builder.hpp"
#include "pico_sh2/util/result.hpp"
#include "session_fixture.hpp"

namespace {

// Named one at a time rather than with a using-directive: this suite
// reaches for a handful of helpers, and a budget line it does not need
// is budget spent for nothing. Each is named in the namespace that
// defines it.
using pico_sh2::sh2_test::drive_to_ready;
using pico_sh2::sh2_test::FrsCapture;
using pico_sh2::sh2_test::make_session;
using pico_sh2::testing::FakeHub;
using pico_sh2::testing::frame;
using pico_sh2::testing::frs_read_response;
using pico_sh2::testing::frs_write_response;

using pico_sh2::Session;
using pico_sh2::Sh2Error;

TEST(Session, FrsReadStreamsRecord) {
    FakeHub hub;
    Session session = make_session(hub);
    FrsCapture cap;
    session.set_frs_sink(
        pico_sh2::Session::FrsSink::bind<FrsCapture, &FrsCapture::on_frs>(
            &cap));
    drive_to_ready(hub, session);

    ASSERT_TRUE(session.frs_read(pico_sh2::frs_type::SYSTEM_ORIENTATION).ok());
    EXPECT_TRUE(session.frs_busy());
    session.service();  // flush the 0xF4 read request
    ASSERT_EQ(hub.writes().size(), 1U);
    EXPECT_EQ(hub.writes()[0].frame[4], 0xF4);

    // Two responses: a data block, then a block carrying the final words plus
    // the record-completed status.
    hub.queue_transfer(
        frame(2, 1,
              frs_read_response(pico_sh2::frs_type::SYSTEM_ORIENTATION, 0, 2, 0,
                                0x11111111, 0x22222222)),
        0);
    hub.queue_transfer(
        frame(2, 2,
              frs_read_response(pico_sh2::frs_type::SYSTEM_ORIENTATION, 3, 2, 2,
                                0x33333333, 0x44444444)),
        0);
    session.service();

    EXPECT_EQ(cap.calls, 1);
    EXPECT_EQ(cap.status, Sh2Error::NONE);
    EXPECT_EQ(cap.type, pico_sh2::frs_type::SYSTEM_ORIENTATION);
    ASSERT_EQ(cap.words.size(), 4U);
    EXPECT_EQ(cap.words[0], 0x11111111U);
    EXPECT_EQ(cap.words[3], 0x44444444U);
    EXPECT_FALSE(session.frs_busy());
    EXPECT_EQ(session.counters().frs_completions, 1U);
}

// SR-3: the record a sink is handed stays valid for the whole call, even when
// the sink chains the next read, as the sink contract allows. The read is
// issued once the sink returns; a write from inside the sink is BUSY, since
// it would need the caller's record to outlive the call.
struct Chainer {
    Session* session = nullptr;
    std::vector<std::uint32_t> before;
    std::vector<std::uint32_t> after;
    pico_sh2::Result<void> chained;
    pico_sh2::Result<void> written;
    int calls = 0;

    void on_frs(Sh2Error /*status*/, std::uint16_t /*type*/,
                std::span<const std::uint32_t> words,
                std::optional<std::uint8_t> /*device_status*/) {
        if (++calls > 1) {
            return;
        }
        before.assign(words.begin(), words.end());
        chained = session->frs_read(pico_sh2::frs_type::SYSTEM_ORIENTATION);
        const std::array<std::uint32_t, 1> record{1};
        written =
            session->frs_write(pico_sh2::frs_type::SYSTEM_ORIENTATION, record);
        after.assign(words.begin(), words.end());
    }
};

TEST(Session, AReadChainedFromTheSinkLeavesTheRecordIntact) {
    FakeHub hub;
    Session session = make_session(hub);
    Chainer chainer;
    chainer.session = &session;
    session.set_frs_sink(
        Session::FrsSink::bind<Chainer, &Chainer::on_frs>(&chainer));
    drive_to_ready(hub, session);

    ASSERT_TRUE(session.frs_read(pico_sh2::frs_type::SYSTEM_ORIENTATION).ok());
    session.service();
    hub.queue_transfer(
        frame(2, 1,
              frs_read_response(pico_sh2::frs_type::SYSTEM_ORIENTATION, 3, 2, 0,
                                0x11111111, 0x22222222)),
        0);
    session.service();

    ASSERT_EQ(chainer.before.size(), 2U);
    EXPECT_EQ(chainer.after, chainer.before);
    EXPECT_TRUE(chainer.chained.ok());
    EXPECT_EQ(chainer.written.error(), Sh2Error::BUSY);
    // The chained read went out once the sink returned.
    EXPECT_TRUE(session.frs_busy());
    session.service();
    EXPECT_EQ(hub.writes().back().frame[4], 0xF4);
}

// A sink that retries on every completion, with the TX queue full, must not
// hold service() forever: a failure completion carries no record to protect,
// so a read asked for from its sink is sent at once and its error returned.
struct RetryingSink {
    Session* session = nullptr;
    int calls = 0;
    std::vector<Sh2Error> statuses;
    std::vector<Sh2Error> retries;

    void on_frs(Sh2Error status, std::uint16_t /*type*/,
                std::span<const std::uint32_t> /*words*/,
                std::optional<std::uint8_t> /*device_status*/) {
        statuses.push_back(status);
        if (++calls > 8) {
            return;  // a hang guard for the test itself
        }
        // Fill the queue the held read would need.
        while (session
                   ->set_feature(pico_sh2::FeatureConfig{
                       .sensor = pico_sh2::reports::SensorId::ACCELEROMETER})
                   .ok()) {
        }
        const auto retried =
            session->frs_read(pico_sh2::frs_type::SYSTEM_ORIENTATION);
        retries.push_back(retried.ok() ? Sh2Error::NONE : retried.error());
    }
};

TEST(Session, ARetryOnAFailedReadDoesNotLoop) {
    FakeHub hub;
    Session session = make_session(hub);
    RetryingSink sink;
    sink.session = &session;
    session.set_frs_sink(
        Session::FrsSink::bind<RetryingSink, &RetryingSink::on_frs>(&sink));
    drive_to_ready(hub, session);

    ASSERT_TRUE(session.frs_read(pico_sh2::frs_type::SYSTEM_ORIENTATION).ok());
    session.service();
    hub.queue_transfer(
        frame(2, 1,
              frs_read_response(pico_sh2::frs_type::SYSTEM_ORIENTATION, 3, 2, 0,
                                0x11111111, 0x22222222)),
        0);
    session.service();

    // The record's sink holds a read; it cannot be queued, so the failure
    // ends at the sink once, whose retry is refused there and then.
    EXPECT_EQ(sink.calls, 2);
    ASSERT_EQ(sink.statuses.size(), 2U);
    EXPECT_EQ(sink.statuses[0], Sh2Error::NONE);
    EXPECT_EQ(sink.statuses[1], Sh2Error::QUEUE_FULL);  // the held read (SR-3)
    ASSERT_EQ(sink.retries.size(), 2U);
    EXPECT_EQ(sink.retries[0], Sh2Error::NONE);  // held behind the record
    EXPECT_NE(sink.retries[1], Sh2Error::NONE);  // refused, not held again
}

// A sink that chains a read and then restarts the session drops the read, as
// begin() drops any running operation: nothing goes out to the restarted hub.
struct ChainThenRestart {
    Session* session = nullptr;
    int calls = 0;

    void on_frs(Sh2Error /*status*/, std::uint16_t /*type*/,
                std::span<const std::uint32_t> /*words*/,
                std::optional<std::uint8_t> /*device_status*/) {
        ++calls;
        (void)session->frs_read(pico_sh2::frs_type::SYSTEM_ORIENTATION);
        session->begin();
    }
};

TEST(Session, ARestartFromTheSinkDropsTheChainedRead) {
    FakeHub hub;
    Session session = make_session(hub);
    ChainThenRestart sink;
    sink.session = &session;
    session.set_frs_sink(
        Session::FrsSink::bind<ChainThenRestart, &ChainThenRestart::on_frs>(
            &sink));
    drive_to_ready(hub, session);

    ASSERT_TRUE(session.frs_read(pico_sh2::frs_type::SYSTEM_ORIENTATION).ok());
    session.service();
    const std::size_t sent = hub.writes().size();
    hub.queue_transfer(
        frame(2, 1,
              frs_read_response(pico_sh2::frs_type::SYSTEM_ORIENTATION, 3, 2, 0,
                                0x11111111, 0x22222222)),
        0);
    session.service();

    EXPECT_EQ(sink.calls, 1);
    EXPECT_FALSE(session.frs_busy());
    EXPECT_EQ(hub.writes().size(), sent);
}

// SR-3: a completed read must cover every word below its highest. Whether the
// hub may answer non-densely is UNCONFIRMED (1000-3625 §6.3.7 does not say), so
// a gap ends the read as PROTOCOL rather than delivering a word nobody sent:
// a dropped response, or the tail of an earlier read of the same record, looks
// exactly like this.
TEST(Session, FrsReadCompletingWithAGapIsProtocol) {
    FakeHub hub;
    Session session = make_session(hub);
    FrsCapture cap;
    session.set_frs_sink(
        pico_sh2::Session::FrsSink::bind<FrsCapture, &FrsCapture::on_frs>(
            &cap));
    drive_to_ready(hub, session);

    // First read: a dense four-word record, whose words and coverage must not
    // carry into the next read.
    ASSERT_TRUE(session.frs_read(pico_sh2::frs_type::SYSTEM_ORIENTATION).ok());
    session.service();
    hub.queue_transfer(
        frame(2, 1,
              frs_read_response(pico_sh2::frs_type::SYSTEM_ORIENTATION, 0, 2, 0,
                                0x11111111, 0x22222222)),
        0);
    hub.queue_transfer(
        frame(2, 2,
              frs_read_response(pico_sh2::frs_type::SYSTEM_ORIENTATION, 3, 2, 2,
                                0x33333333, 0x44444444)),
        0);
    session.service();
    ASSERT_EQ(cap.calls, 1);
    ASSERT_EQ(cap.words.size(), 4U);

    // Second read: only the words at offset 2 are reported, leaving words 0-1
    // uncovered by this read.
    ASSERT_TRUE(session.frs_read(pico_sh2::frs_type::DYNAMIC_CALIBRATION).ok());
    session.service();
    hub.queue_transfer(
        frame(2, 3,
              frs_read_response(pico_sh2::frs_type::DYNAMIC_CALIBRATION, 3, 2,
                                2, 0xAAAAAAAA, 0xBBBBBBBB)),
        0);
    session.service();

    EXPECT_EQ(cap.calls, 2);
    EXPECT_EQ(cap.status, Sh2Error::PROTOCOL);
    EXPECT_TRUE(cap.words.empty());
    EXPECT_FALSE(session.frs_busy());
}

TEST(Session, FrsReadUnrecognizedTypeIsUnsupported) {
    FakeHub hub;
    Session session = make_session(hub);
    FrsCapture cap;
    session.set_frs_sink(
        pico_sh2::Session::FrsSink::bind<FrsCapture, &FrsCapture::on_frs>(
            &cap));
    drive_to_ready(hub, session);
    ASSERT_TRUE(session.frs_read(0x9999).ok());
    session.service();

    hub.queue_transfer(frame(2, 1, frs_read_response(0x9999, 1, 0, 0, 0, 0)),
                       0);
    session.service();
    EXPECT_EQ(cap.status, Sh2Error::UNSUPPORTED);
    EXPECT_FALSE(session.frs_busy());
}

TEST(Session, FrsReadDeviceErrorSurfacesProtocol) {
    FakeHub hub;
    Session session = make_session(hub);
    FrsCapture cap;
    session.set_frs_sink(
        pico_sh2::Session::FrsSink::bind<FrsCapture, &FrsCapture::on_frs>(
            &cap));
    drive_to_ready(hub, session);
    ASSERT_TRUE(session.frs_read(pico_sh2::frs_type::SYSTEM_ORIENTATION).ok());
    session.service();

    // Status 8 = device error (DFU flash unavailable, 1000-3625 §6.3.7): the
    // read aborts rather than accumulating or waiting for a completion that
    // never comes.
    hub.queue_transfer(
        frame(2, 1,
              frs_read_response(pico_sh2::frs_type::SYSTEM_ORIENTATION, 8, 0, 0,
                                0, 0)),
        0);
    session.service();
    EXPECT_EQ(cap.status, Sh2Error::PROTOCOL);
    EXPECT_FALSE(session.frs_busy());
}

// SR-3: a deprecated (4, 6, 7) or reserved (9-15) read status is not one the
// hub defines for a read in progress (1000-3625 §6.3.7), so the read ends at
// once with PROTOCOL rather than storing the report and running on to the
// deadline.
TEST(Session, FrsReadDeprecatedOrReservedStatusSurfacesProtocol) {
    for (const std::uint8_t status :
         {std::uint8_t{4}, std::uint8_t{6}, std::uint8_t{7}, std::uint8_t{9},
          std::uint8_t{15}}) {
        FakeHub hub;
        Session session = make_session(hub);
        FrsCapture cap;
        session.set_frs_sink(
            pico_sh2::Session::FrsSink::bind<FrsCapture, &FrsCapture::on_frs>(
                &cap));
        drive_to_ready(hub, session);
        ASSERT_TRUE(
            session.frs_read(pico_sh2::frs_type::SYSTEM_ORIENTATION).ok());
        session.service();

        hub.queue_transfer(
            frame(2, 1,
                  frs_read_response(pico_sh2::frs_type::SYSTEM_ORIENTATION,
                                    status, 1, 0, 0xAAAAAAAA, 0)),
            0);
        session.service();
        EXPECT_EQ(cap.calls, 1) << "status " << int{status};
        EXPECT_EQ(cap.status, Sh2Error::PROTOCOL) << "status " << int{status};
        EXPECT_TRUE(cap.words.empty()) << "status " << int{status};
        EXPECT_FALSE(session.frs_busy()) << "status " << int{status};
    }
}

// SR-3: busy (2) answers a read made while another FRS operation runs, and
// the hub then completes that operation rather than this one (1000-3625
// §6.3.6), so the read ends with BUSY and the hub's status, carrying none of
// the report's words.
TEST(Session, FrsReadBusyEndsTheReadWithBusy) {
    FakeHub hub;
    Session session = make_session(hub);
    FrsCapture cap;
    session.set_frs_sink(
        pico_sh2::Session::FrsSink::bind<FrsCapture, &FrsCapture::on_frs>(
            &cap));
    drive_to_ready(hub, session);
    ASSERT_TRUE(session.frs_read(pico_sh2::frs_type::SYSTEM_ORIENTATION).ok());
    session.service();

    hub.queue_transfer(
        frame(2, 1,
              frs_read_response(pico_sh2::frs_type::SYSTEM_ORIENTATION,
                                pico_sh2::frs_read_status::BUSY, 1, 0,
                                0xAAAAAAAA, 0)),
        0);
    session.service();
    EXPECT_EQ(cap.calls, 1);
    EXPECT_EQ(cap.status, Sh2Error::BUSY);
    EXPECT_EQ(cap.device_status, pico_sh2::frs_read_status::BUSY);
    EXPECT_TRUE(cap.words.empty());
    EXPECT_FALSE(session.frs_busy());
}

// SR-3: busy (2) ends a write the same way. What the hub does after a busy
// write status is UNCONFIRMED (1000-3625 §6.3.5 lists the code only), so the
// write is not left waiting on a response that may never come.
TEST(Session, FrsWriteBusyEndsTheWriteWithBusy) {
    FakeHub hub;
    Session session = make_session(hub);
    FrsCapture cap;
    session.set_frs_sink(
        pico_sh2::Session::FrsSink::bind<FrsCapture, &FrsCapture::on_frs>(
            &cap));
    drive_to_ready(hub, session);
    std::vector<std::uint32_t> record = {0xAAAA0000};
    ASSERT_TRUE(
        session.frs_write(pico_sh2::frs_type::SYSTEM_ORIENTATION, record).ok());
    session.service();

    hub.queue_transfer(
        frame(2, 1, frs_write_response(pico_sh2::frs_write_status::BUSY)), 0);
    session.service();
    EXPECT_EQ(cap.calls, 1);
    EXPECT_EQ(cap.status, Sh2Error::BUSY);
    EXPECT_EQ(cap.device_status, pico_sh2::frs_write_status::BUSY);
    EXPECT_FALSE(session.frs_busy());
}

// SR-3: record empty (5) is a completed read of zero words.
TEST(Session, FrsReadOfAnEmptyRecordCompletesWithNoWords) {
    FakeHub hub;
    Session session = make_session(hub);
    FrsCapture cap;
    session.set_frs_sink(
        pico_sh2::Session::FrsSink::bind<FrsCapture, &FrsCapture::on_frs>(
            &cap));
    drive_to_ready(hub, session);
    ASSERT_TRUE(session.frs_read(pico_sh2::frs_type::SYSTEM_ORIENTATION).ok());
    session.service();

    hub.queue_transfer(
        frame(2, 1,
              frs_read_response(pico_sh2::frs_type::SYSTEM_ORIENTATION, 5, 0, 0,
                                0, 0)),
        0);
    session.service();
    EXPECT_EQ(cap.calls, 1);
    EXPECT_EQ(cap.status, Sh2Error::NONE);
    EXPECT_TRUE(cap.words.empty());
    EXPECT_EQ(session.counters().frs_completions, 1U);
}

TEST(Session, FrsWriteRoundTrip) {
    FakeHub hub;
    Session session = make_session(hub);
    FrsCapture cap;
    session.set_frs_sink(
        pico_sh2::Session::FrsSink::bind<FrsCapture, &FrsCapture::on_frs>(
            &cap));
    drive_to_ready(hub, session);

    std::vector<std::uint32_t> record = {0xAAAA0000, 0xBBBB1111, 0xCCCC2222};
    ASSERT_TRUE(
        session.frs_write(pico_sh2::frs_type::SYSTEM_ORIENTATION, record).ok());
    session.service();  // flush the 0xF7 write request
    ASSERT_EQ(hub.writes().size(), 1U);
    EXPECT_EQ(hub.writes()[0].frame[4], 0xF7);

    // Ready -> first data chunk (words 0,1) sent within the same service().
    hub.queue_transfer(frame(2, 1, frs_write_response(4)), 0);
    session.service();
    ASSERT_EQ(hub.writes().size(), 2U);
    EXPECT_EQ(hub.writes()[1].frame[4], 0xF6);

    // Words received -> second data chunk (word 2).
    hub.queue_transfer(frame(2, 2, frs_write_response(0)), 0);
    session.service();
    ASSERT_EQ(hub.writes().size(), 3U);

    // Words received for the last chunk -> nothing more to send.
    hub.queue_transfer(frame(2, 3, frs_write_response(0)), 0);
    session.service();
    EXPECT_EQ(hub.writes().size(), 3U);

    // Write completed.
    hub.queue_transfer(frame(2, 4, frs_write_response(3)), 0);
    session.service();
    EXPECT_FALSE(session.frs_busy());
    EXPECT_EQ(cap.status, Sh2Error::NONE);
    EXPECT_EQ(session.counters().frs_completions, 1U);
}

// SR-3: a failed write ends with the device's own status, which a consumer
// needs to tell a full memory from a read-only record. A status the session
// ended on its own, a timeout, carries none.
TEST(Session, FrsWriteFailureSurfacesTheDeviceStatus) {
    FakeHub hub;
    Session session = make_session(hub);
    FrsCapture cap;
    session.set_frs_sink(
        pico_sh2::Session::FrsSink::bind<FrsCapture, &FrsCapture::on_frs>(
            &cap));
    drive_to_ready(hub, session);
    std::vector<std::uint32_t> record = {0xAAAA0000};
    ASSERT_TRUE(
        session.frs_write(pico_sh2::frs_type::SYSTEM_ORIENTATION, record).ok());
    session.service();

    hub.queue_transfer(
        frame(2, 1,
              frs_write_response(pico_sh2::frs_write_status::MEMORY_FULL)),
        0);
    session.service();
    EXPECT_EQ(cap.calls, 1);
    EXPECT_EQ(cap.status, Sh2Error::PROTOCOL);
    EXPECT_EQ(cap.device_status, pico_sh2::frs_write_status::MEMORY_FULL);
    EXPECT_FALSE(session.frs_busy());
}

// SR-3: record valid (8) reports the validation, not the write, so the write
// waits for write completed (3), which ends it with that status.
TEST(Session, ARecordValidWriteWaitsForTheWriteToComplete) {
    FakeHub hub;
    Session session = make_session(hub);
    FrsCapture cap;
    session.set_frs_sink(
        pico_sh2::Session::FrsSink::bind<FrsCapture, &FrsCapture::on_frs>(
            &cap));
    drive_to_ready(hub, session);
    std::vector<std::uint32_t> record = {0xAAAA0000};
    ASSERT_TRUE(
        session.frs_write(pico_sh2::frs_type::SYSTEM_ORIENTATION, record).ok());
    session.service();

    hub.queue_transfer(
        frame(2, 1,
              frs_write_response(pico_sh2::frs_write_status::RECORD_VALID)),
        0);
    session.service();
    EXPECT_EQ(cap.calls, 0);
    EXPECT_TRUE(session.frs_busy());

    hub.queue_transfer(
        frame(2, 2,
              frs_write_response(pico_sh2::frs_write_status::WRITE_COMPLETED)),
        0);
    session.service();
    EXPECT_EQ(cap.calls, 1);
    EXPECT_EQ(cap.status, Sh2Error::NONE);
    EXPECT_EQ(cap.device_status, pico_sh2::frs_write_status::WRITE_COMPLETED);
    EXPECT_FALSE(session.frs_busy());
}

// SR-3: a record the hub validates and never finishes writing is not written.
TEST(Session, ARecordValidWriteThatNeverCompletesTimesOut) {
    FakeHub hub;
    Session session = make_session(hub);
    FrsCapture cap;
    session.set_frs_sink(
        pico_sh2::Session::FrsSink::bind<FrsCapture, &FrsCapture::on_frs>(
            &cap));
    drive_to_ready(hub, session);
    std::vector<std::uint32_t> record = {0xAAAA0000};
    ASSERT_TRUE(
        session.frs_write(pico_sh2::frs_type::SYSTEM_ORIENTATION, record).ok());
    session.service();

    hub.queue_transfer(
        frame(2, 1,
              frs_write_response(pico_sh2::frs_write_status::RECORD_VALID)),
        0);
    session.service();
    hub.advance_time(pico_sh2::sh2_test::TEST_TRANSACTION_TIMEOUT_US + 1);
    session.service();
    EXPECT_EQ(cap.calls, 1);
    EXPECT_EQ(cap.status, Sh2Error::TIMEOUT);
    EXPECT_FALSE(cap.device_status.has_value());
}

// SR-3: a write the hub answers record invalid (9) is not written, and the
// status byte says why.
TEST(Session, AWriteAnsweredRecordInvalidFails) {
    FakeHub hub;
    Session session = make_session(hub);
    FrsCapture cap;
    session.set_frs_sink(
        pico_sh2::Session::FrsSink::bind<FrsCapture, &FrsCapture::on_frs>(
            &cap));
    drive_to_ready(hub, session);
    std::vector<std::uint32_t> record = {0xAAAA0000};
    ASSERT_TRUE(
        session.frs_write(pico_sh2::frs_type::SYSTEM_ORIENTATION, record).ok());
    session.service();

    hub.queue_transfer(
        frame(2, 1,
              frs_write_response(pico_sh2::frs_write_status::RECORD_INVALID)),
        0);
    session.service();
    EXPECT_EQ(cap.calls, 1);
    EXPECT_EQ(cap.status, Sh2Error::PROTOCOL);
    EXPECT_EQ(cap.device_status, pico_sh2::frs_write_status::RECORD_INVALID);
    EXPECT_FALSE(session.frs_busy());
}

// SR-3: a read the hub completes carries the hub's completed status (3).
TEST(Session, ACompletedFrsReadCarriesTheDeviceStatus) {
    FakeHub hub;
    Session session = make_session(hub);
    FrsCapture cap;
    session.set_frs_sink(
        pico_sh2::Session::FrsSink::bind<FrsCapture, &FrsCapture::on_frs>(
            &cap));
    drive_to_ready(hub, session);
    ASSERT_TRUE(session.frs_read(pico_sh2::frs_type::SYSTEM_ORIENTATION).ok());
    session.service();

    hub.queue_transfer(
        frame(2, 1,
              frs_read_response(pico_sh2::frs_type::SYSTEM_ORIENTATION, 3, 1, 0,
                                0x11111111, 0)),
        0);
    session.service();
    EXPECT_EQ(cap.status, Sh2Error::NONE);
    EXPECT_EQ(cap.device_status, pico_sh2::frs_read_status::RECORD_COMPLETED);
}

TEST(Session, AnFrsTimeoutCarriesNoDeviceStatus) {
    FakeHub hub;
    Session session = make_session(hub);
    FrsCapture cap;
    session.set_frs_sink(
        pico_sh2::Session::FrsSink::bind<FrsCapture, &FrsCapture::on_frs>(
            &cap));
    drive_to_ready(hub, session);
    ASSERT_TRUE(session.frs_read(pico_sh2::frs_type::SYSTEM_ORIENTATION).ok());
    session.service();
    hub.advance_time(500'001);
    session.service();
    EXPECT_EQ(cap.status, Sh2Error::TIMEOUT);
    EXPECT_FALSE(cap.device_status.has_value());
}

TEST(Session, FrsBusyRejectsSecondOperation) {
    FakeHub hub;
    Session session = make_session(hub);
    drive_to_ready(hub, session);
    ASSERT_TRUE(session.frs_read(pico_sh2::frs_type::SYSTEM_ORIENTATION).ok());
    std::vector<std::uint32_t> empty;
    EXPECT_EQ(session.frs_write(pico_sh2::frs_type::DYNAMIC_CALIBRATION, empty)
                  .error(),
              Sh2Error::BUSY);
}

TEST(Session, FrsReadTimesOut) {
    FakeHub hub;
    Session session = make_session(hub);
    FrsCapture cap;
    session.set_frs_sink(
        pico_sh2::Session::FrsSink::bind<FrsCapture, &FrsCapture::on_frs>(
            &cap));
    drive_to_ready(hub, session);
    ASSERT_TRUE(session.frs_read(pico_sh2::frs_type::SYSTEM_ORIENTATION).ok());

    hub.advance_time(500'001);  // past frs_timeout_us
    session.service();
    // Ended here, but held while the hub may still be answering it.
    EXPECT_TRUE(session.frs_busy());
    EXPECT_EQ(cap.status, Sh2Error::TIMEOUT);
    EXPECT_EQ(session.counters().frs_timeouts, 1U);
    session.service();
    EXPECT_EQ(session.counters().frs_timeouts, 1U);
}

// TR-1: the longest FRS timeout wraps to the past for neither a read nor a
// write.
TEST(Session, TheLongestFrsReadTimeoutDoesNotWrap) {
    FakeHub hub;
    pico_sh2::SessionTiming timing = pico_sh2::sh2_test::test_timing();
    timing.frs_timeout_us = std::numeric_limits<std::uint64_t>::max();
    Session session{hub, timing, pico_sh2::testing::seeded_channels()};
    drive_to_ready(hub, session);
    ASSERT_TRUE(session.frs_read(pico_sh2::frs_type::SYSTEM_ORIENTATION).ok());
    hub.advance_time(1);
    session.service();
    EXPECT_TRUE(session.frs_busy());
    EXPECT_EQ(session.counters().frs_timeouts, 0U);
}

TEST(Session, TheLongestFrsWriteTimeoutDoesNotWrap) {
    FakeHub hub;
    pico_sh2::SessionTiming timing = pico_sh2::sh2_test::test_timing();
    timing.frs_timeout_us = std::numeric_limits<std::uint64_t>::max();
    Session session{hub, timing, pico_sh2::testing::seeded_channels()};
    drive_to_ready(hub, session);
    std::vector<std::uint32_t> record = {0xAAAA0000};
    ASSERT_TRUE(
        session.frs_write(pico_sh2::frs_type::SYSTEM_ORIENTATION, record).ok());
    hub.advance_time(1);
    session.service();
    EXPECT_TRUE(session.frs_busy());
    EXPECT_EQ(session.counters().frs_timeouts, 0U);
}

TEST(Session, FrsRequestBeforeReadyIsNotReady) {
    FakeHub hub;
    Session session = make_session(hub);
    EXPECT_EQ(session.frs_read(pico_sh2::frs_type::SYSTEM_ORIENTATION).error(),
              Sh2Error::NOT_READY);
}

// SR-3: a record longer than the profile's FRS buffer ends the read with
// TABLE_FULL. Completing it with the words that fitted would hand the consumer
// a truncated record marked as whole.
TEST(Session, FrsReadLongerThanTheBufferEndsWithTableFull) {
    FakeHub hub;
    Session session = make_session(hub);
    FrsCapture cap;
    session.set_frs_sink(
        pico_sh2::Session::FrsSink::bind<FrsCapture, &FrsCapture::on_frs>(
            &cap));
    drive_to_ready(hub, session);
    constexpr auto TYPE = pico_sh2::frs_type::SYSTEM_ORIENTATION;
    ASSERT_TRUE(session.frs_read(TYPE).ok());
    session.service();

    // The last word the buffer holds, then one past it.
    constexpr auto LAST =
        static_cast<std::uint16_t>(pico_sh2::ActiveProfile::FRS_MAX_WORDS - 1);
    hub.queue_transfer(frame(2, 1, frs_read_response(TYPE, 0, 1, LAST, 1, 0)),
                       0);
    hub.queue_transfer(
        frame(2, 2, frs_read_response(TYPE, 3, 1, LAST + 1, 2, 0)), 0);
    session.service();

    EXPECT_EQ(cap.calls, 1);
    EXPECT_EQ(cap.status, Sh2Error::TABLE_FULL);
    EXPECT_FALSE(cap.device_status.has_value());  // the session ended it
    EXPECT_TRUE(cap.words.empty());
    EXPECT_FALSE(session.frs_busy());
    EXPECT_EQ(session.counters().frs_completions, 0U);
}

// SR-3: a response naming another record is the tail of an earlier read that
// already ended. It is counted and never written into the running read.
TEST(Session, AStrayResponseFromAnEarlierReadIsCountedNotApplied) {
    FakeHub hub;
    Session session = make_session(hub);
    FrsCapture cap;
    session.set_frs_sink(
        pico_sh2::Session::FrsSink::bind<FrsCapture, &FrsCapture::on_frs>(
            &cap));
    drive_to_ready(hub, session);
    constexpr auto FIRST = pico_sh2::frs_type::SYSTEM_ORIENTATION;
    constexpr auto SECOND = pico_sh2::frs_type::DYNAMIC_CALIBRATION;

    // The first read times out while the hub is still answering it.
    ASSERT_TRUE(session.frs_read(FIRST).ok());
    session.service();
    hub.advance_time(pico_sh2::sh2_test::TEST_TRANSACTION_TIMEOUT_US);
    session.service();
    ASSERT_EQ(cap.calls, 1);
    ASSERT_EQ(cap.status, Sh2Error::TIMEOUT);
    // No tail arrives inside the hold, so it lapses and the second read runs;
    // the first read's tail then arrives late.
    hub.advance_time(pico_sh2::sh2_test::TEST_TRANSACTION_TIMEOUT_US);
    session.service();

    ASSERT_TRUE(session.frs_read(SECOND).ok());
    session.service();
    hub.queue_transfer(
        frame(2, 1, frs_read_response(FIRST, 3, 2, 0, 0xDEAD0000, 0xDEAD0001)),
        0);
    session.service();
    EXPECT_EQ(session.counters().frs_stray_responses, 1U);
    EXPECT_EQ(cap.calls, 1);
    EXPECT_TRUE(session.frs_busy());

    hub.queue_transfer(
        frame(2, 2, frs_read_response(SECOND, 3, 1, 0, 0x5EC0, 0)), 0);
    session.service();
    EXPECT_EQ(cap.calls, 2);
    EXPECT_EQ(cap.status, Sh2Error::NONE);
    EXPECT_EQ(cap.type, SECOND);
    EXPECT_EQ(cap.words, (std::vector<std::uint32_t>{0x5EC0}));
}

}  // namespace

// SR-3: one FRS operation at a time (1000-3625 §6.3.7). A read the session
// ended itself -- here past its deadline -- may still be running on the hub, so
// a new operation is held until the hub's tail ends. The tail of a timed-out
// read of the same record would otherwise complete the retry with its words.
TEST(Session, ARetryAfterATimeoutIsHeldUntilTheTailEnds) {
    FakeHub hub;
    Session session = make_session(hub);
    FrsCapture cap;
    session.set_frs_sink(
        pico_sh2::Session::FrsSink::bind<FrsCapture, &FrsCapture::on_frs>(
            &cap));
    drive_to_ready(hub, session);
    constexpr auto TYPE = pico_sh2::frs_type::SYSTEM_ORIENTATION;
    ASSERT_TRUE(session.frs_read(TYPE).ok());
    session.service();
    hub.queue_transfer(
        frame(2, 1, frs_read_response(TYPE, 0, 2, 0, 0x11111111, 0x22222222)),
        0);
    session.service();
    hub.advance_time(pico_sh2::sh2_test::TEST_TRANSACTION_TIMEOUT_US + 1);
    session.service();
    ASSERT_EQ(cap.status, Sh2Error::TIMEOUT);

    // The hub is still sending: a retry is refused while it does.
    EXPECT_TRUE(session.frs_busy());
    EXPECT_EQ(session.frs_read(TYPE).error(), Sh2Error::BUSY);
    const std::uint32_t stray = session.counters().frs_stray_responses;
    hub.queue_transfer(
        frame(2, 2, frs_read_response(TYPE, 3, 2, 2, 0x33333333, 0x44444444)),
        0);
    session.service();
    EXPECT_EQ(session.counters().frs_stray_responses, stray + 1);
    EXPECT_EQ(cap.calls, 1);

    // The tail ended with the hub's completion, so the retry runs on its own.
    EXPECT_FALSE(session.frs_busy());
    ASSERT_TRUE(session.frs_read(TYPE).ok());
    session.service();
    hub.queue_transfer(
        frame(2, 3, frs_read_response(TYPE, 3, 2, 0, 0x55555555, 0x66666666)),
        0);
    session.service();
    EXPECT_EQ(cap.calls, 2);
    EXPECT_EQ(cap.status, Sh2Error::NONE);
    ASSERT_EQ(cap.words.size(), 2U);
    EXPECT_EQ(cap.words[0], 0x55555555U);
}

// SR-3: a hub that never finishes the tail holds the next operation for one
// FRS timeout, not for ever.
TEST(Session, AnUnfinishedTailHoldsForOneTimeout) {
    FakeHub hub;
    Session session = make_session(hub);
    drive_to_ready(hub, session);
    constexpr auto TYPE = pico_sh2::frs_type::SYSTEM_ORIENTATION;
    ASSERT_TRUE(session.frs_read(TYPE).ok());
    hub.advance_time(pico_sh2::sh2_test::TEST_TRANSACTION_TIMEOUT_US + 1);
    session.service();
    EXPECT_TRUE(session.frs_busy());
    hub.advance_time(pico_sh2::sh2_test::TEST_TRANSACTION_TIMEOUT_US - 1);
    session.service();
    EXPECT_TRUE(session.frs_busy());
    hub.advance_time(1);
    session.service();
    EXPECT_FALSE(session.frs_busy());
    EXPECT_TRUE(session.frs_read(TYPE).ok());
}

// SR-3: a hub reset ends whatever the hub was doing, so it ends the hold too:
// the hub announcing itself again, and a begin() the caller asks for.
TEST(Session, AHubResetEndsTheHold) {
    FakeHub hub;
    Session session = make_session(hub);
    drive_to_ready(hub, session);
    ASSERT_TRUE(session.frs_read(pico_sh2::frs_type::SYSTEM_ORIENTATION).ok());
    hub.advance_time(pico_sh2::sh2_test::TEST_TRANSACTION_TIMEOUT_US + 1);
    session.service();
    ASSERT_TRUE(session.frs_busy());
    hub.queue_transfer(frame(0, 0, pico_sh2::testing::advertisement()), 0);
    session.service();
    EXPECT_FALSE(session.frs_busy());
}

TEST(Session, ABeginEndsTheHold) {
    FakeHub hub;
    Session session = make_session(hub);
    drive_to_ready(hub, session);
    ASSERT_TRUE(session.frs_read(pico_sh2::frs_type::SYSTEM_ORIENTATION).ok());
    hub.advance_time(pico_sh2::sh2_test::TEST_TRANSACTION_TIMEOUT_US + 1);
    session.service();
    ASSERT_TRUE(session.frs_busy());
    session.begin();
    EXPECT_FALSE(session.frs_busy());
}

// SR-3: the hold ends on the held record's own last status, and on nothing
// else -- not a response that says more follows, not another record's.
TEST(Session, OnlyTheHeldRecordsLastStatusEndsTheHold) {
    FakeHub hub;
    Session session = make_session(hub);
    drive_to_ready(hub, session);
    constexpr auto TYPE = pico_sh2::frs_type::SYSTEM_ORIENTATION;
    constexpr auto OTHER = pico_sh2::frs_type::DYNAMIC_CALIBRATION;
    ASSERT_TRUE(session.frs_read(TYPE).ok());
    hub.advance_time(pico_sh2::sh2_test::TEST_TRANSACTION_TIMEOUT_US + 1);
    session.service();
    ASSERT_TRUE(session.frs_busy());
    const std::uint32_t stray = session.counters().frs_stray_responses;

    hub.queue_transfer(frame(2, 1, frs_read_response(TYPE, 0, 2, 0, 1, 2)), 0);
    session.service();
    EXPECT_TRUE(session.frs_busy());
    hub.queue_transfer(frame(2, 2, frs_read_response(OTHER, 3, 2, 0, 3, 4)), 0);
    session.service();
    EXPECT_TRUE(session.frs_busy());
    EXPECT_EQ(session.counters().frs_stray_responses, stray + 2);

    hub.queue_transfer(frame(2, 3, frs_read_response(TYPE, 3, 2, 2, 5, 6)), 0);
    session.service();
    EXPECT_FALSE(session.frs_busy());
    EXPECT_EQ(session.counters().frs_stray_responses, stray + 3);
}

// SR-3: a write whose next data report cannot be queued ends QUEUE_FULL, and
// the hub, still waiting for that data, holds the next operation.
TEST(Session, AWriteThatCannotBeFedIsHeld) {
    FakeHub hub;
    Session session = make_session(hub);
    FrsCapture cap;
    session.set_frs_sink(
        pico_sh2::Session::FrsSink::bind<FrsCapture, &FrsCapture::on_frs>(
            &cap));
    drive_to_ready(hub, session);
    std::vector<std::uint32_t> record = {1, 2, 3, 4};
    ASSERT_TRUE(
        session.frs_write(pico_sh2::frs_type::SYSTEM_ORIENTATION, record).ok());
    session.service();  // the write request goes out
    // Fill the outbound queue without servicing, so nothing flushes it.
    while (session.get_feature(pico_sh2::reports::SensorId::GRAVITY).ok()) {
    }
    hub.queue_transfer(frame(2, 1, frs_write_response(4)), 0);  // ready
    session.service();
    EXPECT_EQ(cap.status, Sh2Error::QUEUE_FULL);
    EXPECT_TRUE(session.frs_busy());
    EXPECT_EQ(session.frs_read(pico_sh2::frs_type::SYSTEM_ORIENTATION).error(),
              Sh2Error::BUSY);
}

// SR-3: a read ended for a full buffer leaves the hub sending the rest of the
// record, so it is held behind like a timeout.
TEST(Session, ATableFullReadHoldsBehindItsTail) {
    FakeHub hub;
    Session session = make_session(hub);
    drive_to_ready(hub, session);
    constexpr auto TYPE = pico_sh2::frs_type::SYSTEM_ORIENTATION;
    constexpr auto PAST =
        static_cast<std::uint16_t>(pico_sh2::ActiveProfile::FRS_MAX_WORDS);
    ASSERT_TRUE(session.frs_read(TYPE).ok());
    session.service();
    hub.queue_transfer(frame(2, 1, frs_read_response(TYPE, 0, 2, PAST, 1, 2)),
                       0);
    session.service();
    EXPECT_TRUE(session.frs_busy());
    EXPECT_EQ(session.frs_read(TYPE).error(), Sh2Error::BUSY);
    hub.queue_transfer(
        frame(2, 2, frs_read_response(TYPE, 3, 2, PAST + 2, 3, 4)), 0);
    session.service();
    EXPECT_FALSE(session.frs_busy());
}

// SR-3: record valid (8) is not a held write's last status; write completed
// (3) still follows it.
TEST(Session, ARecordValidStatusDoesNotEndAHeldWrite) {
    FakeHub hub;
    Session session = make_session(hub);
    drive_to_ready(hub, session);
    std::vector<std::uint32_t> record = {0xAAAA0000};
    ASSERT_TRUE(
        session.frs_write(pico_sh2::frs_type::SYSTEM_ORIENTATION, record).ok());
    hub.advance_time(pico_sh2::sh2_test::TEST_TRANSACTION_TIMEOUT_US + 1);
    session.service();
    EXPECT_TRUE(session.frs_busy());
    hub.queue_transfer(
        frame(2, 1,
              frs_write_response(pico_sh2::frs_write_status::RECORD_VALID)),
        0);
    session.service();
    EXPECT_TRUE(session.frs_busy());
    hub.queue_transfer(
        frame(2, 2,
              frs_write_response(pico_sh2::frs_write_status::WRITE_COMPLETED)),
        0);
    session.service();
    EXPECT_FALSE(session.frs_busy());
}

// SR-3: a timed-out write is held behind too; its tail is the hub's write
// statuses, which name no record, until one ends the write.
TEST(Session, ATimedOutWriteHoldsUntilTheHubEndsIt) {
    FakeHub hub;
    Session session = make_session(hub);
    drive_to_ready(hub, session);
    std::vector<std::uint32_t> record = {0xAAAA0000};
    ASSERT_TRUE(
        session.frs_write(pico_sh2::frs_type::SYSTEM_ORIENTATION, record).ok());
    hub.advance_time(pico_sh2::sh2_test::TEST_TRANSACTION_TIMEOUT_US + 1);
    session.service();
    EXPECT_TRUE(session.frs_busy());
    const std::uint32_t stray = session.counters().frs_stray_responses;
    const std::uint32_t unrouted = session.counters().unrouted_cargoes;
    hub.queue_transfer(frame(2, 1, frs_write_response(4)), 0);  // ready
    session.service();
    EXPECT_TRUE(session.frs_busy());
    hub.queue_transfer(frame(2, 2, frs_write_response(3)), 0);  // completed
    session.service();
    EXPECT_FALSE(session.frs_busy());
    // The ended write's tail is stray, like a read's, not an unrouted cargo.
    EXPECT_EQ(session.counters().frs_stray_responses, stray + 2);
    EXPECT_EQ(session.counters().unrouted_cargoes, unrouted);
}
