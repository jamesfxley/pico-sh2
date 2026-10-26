// SPDX-License-Identifier: MIT
// Session tests: splitting batched input cargoes into events, reconstructing
// their times (SR-4), and applying decode-time Q overrides (DR-2).
#include <gtest/gtest.h>

#include <array>
#include <cstdint>
#include <vector>

#include "pico_sh2/reports/decoders.hpp"
#include "pico_sh2/reports/metadata_q.hpp"
#include "pico_sh2/reports/q_override.hpp"
#include "pico_sh2/reports/q_point.hpp"
#include "pico_sh2/reports/sensor_event.hpp"
#include "pico_sh2/reports/sensor_id.hpp"
#include "pico_sh2/sh2/session.hpp"
#include "pico_sh2/sh2/startup_phase.hpp"
#include "pico_sh2/testing/fake_hub.hpp"
#include "pico_sh2/testing/frame_builder.hpp"
#include "pico_sh2/util/result.hpp"
#include "session_fixture.hpp"

namespace {

// Named one at a time rather than with a using-directive: this suite
// reaches for a handful of helpers, and a budget line it does not need
// is budget spent for nothing. Each is named in the namespace that
// defines it.
using pico_sh2::reports::GyroIntegratedSample;
using pico_sh2::reports::Vec3;
using pico_sh2::sh2_test::advance_to_wait_advertisement;
using pico_sh2::sh2_test::drive_to_ready;
using pico_sh2::sh2_test::EventCapture;
using pico_sh2::sh2_test::make_session;
using pico_sh2::testing::advertisement;
using pico_sh2::testing::FakeHub;
using pico_sh2::testing::frame;
using pico_sh2::testing::initialize_response;
using pico_sh2::testing::input_gyro_rv_channel;
using pico_sh2::testing::input_normal_channel;
using pico_sh2::testing::input_wake_channel;

using pico_sh2::Session;
using pico_sh2::SessionState;
using pico_sh2::StartupPhase;

TEST(Session, ReportQOverrideRejectsOutOfRangeQ) {
    FakeHub hub;
    Session session = make_session(hub);
    // A Q beyond the representable maximum would make the decoder's 2^Q shift
    // undefined; the override is refused rather than installed (DR-2).
    const auto accel =
        static_cast<std::uint8_t>(pico_sh2::reports::SensorId::ACCELEROMETER);
    EXPECT_FALSE(session.set_report_q(
        accel,
        pico_sh2::reports::ReportQ{
            static_cast<std::uint8_t>(pico_sh2::reports::MAX_Q_POINT + 1), 0}));
    EXPECT_FALSE(
        session.set_report_q(accel, pico_sh2::reports::ReportQ{0, 200}));
    EXPECT_FALSE(session.report_q_overrides().has(accel));
    // The representable maximum itself is accepted.
    EXPECT_TRUE(session.set_report_q(
        accel,
        pico_sh2::reports::ReportQ{
            static_cast<std::uint8_t>(pico_sh2::reports::MAX_Q_POINT), 0}));
}

TEST(Session, DeliversInputEventsWithReconstructedTime) {
    FakeHub hub;
    Session session = make_session(hub);
    EventCapture cap;
    session.set_event_sink(
        pico_sh2::reports::EventSink::bind<EventCapture,
                                           &EventCapture::on_event>(&cap));
    drive_to_ready(hub, session);

    // Input cargo on ch3: a base timestamp (delta 10 ticks) then one
    // accelerometer report with delay 5 ticks. The base delta is subtracted and
    // the delay added: event_us = edge - 100*10 + 100*5 (SR-4).
    std::vector<std::uint8_t> payload = {
        0xFB, 0x0A, 0x00, 0x00, 0x00,  // base delta = 10
        0x01, 0x00, 0x00, 0x05,        // accel: seq, status, delay_lsb = 5
        0x00, 0x01, 0x00, 0x02, 0x00, 0xFF};
    hub.queue_transfer(frame(input_normal_channel(), 0, payload), 1'000'000);
    session.service();

    ASSERT_EQ(cap.events.size(), 1U);
    EXPECT_EQ(cap.events[0].sensor, pico_sh2::reports::SensorId::ACCELEROMETER);
    EXPECT_EQ(cap.events[0].event_us, 1'000'000U - 500U);
    EXPECT_EQ(session.counters().events_decoded, 1U);
}

TEST(Session, ReportQOverrideAppliesAndSurvivesReset) {
    FakeHub hub;
    Session session = make_session(hub);
    EventCapture cap;
    session.set_event_sink(
        pico_sh2::reports::EventSink::bind<EventCapture,
                                           &EventCapture::on_event>(&cap));

    // Override accelerometer to Q6 before READY: X = 256 / 2^6 = 4.0 instead of
    // the compiled Q8's 1.0 (DR-2).
    ASSERT_TRUE(session.set_report_q(
        static_cast<std::uint8_t>(pico_sh2::reports::SensorId::ACCELEROMETER),
        pico_sh2::reports::ReportQ{6, 0}));
    drive_to_ready(hub, session);

    std::vector<std::uint8_t> accel = {0x01, 0x00, 0x00, 0x00, 0x00,
                                       0x01, 0x00, 0x00, 0x00, 0x00};
    hub.queue_transfer(frame(input_normal_channel(), 0, accel), 0);
    session.service();
    ASSERT_EQ(cap.events.size(), 1U);
    EXPECT_FLOAT_EQ(std::get<Vec3>(cap.events[0].payload).x, 4.0F);

    // An unexpected advertisement restarts the handshake (SR-5); the override
    // describes the part, not session state, so it must persist.
    hub.queue_transfer(frame(0, 1, advertisement()), 0);
    session.service();
    hub.queue_transfer(frame(1, 1, {0x01}), 0);
    hub.queue_transfer(frame(2, 1, initialize_response(0)), 0);
    session.service();
    ASSERT_TRUE(session.ready());

    cap.events.clear();
    hub.queue_transfer(frame(input_normal_channel(), 1, accel), 0);
    session.service();
    ASSERT_EQ(cap.events.size(), 1U);
    EXPECT_FLOAT_EQ(std::get<Vec3>(cap.events[0].payload).x, 4.0F);
}

TEST(Session, ApplySensorMetadataInstallsRecordQ) {
    FakeHub hub;
    Session session = make_session(hub);
    EventCapture cap;
    session.set_event_sink(
        pico_sh2::reports::EventSink::bind<EventCapture,
                                           &EventCapture::on_event>(&cap));
    drive_to_ready(hub, session);

    // An accelerometer sensor-metadata record (0xE302) that publishes Q6 in its
    // word 7 low half. Applying it overrides the compiled Q8, so X = 256 / 2^6
    // decodes as 4.0 rather than 1.0 (DR-2).
    std::array<std::uint32_t, 9> record{};
    record[pico_sh2::reports::METADATA_Q_WORD] = 6;  // Q point 1 = 6, Q2 = 0
    ASSERT_TRUE(session.apply_sensor_metadata(0xE302, record));

    std::vector<std::uint8_t> accel = {0x01, 0x00, 0x00, 0x00, 0x00,
                                       0x01, 0x00, 0x00, 0x00, 0x00};
    hub.queue_transfer(frame(input_normal_channel(), 0, accel), 0);
    session.service();
    ASSERT_EQ(cap.events.size(), 1U);
    EXPECT_FLOAT_EQ(std::get<Vec3>(cap.events[0].payload).x, 4.0F);

    // A record type outside the decoded set is refused without touching state.
    EXPECT_FALSE(session.apply_sensor_metadata(0x2D3E, record));
}

TEST(Session, DeliversGyroIntegratedFromTheGyroRvChannel) {
    FakeHub hub;
    Session session = make_session(hub);
    EventCapture cap;
    session.set_event_sink(
        pico_sh2::reports::EventSink::bind<EventCapture,
                                           &EventCapture::on_event>(&cap));
    drive_to_ready(hub, session);

    // The gyro-RV input channel: one 14-byte gyro-integrated RV (real = 1.0
    // Q14, x-rate = 1.0 Q10). Its time is the HINTN edge directly (no delay
    // field).
    std::vector<std::uint8_t> payload = {
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  // i, j, k = 0
        0x00, 0x40,                          // real = 0x4000 = 1.0 (Q14)
        0x00, 0x04,                          // ang vel x = 0x0400 = 1.0 (Q10)
        0x00, 0x00, 0x00, 0x00};             // y, z = 0
    hub.queue_transfer(frame(input_gyro_rv_channel(), 0, payload), 2'000'000);
    session.service();

    ASSERT_EQ(cap.events.size(), 1U);
    EXPECT_EQ(cap.events[0].kind(),
              pico_sh2::reports::EventKind::GYRO_INTEGRATED);
    EXPECT_EQ(cap.events[0].event_us, 2'000'000U);
    EXPECT_FLOAT_EQ(
        std::get<GyroIntegratedSample>(cap.events[0].payload).unit.real, 1.0F);
}

TEST(Session, InputBeforeReadyIsNotDecoded) {
    FakeHub hub;
    Session session = make_session(hub);
    EventCapture cap;
    session.set_event_sink(
        pico_sh2::reports::EventSink::bind<EventCapture,
                                           &EventCapture::on_event>(&cap));
    advance_to_wait_advertisement(hub, session);

    // An input cargo arriving before READY is counted as unrouted, not decoded.
    hub.queue_transfer(
        frame(input_normal_channel(), 0, {0x10, 0x00, 0x00, 0x00, 0x00}), 0);
    session.service();
    EXPECT_TRUE(cap.events.empty());
    EXPECT_EQ(session.counters().events_decoded, 0U);
    EXPECT_GE(session.counters().unrouted_cargoes, 1U);
}

// SR-9: a sensor configured as a wake-up sensor (1000-3625 §6.5.1, feature
// flag bit 2) has its reports delivered on the wake input channel instead of
// the normal one. Both carry identical batched cargoes, so the router must
// treat them the same.
TEST(Session, DeliversInputFromTheWakeChannel) {
    FakeHub hub;
    Session session = make_session(hub);
    EventCapture cap;
    session.set_event_sink(
        pico_sh2::reports::EventSink::bind<EventCapture,
                                           &EventCapture::on_event>(&cap));
    drive_to_ready(hub, session);

    // The same shape of cargo the normal channel carries: a base timestamp
    // then one accelerometer report.
    std::vector<std::uint8_t> payload = {
        0xFB, 0x0A, 0x00, 0x00, 0x00,  // base delta = 10
        0x01, 0x00, 0x00, 0x05,        // accel: seq, status, delay_lsb = 5
        0x00, 0x01, 0x00, 0x02, 0x00, 0xFF};
    hub.queue_transfer(frame(input_wake_channel(), 0, payload), 1'000'000);
    session.service();

    ASSERT_EQ(cap.events.size(), 1U);
    EXPECT_EQ(cap.events[0].sensor, pico_sh2::reports::SensorId::ACCELEROMETER);
    EXPECT_EQ(cap.events[0].event_us, 1'000'000U - 500U);
    EXPECT_EQ(session.counters().events_decoded, 1U);
    EXPECT_EQ(session.counters().unrouted_cargoes, 0U);
}

// SR-9: outside `ready` the wake channel is counted unrouted, not decoded.
TEST(Session, WakeChannelInputBeforeReadyIsNotDecoded) {
    FakeHub hub;
    Session session = make_session(hub);
    EventCapture cap;
    session.set_event_sink(
        pico_sh2::reports::EventSink::bind<EventCapture,
                                           &EventCapture::on_event>(&cap));
    advance_to_wait_advertisement(hub, session);

    hub.queue_transfer(
        frame(input_wake_channel(), 0,
              {0x01, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x02, 0x00, 0xFF}),
        0);
    session.service();

    EXPECT_TRUE(cap.events.empty());
    EXPECT_GT(session.counters().unrouted_cargoes, 0U);
}

// The contract the shipped transports keep and the fake now keeps with them: a
// hub polled with nothing to say hands back a header-only transfer of zero
// length, and that read SUCCEEDS. This case is what holds the rule up: both
// repositories' docs/orientation.md call it load-bearing, and prose is not a
// gate.
TEST(Session, AnIdlePollLeavesAReadySessionUndisturbed) {
    FakeHub hub;
    Session session = make_session(hub);
    drive_to_ready(hub, session);

    const auto before = session.counters();

    hub.queue_idle_transfer(50'000);
    session.service();

    // Not a fault, not a restart, still READY: an empty bus is normal traffic.
    EXPECT_EQ(session.state(), SessionState::READY);
    EXPECT_FALSE(session.faulted());
    EXPECT_EQ(session.fault(), pico_sh2::Sh2Error::NONE);
    EXPECT_EQ(session.counters().transport_restarts, before.transport_restarts);

    // And the session still works afterwards.
    hub.queue_idle_transfer(60'000);
    session.service();
    EXPECT_EQ(session.state(), SessionState::READY);
    EXPECT_FALSE(session.faulted());
}

// The same during startup, which is where a read error is not merely counted
// but faults the session permanently.
TEST(Session, AnIdlePollDuringStartupDoesNotFaultTheSession) {
    FakeHub hub;
    Session session = make_session(hub);
    advance_to_wait_advertisement(hub, session);

    hub.queue_idle_transfer(1'000);
    session.service();

    EXPECT_NE(session.state(), SessionState::FAULTED);
    EXPECT_EQ(session.fault(), pico_sh2::Sh2Error::NONE);
    EXPECT_EQ(session.startup_phase(), StartupPhase::WAIT_ADVERTISEMENT);

    // The handshake still completes once the hub does speak.
    hub.queue_startup();
    session.service();
    EXPECT_EQ(session.state(), SessionState::READY);
}

}  // namespace
