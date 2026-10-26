// SPDX-License-Identifier: MIT
// Wire-format tests for the control-channel report builders and parsers. These
// are pure functions over bytes, exercised here directly against the layouts in
// the SH-2 Reference Manual rather than only through a running session.
#include <gtest/gtest.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <type_traits>

#include "pico_sh2/reports/sensor_id.hpp"
#include "pico_sh2/sh2/command.hpp"
#include "pico_sh2/sh2/feature.hpp"
#include "pico_sh2/sh2/flush.hpp"
#include "pico_sh2/sh2/frs.hpp"

namespace {

using pico_sh2::FeatureConfig;
using pico_sh2::reports::SensorId;

// --- Set / Get Feature (§6.5.2-6.5.5) ----------------------------------------

TEST(Feature, SetFeatureLaysOutEveryField) {
    FeatureConfig config;
    config.sensor = SensorId::ROTATION_VECTOR;
    config.flags = pico_sh2::feature_flag::WAKE;
    config.change_sensitivity = 0x1234;
    config.report_interval_us = 0x00051615;  // 333'333 us (~3 kHz)
    config.batch_interval_us = 0x000186A0;   // 100'000 us
    config.sensor_specific = 0xDEADBEEF;

    std::array<std::uint8_t, pico_sh2::FEATURE_REPORT_SIZE> out{};
    EXPECT_EQ(pico_sh2::serialize_set_feature(config, out),
              pico_sh2::FEATURE_REPORT_SIZE);

    EXPECT_EQ(out[0], pico_sh2::SET_FEATURE_COMMAND);
    EXPECT_EQ(out[1], static_cast<std::uint8_t>(SensorId::ROTATION_VECTOR));
    EXPECT_EQ(out[2], pico_sh2::feature_flag::WAKE);
    // Little-endian throughout.
    EXPECT_EQ(out[3], 0x34);
    EXPECT_EQ(out[4], 0x12);
    EXPECT_EQ(out[5], 0x15);
    EXPECT_EQ(out[8], 0x00);
    EXPECT_EQ(out[9], 0xA0);
    EXPECT_EQ(out[12], 0x00);
    EXPECT_EQ(out[13], 0xEF);
    EXPECT_EQ(out[16], 0xDE);
}

// A report interval of zero is how a sensor is disabled, so it must survive the
// round trip rather than being treated as "unset".
TEST(Feature, ZeroReportIntervalRoundTrips) {
    FeatureConfig config;
    config.sensor = SensorId::ACCELEROMETER;
    config.report_interval_us = 0;

    std::array<std::uint8_t, pico_sh2::FEATURE_REPORT_SIZE> out{};
    pico_sh2::serialize_set_feature(config, out);
    const FeatureConfig parsed = pico_sh2::parse_feature_response(out);
    EXPECT_EQ(parsed.sensor, SensorId::ACCELEROMETER);
    EXPECT_EQ(parsed.report_interval_us, 0U);
}

TEST(Feature, ResponseParsesBackToTheConfigItEncodes) {
    FeatureConfig config;
    config.sensor = SensorId::GYROSCOPE;
    config.flags = pico_sh2::feature_flag::CHANGE_SENSITIVITY_ENABLED |
                   pico_sh2::feature_flag::ALWAYS_ON;
    config.change_sensitivity = 0xBEEF;
    config.report_interval_us = 2'500;
    config.batch_interval_us = 40'000;
    config.sensor_specific = 7;

    std::array<std::uint8_t, pico_sh2::FEATURE_REPORT_SIZE> out{};
    pico_sh2::serialize_set_feature(config, out);
    const FeatureConfig parsed = pico_sh2::parse_feature_response(out);

    EXPECT_EQ(parsed.sensor, config.sensor);
    EXPECT_EQ(parsed.flags, config.flags);
    EXPECT_EQ(parsed.change_sensitivity, config.change_sensitivity);
    EXPECT_EQ(parsed.report_interval_us, config.report_interval_us);
    EXPECT_EQ(parsed.batch_interval_us, config.batch_interval_us);
    EXPECT_EQ(parsed.sensor_specific, config.sensor_specific);
}

// --- Command request (§6.3.8) ------------------------------------------------

// The sequence number and the command are both a byte; a call that passes
// them the wrong way round must not compile.
template <typename Sequence, typename Command>
concept SerializesCommandWith =
    requires(Sequence sequence, Command command,
             std::span<std::uint8_t, pico_sh2::COMMAND_REQUEST_SIZE> out) {
        pico_sh2::serialize_command_request(sequence, command, {}, out);
    };
static_assert(SerializesCommandWith<pico_sh2::CommandSequence, std::uint8_t>);
static_assert(!SerializesCommandWith<std::uint8_t, pico_sh2::CommandSequence>);
static_assert(!SerializesCommandWith<std::uint8_t, std::uint8_t>);

TEST(Command, RequestCarriesSequenceCommandAndParameters) {
    std::array<std::uint8_t, 3> params{0xAA, 0xBB, 0xCC};
    std::array<std::uint8_t, pico_sh2::COMMAND_REQUEST_SIZE> out{};
    EXPECT_EQ(pico_sh2::serialize_command_request(
                  pico_sh2::CommandSequence{0x42}, pico_sh2::command::TARE,
                  params, out),
              pico_sh2::COMMAND_REQUEST_SIZE);

    EXPECT_EQ(out[0], pico_sh2::COMMAND_REQUEST_REPORT);
    EXPECT_EQ(out[1], 0x42);
    EXPECT_EQ(out[2], pico_sh2::command::TARE);
    EXPECT_EQ(out[3], 0xAA);
    EXPECT_EQ(out[4], 0xBB);
    EXPECT_EQ(out[5], 0xCC);
}

// Unsupplied parameters are zero-filled rather than left holding whatever the
// caller's buffer contained.
TEST(Command, RequestZeroFillsUnsuppliedParameters) {
    std::array<std::uint8_t, pico_sh2::COMMAND_REQUEST_SIZE> out{};
    out.fill(0xFF);
    std::array<std::uint8_t, 1> params{0x11};
    pico_sh2::serialize_command_request(
        pico_sh2::CommandSequence{0}, pico_sh2::command::COUNTER, params, out);
    EXPECT_EQ(out[3], 0x11);
    for (std::size_t i = 4; i < pico_sh2::COMMAND_REQUEST_SIZE; ++i) {
        EXPECT_EQ(out[i], 0x00) << "parameter byte " << i;
    }
}

TEST(Command, RequestAcceptsNoParameters) {
    std::array<std::uint8_t, pico_sh2::COMMAND_REQUEST_SIZE> out{};
    pico_sh2::serialize_command_request(pico_sh2::CommandSequence{1},
                                        pico_sh2::command::SAVE_DCD, {}, out);
    EXPECT_EQ(out[2], pico_sh2::command::SAVE_DCD);
    for (std::size_t i = 3; i < pico_sh2::COMMAND_REQUEST_SIZE; ++i) {
        EXPECT_EQ(out[i], 0x00);
    }
}

// --- FRS requests (§6.3.3-6.3.6) ---------------------------------------------

TEST(Frs, ReadRequestCarriesTheRecordTypeAndZeroesTheRest) {
    std::array<std::uint8_t, pico_sh2::FRS_READ_REQUEST_SIZE> out{};
    out.fill(0xFF);
    EXPECT_EQ(pico_sh2::serialize_frs_read_request(
                  pico_sh2::frs_type::SYSTEM_ORIENTATION, out),
              pico_sh2::FRS_READ_REQUEST_SIZE);

    EXPECT_EQ(out[0], pico_sh2::FRS_READ_REQUEST_REPORT);
    EXPECT_EQ(out[1], 0x00);
    // Read offset zero: the whole record from the start.
    EXPECT_EQ(out[2], 0x00);
    EXPECT_EQ(out[3], 0x00);
    EXPECT_EQ(out[4], 0x3E);  // 0x2D3E little-endian
    EXPECT_EQ(out[5], 0x2D);
    EXPECT_EQ(out[6], 0x00);
    EXPECT_EQ(out[7], 0x00);
}

TEST(Frs, WriteRequestCarriesTypeAndLength) {
    std::array<std::uint8_t, pico_sh2::FRS_WRITE_REQUEST_SIZE> out{};
    EXPECT_EQ(pico_sh2::serialize_frs_write_request(
                  pico_sh2::FrsWriteRequest{
                      .type = pico_sh2::frs_type::GYRO_INTEGRATED_RV_CONFIG,
                      .length_words = 7},
                  out),
              pico_sh2::FRS_WRITE_REQUEST_SIZE);

    EXPECT_EQ(out[0], pico_sh2::FRS_WRITE_REQUEST_REPORT);
    EXPECT_EQ(out[1], 0x00);
    EXPECT_EQ(out[2], 7);  // length words, little-endian
    EXPECT_EQ(out[3], 0x00);
    EXPECT_EQ(out[4], 0xA2);  // 0xA1A2 little-endian
    EXPECT_EQ(out[5], 0xA1);
}

// A zero-length write request is the documented way to erase a record, so it
// must encode as such rather than being rejected.
TEST(Frs, WriteRequestOfZeroWordsErasesTheRecord) {
    std::array<std::uint8_t, pico_sh2::FRS_WRITE_REQUEST_SIZE> out{};
    pico_sh2::serialize_frs_write_request(
        pico_sh2::FrsWriteRequest{
            .type = pico_sh2::frs_type::DYNAMIC_CALIBRATION, .length_words = 0},
        out);
    EXPECT_EQ(out[2], 0x00);
    EXPECT_EQ(out[3], 0x00);
}

TEST(Frs, WriteDataCarriesTwoWordsAtTheirOffset) {
    std::array<std::uint8_t, pico_sh2::FRS_WRITE_DATA_SIZE> out{};
    const std::array<std::uint32_t, pico_sh2::FRS_WORDS_PER_REPORT> words{
        0x01020304, 0x05060708};
    EXPECT_EQ(pico_sh2::serialize_frs_write_data(4, words, out),
              pico_sh2::FRS_WRITE_DATA_SIZE);

    EXPECT_EQ(out[0], pico_sh2::FRS_WRITE_DATA_REPORT);
    EXPECT_EQ(out[2], 4);  // word offset, little-endian
    EXPECT_EQ(out[3], 0x00);
    EXPECT_EQ(out[4], 0x04);  // word0 little-endian
    EXPECT_EQ(out[7], 0x01);
    EXPECT_EQ(out[8], 0x08);  // word1 little-endian
    EXPECT_EQ(out[11], 0x05);
}

// The read response packs the status and the data-word count into one byte;
// the split is what the FRS engine decodes every response with.
// The byte as the wire carries it (1000-3625 §6.3.7, Figure 39): data length
// in bits 7:4, status in bits 3:0. A literal rather than one built from the
// same constants it checks, so a wrong shift or mask fails here.
TEST(Frs, ReadResponseStatusAndLengthShareAByte) {
    constexpr std::uint8_t LENGTH_2_STATUS_3 = 0x23;
    EXPECT_EQ(LENGTH_2_STATUS_3 & pico_sh2::FRS_READ_STATUS_MASK, 3U);
    EXPECT_EQ((static_cast<unsigned>(LENGTH_2_STATUS_3) >>
               pico_sh2::FRS_READ_DATA_LEN_SHIFT) &
                  pico_sh2::FRS_READ_STATUS_MASK,
              2U);
}

}  // namespace

// SR-6: a Flush Completed yields the sensor it names, and a report too short
// to name one yields nothing -- carried in the return type, so a caller cannot
// read a sensor the report never supplied.
TEST(Flush, CompletedNamesItsSensorOrNothing) {
    static_assert(std::is_same_v<decltype(pico_sh2::parse_flush_completed({})),
                                 std::optional<SensorId>>);
    const std::array<std::uint8_t, 2> whole{pico_sh2::FLUSH_COMPLETED_REPORT,
                                            0x05};
    EXPECT_EQ(pico_sh2::parse_flush_completed(whole),
              SensorId::ROTATION_VECTOR);
    const std::array<std::uint8_t, 1> truncated{
        pico_sh2::FLUSH_COMPLETED_REPORT};
    EXPECT_FALSE(pico_sh2::parse_flush_completed(truncated).has_value());
}
