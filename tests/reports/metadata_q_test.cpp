// SPDX-License-Identifier: MIT
#include "pico_sh2/reports/metadata_q.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cstdint>
#include <optional>

#include "pico_sh2/reports/q_override.hpp"
#include "pico_sh2/reports/sensor_id.hpp"

namespace pico_sh2::reports {
namespace {

// Build a metadata-record word span whose word 7 packs the two Q points as the
// SH-2 Reference Manual §5.1 (Figures 28/29) lays them out: Q point 1 in the
// low half, Q point 2 in the high half.
std::array<std::uint32_t, 9> record_with_q(std::uint16_t q1, std::uint16_t q2) {
    std::array<std::uint32_t, 9> words{};
    words[METADATA_Q_WORD] = (static_cast<std::uint32_t>(q2) << 16U) |
                             static_cast<std::uint32_t>(q1);
    return words;
}

TEST(MetadataQ, ReadsPrimaryFromLowHalfSecondaryFromHigh) {
    std::array<std::uint32_t, 9> words = record_with_q(8, 0);
    std::optional<ReportQ> record_q = metadata_record_q(words);
    ASSERT_TRUE(record_q.has_value());
    EXPECT_EQ(record_q->primary, 8);
    EXPECT_EQ(record_q->secondary, 0);
}

TEST(MetadataQ, ReadsBothQPointsForRotationVector) {
    // Rotation vector: quaternion Q14, heading-accuracy Q12.
    std::array<std::uint32_t, 9> words = record_with_q(14, 12);
    std::optional<ReportQ> record_q = metadata_record_q(words);
    ASSERT_TRUE(record_q.has_value());
    EXPECT_EQ(record_q->primary, 14);
    EXPECT_EQ(record_q->secondary, 12);
}

TEST(MetadataQ, RejectsRecordTooShortToReachWord7) {
    std::array<std::uint32_t, METADATA_Q_WORD> words{};  // words 0..6 only
    EXPECT_FALSE(metadata_record_q(words).has_value());
}

TEST(MetadataQ, RejectsOutOfRangeQPoint) {
    EXPECT_FALSE(metadata_record_q(record_with_q(100, 0)).has_value());
    // 0x8000 as a signed 16-bit value is negative.
    EXPECT_FALSE(metadata_record_q(record_with_q(0x8000, 0)).has_value());
    EXPECT_FALSE(metadata_record_q(record_with_q(8, 0x8000)).has_value());
}

TEST(MetadataQ, MapsRecordTypesToDecodedReportIds) {
    EXPECT_EQ(report_id_for_metadata_type(0xE302),
              static_cast<std::uint8_t>(SensorId::ACCELEROMETER));
    EXPECT_EQ(report_id_for_metadata_type(0xE30B),
              static_cast<std::uint8_t>(SensorId::ROTATION_VECTOR));
    EXPECT_EQ(
        report_id_for_metadata_type(0xE324),
        static_cast<std::uint8_t>(SensorId::GYRO_INTEGRATED_ROTATION_VECTOR));
    EXPECT_EQ(report_id_for_metadata_type(0xE301),
              static_cast<std::uint8_t>(SensorId::RAW_ACCELEROMETER));
}

TEST(MetadataQ, RejectsRecordTypesOutsideTheDecodedSet) {
    // 0xE316 (significant motion) has a metadata record but no in-scope
    // decoder (FW-2).
    EXPECT_FALSE(report_id_for_metadata_type(0xE316).has_value());
    // A non-metadata FRS record type.
    EXPECT_FALSE(report_id_for_metadata_type(0x2D3E).has_value());
    // Below and above the metadata range.
    EXPECT_FALSE(report_id_for_metadata_type(0xE300).has_value());
    EXPECT_FALSE(report_id_for_metadata_type(0xE325).has_value());
}

}  // namespace
}  // namespace pico_sh2::reports
