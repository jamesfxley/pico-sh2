// SPDX-License-Identifier: MIT
#pragma once

/// \file report_length_table.hpp
/// Report-length table: maps an SH-2 report ID to its fixed on-wire length in
/// bytes (the length *includes* the report-ID byte). A single input or control
/// cargo from the hub may concatenate several fixed-length reports; splitting
/// them requires knowing each report's length up front. A hub publishes an
/// authoritative table in its channel-0 advertisement (tag 0x81), but a seed
/// table lets the driver split reports before the advertisement is parsed and
/// covers hubs that omit the tag.
///
/// Indexing is direct by report ID (0..255), so lookup and override are O(1)
/// and the table cannot overflow. A length of 0 means "unknown" (UNSUPPORTED).

#include <array>
#include <cstddef>
#include <cstdint>

#include "pico_sh2/util/result.hpp"

namespace pico_sh2::reports {

/// A report's on-wire length in bytes, including the report-ID byte. Its own
/// type so that set() cannot take a length and a report ID the wrong way
/// round: both are a byte.
struct ReportLength {
    std::uint8_t bytes;  ///< The length, in bytes.
};

/// On-wire length per report ID, seeded from the compiled defaults and
/// overridden by the hub's advertisement.
class ReportLengthTable {
   public:
    /// One entry per possible report ID, so no insertion can be refused.
    static constexpr std::size_t CAPACITY = 256;

    constexpr ReportLengthTable() = default;

    /// On-wire length for `report_id`, including the report-ID byte.
    /// UNSUPPORTED when the ID is not in the table (length 0).
    [[nodiscard]] constexpr Result<std::uint8_t> length_of(
        std::uint8_t report_id) const {
        const std::uint8_t length = lengths_[report_id];
        if (length == 0) {
            return Sh2Error::UNSUPPORTED;
        }
        return length;
    }

    /// \param report_id the report ID to test.
    /// \return whether the table holds a length for it.
    [[nodiscard]] constexpr bool contains(std::uint8_t report_id) const {
        return lengths_[report_id] != 0;
    }

    /// Insert or override a length. Called for each advertisement (id, length)
    /// pair; the advertisement is authoritative and wins over the seed.
    constexpr void set(std::uint8_t report_id, ReportLength length) {
        lengths_[report_id] = length.bytes;
    }

    /// The default table for the core sensor set. The advertisement overrides
    /// any of these at runtime; values here match the input-report definitions
    /// of 1000-3625 (length includes the report-ID byte, the
    /// sequence/status/delay prefix, and the report body).
    ///
    /// The bracketed number after each entry is that report's own subsection of
    /// 1000-3625, so a value can be checked against the document that defines
    /// it without first working out where to look. A table of report IDs and
    /// byte counts is the most copyable artefact in an SH-2 implementation, so
    /// each entry naming its source is what distinguishes this one from a
    /// transcription.
    static constexpr ReportLengthTable seeded() {
        ReportLengthTable table;
        table.set(0x01, ReportLength{10});  // Accelerometer  [6.5.9]
        table.set(0x02, ReportLength{10});  // Gyroscope (calibrated)  [6.5.13]
        table.set(0x03,
                  ReportLength{10});  // Magnetic field (calibrated)  [6.5.16]
        table.set(0x04, ReportLength{10});  // Linear acceleration  [6.5.10]
        table.set(0x05, ReportLength{14});  // Rotation vector  [6.5.18]
        table.set(0x06, ReportLength{10});  // Gravity  [6.5.11]
        table.set(0x07, ReportLength{16});  // Gyroscope uncalibrated  [6.5.14]
        table.set(0x08, ReportLength{12});  // Game rotation vector  [6.5.19]
        table.set(0x09,
                  ReportLength{14});  // Geomagnetic rotation vector  [6.5.20]
        table.set(0x0F,
                  ReportLength{16});  // Magnetic field uncalibrated  [6.5.17]
        table.set(0x10, ReportLength{5});   // Tap detector  [6.5.27]
        table.set(0x11, ReportLength{12});  // Step counter  [6.5.29]
        table.set(0x12, ReportLength{6});   // Significant motion  [6.5.30]
        table.set(0x13, ReportLength{6});   // Stability classifier  [6.5.31]
        table.set(0x14, ReportLength{16});  // Raw accelerometer  [6.5.8]
        table.set(0x15, ReportLength{16});  // Raw gyroscope  [6.5.12]
        table.set(0x16, ReportLength{16});  // Raw magnetometer  [6.5.15]
        table.set(0x18, ReportLength{8});   // Step detector  [6.5.28]
        table.set(0x28, ReportLength{
                            14});  // ARVR-stabilized rotation vector  [6.5.42]
        table.set(
            0x29,
            ReportLength{12});  // ARVR-stabilized game rotation vector [6.5.43]
        /// Timestamp records embedded in input cargoes (report ID + i32).
        table.set(0xFB, ReportLength{5});  // Base timestamp reference  [7.2.1]
        table.set(0xFA, ReportLength{5});  // Timestamp rebase  [7.2.2]
        /// Flush Completed (§6.5.7) travels on the control channel, whose read
        /// direction carries "all reports other than sensor data reports"
        /// (1000-3600 §2.2). Seeded anyway so that a cargo batching it with
        /// input reports costs one skipped report rather than the remainder.
        table.set(0xEF, ReportLength{2});  // Flush Completed  [6.5.7]
        /// The other control-channel responses the session routes. A control
        /// cargo may batch several reports too (1000-3600 §2.3), and is split
        /// by this table.
        table.set(0xF1, ReportLength{16});  // Command Response  [6.3.9]
        table.set(0xF3, ReportLength{16});  // FRS Read Response  [6.3.7]
        table.set(0xF5, ReportLength{4});   // FRS Write Response  [6.3.5]
        table.set(0xFC, ReportLength{17});  // Get Feature Response  [6.5.5]
        return table;
    }

   private:
    std::array<std::uint8_t, CAPACITY> lengths_{};
};

}  // namespace pico_sh2::reports
