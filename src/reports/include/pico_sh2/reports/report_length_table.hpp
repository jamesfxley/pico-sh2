// SPDX-License-Identifier: MIT
#pragma once

/// \file report_length_table.hpp
/// Wire length in bytes per report ID, including the ID byte.
/// A cargo can concatenate reports.
/// The seed covers startup and hubs omitting report lengths; advertisement
/// wins. Direct indexing is O(1); zero means unknown (UNSUPPORTED).

#include <array>
#include <cstddef>
#include <cstdint>

#include "pico_sh2/util/result.hpp"

namespace pico_sh2::reports {

/// Distinct byte length type to prevent swapping it with a report ID.
struct ReportLength {
    std::uint8_t bytes;  ///< The length, in bytes.
};

/// On-wire length per report ID, seeded from the compiled defaults and
/// overridden by the hub's advertisement.
class ReportLengthTable {
   public:
    /// Entry count, covering every possible byte-sized report ID.
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

    /// Set an advertised length, overriding the seed.
    constexpr void set(std::uint8_t report_id, ReportLength length) {
        lengths_[report_id] = length.bytes;
    }

    /// Default lengths in bytes, including ID, prefix and body (1000-3625
    /// §6.5). Entry comments cite their report subsection; advertisement
    /// overrides defaults.
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
                            14});  // ARVR-stabilised rotation vector  [6.5.42]
        table.set(
            0x29,
            ReportLength{12});  // ARVR-stabilised game rotation vector [6.5.43]
        /// Timestamp records embedded in input cargoes (report ID + i32).
        table.set(0xFB, ReportLength{5});  // Base timestamp reference  [7.2.1]
        table.set(0xFA, ReportLength{5});  // Timestamp rebase  [7.2.2]
        /// Flush Completed uses control (1000-3600 §2.2); seeding its length
        /// lets input walks skip it without losing the remaining reports.
        table.set(0xEF, ReportLength{2});  // Flush Completed  [6.5.7]
        /// Control-channel responses; control cargoes also batch reports and
        /// are split by this table (1000-3600 §2.3).
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
