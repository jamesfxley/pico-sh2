// SPDX-License-Identifier: MIT
#pragma once

/// \file decoders.hpp
/// Report decoding. A cargo delivered on an input channel (3/4) is a run of
/// concatenated fixed-length input reports; `walk_input_cargo` splits it using
/// the report-length table and decodes each report to a SensorEvent, delivered
/// to a sink. Individual reports can also be decoded directly for testing.
///
/// Decoding is table-driven: a constexpr registry maps each report ID to its
/// wire length and a decode function, and a compile-time check binds those
/// lengths to the report-length seed so the two cannot drift. Adding a sensor
/// is a struct, a decode function, and one table row.

#include <cstdint>
#include <span>

#include "pico_sh2/reports/q_override.hpp"
#include "pico_sh2/reports/report_length_table.hpp"
#include "pico_sh2/reports/sensor_event.hpp"
#include "pico_sh2/util/result.hpp"
#include "pico_util/result/callback.hpp"

namespace pico_sh2::reports {

/// Sink invoked once per successfully decoded event during a cargo walk.
using EventSink = pico_util::Callback<void(const SensorEvent&)>;

/// Outcome tallies for one cargo walk. `unknown_reports` and
/// `truncated_reports` are terminal for the walk (it cannot resynchronize past
/// a report of unknown length or one that overruns the cargo);
/// `reports_skipped` counts reports of known length with no compiled decoder,
/// which are stepped over.
struct WalkStats {
    /// Reports decoded into an event and delivered.
    std::uint32_t events_decoded = 0;
    /// Reports of known length with no compiled decoder, stepped over.
    std::uint32_t reports_skipped = 0;
    /// Reports whose ID has no known length, which ends the walk.
    std::uint32_t unknown_reports = 0;
    /// Reports overrunning the cargo, which ends the walk.
    std::uint32_t truncated_reports = 0;
};

/// Decode a single input report (report ID at byte 0, common prefix, payload).
/// Returns UNSUPPORTED if no decoder is registered for the report ID, or
/// INVALID_ARGUMENT if the span is shorter than the report's wire length.
/// `q_overrides` supplies per-report Q points that replace the compiled
/// defaults (DR-2); an empty table (the default) decodes with the compiled Q.
Result<SensorEvent> decode_report(std::span<const std::uint8_t> report,
                                  const QOverrideTable& q_overrides = {});

/// Decode a compact gyro-integrated rotation vector (report 0x2A) delivered
/// one-per-transfer on the gyro-RV input channel. This report has no common
/// prefix, report ID, or delay field; the transfer's payload is exactly the
/// 14-byte report. `q_overrides` may override the quaternion's Q point; the
/// angular velocity's is fixed at Q10 (DR-2).
Result<SensorEvent> decode_gyro_integrated(
    std::span<const std::uint8_t> payload,
    const QOverrideTable& q_overrides = {});

/// Split and decode a cargo from the normal or wake input channel. `lengths`
/// supplies each report's on-wire length (the advertisement-overridable table)
/// and `q_overrides` its decode-time Q points (DR-2); every decoded event is
/// passed to `sink`. Embedded Base Timestamp (0xFB) and Timestamp Rebase (0xFA)
/// records set the running timebase; `intn_edge_us` is the HINTN-edge time the
/// cargo was read at, and each decoded event's `event_us` is reconstructed from
/// it (SR-4). Returns PROTOCOL if a report of unknown length or one that
/// overruns the cargo is encountered (the walk stops, having counted it); NONE
/// when the whole cargo is consumed.
Result<void> walk_input_cargo(std::span<const std::uint8_t> cargo,
                              const ReportLengthTable& lengths,
                              const QOverrideTable& q_overrides,
                              std::uint64_t intn_edge_us, const EventSink& sink,
                              WalkStats& stats);

}  // namespace pico_sh2::reports
