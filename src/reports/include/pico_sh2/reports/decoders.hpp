// SPDX-License-Identifier: MIT
#pragma once

/// \file decoders.hpp
/// Decoders that turn cargo into typed events.
/// Batched reports use normal/wake roles; the decoder table's lengths are
/// checked against the report-length seed.

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

/// Accumulated walk counts; unknown lengths and overruns end a cargo's walk.
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

/// Decode one prefixed report with optional Q overrides (DR-2).
/// UNSUPPORTED means no decoder; INVALID_ARGUMENT means empty or short input.
Result<SensorEvent> decode_report(std::span<const std::uint8_t> report,
                                  const QOverrideTable& q_overrides = {});

/// Decode one 14-byte compact gyro report, without prefix, ID or delay.
/// Overrides may change quaternion Q; angular velocity stays Q10 (DR-2).
Result<SensorEvent> decode_gyro_integrated(
    std::span<const std::uint8_t> payload,
    const QOverrideTable& q_overrides = {});

/// Split normal/wake input by lengths, applying Q overrides (DR-2).
/// Timestamp records and the lead HINTN edge set event_us (SR-4).
/// Decoded events reach sink during the walk. Unknown lengths or overruns stop
/// it with PROTOCOL and a count; consuming the whole cargo returns success.
Result<void> walk_input_cargo(std::span<const std::uint8_t> cargo,
                              const ReportLengthTable& lengths,
                              const QOverrideTable& q_overrides,
                              std::uint64_t intn_edge_us, const EventSink& sink,
                              WalkStats& stats);

}  // namespace pico_sh2::reports
