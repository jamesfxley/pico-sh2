// SPDX-License-Identifier: MIT
#pragma once

/// \file input_router.hpp
/// Input-cargo routing: the report-length table the advertisement establishes,
/// the decode-time Q overrides (DR-2), and the walk that turns a batched cargo
/// into events.
///
/// The `inputNormal` and `inputWake` roles carry batched fixed-length reports
/// which are split by the length table and timed from the cargo's
/// lead-transfer edge plus their own timestamp records (SR-4). The
/// `inputGyroRv` role carries exactly one compact gyro-integrated rotation
/// vector with no prefix or delay, timed by the edge directly
/// (1000-3535 §2.6). SR-10 binds each role to a number from the hub's
/// advertisement, so the numbers depend on the base the hub selected and are
/// not named here.
///
/// This is an internal collaborator of Session, not public API.

#include <cstdint>
#include <span>

#include "pico_sh2/reports/decoders.hpp"
#include "pico_sh2/reports/q_override.hpp"
#include "pico_sh2/reports/report_length_table.hpp"
#include "pico_sh2/reports/sensor_event.hpp"
#include "pico_sh2/util/result.hpp"

namespace pico_sh2::detail {

class InputRouter {
   public:
    void set_event_sink(reports::EventSink sink) { event_sink_ = sink; }

    /// Split and decode an `inputNormal` or `inputWake` cargo. An unknown or
    /// overrunning report aborts this cargo's walk, counted in stats(), rather
    /// than the session.
    void route_batched(std::span<const std::uint8_t> cargo,
                       std::uint64_t lead_edge_us) {
        // Discarded deliberately. PROTOCOL here means the hub sent a report
        // this build cannot size; the walk stopped and counted it in stats_,
        // and there is nothing the router can do about a peer's malformed
        // cargo that stats_ has not already recorded. Faulting the session on
        // it would take down a link that is otherwise healthy.
        static_cast<void>(reports::walk_input_cargo(
            cargo, lengths_, q_overrides_, lead_edge_us, event_sink_, stats_));
    }

    /// Decode the single `inputGyroRv` report.
    void route_gyro_integrated(std::span<const std::uint8_t> cargo,
                               std::uint64_t lead_edge_us) {
        auto event = reports::decode_gyro_integrated(cargo, q_overrides_);
        if (!event.ok()) {
            ++stats_.reports_skipped;
            return;
        }
        reports::SensorEvent decoded = event.value();
        decoded.event_us = lead_edge_us;
        ++stats_.events_decoded;
        event_sink_(decoded);
    }

    /// Apply one advertised (report ID, on-wire length) pair.
    void set_report_length(std::uint8_t report_id, std::uint8_t length) {
        lengths_.set(report_id, reports::ReportLength{length});
    }

    /// Drop the advertised lengths back to the compiled seed. The advertisement
    /// re-establishes them, so they belong to a hub incarnation; the Q
    /// overrides describe the part itself and deliberately survive (DR-2).
    void reseed_lengths() { lengths_ = reports::ReportLengthTable::seeded(); }

    /// DR-2: install or replace the decode-time Q points for a report ID.
    /// INVALID_ARGUMENT if either Q point exceeds reports::MAX_Q_POINT, which
    /// would make the decoder's 2^Q shift undefined; TABLE_FULL if there is no
    /// room for a new entry.
    Result<void> set_report_q(std::uint8_t report_id,
                              reports::ReportQ q_points) {
        return q_overrides_.set(report_id, q_points);
    }

    [[nodiscard]] const reports::ReportLengthTable& lengths() const {
        return lengths_;
    }

    [[nodiscard]] const reports::QOverrideTable& q_overrides() const {
        return q_overrides_;
    }

    [[nodiscard]] const reports::WalkStats& stats() const { return stats_; }

   private:
    reports::ReportLengthTable lengths_ = reports::ReportLengthTable::seeded();
    reports::QOverrideTable q_overrides_{};
    reports::EventSink event_sink_;
    reports::WalkStats stats_{};  // accumulated across cargoes
};

}  // namespace pico_sh2::detail
