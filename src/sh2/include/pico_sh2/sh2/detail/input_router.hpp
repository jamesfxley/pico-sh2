// SPDX-License-Identifier: MIT
#pragma once

/// \file input_router.hpp
/// Internal batched input decoding with Q overrides (DR-2).
/// Batched events use the lead edge and timestamp records (SR-4). Compact gyro
/// reports use the edge directly (1000-3535 §2.6). Roles are advertised
/// (SR-10).

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

    /// Decode batched input; unknown or overrunning reports stop only this
    /// cargo's walk.
    void route_batched(std::span<const std::uint8_t> cargo,
                       std::uint64_t lead_edge_us) {
        // The result is discarded: the walk counts malformed input, and
        // faulting the session would drop an otherwise healthy link.
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

    /// Restore seeded lengths; part-specific Q overrides survive reset (DR-2).
    void reseed_lengths() { lengths_ = reports::ReportLengthTable::seeded(); }

    /// Replace decode-time Q points (DR-2).
    /// INVALID_ARGUMENT means a Q would make the shift undefined; TABLE_FULL
    /// means no room for a new entry.
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
