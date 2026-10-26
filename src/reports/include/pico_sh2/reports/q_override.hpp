// SPDX-License-Identifier: MIT
#pragma once

/// \file q_override.hpp
/// Decode-time Q-point overrides (DR-2). Every input report decodes its
/// fixed-point fields with a Q point that defaults to the value compiled into
/// the decoder table (1000-3625 §6.5). A given part may carry
/// authoritative Q points in its sensor-metadata FRS records (0xE301–0xE324);
/// when an override is installed for a report ID, the decoders apply it in
/// place of the compiled default, so a part whose scaling differs from the
/// compiled assumption still decodes correctly.
///
/// A report uses at most two distinct Q points: a `primary` scaling its main
/// data field(s) and a `secondary` scaling the single auxiliary field a few
/// reports carry (rotation-vector heading accuracy, an uncalibrated sensor's
/// bias). The gyro-integrated rotation vector's angular velocity is fixed at
/// Q10 and takes neither (DR-2). Reports with no fixed-point fields ignore
/// both.

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>

#include "pico_sh2/reports/q_point.hpp"
#include "pico_sh2/util/result.hpp"

namespace pico_sh2::reports {

/// The Q point(s) a report's decoder applies, in the two roles 1000-3625 §5.1
/// gives a sensor's metadata Q points. `primary` scales the data field(s);
/// `secondary` scales a bias or accuracy field, which only the uncalibrated
/// reports (their bias) and the rotation vectors with accuracy (their heading
/// accuracy) carry. Every other decoder ignores it.
struct ReportQ {
    std::uint8_t primary;    ///< Scales the report's data field(s).
    std::uint8_t secondary;  ///< Scales its bias or accuracy field, if any.
};

/// A fixed-capacity report-ID -> ReportQ map consulted at decode time. Empty by
/// default (every report decodes with its compiled Q); an installed entry
/// overrides that report's Q. Storage is a compile-time array — no allocation
/// (AR-1); installing a new report ID past capacity is refused rather than
/// dropped silently.
class QOverrideTable {
   public:
    /// Enough slots for the full decodable report set (the decoder registry has
    /// 19 entries plus the gyro-integrated report).
    static constexpr std::size_t CAPACITY = 24;

    /// Install or replace the override for `report_id`. INVALID_ARGUMENT if
    /// either Q point exceeds MAX_Q_POINT, which would make the decoders' 2^Q
    /// shift undefined; TABLE_FULL only when the report ID is new and the table
    /// is already full.
    Result<void> set(std::uint8_t report_id, ReportQ q_points) {
        // The invariant belongs here, with the storage the decoders read from.
        // Every path that installs a Q -- the session's, the metadata record's,
        // and a consumer decoding a captured cargo with this table directly --
        // goes through this one function.
        if (q_points.primary > MAX_Q_POINT ||
            q_points.secondary > MAX_Q_POINT) {
            return Sh2Error::INVALID_ARGUMENT;
        }
        for (Entry& entry : entries_) {
            if (entry.used && entry.id == report_id) {
                entry.q = q_points;
                return Result<void>{};
            }
        }
        for (Entry& entry : entries_) {
            if (!entry.used) {
                entry = Entry{.id = report_id, .q = q_points, .used = true};
                return Result<void>{};
            }
        }
        return Sh2Error::TABLE_FULL;
    }

    /// The installed override for `report_id`, or `fallback` if none is set.
    [[nodiscard]] ReportQ resolve(std::uint8_t report_id,
                                  ReportQ fallback) const {
        for (const Entry& entry : entries_) {
            if (entry.used && entry.id == report_id) {
                return entry.q;
            }
        }
        return fallback;
    }

    /// \param report_id the report ID to test.
    /// \return whether an override is installed for it.
    [[nodiscard]] bool has(std::uint8_t report_id) const {
        return std::ranges::any_of(entries_, [report_id](const Entry& entry) {
            return entry.used && entry.id == report_id;
        });
    }

   private:
    struct Entry {
        std::uint8_t id;
        ReportQ q;
        bool used;
    };

    std::array<Entry, CAPACITY> entries_{};
};

}  // namespace pico_sh2::reports
