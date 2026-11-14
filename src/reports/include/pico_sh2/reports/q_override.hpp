// SPDX-License-Identifier: MIT
#pragma once

/// \file q_override.hpp
/// Per-report overrides for compiled Q defaults (1000-3625 §6.5, DR-2).

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>

#include "pico_sh2/reports/q_point.hpp"
#include "pico_sh2/util/result.hpp"

namespace pico_sh2::reports {

/// Q exponents for data and optional bias or accuracy (1000-3625 §5.1).
struct ReportQ {
    std::uint8_t primary;    ///< Scales the report's data field(s).
    std::uint8_t secondary;  ///< Scales its bias or accuracy field, if any.
};

/// Fixed-capacity override map, empty by default and allocation-free (AR-1).
/// A new ID beyond capacity returns TABLE_FULL.
class QOverrideTable {
   public:
    /// Override capacity in report IDs.
    static constexpr std::size_t CAPACITY = 24;

    /// Replace an override; INVALID_ARGUMENT means a Q above MAX_Q_POINT.
    /// TABLE_FULL means a new ID has no slot; invalid Q values would make
    /// shifts undefined.
    Result<void> set(std::uint8_t report_id, ReportQ q_points) {
        // All override paths pass this check before storing values used in
        // shifts.
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
