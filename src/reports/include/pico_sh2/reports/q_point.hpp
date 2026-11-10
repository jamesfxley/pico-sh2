// SPDX-License-Identifier: MIT
#pragma once

/// \file q_point.hpp
/// Fixed-point scaling: real = raw / 2^Q (1000-3625 §6.5).
/// Metadata may override the decoder table's compiled defaults.

#include <cstdint>

namespace pico_sh2::reports {

/// Largest accepted Q exponent; 1u << 32 would be undefined.
inline constexpr unsigned MAX_Q_POINT = 31;

/// Convert raw / 2^Q to float; Q must be <= MAX_Q_POINT for a defined shift.
constexpr float q_to_float(std::int32_t raw, unsigned q_point) {
    return static_cast<float>(raw) /
           static_cast<float>(std::uint32_t{1} << q_point);
}

}  // namespace pico_sh2::reports
