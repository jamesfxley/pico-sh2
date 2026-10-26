// SPDX-License-Identifier: MIT
#pragma once

/// \file q_point.hpp
/// Fixed-point conversion. SH-2 input reports carry signed fixed-point fields
/// whose scaling is a per-field "Q point": the value on the wire is the real
/// value multiplied by 2^Q, so real = raw / 2^Q. Q points are fixed per report
/// (SH-2 Reference Manual, section 6.5) and may be overridden by
/// sensor-metadata FRS records at runtime; the compiled defaults live in the
/// decoder table.

#include <cstdint>

namespace pico_sh2::reports {

/// Largest Q point the fixed-point conversion accepts. The largest Q any SH-2
/// report actually uses is 30 (system-orientation FRS quaternions); 31 is the
/// representable ceiling — 2^31 still fits in a u32, but 1u << 32 would be
/// undefined. Callers that accept a Q from outside the compiled table (the
/// metadata-override and direct-override paths) reject anything above this.
inline constexpr unsigned MAX_Q_POINT = 31;

/// Convert a signed fixed-point sample with the given Q point to float.
/// real = raw * 2^-Q. Q must be <= MAX_Q_POINT so 2^Q is exact in a u32 and the
/// shift is defined.
constexpr float q_to_float(std::int32_t raw, unsigned q_point) {
    return static_cast<float>(raw) /
           static_cast<float>(std::uint32_t{1} << q_point);
}

}  // namespace pico_sh2::reports
