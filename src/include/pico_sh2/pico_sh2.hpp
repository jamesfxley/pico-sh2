// SPDX-License-Identifier: MIT
#pragma once

/// \file pico_sh2.hpp
/// Public SH-2/SHTP headers; sh2/detail/ remains internal to Session.
/// Cargo is the payload carried by one or more SHTP transfers.

// The umbrella re-exports the public surface.
// IWYU pragma: begin_exports

/// Library errors and pico_util::Result; the headers using other pico-util
/// types include them.
#include "pico_sh2/util/result.hpp"

/// SHTP transport: channels, header, advertisement, framing engine.
#include "pico_sh2/shtp/advertisement.hpp"
#include "pico_sh2/shtp/channels.hpp"
#include "pico_sh2/shtp/engine.hpp"
#include "pico_sh2/shtp/header.hpp"

/// Sensor reports: identifiers, decoded events, decoders, and Q handling.
#include "pico_sh2/reports/decoders.hpp"
#include "pico_sh2/reports/metadata_q.hpp"
#include "pico_sh2/reports/q_override.hpp"
#include "pico_sh2/reports/q_point.hpp"
#include "pico_sh2/reports/report_layout.hpp"
#include "pico_sh2/reports/report_length_table.hpp"
#include "pico_sh2/reports/sensor_event.hpp"
#include "pico_sh2/reports/sensor_id.hpp"
#include "pico_sh2/reports/timestamp.hpp"

/// SH-2 session: the HAL seam, the sizing profile, the control-report types,
/// and the session itself.
#include "pico_sh2/sh2/channel_binding.hpp"
#include "pico_sh2/sh2/command.hpp"
#include "pico_sh2/sh2/feature.hpp"
#include "pico_sh2/sh2/flush.hpp"
#include "pico_sh2/sh2/frs.hpp"
#include "pico_sh2/sh2/hal_transport.hpp"
#include "pico_sh2/sh2/profile.hpp"
#include "pico_sh2/sh2/session.hpp"
#include "pico_sh2/sh2/session_timing.hpp"

// IWYU pragma: end_exports

namespace pico_sh2 {

/// Major version, bound to project(pico_sh2 VERSION ...).
inline constexpr int VERSION_MAJOR = 0;
/// Minor version, bound to project(pico_sh2 VERSION ...).
inline constexpr int VERSION_MINOR = 1;
/// Patch version, bound to project(pico_sh2 VERSION ...).
inline constexpr int VERSION_PATCH = 0;

}  // namespace pico_sh2
