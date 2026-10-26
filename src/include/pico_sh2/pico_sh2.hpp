// SPDX-License-Identifier: MIT
#pragma once

/// \file pico_sh2.hpp
/// pico-sh2 umbrella header. Pulls in the full public surface of the SH-2 /
/// SHTP protocol library, so a consumer can include this alone and construct
/// and drive a Session. Platform-independent: no Pico SDK, no OS headers.
///
/// Everything under pico_sh2/sh2/detail/ is internal to Session and is
/// deliberately not exported here.

// The umbrella re-exports the public surface.
// IWYU pragma: begin_exports

/// This library's error taxonomy and the Result carrying it. The Result,
/// Callback and byte accessors themselves are pico_util's, reached through the
/// headers below rather than re-exported here.
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

/// Semantic version of the library, mirroring project(pico_sh2 VERSION ...).
/// Major version; changes when the public API breaks.
inline constexpr int VERSION_MAJOR = 0;
/// Minor version; changes when the public API grows compatibly.
inline constexpr int VERSION_MINOR = 1;
/// Patch version; changes when nothing in the public API does.
inline constexpr int VERSION_PATCH = 0;

}  // namespace pico_sh2
