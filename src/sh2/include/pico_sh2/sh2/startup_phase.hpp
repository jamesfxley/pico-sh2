// SPDX-License-Identifier: MIT
#pragma once

#include <cstdint>

namespace pico_sh2 {

/// Where the hub is in its startup sequence. Reported by Session as the phase a
/// fault happened in, so a deadline expiry is attributable to a distinct step.
enum class StartupPhase : std::uint8_t {
    IDLE = 0,             ///< constructed; begin() not yet called
    RESET_ASSERTED,       ///< NRST held low, waiting out the hold time
    BOOTING,              ///< NRST released, waiting the power-on boot delay
    WAIT_ADVERTISEMENT,   ///< expecting the channel-0 advertisement
    WAIT_RESET_COMPLETE,  ///< expecting the executable-channel reset-complete
    WAIT_INITIALIZE,      ///< expecting the unsolicited Initialize response
    COMPLETE,             ///< handshake done; the session is READY
};

}  // namespace pico_sh2
