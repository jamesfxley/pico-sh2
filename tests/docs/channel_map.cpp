// SPDX-License-Identifier: MIT
// Compiles channel seeding with the guide's transport and timing.
#include <pico_sh2/pico_sh2.hpp>

namespace doc_examples {
void channel_map(pico_sh2::IHalTransport& transport,
                 const pico_sh2::SessionTiming& timing) {
#include "channel_map.inc"
    session.begin();
}
}  // namespace doc_examples
