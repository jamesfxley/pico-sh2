// SPDX-License-Identifier: MIT
// Seeding the channel map, over the transport and timing the guide has built,
// and starting the session as the program that follows would.
#include <pico_sh2/pico_sh2.hpp>

namespace doc_examples {
void channel_map(pico_sh2::IHalTransport& transport,
                 const pico_sh2::SessionTiming& timing) {
#include "channel_map.inc"
    session.begin();
}
}  // namespace doc_examples
