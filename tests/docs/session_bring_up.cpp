// SPDX-License-Identifier: MIT
// Bringing a session up, over the transport, event handler and context the
// guide leaves to the reader.
#include <pico_sh2/pico_sh2.hpp>

namespace doc_examples {
struct Context {};

void on_event(void* ctx, const pico_sh2::reports::SensorEvent& event);

void session_bring_up(pico_sh2::IHalTransport& transport, Context& ctx) {
#include "session_bring_up.inc"
}
}  // namespace doc_examples
