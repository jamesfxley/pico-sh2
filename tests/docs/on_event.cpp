// SPDX-License-Identifier: MIT
// An event handler, with the reader's own use of the sample declared, and
// checked against the sink the guide binds it to.
#include <pico_sh2/pico_sh2.hpp>
#include <type_traits>
#include <variant>

namespace doc_examples {
template <typename... Values>
void use(const Values&... values);

#include "on_event.inc"

static_assert(std::is_constructible_v<pico_sh2::reports::EventSink,
                                      decltype(&on_event), void*>);
}  // namespace doc_examples
