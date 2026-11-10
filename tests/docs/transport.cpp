// SPDX-License-Identifier: MIT
// Checks that the guide's transport declares every required override.
#include <cstddef>
#include <cstdint>
#include <pico_sh2/pico_sh2.hpp>
#include <span>
#include <type_traits>

namespace doc_examples {
#include "transport.inc"

static_assert(!std::is_abstract_v<MyTransport>);
}  // namespace doc_examples
