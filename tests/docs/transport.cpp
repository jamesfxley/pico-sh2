// SPDX-License-Identifier: MIT
// The transport a consumer writes, declared as the usage guide shows it, and
// checked to be complete: a class missing an override would be abstract, and a
// reader could not declare one.
#include <cstddef>
#include <cstdint>
#include <pico_sh2/pico_sh2.hpp>
#include <span>
#include <type_traits>

namespace doc_examples {
#include "transport.inc"

static_assert(!std::is_abstract_v<MyTransport>);
}  // namespace doc_examples
