// SPDX-License-Identifier: MIT
// Reporting every counter of a session, through the reader's own reporter.
#include <cstdint>
#include <pico_sh2/pico_sh2.hpp>

namespace doc_examples {
void report_counter(const char* name, std::uint32_t value);

void counters(const pico_sh2::Session& session) {
#include "counters.inc"
}
}  // namespace doc_examples
