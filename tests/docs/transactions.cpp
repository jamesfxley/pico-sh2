// SPDX-License-Identifier: MIT
// A command and an FRS read, with the sinks and context the guide names.
#include <cstdint>
#include <optional>
#include <pico_sh2/pico_sh2.hpp>
#include <span>

namespace doc_examples {
struct Context {};

void on_command(void* ctx, const pico_sh2::CommandResponse& response);
void on_frs(void* ctx, pico_sh2::Sh2Error status, std::uint16_t record_type,
            std::span<const std::uint32_t> words,
            std::optional<std::uint8_t> hub_status);

void transactions(pico_sh2::Session& session, Context& ctx) {
#include "transactions.inc"
}
}  // namespace doc_examples
