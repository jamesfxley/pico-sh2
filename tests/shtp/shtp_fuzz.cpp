// SPDX-License-Identifier: MIT
#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <span>

#include "pico_sh2/shtp/advertisement.hpp"
#include "pico_sh2/shtp/channels.hpp"
#include "pico_sh2/shtp/engine.hpp"

// Arbitrary transfers must terminate without out-of-bounds reads or writes
// (ASan/UBSan) and never deliver cargo beyond buffer capacity.
// Unconstrained fragment splits must produce no crash or hang (NR-5).

// readability-identifier-naming: libFuzzer requires this entry point name.
// NOLINTNEXTLINE(readability-identifier-naming)
extern "C" int LLVMFuzzerTestOneInput(const std::uint8_t* data,
                                      std::size_t size) {
    std::array<std::uint8_t, 512> buffer{};
    pico_sh2::shtp::ShtpEngine engine(buffer);

    // Reading every delivered byte exposes bad spans to the sanitiser;
    // the capacity check traps oversized delivery.
    static std::uint64_t sink_sum = 0;
    engine.set_cargo_sink(pico_sh2::shtp::ShtpEngine::CargoSink(
        [](void* ctx, const pico_sh2::shtp::ShtpEngine::Cargo& cargo) {
            if (cargo.payload.size() > 512) {
                __builtin_trap();
            }
            auto* sum = static_cast<std::uint64_t*>(ctx);
            for (const std::uint8_t byte : cargo.payload) {
                *sum += byte;
            }
            *sum += cargo.lead_edge_us;
        },
        &sink_sum));

    const std::span<const std::uint8_t> input(data, size);
    std::size_t offset = 0;
    while (offset + pico_sh2::shtp::HEADER_SIZE <= input.size()) {
        // Physical size varies independently of the header, exercising
        // mismatches.
        std::size_t chunk = pico_sh2::shtp::HEADER_SIZE + (input[offset] % 33U);
        chunk = std::min(chunk, input.size() - offset);
        (void)engine.ingest(input.subspan(offset, chunk), offset);
        offset += chunk;
    }

    // Exercise outbound framing with a data-derived payload length.
    if (size > 0) {
        std::array<std::uint8_t, 64> out{};
        const std::size_t payload_len = data[0] % out.size();
        const std::span<const std::uint8_t> payload(
            data, payload_len <= size ? payload_len : size);
        // Framing accepts any wire channel, so the input supplies it too.
        (void)engine.frame_outbound(data[0], payload, out);
    }

    // Advertisement parsing must stay inside the cargo; emitted names must too.
    static std::uint64_t advertised_sum = 0;
    const auto lengths = pico_sh2::shtp::ReportLengthSink(
        [](void* ctx, std::uint8_t report_id, std::uint8_t length) {
            *static_cast<std::uint64_t*>(ctx) += report_id + length;
        },
        &advertised_sum);
    const auto channels = pico_sh2::shtp::ChannelSink(
        [](void* ctx, const pico_sh2::shtp::AdvertisedChannel& channel) {
            auto* sum = static_cast<std::uint64_t*>(ctx);
            *sum += channel.guid + channel.number;
            for (const std::uint8_t byte : channel.name) {
                *sum += byte;
            }
        },
        &advertised_sum);
    (void)pico_sh2::shtp::parse_advertisement(input, lengths, channels);
    return 0;
}
