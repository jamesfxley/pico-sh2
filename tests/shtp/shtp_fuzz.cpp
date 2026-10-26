// SPDX-License-Identifier: MIT
#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <span>

#include "pico_sh2/shtp/advertisement.hpp"
#include "pico_sh2/shtp/channels.hpp"
#include "pico_sh2/shtp/engine.hpp"

// libFuzzer entry point for the SHTP framing engine. Built only when
// PICO_SH2_ENABLE_FUZZERS is set and the compiler supports -fsanitize=fuzzer
// (Clang). The always-on engine tests pin the contract in CI; this target
// explores it continuously against arbitrary hub byte streams.
//
// The engine must, for any byte sequence: terminate, never read outside a
// transfer or write past the reassembly buffer (ASan/UBSan guard this), and
// never deliver a cargo larger than the buffer it was given. The fuzzer feeds
// the input as a sequence of transfers whose physical size is data-derived, so
// it exercises both single-transfer cargoes and fragmentation (a transfer
// shorter than the cargo its header claims).

// NR-5: reassembly holds across arbitrary fragment boundaries — the fuzzer
// drives the framing engine with unconstrained transfer splits and must find no
// crash or hang.
//
// The entry point's name is libFuzzer's, not ours.
// NOLINTNEXTLINE(readability-identifier-naming)
extern "C" int LLVMFuzzerTestOneInput(const std::uint8_t* data,
                                      std::size_t size) {
    std::array<std::uint8_t, 512> buffer{};
    pico_sh2::shtp::ShtpEngine engine(buffer);

    // Touch every delivered byte so a bad span is caught by the sanitizer, and
    // trap if a cargo ever exceeds the reassembly buffer.
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
        // Physical transfer size is derived from the stream, independent of the
        // header's length field, so fragments and mismatches both occur.
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
        // A channel is a wire number; which one does not matter to framing,
        // so drive it from the input too.
        (void)engine.frame_outbound(data[0], payload, out);
    }

    // The same bytes as an advertisement: the TLV walk must stay inside the
    // cargo, and every name it hands out must lie inside it too.
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
