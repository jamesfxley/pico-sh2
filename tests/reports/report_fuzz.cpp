// SPDX-License-Identifier: MIT
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <span>

#include "pico_sh2/reports/decoders.hpp"
#include "pico_sh2/reports/report_length_table.hpp"
#include "pico_sh2/reports/sensor_event.hpp"

// libFuzzer entry point for the report decoders and the input-cargo walk. Built
// only when PICO_SH2_ENABLE_FUZZERS is set and the compiler supports
// -fsanitize=fuzzer (Clang). The always-on golden-vector tests pin the
// contract; this target explores it against arbitrary cargo bytes.
//
// For any byte sequence the decoders and the walk must terminate and never read
// outside the supplied span (ASan/UBSan guard this). The walk splits reports by
// the length table, so a fuzz cargo exercises unknown IDs, truncated tails, and
// concatenation. The compact gyro-integrated report (no prefix) is exercised
// separately from the same bytes.
//
// The hub supplies the length table too, through its advertisement, so a
// second walk runs under a table whose entries the input sets: a length a
// record cannot hold, or zero, must be refused by the walk rather than read.

// The entry point's name is libFuzzer's, not ours.
// NOLINTNEXTLINE(readability-identifier-naming)
extern "C" int LLVMFuzzerTestOneInput(const std::uint8_t* data,
                                      std::size_t size) {
    const std::span<const std::uint8_t> input(data, size);

    static std::uint64_t sink_sum = 0;
    auto sink = pico_sh2::reports::EventSink(
        [](void* ctx, const pico_sh2::reports::SensorEvent& event) {
            auto* sum = static_cast<std::uint64_t*>(ctx);
            *sum += static_cast<std::uint64_t>(event.kind());
        },
        &sink_sum);

    pico_sh2::reports::WalkStats stats{};
    // The first bytes double as a fuzzed HINTN edge so timestamp reconstruction
    // (0xFB/0xFA records + report delay) is exercised too.
    std::uint64_t edge = 0;
    if (size >= sizeof(edge)) {
        std::memcpy(&edge, data, sizeof(edge));
    }
    (void)pico_sh2::reports::walk_input_cargo(
        input, pico_sh2::reports::ReportLengthTable::seeded(),
        pico_sh2::reports::QOverrideTable{}, edge, sink, stats);

    // Leading (report ID, length) pairs override the seed, as an advertisement
    // would; the rest of the input is the cargo walked under them.
    constexpr std::size_t ADVERTISED_PAIRS = 4;
    constexpr std::size_t PAIR_BYTES = 2 * ADVERTISED_PAIRS;
    if (size > PAIR_BYTES) {
        pico_sh2::reports::ReportLengthTable advertised =
            pico_sh2::reports::ReportLengthTable::seeded();
        for (std::size_t pair = 0; pair < ADVERTISED_PAIRS; ++pair) {
            advertised.set(input[2 * pair], pico_sh2::reports::ReportLength{
                                                input[(2 * pair) + 1]});
        }
        (void)pico_sh2::reports::walk_input_cargo(
            input.subspan(PAIR_BYTES), advertised,
            pico_sh2::reports::QOverrideTable{}, edge, sink, stats);
    }

    // A single-report decode and the prefix-less gyro-integrated decode over
    // the same bytes; both must bounds-check the span internally.
    (void)pico_sh2::reports::decode_report(input);
    (void)pico_sh2::reports::decode_gyro_integrated(input);
    return 0;
}
