// SPDX-License-Identifier: MIT
#include "pico_sh2/shtp/engine.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>
#include <vector>

#include "pico_sh2/shtp/channels.hpp"
#include "pico_sh2/util/result.hpp"

namespace {

using pico_sh2::Sh2Error;

using pico_sh2::shtp::HEADER_SIZE;
using pico_sh2::shtp::ShtpEngine;

// Captures cargo fields and payload; origin distinguishes in-place
// delivery from reassembly (AR-2).
struct Capture {
    struct Cargo {
        std::uint8_t channel;
        std::uint8_t sequence;
        std::uint64_t lead_edge_us;
        std::vector<std::uint8_t> data;
        const std::uint8_t* origin;
    };

    std::vector<Cargo> cargoes;

    void on_cargo(const ShtpEngine::Cargo& cargo) {
        cargoes.push_back({cargo.channel,
                           cargo.sequence,
                           cargo.lead_edge_us,
                           {cargo.payload.begin(), cargo.payload.end()},
                           cargo.payload.data()});
    }
};

// Edge timestamp used where a test does not exercise edge propagation.
constexpr std::uint64_t NO_EDGE = 0;

ShtpEngine::CargoSink sink_for(Capture& capture) {
    return ShtpEngine::CargoSink::bind<Capture, &Capture::on_cargo>(&capture);
}

// Delivery is identical at every physical split (EN-04.5; NR-2, NR-5).
TEST(ShtpEngine, ReassemblyIsIndependentOfEverySplit) {
    const std::vector<std::uint8_t> expected{0, 1, 2, 3, 4,  5,
                                             6, 7, 8, 9, 10, 11};
    for (std::size_t split = 0; split <= expected.size(); ++split) {
        for (const bool one_byte : {false, true}) {
            Capture capture;
            std::array<std::uint8_t, 64> buffer{};
            ShtpEngine engine(buffer);
            engine.set_cargo_sink(sink_for(capture));
            const std::size_t lead = split == 0 ? expected.size() : split;
            std::vector<std::uint8_t> transfer{16, 0, 2, 0};
            transfer.insert(
                transfer.end(), expected.begin(),
                expected.begin() + static_cast<std::ptrdiff_t>(lead));
            ASSERT_TRUE(engine.ingest(transfer, 1234).ok());
            std::size_t offset = lead;
            std::uint8_t sequence = 1;
            while (offset < expected.size()) {
                const auto take =
                    one_byte ? std::size_t{1} : expected.size() - offset;
                const auto remaining = expected.size() - offset + HEADER_SIZE;
                transfer = {static_cast<std::uint8_t>(remaining), 0x80, 2,
                            sequence++};
                transfer.insert(
                    transfer.end(),
                    expected.begin() + static_cast<std::ptrdiff_t>(offset),
                    expected.begin() +
                        static_cast<std::ptrdiff_t>(offset + take));
                ASSERT_TRUE(engine.ingest(transfer, 9999).ok());
                offset += take;
            }
            ASSERT_EQ(capture.cargoes.size(), 1U);
            EXPECT_EQ(capture.cargoes[0].data, expected);
            EXPECT_EQ(capture.cargoes[0].channel, 2);
            EXPECT_EQ(capture.cargoes[0].sequence, 0);
            EXPECT_EQ(capture.cargoes[0].lead_edge_us, 1234U);
            EXPECT_EQ(engine.stats().cargoes_delivered, 1U);
            EXPECT_EQ(engine.stats().reassembly_drops, 0U);
        }
    }
}

TEST(ShtpEngine, DeliversSingleTransferCargo) {
    Capture capture;
    std::array<std::uint8_t, 64> buffer{};
    ShtpEngine engine(buffer);
    engine.set_cargo_sink(sink_for(capture));

    // length 8, channel 2, seq 5, payload {0xA0,0xA1,0xA2,0xA3}.
    std::array<std::uint8_t, 8> transfer{0x08, 0x00, 0x02, 0x05,
                                         0xA0, 0xA1, 0xA2, 0xA3};
    ASSERT_TRUE(engine.ingest(transfer, NO_EDGE).ok());

    ASSERT_EQ(capture.cargoes.size(), 1U);
    EXPECT_EQ(capture.cargoes[0].channel, 2);
    EXPECT_EQ(capture.cargoes[0].sequence, 5);
    EXPECT_EQ(capture.cargoes[0].data,
              (std::vector<std::uint8_t>{0xA0, 0xA1, 0xA2, 0xA3}));
    EXPECT_EQ(engine.stats().cargoes_delivered, 1U);
}

// Reassembly delivers once at the lead header's declared length,
// independently of short transfers (NR-5).
TEST(ShtpEngine, ReassemblesFragmentedCargo) {
    Capture capture;
    std::array<std::uint8_t, 64> buffer{};
    ShtpEngine engine(buffer);
    engine.set_cargo_sink(sink_for(capture));

    // Six payload bytes: lead length 10 declares all six but carries four;
    // the continuation carries two.
    std::array<std::uint8_t, 8> lead{0x0A, 0x00, 0x02, 0x00,
                                     0xAA, 0xBB, 0xCC, 0xDD};
    std::array<std::uint8_t, 6> cont{0x06, 0x80, 0x02, 0x01, 0xEE, 0xFF};

    ASSERT_TRUE(engine.ingest(lead, NO_EDGE).ok());
    EXPECT_EQ(capture.cargoes.size(), 0U);  // not complete yet
    ASSERT_TRUE(engine.ingest(cont, NO_EDGE).ok());

    ASSERT_EQ(capture.cargoes.size(), 1U);
    EXPECT_EQ(capture.cargoes[0].sequence, 0);  // lead sequence
    EXPECT_EQ(capture.cargoes[0].data,
              (std::vector<std::uint8_t>{0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF}));
}

// Bytes beyond the declared length are ignored in both whole
// and fragmented cargoes (NR-5).
TEST(ShtpEngine, BytesBeyondTheDeclaredLengthAreIgnored) {
    Capture capture;
    std::array<std::uint8_t, 64> buffer{};
    ShtpEngine engine(buffer);
    engine.set_cargo_sink(sink_for(capture));

    // Declares 2 cargo bytes (length 6) and carries 4.
    std::array<std::uint8_t, 8> single{0x06, 0x00, 0x02, 0x00,
                                       0xA0, 0xA1, 0xB0, 0xB1};
    ASSERT_TRUE(engine.ingest(single, NO_EDGE).ok());
    ASSERT_EQ(capture.cargoes.size(), 1U);
    EXPECT_EQ(capture.cargoes[0].data, (std::vector<std::uint8_t>{0xA0, 0xA1}));

    // Declares 5 cargo bytes (length 9): 4 in the lead, and a continuation
    // carrying 3 where 1 remains.
    std::array<std::uint8_t, 8> lead{0x09, 0x00, 0x02, 0x01,
                                     0xC0, 0xC1, 0xC2, 0xC3};
    std::array<std::uint8_t, 7> cont{0x07, 0x80, 0x02, 0x02, 0xC4, 0xD0, 0xD1};
    ASSERT_TRUE(engine.ingest(lead, NO_EDGE).ok());
    ASSERT_TRUE(engine.ingest(cont, NO_EDGE).ok());
    ASSERT_EQ(capture.cargoes.size(), 2U);
    EXPECT_EQ(capture.cargoes[1].data,
              (std::vector<std::uint8_t>{0xC0, 0xC1, 0xC2, 0xC3, 0xC4}));
}

// A whole cargo carries the edge that announced it (SR-4).
TEST(ShtpEngine, SingleTransferCargoCarriesItsEdge) {
    Capture capture;
    std::array<std::uint8_t, 64> buffer{};
    ShtpEngine engine(buffer);
    engine.set_cargo_sink(sink_for(capture));

    std::array<std::uint8_t, 8> transfer{0x08, 0x00, 0x02, 0x05,
                                         0xA0, 0xA1, 0xA2, 0xA3};
    ASSERT_TRUE(engine.ingest(transfer, 12'345).ok());

    ASSERT_EQ(capture.cargoes.size(), 1U);
    EXPECT_EQ(capture.cargoes[0].lead_edge_us, 12'345U);
}

// A fragmented cargo carries its lead edge (1000-3535 §2.6; SR-4).
TEST(ShtpEngine, FragmentedCargoCarriesTheLeadEdge) {
    Capture capture;
    std::array<std::uint8_t, 64> buffer{};
    ShtpEngine engine(buffer);
    engine.set_cargo_sink(sink_for(capture));

    std::array<std::uint8_t, 8> lead{0x0A, 0x00, 0x02, 0x00,
                                     0xAA, 0xBB, 0xCC, 0xDD};
    std::array<std::uint8_t, 6> cont{0x06, 0x80, 0x02, 0x01, 0xEE, 0xFF};

    ASSERT_TRUE(engine.ingest(lead, 1'000).ok());
    ASSERT_TRUE(engine.ingest(cont, 2'000).ok());

    ASSERT_EQ(capture.cargoes.size(), 1U);
    EXPECT_EQ(capture.cargoes[0].lead_edge_us, 1'000U);
}

// An idle transfer leaves the reassembly's lead edge intact.
TEST(ShtpEngine, IdleTransferDoesNotDisturbTheLeadEdge) {
    Capture capture;
    std::array<std::uint8_t, 64> buffer{};
    ShtpEngine engine(buffer);
    engine.set_cargo_sink(sink_for(capture));

    std::array<std::uint8_t, 8> lead{0x0A, 0x00, 0x02, 0x00,
                                     0xAA, 0xBB, 0xCC, 0xDD};
    std::array<std::uint8_t, 4> idle{0x00, 0x00, 0x02, 0x00};
    std::array<std::uint8_t, 6> cont{0x06, 0x80, 0x02, 0x01, 0xEE, 0xFF};

    ASSERT_TRUE(engine.ingest(lead, 1'000).ok());
    ASSERT_TRUE(engine.ingest(idle, 1'500).ok());
    ASSERT_TRUE(engine.ingest(cont, 2'000).ok());

    ASSERT_EQ(capture.cargoes.size(), 1U);
    EXPECT_EQ(capture.cargoes[0].lead_edge_us, 1'000U);
}

// A whole cargo is delivered from the caller's staging buffer (AR-2).
TEST(ShtpEngine, SingleTransferCargoIsDeliveredInPlace) {
    Capture capture;
    std::array<std::uint8_t, 64> buffer{};
    ShtpEngine engine(buffer);
    engine.set_cargo_sink(sink_for(capture));

    std::array<std::uint8_t, 8> transfer{0x08, 0x00, 0x02, 0x05,
                                         0xA0, 0xA1, 0xA2, 0xA3};
    ASSERT_TRUE(engine.ingest(transfer, NO_EDGE).ok());

    ASSERT_EQ(capture.cargoes.size(), 1U);
    EXPECT_EQ(capture.cargoes[0].origin, transfer.data() + HEADER_SIZE);
}

// A fragmented cargo is delivered from the reassembly buffer (AR-2).
TEST(ShtpEngine, FragmentedCargoIsDeliveredFromReassemblyBuffer) {
    Capture capture;
    std::array<std::uint8_t, 64> buffer{};
    ShtpEngine engine(buffer);
    engine.set_cargo_sink(sink_for(capture));

    std::array<std::uint8_t, 8> lead{0x0A, 0x00, 0x02, 0x00,
                                     0xAA, 0xBB, 0xCC, 0xDD};
    std::array<std::uint8_t, 6> cont{0x06, 0x80, 0x02, 0x01, 0xEE, 0xFF};
    ASSERT_TRUE(engine.ingest(lead, NO_EDGE).ok());
    ASSERT_TRUE(engine.ingest(cont, NO_EDGE).ok());

    ASSERT_EQ(capture.cargoes.size(), 1U);
    EXPECT_EQ(capture.cargoes[0].origin, buffer.data());
}

TEST(ShtpEngine, CountsBadHeaderAndDoesNotDeliver) {
    Capture capture;
    std::array<std::uint8_t, 64> buffer{};
    ShtpEngine engine(buffer);
    engine.set_cargo_sink(sink_for(capture));

    std::array<std::uint8_t, 4> dead{0xFF, 0xFF, 0x02, 0x00};
    EXPECT_EQ(engine.ingest(dead, NO_EDGE).error(), Sh2Error::BAD_HEADER);
    EXPECT_EQ(engine.stats().bad_headers, 1U);
    EXPECT_EQ(capture.cargoes.size(), 0U);
}

TEST(ShtpEngine, EmptyTransferIsIdleNoOp) {
    Capture capture;
    std::array<std::uint8_t, 64> buffer{};
    ShtpEngine engine(buffer);
    engine.set_cargo_sink(sink_for(capture));

    std::array<std::uint8_t, 4> idle{0x00, 0x00, 0x03, 0x00};
    ASSERT_TRUE(engine.ingest(idle, NO_EDGE).ok());
    EXPECT_EQ(engine.stats().empty_transfers, 1U);
    EXPECT_EQ(capture.cargoes.size(), 0U);
}

// An oversize cargo is dropped whole and counted (BR-1).
TEST(ShtpEngine, DropsOversizeCargo) {
    Capture capture;
    std::array<std::uint8_t, 16> buffer{};  // small on purpose
    ShtpEngine engine(buffer);
    engine.set_cargo_sink(sink_for(capture));

    // Lead claims a 100-byte cargo (104 total) that cannot fit the buffer.
    std::array<std::uint8_t, 8> lead{0x68, 0x00, 0x02, 0x00,
                                     0x01, 0x02, 0x03, 0x04};
    EXPECT_EQ(engine.ingest(lead, NO_EDGE).error(), Sh2Error::REASSEMBLY_DROP);
    EXPECT_EQ(engine.stats().reassembly_drops, 1U);
    EXPECT_EQ(capture.cargoes.size(), 0U);
}

// A continuation with no cargo in progress is dropped and counted (NR-2).
TEST(ShtpEngine, DropsStrayContinuation) {
    Capture capture;
    std::array<std::uint8_t, 64> buffer{};
    ShtpEngine engine(buffer);
    engine.set_cargo_sink(sink_for(capture));

    // Continuation flag set with no cargo in progress.
    std::array<std::uint8_t, 6> cont{0x06, 0x80, 0x02, 0x00, 0xEE, 0xFF};
    EXPECT_EQ(engine.ingest(cont, NO_EDGE).error(), Sh2Error::REASSEMBLY_DROP);
    EXPECT_EQ(engine.stats().reassembly_drops, 1U);
}

// A cross-channel continuation drops the whole cargo (NR-2).
TEST(ShtpEngine, DropsCrossChannelContinuation) {
    Capture capture;
    std::array<std::uint8_t, 64> buffer{};
    ShtpEngine engine(buffer);
    engine.set_cargo_sink(sink_for(capture));

    // Lead on channel 2, then a continuation on channel 3.
    std::array<std::uint8_t, 8> lead{0x0A, 0x00, 0x02, 0x00,
                                     0xAA, 0xBB, 0xCC, 0xDD};
    std::array<std::uint8_t, 6> cont{0x06, 0x80, 0x03, 0x01, 0xEE, 0xFF};
    ASSERT_TRUE(engine.ingest(lead, NO_EDGE).ok());
    EXPECT_EQ(engine.ingest(cont, NO_EDGE).error(), Sh2Error::REASSEMBLY_DROP);
    EXPECT_EQ(capture.cargoes.size(), 0U);
}

// A new lead drops the interrupted cargo without partial delivery (NR-2).
TEST(ShtpEngine, NewLeadDropsInterruptedCargo) {
    Capture capture;
    std::array<std::uint8_t, 64> buffer{};
    ShtpEngine engine(buffer);
    engine.set_cargo_sink(sink_for(capture));

    // Lead promising 6 payload bytes but only 4 arrive, then a fresh complete
    // cargo interrupts it.
    std::array<std::uint8_t, 8> partial{0x0A, 0x00, 0x02, 0x00,
                                        0xAA, 0xBB, 0xCC, 0xDD};
    std::array<std::uint8_t, 6> fresh{0x06, 0x00, 0x02, 0x01, 0x11, 0x22};
    ASSERT_TRUE(engine.ingest(partial, NO_EDGE).ok());
    ASSERT_TRUE(engine.ingest(fresh, NO_EDGE).ok());

    EXPECT_EQ(engine.stats().reassembly_drops, 1U);
    ASSERT_EQ(capture.cargoes.size(), 1U);
    EXPECT_EQ(capture.cargoes[0].data, (std::vector<std::uint8_t>{0x11, 0x22}));
}

// A sequence gap counts once and still delivers the cargo (NR-3).
TEST(ShtpEngine, CountsSequenceGap) {
    Capture capture;
    std::array<std::uint8_t, 64> buffer{};
    ShtpEngine engine(buffer);
    engine.set_cargo_sink(sink_for(capture));

    std::array<std::uint8_t, 6> seq0{0x06, 0x00, 0x03, 0x00, 0x01, 0x02};
    std::array<std::uint8_t, 6> seq2{0x06, 0x00, 0x03, 0x02, 0x03, 0x04};
    ASSERT_TRUE(engine.ingest(seq0, NO_EDGE).ok());
    ASSERT_TRUE(engine.ingest(seq2, NO_EDGE).ok());  // skipped seq 1
    EXPECT_EQ(engine.stats().sequence_gaps, 1U);
}

TEST(ShtpEngine, FramesOutboundWithIncrementingSequence) {
    std::array<std::uint8_t, 64> buffer{};
    ShtpEngine engine(buffer);

    std::array<std::uint8_t, 3> payload{0xFD, 0x05, 0x01};
    std::array<std::uint8_t, 32> out{};

    auto first = engine.frame_outbound(2, payload, out);
    ASSERT_TRUE(first.ok());
    EXPECT_EQ(first.value(), payload.size() + 4);
    EXPECT_EQ(out[0], 7);     // 3 payload + 4 header
    EXPECT_EQ(out[2], 2);     // channel
    EXPECT_EQ(out[3], 0);     // first sequence
    EXPECT_EQ(out[4], 0xFD);  // payload begins after header

    auto second = engine.frame_outbound(2, payload, out);
    ASSERT_TRUE(second.ok());
    EXPECT_EQ(out[3], 1);  // sequence advanced

    // A different channel keeps its own sequence counter.
    auto other = engine.frame_outbound(1, payload, out);
    ASSERT_TRUE(other.ok());
    EXPECT_EQ(out[3], 0);
}

// Outbound framing returns CARGO_TOO_LARGE without fragmentation (NR-2).
TEST(ShtpEngine, RejectsOutboundThatDoesNotFit) {
    std::array<std::uint8_t, 64> buffer{};
    ShtpEngine engine(buffer);

    std::array<std::uint8_t, 30> payload{};
    std::array<std::uint8_t, 32> out{};  // 30 + 4 header > 32
    EXPECT_EQ(engine.frame_outbound(2, payload, out).error(),
              Sh2Error::CARGO_TOO_LARGE);
}

TEST(ShtpEngine, ResetClearsInProgressAndSequences) {
    Capture capture;
    std::array<std::uint8_t, 64> buffer{};
    ShtpEngine engine(buffer);
    engine.set_cargo_sink(sink_for(capture));

    std::array<std::uint8_t, 8> partial{0x0A, 0x00, 0x02, 0x00,
                                        0xAA, 0xBB, 0xCC, 0xDD};
    ASSERT_TRUE(engine.ingest(partial, NO_EDGE).ok());
    engine.reset();

    // After reset the continuation is stray (no cargo in progress).
    std::array<std::uint8_t, 6> cont{0x06, 0x80, 0x02, 0x01, 0xEE, 0xFF};
    EXPECT_EQ(engine.ingest(cont, NO_EDGE).error(), Sh2Error::REASSEMBLY_DROP);
}

// Reset restarts TX sequences at zero and RX gap tracking at the next input.
TEST(ShtpEngine, ResetRestartsBothSequenceCounts) {
    Capture capture;
    std::array<std::uint8_t, 64> buffer{};
    ShtpEngine engine(buffer);
    engine.set_cargo_sink(sink_for(capture));

    std::array<std::uint8_t, 1> payload{0x01};
    std::array<std::uint8_t, 16> out{};
    ASSERT_TRUE(engine.frame_outbound(2, payload, out).ok());
    ASSERT_TRUE(engine.frame_outbound(2, payload, out).ok());
    std::array<std::uint8_t, 5> inbound{0x05, 0x00, 0x03, 0x07, 0xAA};
    ASSERT_TRUE(engine.ingest(inbound, NO_EDGE).ok());

    engine.reset();

    ASSERT_TRUE(engine.frame_outbound(2, payload, out).ok());
    EXPECT_EQ(out[3], 0) << "outbound sequence did not restart";
    const auto gaps = engine.stats().sequence_gaps;
    std::array<std::uint8_t, 5> first{0x05, 0x00, 0x03, 0x00, 0xBB};
    ASSERT_TRUE(engine.ingest(first, NO_EDGE).ok());
    EXPECT_EQ(engine.stats().sequence_gaps, gaps) << "a restart read as a gap";
}

// An oversize cargo counts one drop across its transfers;
// the channel's next lead ends discard mode (OR-1).
TEST(ShtpEngine, AnOversizeCargoCountsOneDropAcrossItsTransfers) {
    Capture capture;
    std::array<std::uint8_t, 16> buffer{};
    ShtpEngine engine(buffer);
    engine.set_cargo_sink(sink_for(capture));

    std::array<std::uint8_t, 8> lead{0x68, 0x00, 0x02, 0x00,
                                     0x01, 0x02, 0x03, 0x04};
    std::array<std::uint8_t, 8> cont{0x08, 0x80, 0x02, 0x01,
                                     0x05, 0x06, 0x07, 0x08};
    EXPECT_EQ(engine.ingest(lead, NO_EDGE).error(), Sh2Error::REASSEMBLY_DROP);
    for (int i = 0; i < 16; ++i) {
        EXPECT_EQ(engine.ingest(cont, NO_EDGE).error(),
                  Sh2Error::REASSEMBLY_DROP);
    }
    EXPECT_EQ(engine.stats().reassembly_drops, 1U);

    // The channel's next lead starts a cargo that is delivered, and a stray
    // continuation after it is a new drop.
    std::array<std::uint8_t, 6> fresh{0x06, 0x00, 0x02, 0x02, 0x11, 0x22};
    ASSERT_TRUE(engine.ingest(fresh, NO_EDGE).ok());
    EXPECT_EQ(capture.cargoes.size(), 1U);
    EXPECT_EQ(engine.ingest(cont, NO_EDGE).error(), Sh2Error::REASSEMBLY_DROP);
    EXPECT_EQ(engine.stats().reassembly_drops, 2U);
}

// Stray continuations from one lost cargo count one drop (OR-1).
TEST(ShtpEngine, StrayContinuationsCountOneDropPerCargo) {
    Capture capture;
    std::array<std::uint8_t, 64> buffer{};
    ShtpEngine engine(buffer);
    engine.set_cargo_sink(sink_for(capture));

    std::array<std::uint8_t, 6> cont{0x06, 0x80, 0x02, 0x00, 0xEE, 0xFF};
    for (int i = 0; i < 3; ++i) {
        EXPECT_EQ(engine.ingest(cont, NO_EDGE).error(),
                  Sh2Error::REASSEMBLY_DROP);
    }
    EXPECT_EQ(engine.stats().reassembly_drops, 1U);
}

}  // namespace
