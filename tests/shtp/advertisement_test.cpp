// SPDX-License-Identifier: MIT
#include "pico_sh2/shtp/advertisement.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <string>
#include <utility>
#include <vector>

#include "pico_sh2/util/result.hpp"

namespace {

using pico_sh2::Sh2Error;
using pico_sh2::shtp::parse_advertisement;
using pico_sh2::shtp::ReportLengthSink;

// Collects the (report ID, length) pairs the parser emits. The advertisement is
// authoritative and a later pair overrides an earlier one, so lookups report
// the last value seen for an ID.
class Collector {
   public:
    ReportLengthSink sink() {
        return ReportLengthSink::bind<Collector, &Collector::on_pair>(this);
    }

    [[nodiscard]] std::size_t count() const { return pairs_.size(); }

    [[nodiscard]] std::optional<std::uint8_t> length_of(
        std::uint8_t report_id) const {
        std::optional<std::uint8_t> found;
        for (const auto& pair : pairs_) {
            if (pair.first == report_id) {
                found = pair.second;
            }
        }
        return found;
    }

   private:
    void on_pair(std::uint8_t report_id, std::uint8_t length) {
        pairs_.emplace_back(report_id, length);
    }

    std::vector<std::pair<std::uint8_t, std::uint8_t>> pairs_;
};

// NR-4: tag-0x81 pairs under a sensorhub GUID are emitted in wire order.
TEST(Advertisement, EmitsReportLengthPairs) {
    // GUID 2 (sensorhub) opens application scope, then tag 0x81, length 4, two
    // (id, length) pairs.
    std::array<std::uint8_t, 9> cargo{0x01, 0x01, 0x02, 0x81, 0x04,
                                      0x05, 0x0E, 0x08, 0x0C};
    Collector collector;
    auto applied = parse_advertisement(cargo, collector.sink());
    ASSERT_TRUE(applied.ok());
    EXPECT_EQ(applied.value(), 2U);
    EXPECT_EQ(collector.length_of(0x05).value_or(0), 0x0E);
    EXPECT_EQ(collector.length_of(0x08).value_or(0), 0x0C);
}

// NR-4: under the SHTP application (GUID 0), tag 0x81 is the UART-timeout value
// rather than report lengths (1000-3535 §5.3), so nothing is emitted.
TEST(Advertisement, IgnoresShtpApplicationTag81) {
    std::array<std::uint8_t, 7> cargo{0x01, 0x01, 0x00, 0x81, 0x02, 0x05, 0x0E};
    Collector collector;
    auto applied = parse_advertisement(cargo, collector.sink());
    ASSERT_TRUE(applied.ok());
    EXPECT_EQ(applied.value(), 0U);
    EXPECT_EQ(collector.count(), 0U);
}

// NR-4: scope tracks the nearest preceding GUID, so the SHTP 0x81 is suppressed
// and the sensorhub 0x81 that follows is emitted.
TEST(Advertisement, EmitsTag81OnlyAfterTheSensorhubGuid) {
    std::array<std::uint8_t, 14> cargo{
        0x01, 0x01, 0x00,        // GUID 0 (SHTP)
        0x81, 0x02, 0x05, 0xFF,  // UART timeout here: ignored
        0x01, 0x01, 0x02,        // GUID 2 (sensorhub)
        0x81, 0x02, 0x05, 0x18,  // report lengths here: emitted
    };
    Collector collector;
    auto applied = parse_advertisement(cargo, collector.sink());
    ASSERT_TRUE(applied.ok());
    EXPECT_EQ(applied.value(), 1U);
    EXPECT_EQ(collector.length_of(0x05).value_or(0), 0x18);
}

// NR-4: tags 0x80-0xFF are defined by each application (1000-3535 §5.2), so
// 0x81 is the report-length list only in the sensorhub's scope. Under the
// executable or a vendor application it means something else and emits
// nothing.
TEST(Advertisement, Tag81OutsideTheSensorhubEmitsNothing) {
    std::array<std::uint8_t, 17> cargo{
        0x01, 0x01, 0x01,                    // GUID 1 (executable)
        0x81, 0x02, 0x05, 0x0E,              // not report lengths here
        0x01, 0x04, 0xDE, 0xAD, 0xBE, 0xEF,  // a vendor GUID
        0x81, 0x02, 0x06, 0x0E,              // nor here
    };
    Collector collector;
    auto applied = parse_advertisement(cargo, collector.sink());
    ASSERT_TRUE(applied.ok());
    EXPECT_EQ(applied.value(), 0U);
    EXPECT_EQ(collector.count(), 0U);
}

// NR-4: an unknown or vendor tag is skipped by its length and never breaks the
// walk.
TEST(Advertisement, SkipsUnknownTags) {
    // Sensorhub GUID (tag 1, len 4), AppName (tag 8, len 3), then
    // ReportLengths.
    std::array<std::uint8_t, 15> cargo{0x01, 0x04, 0x02, 0x00, 0x00,
                                       0x00, 0x08, 0x03, 'a',  'b',
                                       'c',  0x81, 0x02, 0x01, 0x0A};
    Collector collector;
    auto applied = parse_advertisement(cargo, collector.sink());
    ASSERT_TRUE(applied.ok());
    EXPECT_EQ(applied.value(), 1U);
    EXPECT_EQ(collector.length_of(0x01).value_or(0), 0x0A);
}

TEST(Advertisement, EmptyCargoEmitsNothing) {
    std::array<std::uint8_t, 0> cargo{};
    Collector collector;
    auto applied = parse_advertisement(cargo, collector.sink());
    ASSERT_TRUE(applied.ok());
    EXPECT_EQ(applied.value(), 0U);
}

// NR-4: a TLV entry running past the end of the cargo is malformed.
TEST(Advertisement, RejectsTruncatedValue) {
    // tag 0x81 claims length 6 but only 2 value bytes are present.
    std::array<std::uint8_t, 4> cargo{0x81, 0x06, 0x05, 0x0E};
    Collector collector;
    EXPECT_EQ(parse_advertisement(cargo, collector.sink()).error(),
              Sh2Error::PROTOCOL);
}

TEST(Advertisement, RejectsTagWithoutLengthByte) {
    std::array<std::uint8_t, 1> cargo{0x81};
    Collector collector;
    EXPECT_EQ(parse_advertisement(cargo, collector.sink()).error(),
              Sh2Error::PROTOCOL);
}

TEST(Advertisement, RejectsOddReportLengthArray) {
    // Odd value length leaves a dangling report ID with no length. Under a
    // sensorhub (non-SHTP) GUID so the report-lengths branch is reached.
    std::array<std::uint8_t, 8> cargo{0x01, 0x01, 0x02, 0x81,
                                      0x03, 0x05, 0x0E, 0x08};
    Collector collector;
    EXPECT_EQ(parse_advertisement(cargo, collector.sink()).error(),
              Sh2Error::PROTOCOL);
}

// An unset sink is a no-op, so a caller that only wants the pair count (or the
// malformed-entry check) need not bind one.
TEST(Advertisement, UnsetSinkStillCounts) {
    std::array<std::uint8_t, 9> cargo{0x01, 0x01, 0x02, 0x81, 0x04,
                                      0x05, 0x0E, 0x08, 0x0C};
    auto applied = parse_advertisement(cargo, ReportLengthSink{});
    ASSERT_TRUE(applied.ok());
    EXPECT_EQ(applied.value(), 2U);
}

// --- channel assignments (1000-3535 §5.2) ---

struct ChannelCollector {
    struct Seen {
        std::uint32_t guid;
        std::uint8_t number;
        bool wake;
        std::string name;
    };

    std::vector<Seen> channels;

    pico_sh2::shtp::ChannelSink sink() {
        return pico_sh2::shtp::ChannelSink::bind<ChannelCollector,
                                                 &ChannelCollector::on_channel>(
            this);
    }

    void on_channel(const pico_sh2::shtp::AdvertisedChannel& channel) {
        std::string name;
        for (const std::uint8_t byte : channel.name) {
            name.push_back(static_cast<char>(byte));
        }
        channels.push_back({channel.guid, channel.number, channel.wake, name});
    }
};

// SR-10: a channel tag gives the number and the ChannelName that follows names
// it, both scoped to the preceding GUID.
TEST(Advertisement, EmitsChannelsWithTheirGuidAndName) {
    // GUID 1, NormalChannel 3, ChannelName "device".
    std::array<std::uint8_t, 18> cargo{
        0x01, 0x04, 0x01, 0x00, 0x00, 0x00,  // GUID 1
        0x06, 0x01, 0x03,                    // NormalChannel 3
        0x09, 0x07, 'd',  'e',  'v',  'i',  'c', 'e', 0x00};
    Collector lengths;
    ChannelCollector channels;
    ASSERT_TRUE(
        parse_advertisement(cargo, lengths.sink(), channels.sink()).ok());

    ASSERT_EQ(channels.channels.size(), 1U);
    EXPECT_EQ(channels.channels[0].guid, 1U);
    EXPECT_EQ(channels.channels[0].number, 3);
    EXPECT_FALSE(channels.channels[0].wake);
    EXPECT_EQ(channels.channels[0].name, "device");
}

// A wake channel is distinguished by its tag, not by its name.
TEST(Advertisement, MarksWakeChannels) {
    std::array<std::uint8_t, 3> cargo{0x07, 0x01, 0x04};
    Collector lengths;
    ChannelCollector channels;
    ASSERT_TRUE(
        parse_advertisement(cargo, lengths.sink(), channels.sink()).ok());
    ASSERT_EQ(channels.channels.size(), 1U);
    EXPECT_TRUE(channels.channels[0].wake);
    EXPECT_TRUE(channels.channels[0].name.empty());
}

// The name that matters is the one under the right GUID: the SHTP application
// and the SH-2 sensorhub both advertise a channel called "control", so an
// emitted channel carries the GUID that disambiguates them. Observed on a real
// BNO085, whose channel 0 is SHTP's "control" and channel 2 the sensorhub's.
TEST(Advertisement, TheSameNameUnderTwoGuidsIsTwoChannels) {
    std::vector<std::uint8_t> cargo{
        0x01, 0x04, 0x00, 0x00, 0x00, 0x00,  // GUID 0 (SHTP)
        0x06, 0x01, 0x00,                    // NormalChannel 0
        0x09, 0x08, 'c',  'o',  'n',  't',  'r', 'o', 'l', 0x00,
        0x01, 0x04, 0x02, 0x00, 0x00, 0x00,  // GUID 2 (sensorhub)
        0x06, 0x01, 0x02,                    // NormalChannel 2
        0x09, 0x08, 'c',  'o',  'n',  't',  'r', 'o', 'l', 0x00};
    Collector lengths;
    ChannelCollector channels;
    ASSERT_TRUE(
        parse_advertisement(cargo, lengths.sink(), channels.sink()).ok());

    ASSERT_EQ(channels.channels.size(), 2U);
    EXPECT_EQ(channels.channels[0].guid, 0U);
    EXPECT_EQ(channels.channels[0].number, 0);
    EXPECT_EQ(channels.channels[1].guid, 2U);
    EXPECT_EQ(channels.channels[1].number, 2);
    EXPECT_EQ(channels.channels[0].name, channels.channels[1].name);
}

// A ChannelName only names the channel tag immediately before it (1000-3535
// §5.2), so one separated by another tag names nothing.
TEST(Advertisement, AChannelNameMustFollowItsChannelTag) {
    std::vector<std::uint8_t> cargo{
        0x06, 0x01, 0x03,             // NormalChannel 3
        0x08, 0x03, 'a',  'b', 0x00,  // AppName in between
        0x09, 0x07, 'd',  'e', 'v',  'i', 'c', 'e', 0x00};
    Collector lengths;
    ChannelCollector channels;
    ASSERT_TRUE(
        parse_advertisement(cargo, lengths.sink(), channels.sink()).ok());
    ASSERT_EQ(channels.channels.size(), 1U);
    EXPECT_EQ(channels.channels[0].number, 3);
    EXPECT_TRUE(channels.channels[0].name.empty());
}

// A channel advertised last in the cargo has no following tag to end it, and
// must still be emitted.
TEST(Advertisement, EmitsAChannelThatEndsTheCargo) {
    std::array<std::uint8_t, 12> cargo{0x06, 0x01, 0x05, 0x09, 0x03, 'r',
                                       'v',  0x00, 0x00, 0x00, 0x00, 0x00};
    Collector lengths;
    ChannelCollector channels;
    // Trailing zero bytes parse as tag-0 entries of length 0 and are ignored.
    ASSERT_TRUE(parse_advertisement(std::span(cargo).first(8), lengths.sink(),
                                    channels.sink())
                    .ok());
    ASSERT_EQ(channels.channels.size(), 1U);
    EXPECT_EQ(channels.channels[0].number, 5);
    EXPECT_EQ(channels.channels[0].name, "rv");
}

TEST(Advertisement, DoesNotOverrunOnAdversarialLengths) {
    // Every one-byte prefix of a plausible advertisement must be rejected or
    // parsed, never read out of bounds (ASan/UBSan guard this).
    std::vector<std::uint8_t> full{0x01, 0x04, 0xDE, 0xAD, 0xBE,
                                   0xEF, 0x81, 0x02, 0x05, 0x0E};
    for (std::size_t entry = 0; entry <= full.size(); ++entry) {
        Collector collector;
        const std::span<const std::uint8_t> cargo(full.data(), entry);
        (void)parse_advertisement(cargo, collector.sink());
    }
    SUCCEED();
}

// SR-10: a GUID is at most four bytes -- this library's limit, UNCONFIRMED
// against 1000-3535 §5.2 (NR-4). A longer value names no
// application the roles could be bound to, and parsing it would shift past the
// width of the GUID.
TEST(Advertisement, RejectsAGuidLongerThanFourBytes) {
    std::array<std::uint8_t, 7> cargo{0x01, 0x05, 0x02, 0x00, 0x00, 0x00, 0x07};
    Collector collector;
    EXPECT_EQ(parse_advertisement(cargo, collector.sink()).error(),
              Sh2Error::PROTOCOL);
}

}  // namespace
