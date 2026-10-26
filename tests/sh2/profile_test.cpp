// SPDX-License-Identifier: MIT
#include "pico_sh2/sh2/profile.hpp"

#include <gtest/gtest.h>

#include <cstddef>

#include "pico_sh2/sh2/session.hpp"

namespace pico_sh2 {
namespace {

// The constructor aborts on a profile disagreement, which on a nosys target is
// a board dead at boot with nothing said. binding_matches() is what a consumer
// tests and logs first, so it has to agree with the constructor's own check --
// here, in a build where the two necessarily match.
TEST(Profile, BindingMatchesTheLibrarysOwnView) {
    EXPECT_TRUE(binding_matches());
    EXPECT_EQ(library_binding(), Session::caller_binding());
}

// AR-5. The concept is what a downstream profile is checked against. A struct
// missing one knob must not satisfy it -- otherwise the failure moves back
// inside FrsEngine, which is the diagnostic the concept exists to replace.
struct MissingFrsWords {
    static constexpr std::size_t MAX_RX_TRANSFER_LEN = 288;
    static constexpr std::size_t MAX_CARGO_LEN = 384;
    static constexpr std::size_t MAX_TX_CARGO = 32;
    static constexpr std::size_t TX_QUEUE_DEPTH = 4;
    static constexpr std::size_t MAX_ENABLED_SENSORS = 8;
    static constexpr std::size_t MAX_TRANSFERS_PER_SERVICE = 64;
    static constexpr std::size_t SESSION_SIZE_CEILING = 2304;
};

// The same near miss for the drain bound, which the session reads in
// service() and a profile missing it would otherwise fail inside.
struct MissingTransfersPerService {
    static constexpr std::size_t MAX_RX_TRANSFER_LEN = 288;
    static constexpr std::size_t MAX_CARGO_LEN = 384;
    static constexpr std::size_t MAX_TX_CARGO = 32;
    static constexpr std::size_t TX_QUEUE_DEPTH = 4;
    static constexpr std::size_t MAX_ENABLED_SENSORS = 8;
    static constexpr std::size_t FRS_MAX_WORDS = 64;
    static constexpr std::size_t SESSION_SIZE_CEILING = 2304;
};

// A knob of another type, which a conversion would take in: -1 becomes the
// largest std::size_t and passes every lower bound AR-5 sets.
struct SignedFrsWords {
    static constexpr std::size_t MAX_RX_TRANSFER_LEN = 288;
    static constexpr std::size_t MAX_CARGO_LEN = 384;
    static constexpr std::size_t MAX_TX_CARGO = 32;
    static constexpr std::size_t TX_QUEUE_DEPTH = 4;
    static constexpr std::size_t MAX_ENABLED_SENSORS = 8;
    static constexpr int FRS_MAX_WORDS = -1;
    static constexpr std::size_t MAX_TRANSFERS_PER_SERVICE = 64;
    static constexpr std::size_t SESSION_SIZE_CEILING = 2304;
};

// AR-5's knob table, written out as the specification states it rather than
// through the constants profile.hpp compares with, so a constant that drifts
// from the table fails here. Each knob is set on its own over DefaultProfile.
template <std::size_t V>
struct Cargo : DefaultProfile {
    static constexpr std::size_t MAX_CARGO_LEN = V;
};

template <std::size_t V>
struct RxTransfer : DefaultProfile {
    static constexpr std::size_t MAX_RX_TRANSFER_LEN = V;
};

template <std::size_t V>
struct TxCargo : DefaultProfile {
    static constexpr std::size_t MAX_TX_CARGO = V;
};

template <std::size_t V>
struct TxQueue : DefaultProfile {
    static constexpr std::size_t TX_QUEUE_DEPTH = V;
};

template <std::size_t V>
struct Sensors : DefaultProfile {
    static constexpr std::size_t MAX_ENABLED_SENSORS = V;
};

template <std::size_t V>
struct FrsWords : DefaultProfile {
    static constexpr std::size_t FRS_MAX_WORDS = V;
};

template <std::size_t V>
struct Transfers : DefaultProfile {
    static constexpr std::size_t MAX_TRANSFERS_PER_SERVICE = V;
};

template <std::size_t V>
struct Ceiling : DefaultProfile {
    static constexpr std::size_t SESSION_SIZE_CEILING = V;
};

// AR-5: each range holds at its limit and refuses one past it, so a bound
// deleted from ProfileBounds -- which the active profile's static_asserts read
// -- fails here rather than letting an out-of-range profile compile.
TEST(Profile, EachBoundHoldsAtItsLimitAndRefusesOnePast) {
    static_assert(ProfileBounds<Cargo<21>>::CARGO_LEN);
    static_assert(!ProfileBounds<Cargo<20>>::CARGO_LEN);
    static_assert(ProfileBounds<RxTransfer<32>>::RX_TRANSFER_MIN);
    static_assert(!ProfileBounds<RxTransfer<31>>::RX_TRANSFER_MIN);
    constexpr std::size_t CARGO = DefaultProfile::MAX_CARGO_LEN;
    static_assert(ProfileBounds<RxTransfer<CARGO + 4>>::RX_TRANSFER_MAX);
    static_assert(!ProfileBounds<RxTransfer<CARGO + 5>>::RX_TRANSFER_MAX);
    static_assert(ProfileBounds<TxCargo<21>>::TX_CARGO);
    static_assert(!ProfileBounds<TxCargo<20>>::TX_CARGO);
    static_assert(ProfileBounds<TxQueue<2>>::TX_QUEUE);
    static_assert(!ProfileBounds<TxQueue<1>>::TX_QUEUE);
    static_assert(ProfileBounds<Sensors<4>>::ENABLED_SENSORS);
    static_assert(!ProfileBounds<Sensors<3>>::ENABLED_SENSORS);
    static_assert(ProfileBounds<Sensors<64>>::ENABLED_SENSORS);
    static_assert(!ProfileBounds<Sensors<65>>::ENABLED_SENSORS);
    static_assert(ProfileBounds<FrsWords<64>>::FRS_WORDS_MIN);
    static_assert(!ProfileBounds<FrsWords<63>>::FRS_WORDS_MIN);
    static_assert(ProfileBounds<FrsWords<65535>>::FRS_WORDS_MAX);
    static_assert(!ProfileBounds<FrsWords<65536>>::FRS_WORDS_MAX);
    static_assert(ProfileBounds<Transfers<1>>::TRANSFERS_PER_SERVICE);
    static_assert(!ProfileBounds<Transfers<0>>::TRANSFERS_PER_SERVICE);
    // The ceiling's range is the buffers, the FRS record's coverage bits
    // included, that the same profile asks for.
    constexpr std::size_t BUFFERS = 288 + 384 + (4 * 64) + (4 * 2);
    static_assert(ProfileBounds<Ceiling<BUFFERS>>::SESSION_CEILING);
    static_assert(!ProfileBounds<Ceiling<BUFFERS - 1>>::SESSION_CEILING);
    SUCCEED();
}

// AR-5: ALL is every bound, so a bound left out of it is one the catch-all
// assert would not hold the active profile to.
TEST(Profile, AllIsEveryBound) {
    static_assert(ProfileBounds<DefaultProfile>::ALL);
    static_assert(ProfileBounds<LargeProfile>::ALL);
    static_assert(!ProfileBounds<Cargo<20>>::ALL);
    static_assert(!ProfileBounds<RxTransfer<31>>::ALL);
    static_assert(
        !ProfileBounds<RxTransfer<DefaultProfile::MAX_CARGO_LEN + 5>>::ALL);
    static_assert(!ProfileBounds<TxCargo<20>>::ALL);
    static_assert(!ProfileBounds<TxQueue<1>>::ALL);
    static_assert(!ProfileBounds<Sensors<3>>::ALL);
    static_assert(!ProfileBounds<Sensors<65>>::ALL);
    static_assert(!ProfileBounds<FrsWords<63>>::ALL);
    static_assert(!ProfileBounds<FrsWords<65536>>::ALL);
    static_assert(!ProfileBounds<Transfers<0>>::ALL);
    static_assert(!ProfileBounds<Ceiling<0>>::ALL);
    SUCCEED();
}

// AR-5: every knob is a std::size_t, so one declared as another type is
// refused by the concept rather than converted.
TEST(Profile, ConceptRejectsAKnobThatIsNotASize) {
    // Reads the knobs the concept stops before, as the near misses below do.
    static_assert(SignedFrsWords::MAX_TRANSFERS_PER_SERVICE > 0);
    static_assert(SignedFrsWords::SESSION_SIZE_CEILING > 0);
    static_assert(SignedFrsWords::FRS_MAX_WORDS < 0);
    static_assert(!Sh2Profile<SignedFrsWords>);
}

TEST(Profile, ConceptRejectsAProfileMissingAKnob) {
    // Reads the knobs the concept stops before, so each struct is a near miss
    // in exactly one respect -- its absent knob -- and not also in being
    // unused.
    static_assert(MissingFrsWords::MAX_TRANSFERS_PER_SERVICE > 0);
    static_assert(MissingFrsWords::SESSION_SIZE_CEILING > 0);
    static_assert(!Sh2Profile<MissingFrsWords>);
    static_assert(MissingTransfersPerService::SESSION_SIZE_CEILING > 0);
    static_assert(!Sh2Profile<MissingTransfersPerService>);
    static_assert(Sh2Profile<DefaultProfile>);
    static_assert(Sh2Profile<LargeProfile>);
    SUCCEED();
}

// LargeProfile inherits from DefaultProfile, so a knob it does not override
// must read through to the default rather than being absent. MAX_TX_CARGO is
// the one they agree on, and it is why the inheritance is not merely tidier: a
// knob added to DefaultProfile later reaches LargeProfile the same way.
TEST(Profile, LargeInheritsTheKnobsItDoesNotChange) {
    EXPECT_EQ(LargeProfile::MAX_TX_CARGO, DefaultProfile::MAX_TX_CARGO);
    EXPECT_GT(LargeProfile::MAX_CARGO_LEN, DefaultProfile::MAX_CARGO_LEN);
    EXPECT_GT(LargeProfile::FRS_MAX_WORDS, DefaultProfile::FRS_MAX_WORDS);
}

// AR-5: the ceiling is the one knob with no protocol minimum, so it is bounded
// by the buffers the same profile asks for --
// MAX_RX_TRANSFER_LEN + MAX_CARGO_LEN + 4 * FRS_MAX_WORDS, plus the FRS
// coverage bitmap's words. Below that it could
// never be met, and the assert that fired would name Session's size rather
// than the profile.
template <typename Profile>
constexpr std::size_t ar5_buffer_bytes() {
    return Profile::MAX_RX_TRANSFER_LEN + Profile::MAX_CARGO_LEN +
           (4 * Profile::FRS_MAX_WORDS) +
           (4 * ((Profile::FRS_MAX_WORDS + 31) / 32));
}

TEST(Profile, CeilingClearsTheBuffersTheProfileAsksFor) {
    EXPECT_EQ(profile_invariants::PROFILE_BUFFER_BYTES,
              ar5_buffer_bytes<ActiveProfile>());
    EXPECT_GE(DefaultProfile::SESSION_SIZE_CEILING,
              ar5_buffer_bytes<DefaultProfile>());
    EXPECT_GE(LargeProfile::SESSION_SIZE_CEILING,
              ar5_buffer_bytes<LargeProfile>());
}

}  // namespace
}  // namespace pico_sh2
