// SPDX-License-Identifier: MIT
#include "pico_sh2/sh2/profile.hpp"

#include <gtest/gtest.h>

#include <cstddef>

#include "pico_sh2/sh2/session.hpp"

namespace pico_sh2 {
namespace {

// binding_matches() and the constructor agree for a matching build.
TEST(Profile, BindingMatchesTheLibrarysOwnView) {
    EXPECT_TRUE(binding_matches());
    EXPECT_EQ(library_binding(), Session::caller_binding());
}

// A profile missing a knob must not satisfy the concept (AR-5).
// The failure is reported before engine instantiation.
struct MissingFrsWords {
    static constexpr std::size_t MAX_RX_TRANSFER_LEN = 288;
    static constexpr std::size_t MAX_CARGO_LEN = 384;
    static constexpr std::size_t MAX_TX_CARGO = 32;
    static constexpr std::size_t TX_QUEUE_DEPTH = 4;
    static constexpr std::size_t MAX_ENABLED_SENSORS = 8;
    static constexpr std::size_t MAX_TRANSFERS_PER_SERVICE = 64;
    static constexpr std::size_t SESSION_SIZE_CEILING = 2304;
};

// The same missing-knob check for the service drain bound.
struct MissingTransfersPerService {
    static constexpr std::size_t MAX_RX_TRANSFER_LEN = 288;
    static constexpr std::size_t MAX_CARGO_LEN = 384;
    static constexpr std::size_t MAX_TX_CARGO = 32;
    static constexpr std::size_t TX_QUEUE_DEPTH = 4;
    static constexpr std::size_t MAX_ENABLED_SENSORS = 8;
    static constexpr std::size_t FRS_MAX_WORDS = 64;
    static constexpr std::size_t SESSION_SIZE_CEILING = 2304;
};

// An int knob admits -1, which converts to the largest std::size_t
// and passes the lower bounds (AR-5).
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

// Literal specification bounds detect drift in profile.hpp; each knob
// is varied independently over DefaultProfile (AR-5).
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

// Each range accepts its boundary and rejects one past it (AR-5).
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
    // The minimum ceiling includes the record coverage bitmap.
    constexpr std::size_t BUFFERS = 288 + 384 + (4 * 64) + (4 * 2);
    static_assert(ProfileBounds<Ceiling<BUFFERS>>::SESSION_CEILING);
    static_assert(!ProfileBounds<Ceiling<BUFFERS - 1>>::SESSION_CEILING);
    SUCCEED();
}

// ALL includes every bound checked by the active profile's asserts (AR-5).
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

// The concept rejects a knob whose type is not std::size_t (AR-5).
TEST(Profile, ConceptRejectsAKnobThatIsNotASize) {
    // Reads the knobs the concept stops before, as the near misses below do.
    static_assert(SignedFrsWords::MAX_TRANSFERS_PER_SERVICE > 0);
    static_assert(SignedFrsWords::SESSION_SIZE_CEILING > 0);
    static_assert(SignedFrsWords::FRS_MAX_WORDS < 0);
    static_assert(!Sh2Profile<SignedFrsWords>);
}

TEST(Profile, ConceptRejectsAProfileMissingAKnob) {
    // Accesses keep each struct a near miss only in its missing knob.
    static_assert(MissingFrsWords::MAX_TRANSFERS_PER_SERVICE > 0);
    static_assert(MissingFrsWords::SESSION_SIZE_CEILING > 0);
    static_assert(!Sh2Profile<MissingFrsWords>);
    static_assert(MissingTransfersPerService::SESSION_SIZE_CEILING > 0);
    static_assert(!Sh2Profile<MissingTransfersPerService>);
    static_assert(Sh2Profile<DefaultProfile>);
    static_assert(Sh2Profile<LargeProfile>);
    SUCCEED();
}

// LargeProfile must inherit knobs it does not override, including MAX_TX_CARGO.
TEST(Profile, LargeInheritsTheKnobsItDoesNotChange) {
    EXPECT_EQ(LargeProfile::MAX_TX_CARGO, DefaultProfile::MAX_TX_CARGO);
    EXPECT_GT(LargeProfile::MAX_CARGO_LEN, DefaultProfile::MAX_CARGO_LEN);
    EXPECT_GT(LargeProfile::FRS_MAX_WORDS, DefaultProfile::FRS_MAX_WORDS);
}

// The minimum ceiling covers this profile's buffers and FRS coverage bitmap,
// catching an impossible ceiling before the Session size assert (AR-5).
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
