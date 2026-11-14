// SPDX-License-Identifier: MIT
#pragma once

/// \file profile.hpp
/// Compile-time buffer sizes, checked against protocol bounds.
/// PICO_SH2_ACTIVE_PROFILE selects the type; DefaultProfile is the default.

#include <concepts>
#include <cstddef>
#include <cstdint>

#include "pico_sh2/shtp/channels.hpp"

namespace pico_sh2 {

/// Minimum receive-transfer capacity in bytes, including header and margin.
inline constexpr std::size_t MIN_RX_TRANSFER = 32;
/// Set Feature capacity bound in bytes, including the four-byte header.
inline constexpr std::size_t SET_FEATURE_CARGO_SIZE = 21;
/// Upper bound on concurrently enabled sensors any profile may request.
inline constexpr std::size_t MAX_ENABLED_SENSORS_CEILING = 64;
/// Largest FRS record the manual bounds: the User Record, capped at 64 words
/// (1000-3625 §4.3.21). Every other record is a fixed layout well under it.
inline constexpr std::size_t FRS_USER_RECORD_WORDS = 64;
/// Largest FRS record the wire can address: the write request's length and the
/// write data's offset are 16-bit word counts (1000-3625 §6.3.3, §6.3.4).
inline constexpr std::size_t FRS_MAX_RECORD_WORDS = 0xFFFF;

/// Minimum reassembly capacity in bytes; must hold the largest outgoing cargo
/// (AR-5).
inline constexpr std::size_t MIN_CARGO_LEN = SET_FEATURE_CARGO_SIZE;

/// The interface a sizing profile must supply (AR-5).
/// Each knob must be std::size_t; signed negatives could pass lower bounds.
/// A static member is an lvalue in requires, hence const std::size_t&.
template <typename P>
concept Sh2Profile = requires {
    { P::MAX_RX_TRANSFER_LEN } -> std::same_as<const std::size_t&>;
    { P::MAX_CARGO_LEN } -> std::same_as<const std::size_t&>;
    { P::MAX_TX_CARGO } -> std::same_as<const std::size_t&>;
    { P::TX_QUEUE_DEPTH } -> std::same_as<const std::size_t&>;
    { P::MAX_ENABLED_SENSORS } -> std::same_as<const std::size_t&>;
    { P::FRS_MAX_WORDS } -> std::same_as<const std::size_t&>;
    { P::MAX_TRANSFERS_PER_SERVICE } -> std::same_as<const std::size_t&>;
    { P::SESSION_SIZE_CEILING } -> std::same_as<const std::size_t&>;
};

/// A single-IMU host driving a handful of sensors.
struct DefaultProfile {
    /// Receive-transfer capacity in bytes.
    static constexpr std::size_t MAX_RX_TRANSFER_LEN = 288;
    /// Reassembly capacity in bytes.
    static constexpr std::size_t MAX_CARGO_LEN = 384;
    /// Outbound payload capacity in bytes, with headroom above Set Feature.
    static constexpr std::size_t MAX_TX_CARGO = 32;
    /// Outbound queue capacity in cargoes; any depth of at least two works.
    static constexpr std::size_t TX_QUEUE_DEPTH = 4;
    /// Consumer enabled-sensor capacity; it does not size Session.
    static constexpr std::size_t MAX_ENABLED_SENSORS = 8;
    /// Largest FRS record (32-bit words) buffered for a read or write.
    static constexpr std::size_t FRS_MAX_WORDS = 64;
    /// Maximum transfers per service() call; remaining input waits for the next
    /// call.
    static constexpr std::size_t MAX_TRANSFERS_PER_SERVICE = 64;
    /// Maximum sizeof(Session) in bytes, including 64-bit host pointers.
    static constexpr std::size_t SESSION_SIZE_CEILING = 2304;
};

/// Larger capacities, inheriting unchanged and newly added defaults.
struct LargeProfile : DefaultProfile {
    /// Receive-transfer capacity in bytes.
    static constexpr std::size_t MAX_RX_TRANSFER_LEN = 640;
    /// Reassembly capacity in bytes.
    static constexpr std::size_t MAX_CARGO_LEN = 1024;
    /// Outbound cargo queue depth.
    static constexpr std::size_t TX_QUEUE_DEPTH = 8;
    /// Concurrently enabled sensors.
    static constexpr std::size_t MAX_ENABLED_SENSORS = 16;
    /// Largest FRS record (32-bit words) buffered for a read or write.
    static constexpr std::size_t FRS_MAX_WORDS = 128;
    /// Maximum sizeof(Session) in bytes.
    static constexpr std::size_t SESSION_SIZE_CEILING = 3840;
};

static_assert(Sh2Profile<DefaultProfile>);
static_assert(Sh2Profile<LargeProfile>);

/// Named profile bounds, exposed for tests of invalid profiles (AR-5).
/// \tparam P the profile to check.
template <Sh2Profile P>
struct ProfileBounds {
    /// The largest cargo the library builds fits.
    static constexpr bool CARGO_LEN = P::MAX_CARGO_LEN >= MIN_CARGO_LEN;
    /// A minimal transfer fits.
    static constexpr bool RX_TRANSFER_MIN =
        P::MAX_RX_TRANSFER_LEN >= MIN_RX_TRANSFER;
    /// A transfer is no more than one cargo and its header.
    static constexpr bool RX_TRANSFER_MAX =
        P::MAX_RX_TRANSFER_LEN <= P::MAX_CARGO_LEN + shtp::HEADER_SIZE;
    /// A Set Feature cargo fits.
    static constexpr bool TX_CARGO = P::MAX_TX_CARGO >= SET_FEATURE_CARGO_SIZE;
    /// At least two cargoes can be staged.
    static constexpr bool TX_QUEUE = P::TX_QUEUE_DEPTH >= 2;
    /// The enabled-sensor table is between four and the ceiling.
    static constexpr bool ENABLED_SENSORS =
        P::MAX_ENABLED_SENSORS >= 4 &&
        P::MAX_ENABLED_SENSORS <= MAX_ENABLED_SENSORS_CEILING;
    /// The largest bounded FRS record fits.
    static constexpr bool FRS_WORDS_MIN =
        P::FRS_MAX_WORDS >= FRS_USER_RECORD_WORDS;
    /// A record's word count and offsets fit the request's 16-bit fields.
    static constexpr bool FRS_WORDS_MAX =
        P::FRS_MAX_WORDS <= FRS_MAX_RECORD_WORDS;
    /// At least one transfer is admitted per service().
    static constexpr bool TRANSFERS_PER_SERVICE =
        P::MAX_TRANSFERS_PER_SERVICE >= 1;
    /// Minimum storage in bytes for staging, reassembly, FRS words and coverage
    /// bits.
    static constexpr std::size_t BUFFER_BYTES =
        P::MAX_RX_TRANSFER_LEN + P::MAX_CARGO_LEN +
        (P::FRS_MAX_WORDS * sizeof(std::uint32_t)) +
        (((P::FRS_MAX_WORDS + 31) / 32) * sizeof(std::uint32_t));
    /// The session ceiling clears the buffers the same profile asks for.
    static constexpr bool SESSION_CEILING =
        P::SESSION_SIZE_CEILING >= BUFFER_BYTES;
    /// Every bound at once.
    static constexpr bool ALL =
        CARGO_LEN && RX_TRANSFER_MIN && RX_TRANSFER_MAX && TX_CARGO &&
        TX_QUEUE && ENABLED_SENSORS && FRS_WORDS_MIN && FRS_WORDS_MAX &&
        TRANSFERS_PER_SERVICE && SESSION_CEILING;
};

}  // namespace pico_sh2

// A custom profile follows DefaultProfile so it may inherit from it;
// the header defines its type and PICO_SH2_ACTIVE_PROFILE (AR-5).
#ifdef PICO_SH2_PROFILE_HEADER
#include PICO_SH2_PROFILE_HEADER
#endif

namespace pico_sh2 {

#ifndef PICO_SH2_ACTIVE_PROFILE
/// The profile every buffer is sized from. The build sets it from the
/// PICO_SH2_PROFILE cache variable; DefaultProfile when unset.
#define PICO_SH2_ACTIVE_PROFILE ::pico_sh2::DefaultProfile
#endif

/// The profile PICO_SH2_ACTIVE_PROFILE selected, named as a type.
using ActiveProfile = PICO_SH2_ACTIVE_PROFILE;

// This check diagnoses missing profile knobs before buffer use.
static_assert(Sh2Profile<ActiveProfile>,
              "PICO_SH2_ACTIVE_PROFILE must name a type satisfying Sh2Profile: "
              "see the knob table in docs/technical-specification.md");

/// Compile-time checks of active-profile bounds.
namespace profile_invariants {
/// The active profile's bounds, which every assert below reads.
using Bounds = ProfileBounds<ActiveProfile>;
static_assert(Bounds::CARGO_LEN,
              "MAX_CARGO_LEN must hold the largest cargo the library builds");
static_assert(Bounds::RX_TRANSFER_MIN,
              "MAX_RX_TRANSFER_LEN too small for a minimal transfer");
static_assert(Bounds::RX_TRANSFER_MAX,
              "a single transfer cannot exceed one cargo plus header");
static_assert(Bounds::TX_CARGO, "MAX_TX_CARGO must hold a Set Feature cargo");
// The ring counts elements, so the depth need not be a power of two.
static_assert(Bounds::TX_QUEUE,
              "TX_QUEUE_DEPTH must hold at least two cargoes");
static_assert(Bounds::ENABLED_SENSORS,
              "MAX_ENABLED_SENSORS out of supported range");
static_assert(Bounds::FRS_WORDS_MIN,
              "FRS_MAX_WORDS must hold the largest bounded FRS record");
static_assert(Bounds::FRS_WORDS_MAX,
              "FRS_MAX_WORDS beyond a 16-bit word count cannot be written: "
              "the request's length and offset would truncate");
static_assert(Bounds::TRANSFERS_PER_SERVICE,
              "MAX_TRANSFERS_PER_SERVICE must admit at least one transfer");
// This catches bounds added to ALL without an individual assertion.
static_assert(Bounds::ALL, "the active profile is outside AR-5's ranges");

/// The storage the active profile's knobs demand of a Session, in bytes.
inline constexpr std::size_t PROFILE_BUFFER_BYTES = Bounds::BUFFER_BYTES;

// This diagnoses an impossible profile ceiling before Session's size check.
static_assert(Bounds::SESSION_CEILING,
              "SESSION_SIZE_CEILING is below the buffers this profile's own "
              "knobs require; raise the ceiling or lower the buffers");
}  // namespace profile_invariants

}  // namespace pico_sh2
