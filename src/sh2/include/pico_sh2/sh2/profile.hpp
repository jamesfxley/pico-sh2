// SPDX-License-Identifier: MIT
#pragma once

/// \file profile.hpp
/// Compile-time sizing profile. Every static buffer and fixed-capacity array in
/// the session is sized from the active profile struct, and the static_asserts
/// at the bottom bind those sizes to protocol invariants so a mis-set knob is a
/// compile error rather than a runtime overflow. The active profile is selected
/// by the PICO_SH2_ACTIVE_PROFILE macro (set by the build's PICO_SH2_PROFILE
/// cache variable, or by the header PICO_SH2_PROFILE_HEADER names); it
/// defaults to DefaultProfile.

#include <concepts>
#include <cstddef>
#include <cstdint>

#include "pico_sh2/shtp/channels.hpp"

namespace pico_sh2 {

/// Protocol-derived minimums the profile knobs are checked against. Named so
/// the static_asserts read by intent and so the wire facts have a single home;
/// the SHTP header size comes from shtp::HEADER_SIZE.
///
/// A minimal SHTP transfer is a header plus a one-byte cargo, with margin.
inline constexpr std::size_t MIN_RX_TRANSFER = 32;
/// Largest host->hub cargo is Set Feature: 17-byte payload + 4-byte header.
inline constexpr std::size_t SET_FEATURE_CARGO_SIZE = 21;
/// Upper bound on concurrently enabled sensors any profile may request.
inline constexpr std::size_t MAX_ENABLED_SENSORS_CEILING = 64;
/// Largest FRS record the manual bounds: the User Record, capped at 64 words
/// (1000-3625 §4.3.21). Every other record is a fixed layout well under it.
inline constexpr std::size_t FRS_USER_RECORD_WORDS = 64;
/// Largest FRS record the wire can address: the write request's length and the
/// write data's offset are 16-bit word counts (1000-3625 §6.3.3, §6.3.4).
inline constexpr std::size_t FRS_MAX_RECORD_WORDS = 0xFFFF;

/// The reassembly buffer must at least hold the largest cargo this library
/// itself builds. Nothing in SHTP bounds it above that: a hub's advertisement
/// is typically far larger (a few hundred bytes) and its size is a property of
/// the part, so sizing a profile to receive one is the consumer's call. An
/// undersized buffer is not a correctness fault — an oversize cargo is dropped
/// whole and counted (BR-1) — but a hub whose advertisement does not fit will
/// never be routed, because the session cannot learn its channels.
inline constexpr std::size_t MIN_CARGO_LEN = SET_FEATURE_CARGO_SIZE;

/// The compile-time interface a sizing profile must supply (AR-5).
///
/// Stating it as a concept turns a mis-specified downstream profile into one
/// diagnostic naming the missing knob, rather than a wall of substitution
/// failures inside FrsEngine or the TX path -- which is where a profile without
/// `FRS_MAX_WORDS` currently fails, several headers away from the mistake.
///
/// Each knob must be a `std::size_t` itself, not merely convert to one: a
/// signed knob of -1 would convert to the largest size and pass every lower
/// bound the profile invariants check. A static data member named in a
/// requires-expression is an lvalue, hence `const std::size_t&`.
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
    /// Largest single transfer staged from the HAL. Must hold the header plus a
    /// typical hub transfer; the advertisement fits so it is read in one shot
    /// on the fast path. Partial reads are legal, so this bounds a transfer,
    /// not correctness.
    static constexpr std::size_t MAX_RX_TRANSFER_LEN = 288;
    /// Largest reassembled cargo. Must hold the hub's advertisement, which is
    /// the largest cargo the handshake depends on; batched input cargoes are
    /// capped by configuration (see spec), and an oversize cargo is dropped
    /// whole rather than truncated.
    static constexpr std::size_t MAX_CARGO_LEN = 384;
    /// Largest host->hub cargo payload. Set Feature is 17 B; add headroom.
    static constexpr std::size_t MAX_TX_CARGO = 32;
    /// Outbound cargo queue depth. Counted, not masked, so any value >= 2
    /// works.
    static constexpr std::size_t TX_QUEUE_DEPTH = 4;
    /// Concurrently enabled sensors. Nothing here is sized by it: a driver
    /// built on this library sizes its desired-sensor table from it, so the
    /// forwarded profile scales the driver and the session together.
    static constexpr std::size_t MAX_ENABLED_SENSORS = 8;
    /// Largest FRS record (32-bit words) buffered for a read or write.
    static constexpr std::size_t FRS_MAX_WORDS = 64;
    /// Transfers drained from the HAL in one service() call. The bound is what
    /// stops a hub asserting data-ready continuously from holding the caller's
    /// context indefinitely; the remainder is drained on the next call, so this
    /// bounds latency rather than throughput.
    static constexpr std::size_t MAX_TRANSFERS_PER_SERVICE = 64;
    /// Ceiling on sizeof(Session), asserted where the class is defined. Bounds
    /// the whole session in one number so a new member cannot grow it
    /// unnoticed. Sized for a 64-bit host, where pointers are twice the
    /// target's, so a firmware build sits comfortably under it.
    static constexpr std::size_t SESSION_SIZE_CEILING = 2304;
};

/// Wider buffers for hubs advertising larger cargoes or many enabled sensors.
///
/// Inherits rather than restating: `MAX_TX_CARGO` is the same here, and an
/// eighth knob added to DefaultProfile reaches this profile instead of silently
/// leaving it behind at whatever the default was when this struct was written.
/// Every member below is one this profile genuinely changes.
struct LargeProfile : DefaultProfile {
    /// Largest single transfer staged from the HAL.
    static constexpr std::size_t MAX_RX_TRANSFER_LEN = 640;
    /// Largest reassembled cargo.
    static constexpr std::size_t MAX_CARGO_LEN = 1024;
    /// Outbound cargo queue depth.
    static constexpr std::size_t TX_QUEUE_DEPTH = 8;
    /// Concurrently enabled sensors.
    static constexpr std::size_t MAX_ENABLED_SENSORS = 16;
    /// Largest FRS record (32-bit words) buffered for a read or write.
    static constexpr std::size_t FRS_MAX_WORDS = 128;
    /// Ceiling on sizeof(Session), asserted where the class is defined.
    static constexpr std::size_t SESSION_SIZE_CEILING = 3840;
};

static_assert(Sh2Profile<DefaultProfile>);
static_assert(Sh2Profile<LargeProfile>);

/// AR-5's range for each knob of a profile, one named bound apiece.
///
/// Public so the bounds can be tested on profiles at and past each one, which
/// the static_asserts on the active profile cannot be: a profile outside a
/// bound does not compile. The asserts below read these, so a bound deleted
/// here is one no profile is held to, and one a test sees go.
///
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
    /// The storage the profile's own knobs demand of a Session, in bytes: the
    /// two staged buffers, the FRS record and one coverage bit per record word.
    /// Every other member is a pointer, a callback or a small engine, and none
    /// of them scales with the profile.
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

// A consumer's own profile (AR-5): the build's PICO_SH2_PROFILE_HEADER names a
// header that defines a profile type -- after DefaultProfile, so it may inherit
// from it -- and defines PICO_SH2_ACTIVE_PROFILE to name that type.
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

// A downstream profile reaches the library through this alias, so this is where
// the concept has to hold. Checked here rather than at each use so the
// diagnostic names the profile rather than the buffer that tried to use it.
static_assert(Sh2Profile<ActiveProfile>,
              "PICO_SH2_ACTIVE_PROFILE must name a type satisfying Sh2Profile: "
              "see the knob table in docs/technical-specification.md");

/// The static_asserts binding the active profile's knobs to protocol
/// invariants, so a mis-set knob fails the build rather than the device.
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
// No power-of-two requirement. The outbound ring counts its elements rather
// than masking an index, so any depth works and TX_QUEUE_DEPTH is simply the
// number of cargoes that can be staged at once.
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
// Every bound has its own assert above, for its message; this one is what a
// bound added to ProfileBounds::ALL without an assert of its own still meets.
static_assert(Bounds::ALL, "the active profile is outside AR-5's ranges");

/// The storage the active profile's knobs demand of a Session, in bytes.
inline constexpr std::size_t PROFILE_BUFFER_BYTES = Bounds::BUFFER_BYTES;

// The ceiling's own bound. A ceiling below the buffers the same
// profile asks for cannot ever be met, so the assert at the bottom of
// session.hpp would fire with a message about Session's size when the mistake
// is in the profile. This says where it is.
static_assert(Bounds::SESSION_CEILING,
              "SESSION_SIZE_CEILING is below the buffers this profile's own "
              "knobs require; raise the ceiling or lower the buffers");
}  // namespace profile_invariants

}  // namespace pico_sh2
