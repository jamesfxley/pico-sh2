// SPDX-License-Identifier: MIT
#pragma once

/// \file channels.hpp
/// SHTP channel assignment: what a channel is *for*, and which wire number
/// carries it. The two are not the same thing and only one of them is fixed.
///
/// SHTP fixes exactly one number: "The command channel is channel 0"
/// (1000-3535 §5.1). SH-2's own channels sit at N..N+4 for a base N the hub
/// chooses, and 1000-3600 §2.3 is explicit that "the value for N may be
/// selected at compile time or run time depending on the specific
/// implementation of SH-2". A number written into this header would therefore
/// be a property of one part, not of the protocol, so the numbers are learned
/// from the hub's channel-0 advertisement instead.
///
/// Roles are matched to advertised channels by name, scoped to the advertising
/// application's GUID. The scoping is not optional: SHTP's own application and
/// the SH-2 sensorhub both name a channel "control", so a name-only match binds
/// the SH-2 control role to the command channel.

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>

namespace pico_sh2::shtp {

/// What a channel is used for. COMMAND is SHTP's own (1000-3535 §5.1); the rest
/// are SH-2's, named in 1000-3600 §2.2 as device, control, inputNormal,
/// inputWake and inputGyroRv.
enum class ChannelRole : std::uint8_t {
    COMMAND = 0,
    DEVICE,
    CONTROL,
    INPUT_NORMAL,
    INPUT_WAKE,
    INPUT_GYRO_RV,
};

/// Number of values in ChannelRole, sizing the role-to-number map.
inline constexpr std::size_t NUM_ROLES = 6;

/// The one channel number the protocol fixes (1000-3535 §5.1).
inline constexpr std::uint8_t COMMAND_CHANNEL = 0;

/// How many channel numbers the framing engine keeps sequence state for. The
/// channel field is one byte, so tracking every possible value would cost far
/// more than it is worth; SH-2 needs the command channel plus five at N..N+4,
/// and this covers every base N up to 11. A channel above it is still routed
/// and delivered — only its sequence-gap accounting is skipped.
inline constexpr std::size_t MAX_TRACKED_CHANNELS = 16;

/// SR-10: role to wire number, and back. Seeded by the consumer for the
/// operations that must work before a hub has advertised, then corrected by the
/// advertisement itself.
class ChannelMap {
   public:
    /// The command channel is known without an advertisement; every other role
    /// starts unbound.
    constexpr ChannelMap() { seed(ChannelRole::COMMAND, COMMAND_CHANNEL); }

    /// Supply a role's number ahead of the first advertisement. A consumer that
    /// knows its part uses this so a reset can be requested before the
    /// handshake completes; the advertisement overrides whatever is seeded
    /// here, so a wrong seed costs nothing once the hub has spoken.
    constexpr void seed(ChannelRole role, std::uint8_t number) {
        const std::size_t i = index(role);
        seed_[i] = number;
        seeded_[i] = true;
        if (!learned_[i]) {
            number_[i] = number;
            bound_[i] = true;
        }
    }

    /// Bind a role from the advertisement. Takes precedence over a seed.
    constexpr void learn(ChannelRole role, std::uint8_t number) {
        const std::size_t i = index(role);
        number_[i] = number;
        bound_[i] = true;
        learned_[i] = true;
    }

    /// \param role the role to look up.
    /// \return the channel number bound to `role`, or nothing if unbound.
    [[nodiscard]] constexpr std::optional<std::uint8_t> number_of(
        ChannelRole role) const {
        const std::size_t i = index(role);
        return bound_[i] ? std::optional<std::uint8_t>(number_[i])
                         : std::nullopt;
    }

    /// \param number the channel number to look up.
    /// \return the role bound to `number`, or nothing if it is not one of
    ///         SH-2's channels.
    [[nodiscard]] constexpr std::optional<ChannelRole> role_of(
        std::uint8_t number) const {
        for (std::size_t i = 0; i < NUM_ROLES; ++i) {
            if (bound_[i] && number_[i] == number) {
                return static_cast<ChannelRole>(i);
            }
        }
        return std::nullopt;
    }

    /// True once every role has a number. The session needs this before it can
    /// route, which is why it is part of reaching ready.
    [[nodiscard]] constexpr bool complete() const {
        return std::ranges::all_of(bound_, [](bool bound) { return bound; });
    }

    /// Discard everything an advertisement taught, leaving the seed. A hub that
    /// reset may come back with a different N, so learned numbers cannot be
    /// trusted across one; the seed survives so a reset can still be requested.
    constexpr void forget_learned() {
        for (std::size_t i = 0; i < NUM_ROLES; ++i) {
            learned_[i] = false;
            bound_[i] = seeded_[i];
            number_[i] = seeded_[i] ? seed_[i] : 0;
        }
    }

   private:
    static constexpr std::size_t index(ChannelRole role) {
        return static_cast<std::size_t>(role);
    }

    std::array<std::uint8_t, NUM_ROLES> number_{};
    std::array<std::uint8_t, NUM_ROLES> seed_{};
    std::array<bool, NUM_ROLES> bound_{};
    std::array<bool, NUM_ROLES> seeded_{};
    std::array<bool, NUM_ROLES> learned_{};
};

/// A header length field of 0xFFFF is reserved and signals a dead/failed bus
/// read; it is never a valid cargo length.
inline constexpr std::uint16_t RESERVED_LENGTH = 0xFFFF;

/// Continuation flag lives in bit 15 of the 16-bit length field.
inline constexpr std::uint16_t CONTINUATION_MASK = 0x8000;
/// The length occupies the remaining 15 bits of that field.
inline constexpr std::uint16_t LENGTH_MASK = 0x7FFF;

/// Every SHTP transfer is prefixed by a 4-byte header (len LSB, len MSB,
/// channel, sequence). The length field counts these 4 bytes.
inline constexpr std::size_t HEADER_SIZE = 4;

}  // namespace pico_sh2::shtp
