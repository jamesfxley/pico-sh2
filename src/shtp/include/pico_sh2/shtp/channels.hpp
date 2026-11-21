// SPDX-License-Identifier: MIT
#pragma once

/// \file channels.hpp
/// SHTP channel roles and advertised wire numbers.
/// Channel 0 is fixed (1000-3535 §5.1). SH-2's base may be selected at compile
/// time or run time, depending on the implementation (1000-3600 §2.3).

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>

namespace pico_sh2::shtp {

/// SHTP command role (1000-3535 §5.1) and SH-2 roles (1000-3600 §2.2).
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

/// Channels with sequence tracking; higher channels route without
/// sequence-gap accounting.
inline constexpr std::size_t MAX_TRACKED_CHANNELS = 16;

/// Role-to-number mapping; seeds support operations that must work before
/// advertisement, which overrides the seeds (SR-10).
class ChannelMap {
   public:
    /// The command channel is known without an advertisement; every other role
    /// starts unbound.
    constexpr ChannelMap() { seed(ChannelRole::COMMAND, COMMAND_CHANNEL); }

    /// Seed a role for early reset requests; advertisement takes precedence.
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

    /// True once every role has a number, allowing session routing.
    [[nodiscard]] constexpr bool complete() const {
        return std::ranges::all_of(bound_, [](bool bound) { return bound; });
    }

    /// Discard learned numbers that can change at reset; keep seeds for early
    /// requests.
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

/// Reserved raw length; a failed peripheral can produce it (1000-3535 §2.2.1).
inline constexpr std::uint16_t RESERVED_LENGTH = 0xFFFF;

/// Continuation flag lives in bit 15 of the 16-bit length field.
inline constexpr std::uint16_t CONTINUATION_MASK = 0x8000;
/// The length occupies the remaining 15 bits of that field.
inline constexpr std::uint16_t LENGTH_MASK = 0x7FFF;

/// Transfer header length in bytes, counted in the length field.
inline constexpr std::size_t HEADER_SIZE = 4;

}  // namespace pico_sh2::shtp
