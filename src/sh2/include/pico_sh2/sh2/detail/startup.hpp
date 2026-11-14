// SPDX-License-Identifier: MIT
#pragma once

/// \file startup.hpp
/// Internal reset, boot and announcement sequencing with phase deadlines
/// (SR-1). Advertisement precedes reset-complete and Initialize (1000-3535
/// §2.3.1, 1000-3600 §2.2, 1000-3625 §6.4.4); their mutual order is
/// UNCONFIRMED. The sequencer tracks both independently; Session performs bus
/// actions and counts events.

#include <cstdint>

#include "pico_sh2/sh2/detail/deadline.hpp"
#include "pico_sh2/sh2/session_timing.hpp"
#include "pico_sh2/sh2/startup_phase.hpp"

namespace pico_sh2::detail {

using StartupPhase = pico_sh2::StartupPhase;

class StartupSequencer {
   public:
    explicit StartupSequencer(SessionTiming timing) : timing_(timing) {}

    /// What a tick asks of the caller.
    struct Tick {
        bool release_reset = false;  // the reset hold has elapsed
        bool timed_out = false;      // the current phase passed its deadline
    };

    /// Start the sequence with the reset line asserted.
    void begin(std::uint64_t now_us) {
        saw_reset_complete_ = false;
        saw_initialize_ = false;
        advertisement_requested_ = false;
        phase_ = StartupPhase::RESET_ASSERTED;
        phase_start_us_ = now_us;
        deadline_us_ = 0;
    }

    /// Restart the handshake without driving reset; the hub is already
    /// restarting (SR-5).
    void restart(std::uint64_t now_us) {
        saw_reset_complete_ = false;
        saw_initialize_ = false;
        enter(StartupPhase::WAIT_ADVERTISEMENT, now_us,
              timing_.advertisement_timeout_us);
    }

    /// Advance the timed phases and test the current deadline.
    Tick tick(std::uint64_t now_us) {
        Tick result;
        if (phase_ == StartupPhase::RESET_ASSERTED) {
            if (now_us - phase_start_us_ >= timing_.reset_hold_us) {
                phase_ = StartupPhase::BOOTING;
                phase_start_us_ = now_us;
                result.release_reset = true;
            }
            return result;
        }
        if (phase_ == StartupPhase::BOOTING) {
            if (now_us - phase_start_us_ >= timing_.boot_delay_us) {
                enter(StartupPhase::WAIT_ADVERTISEMENT, now_us,
                      timing_.advertisement_timeout_us);
            }
            return result;
        }
        result.timed_out = deadline_us_ != 0 && now_us >= deadline_us_;
        return result;
    }

    /// True during timed reset or boot, before traffic is expected.
    [[nodiscard]] bool in_reset_window() const {
        return phase_ == StartupPhase::RESET_ASSERTED ||
               phase_ == StartupPhase::BOOTING;
    }

    /// Start announcement waits; a requested advertisement keeps prior
    /// announcements.
    void on_advertisement(std::uint64_t now_us) {
        if (!advertisement_requested_) {
            saw_reset_complete_ = false;
            saw_initialize_ = false;
        }
        advertisement_requested_ = false;
        advance(now_us);
    }

    /// Records reset-complete while awaiting a requested advertisement; the
    /// deadline set by restart() remains (SR-5).
    void on_reset_complete_without_advertisement() {
        saw_reset_complete_ = true;
        advertisement_requested_ = true;
    }

    /// True if the reset-complete is expected in the current phase.
    [[nodiscard]] bool accepts_reset_complete() const {
        return phase_ == StartupPhase::WAIT_RESET_COMPLETE ||
               phase_ == StartupPhase::WAIT_INITIALIZE;
    }

    /// True when Initialize is expected, including while an advertisement is
    /// requested.
    [[nodiscard]] bool accepts_initialize() const {
        return accepts_reset_complete() || advertisement_requested_;
    }

    void on_reset_complete(std::uint64_t now_us) {
        saw_reset_complete_ = true;
        advance(now_us);
    }

    void on_initialize(std::uint64_t now_us) {
        saw_initialize_ = true;
        if (!advertisement_requested_) {
            advance(now_us);
        }
    }

    [[nodiscard]] StartupPhase phase() const { return phase_; }

    [[nodiscard]] bool complete() const {
        return phase_ == StartupPhase::COMPLETE;
    }

    [[nodiscard]] bool idle() const { return phase_ == StartupPhase::IDLE; }

   private:
    void enter(StartupPhase phase, std::uint64_t now_us,
               std::uint64_t timeout_us) {
        phase_ = phase;
        deadline_us_ = deadline_after(now_us, timeout_us);
    }

    /// Complete after both announcements; otherwise report the one still
    /// awaited.
    void advance(std::uint64_t now_us) {
        if (saw_reset_complete_ && saw_initialize_) {
            phase_ = StartupPhase::COMPLETE;
            deadline_us_ = 0;  // a running session carries no phase deadline
        } else if (saw_reset_complete_) {
            enter(StartupPhase::WAIT_INITIALIZE, now_us,
                  timing_.initialize_timeout_us);
        } else {
            enter(StartupPhase::WAIT_RESET_COMPLETE, now_us,
                  timing_.reset_complete_timeout_us);
        }
    }

    SessionTiming timing_;
    StartupPhase phase_ = StartupPhase::IDLE;
    std::uint64_t phase_start_us_ = 0;  // for the timed reset/boot phases
    std::uint64_t deadline_us_ = 0;     // absolute; 0 = no deadline
    bool saw_reset_complete_ = false;
    bool saw_initialize_ = false;
    bool advertisement_requested_ = false;  // held announcements await it
};

}  // namespace pico_sh2::detail
