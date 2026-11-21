// SPDX-License-Identifier: MIT
#pragma once

/// \file session.hpp
/// SH-2 session: startup, commands, FRS, routing and counters
/// (SR-1, SR-2, SR-3, OR-1).
/// Every call must use the context that drives service(); concurrent calls race
/// with unsynchronised state (AR-3).
/// Only begin()/service() transact on the bus (AR-3).

#include <array>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <optional>
#include <span>

#include "pico_sh2/reports/decoders.hpp"
#include "pico_sh2/reports/metadata_q.hpp"
#include "pico_sh2/reports/q_override.hpp"
#include "pico_sh2/reports/report_length_table.hpp"
#include "pico_sh2/reports/sensor_event.hpp"
#include "pico_sh2/reports/sensor_id.hpp"
#include "pico_sh2/sh2/command.hpp"
#include "pico_sh2/sh2/detail/command_engine.hpp"
#include "pico_sh2/sh2/detail/frs_engine.hpp"
#include "pico_sh2/sh2/detail/input_router.hpp"
#include "pico_sh2/sh2/detail/startup.hpp"
#include "pico_sh2/sh2/detail/tx_path.hpp"
#include "pico_sh2/sh2/feature.hpp"
#include "pico_sh2/sh2/hal_transport.hpp"
#include "pico_sh2/sh2/profile.hpp"
#include "pico_sh2/sh2/session_timing.hpp"
#include "pico_sh2/sh2/startup_phase.hpp"
#include "pico_sh2/shtp/advertisement.hpp"
#include "pico_sh2/shtp/channels.hpp"
#include "pico_sh2/shtp/engine.hpp"
#include "pico_sh2/util/result.hpp"
#include "pico_util/result/callback.hpp"

namespace pico_sh2 {

/// Session lifecycle; startup_phase() reports progress within startup.
enum class SessionState : std::uint8_t {
    IDLE = 0,  // constructed; begin() not yet called
    STARTING,  // running the reset and handshake sequence
    READY,     // handshake complete; routing sensor traffic
    FAULTED,   // startup failed (deadline or protocol)
};

/// Events reported by the optional allocation-free diagnostics hook (OR-2).
enum class DiagEvent : std::uint8_t {
    STARTUP_FAULT,    // a startup phase failed (deadline, bus error, or status)
    HUB_RESET,        // an unexpected hub reset was recognised (SR-5)
    COMMAND_TIMEOUT,  // an outstanding command passed its deadline
    FRS_TIMEOUT,      // an FRS operation passed its deadline
    WRITE_ERROR,      // the transport write failed; the frame stays queued
};

// The X-macro expands one definition into a struct, visitor and count;
// templates and constexpr functions cannot do this
// (cppcoreguidelines-macro-usage). The expansions are declarations and
// statements; bugprone-macro-parentheses would require parentheses that do not
// compile.
// NOLINTBEGIN(cppcoreguidelines-macro-usage,bugprone-macro-parentheses)
/// Diagnostic counters generated from session_counters.def (OR-1).
struct SessionCounters {
/// One counter from session_counters.def; undefined again after use.
#define PICO_SH2_COUNTER(name, description)                       \
    /** One counter; session_counters.def says what it counts. */ \
    std::uint32_t name = 0;
#include "pico_sh2/sh2/session_counters.def"
#undef PICO_SH2_COUNTER
};

/// Visit every counter as (name, description, value), in declaration order.
/// The visitor is held by value and reused for each counter.
template <typename Visitor>
constexpr void for_each_counter(const SessionCounters& counters,
                                Visitor visit) {
/// One counter from session_counters.def; undefined again after use.
#define PICO_SH2_COUNTER(name, description) \
    visit(#name, description, counters.name);
#include "pico_sh2/sh2/session_counters.def"
#undef PICO_SH2_COUNTER
}

/// Number of counters in the set, for a consumer sizing a fixed array.
inline constexpr std::size_t COUNTER_COUNT = 0
/// One counter from session_counters.def; undefined again after use.
#define PICO_SH2_COUNTER(name, description) +1
#include "pico_sh2/sh2/session_counters.def"
#undef PICO_SH2_COUNTER
    ;

// Closes the region opened above; the X-macro reason is there.
// NOLINTEND(cppcoreguidelines-macro-usage,bugprone-macro-parentheses)

/// Profile values compiled into the caller, checked against the library's
/// layout. A mismatch can corrupt memory; CMake propagates the profile to
/// consumers.
struct SessionBinding {
    /// ActiveProfile::MAX_RX_TRANSFER_LEN, the staging buffer extent.
    std::size_t max_rx_transfer_len;
    /// Reassembly buffer size in bytes.
    std::size_t max_cargo_len;
    /// ActiveProfile::MAX_TX_CARGO, the width of one queued outbound cargo.
    std::size_t max_tx_cargo;
    /// ActiveProfile::TX_QUEUE_DEPTH, the outbound queue's element count.
    std::size_t tx_queue_depth;
    /// ActiveProfile::FRS_MAX_WORDS, the FRS record buffer extent.
    std::size_t frs_max_words;
    /// Enabled-sensor capacity; it sizes consumer tables, not Session.
    std::size_t max_enabled_sensors;
    /// Size of Session in bytes, including members not listed separately.
    std::size_t session_size;
    /// Alignment of Session, paired with session_size.
    std::size_t session_align;

    /// Member-wise profile and layout comparison.
    bool operator==(const SessionBinding&) const = default;
};

/// Reset, handshake, configuration and events over one transport.
class Session {
   public:
    /// Unexpected-reset notification, after the handshake has restarted.
    using ResetSink = pico_util::Callback<void()>;
    /// A Get Feature response carrying the hub's granted configuration.
    using FeatureSink = pico_util::Callback<void(const FeatureConfig&)>;
    /// A command response (solicited or unsolicited).
    using CommandSink = pico_util::Callback<void(const CommandResponse&)>;
    /// Completion with error, record type, borrowed words and optional hub
    /// status. Words stay valid throughout the call; re-entrant reads wait and
    /// writes return BUSY while words are borrowed.
    using FrsSink = pico_util::Callback<void(Sh2Error, std::uint16_t,
                                             std::span<const std::uint32_t>,
                                             std::optional<std::uint8_t>)>;
    /// Flush completion for a sensor (1000-3625 §6.5.7).
    using FlushSink = pico_util::Callback<void(reports::SensorId)>;
    /// Optional allocation-free diagnostics callback; unset means no-op (OR-2).
    using DiagSink = pico_util::Callback<void(DiagEvent)>;

    /// Takes the consumer's timing. `channels` seeds the role-to-channel map
    /// for resets that must work before advertisement (SR-7); advertisement
    /// overrides the seeds. The default binding comes from the caller; a
    /// profile mismatch aborts. Callers leave binding at its default.
    Session(IHalTransport& hal, SessionTiming timing,
            shtp::ChannelMap channels = {},
            SessionBinding binding = caller_binding());

    /// Copying or moving would invalidate pointers held by engines and sinks.
    Session(const Session&) = delete;
    Session& operator=(const Session&) = delete;
    Session(Session&&) = delete;
    Session& operator=(Session&&) = delete;
    ~Session() = default;

    /// Profile values compiled into the caller.
    [[nodiscard]] static constexpr SessionBinding caller_binding() noexcept;

    /// Optional event sink; unset means no-op.
    /// Sinks may call requests or begin(); begin() ends the service pass.
    /// A re-entrant service() returns immediately.
    void set_event_sink(reports::EventSink sink) { event_sink_ = sink; }

    /// \param sink notified when the hub resets unexpectedly.
    void set_reset_sink(ResetSink sink) { reset_sink_ = sink; }

    /// \param sink receives each Get Feature response.
    void set_feature_sink(FeatureSink sink) { feature_sink_ = sink; }

    /// \param sink receives each command response.
    void set_command_sink(CommandSink sink) { command_sink_ = sink; }

    /// \param sink receives each FRS read or write completion.
    void set_frs_sink(FrsSink sink) { frs_sink_ = sink; }

    /// \param sink receives each flush completion.
    void set_flush_sink(FlushSink sink) { flush_sink_ = sink; }

    /// \param sink receives the diagnostics events (OR-2).
    void set_diag_sink(DiagSink sink) { diag_sink_ = sink; }

    // Requests enqueue cargo for service(). They return NOT_READY before READY
    // or QUEUE_FULL with no free slot; neither error sends the cargo (BR-2).

    /// Enable/configure a sensor (Set Feature, 0xFD).
    Result<void> set_feature(const FeatureConfig& config);

    /// Request a sensor's current configuration (Get Feature, 0xFE); the
    /// granted config arrives at the feature sink.
    Result<void> get_feature(reports::SensorId sensor);

    /// Queue a command; only one may be outstanding (SR-2). Another returns
    /// BUSY. INVALID_ARGUMENT means command is outside 1–127 or params exceeds
    /// COMMAND_MAX_PARAMS. Responses reach the command sink; the deadline ends
    /// an unanswered request (TR-1).
    Result<void> send_command(std::uint8_t command,
                              std::span<const std::uint8_t> params = {});

    /// True while a command is outstanding; another returns BUSY (SR-2).
    [[nodiscard]] bool command_pending() const { return commands_.pending(); }

    /// Read a record asynchronously; completion reaches the FRS sink.
    /// BUSY means an FRS operation is running; hub BUSY reaches the sink
    /// (SR-3).
    Result<void> frs_read(std::uint16_t frs_type);

    /// Queue an executable-channel reset without clearing calibration
    /// (1000-3600 §2.2). Works before READY once the channel is known; the hub
    /// re-announces itself (SR-5).
    Result<void> request_reset();

    /// Request outstanding batched data now (0xF0, 1000-3625 §6.5.6).
    /// The hub may also flush other sensors; each completion reaches the flush
    /// sink.
    Result<void> force_flush(reports::SensorId sensor);

    /// Write a record asynchronously; empty words erase it.
    /// words must not exceed the profile's FRS buffer: INVALID_ARGUMENT
    /// otherwise. BUSY means an operation is running; hub BUSY reaches the sink
    /// (SR-3).
    Result<void> frs_write(std::uint16_t frs_type,
                           std::span<const std::uint32_t> words);

    /// True while an FRS operation runs here or remains on the hub (SR-3).
    /// A new operation returns BUSY.
    [[nodiscard]] bool frs_busy() const {
        return frs_.busy() || frs_.holding();
    }

    /// Start the non-blocking handshake; invalid timing faults with
    /// INVALID_ARGUMENT and calls no sink. Otherwise assert reset and enter
    /// STARTING; service() advances startup.
    void begin();

    /// Advance startup, drain and route transfers, then enforce the phase
    /// deadline.
    void service();

    /// Where the session is in its lifecycle.
    [[nodiscard]] SessionState state() const { return state_; }

    /// Whether the handshake is complete and sensor traffic is routed.
    [[nodiscard]] bool ready() const { return state_ == SessionState::READY; }

    /// Whether startup failed; fault() names the error and startup_phase() the
    /// step it happened in.
    [[nodiscard]] bool faulted() const {
        return state_ == SessionState::FAULTED;
    }

    /// The error that moved the session to FAULTED (NONE otherwise).
    [[nodiscard]] Sh2Error fault() const { return fault_; }

    /// Startup progress, preserved at the failed phase while FAULTED.
    [[nodiscard]] StartupPhase startup_phase() const {
        return startup_.phase();
    }

    /// Normative diagnostic counters, updated as the session runs (OR-1).
    [[nodiscard]] const SessionCounters& counters() const { return counters_; }

    /// The report lengths the hub advertised, as the decoders see them.
    [[nodiscard]] const reports::ReportLengthTable& report_lengths() const {
        return input_.lengths();
    }

    /// Replace decode-time Q points; overrides survive hub resets (DR-2).
    /// INVALID_ARGUMENT means a Q exceeds reports::MAX_Q_POINT and would make
    /// the shift undefined; TABLE_FULL means no room for a new entry.
    Result<void> set_report_q(std::uint8_t report_id,
                              reports::ReportQ q_points) {
        return input_.set_report_q(report_id, q_points);
    }

    /// Apply metadata-record Q points to its sensor (1000-3625 §5.1) (DR-2).
    /// UNSUPPORTED means no decoded metadata type; INVALID_ARGUMENT means a
    /// short record or out-of-range Q; TABLE_FULL means no room for a new
    /// override.
    Result<void> apply_sensor_metadata(std::uint16_t record_type,
                                       std::span<const std::uint32_t> words) {
        std::optional<std::uint8_t> report_id =
            reports::report_id_for_metadata_type(record_type);
        if (!report_id.has_value()) {
            return Sh2Error::UNSUPPORTED;
        }
        std::optional<reports::ReportQ> record_q =
            reports::metadata_record_q(words);
        if (!record_q.has_value()) {
            return Sh2Error::INVALID_ARGUMENT;
        }
        return input_.set_report_q(*report_id, *record_q);
    }

    /// The decode-time Q-point overrides currently installed (DR-2).
    [[nodiscard]] const reports::QOverrideTable& report_q_overrides() const {
        return input_.q_overrides();
    }

   private:
    /// Engine cargo sink trampoline target.
    void on_cargo(const shtp::ShtpEngine::Cargo& cargo);

    /// Advertisement report-length sink trampoline target.
    void on_advertised_channel(const shtp::AdvertisedChannel& channel);

    void on_report_length(std::uint8_t report_id, std::uint8_t length) {
        input_.set_report_length(report_id, length);
    }

    void drain_transfers();

    /// Publish framing and input counters after every drain, including early
    /// exits.
    void mirror_counters();

    void fail(Sh2Error error);

    /// Whether recovery preserves learned channel numbers.
    enum class LearnedChannels : std::uint8_t {
        FORGET,  ///< The hub can come back numbered differently.
        KEEP,    ///< The hub has just spoken on its learned device channel.
    };

    /// Discards prior hub state and restarts the handshake (SR-5).
    /// Returns false if a sink called begin(), so the caller stops.
    bool restart_handshake(LearnedChannels learned = LearnedChannels::FORGET);

    /// One service() pass, behind its re-entry guard.
    void service_pass();

    /// Forwards events unless a sink restarted the session mid-cargo.
    void on_event(const reports::SensorEvent& event);

    /// Routes channel-0 responses, including advertisement bodies.
    void handle_command_channel(std::span<const std::uint8_t> payload);
    void handle_advertisement(std::span<const std::uint8_t> payload);
    void handle_reset_complete(std::span<const std::uint8_t> payload);
    void handle_initialize(std::span<const std::uint8_t> payload);

    /// Split control cargo by report length and route each report for the
    /// state.
    void walk_control(std::span<const std::uint8_t> cargo);

    /// READY-state control routing: one report from the control channel.
    void route_control(std::span<const std::uint8_t> payload);

    /// A Flush Completed report (0xEF), from whichever channel carried it.
    void handle_flush_completed(std::span<const std::uint8_t> report);
    void handle_command_response(std::span<const std::uint8_t> payload);
    void handle_feature_response(std::span<const std::uint8_t> payload);
    /// Count unusable FRS responses and deliver completions.
    void account_frs(const detail::FrsEngine::Completion& completion,
                     detail::FrsEngine::Outcome outcome);
    void deliver_frs(const detail::FrsEngine::Completion& completion);

    /// The borrowed transport.
    [[nodiscard]] IHalTransport& hal() const noexcept { return hal_.get(); }

    /// Consumer-owned transport; it outlives this Session.
    std::reference_wrapper<IHalTransport> hal_;
    SessionTiming timing_;
    shtp::ChannelMap channels_;

    std::array<std::uint8_t, ActiveProfile::MAX_RX_TRANSFER_LEN> rx_staging_{};

    // reassembly_ MUST precede engine_: members initialise in declaration
    // order, and the engine borrows this buffer. Keep them adjacent and in this
    // order.
    std::array<std::uint8_t, ActiveProfile::MAX_CARGO_LEN> reassembly_{};
    shtp::ShtpEngine engine_;

    detail::StartupSequencer startup_;
    detail::CommandEngine commands_;
    detail::FrsEngine frs_;
    detail::InputRouter input_;
    detail::TxPath tx_;

    reports::EventSink event_sink_;
    ResetSink reset_sink_;
    FeatureSink feature_sink_;
    CommandSink command_sink_;
    FrsSink frs_sink_;
    /// Marks a borrowed FRS record and defers a re-entrant read (SR-3).
    bool delivering_frs_record_ = false;
    std::optional<std::uint16_t> deferred_frs_read_;

    /// An unsolicited Initialize seen while READY, kept for the handshake a
    /// following reset-complete restarts (SR-5).
    struct HeldInitialize {
        std::uint64_t at_us = 0;
        bool ok = false;
    };

    std::optional<HeldInitialize> held_initialize_;
    FlushSink flush_sink_;
    DiagSink diag_sink_;

    SessionCounters counters_;
    SessionState state_ = SessionState::IDLE;
    Sh2Error fault_ = Sh2Error::NONE;
    /// begin() advances this; service() checks it after every sink call.
    std::uint32_t generation_ = 0;
    /// generation_ as the current drain began.
    std::uint32_t drain_generation_ = 0;
    /// Set for the duration of service(), so a sink's call to it is a no-op.
    bool in_service_ = false;
};

/// Bound the session's static storage to the profile ceiling (AR-1).
static_assert(sizeof(Session) <= ActiveProfile::SESSION_SIZE_CEILING,
              "Session exceeds the active profile's static footprint ceiling");

// Defined out of line because Session must be complete for sizeof/alignof;
// the default argument is still evaluated in the caller's translation unit.
constexpr SessionBinding Session::caller_binding() noexcept {
    return SessionBinding{
        .max_rx_transfer_len = ActiveProfile::MAX_RX_TRANSFER_LEN,
        .max_cargo_len = ActiveProfile::MAX_CARGO_LEN,
        .max_tx_cargo = ActiveProfile::MAX_TX_CARGO,
        .tx_queue_depth = ActiveProfile::TX_QUEUE_DEPTH,
        .frs_max_words = ActiveProfile::FRS_MAX_WORDS,
        .max_enabled_sensors = ActiveProfile::MAX_ENABLED_SENSORS,
        .session_size = sizeof(Session),
        .session_align = alignof(Session),
    };
}

/// Profile values compiled into the library archive.
/// This is out of line so the result comes from the library, not the caller.
[[nodiscard]] SessionBinding library_binding() noexcept;

/// True when the caller's profile matches the library's.
/// Checking before construction lets a consumer report a mismatch before abort.
[[nodiscard]] inline bool binding_matches() noexcept {
    return library_binding() == Session::caller_binding();
}

}  // namespace pico_sh2
