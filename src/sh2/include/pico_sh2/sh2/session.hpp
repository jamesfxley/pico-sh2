// SPDX-License-Identifier: MIT
#pragma once

/// \file session.hpp
/// SH-2 session: the object a consumer drives to bring a sensor hub from reset
/// to a running sensor stream. It owns the SHTP framing engine and the
/// reassembly and staging buffers (sized from the active profile), and composes
/// the startup sequencer, the command and FRS transaction engines, the input
/// router, and the outbound path — routing each completed cargo to whichever of
/// them owns it.
///
/// The design is non-blocking and service-context confined (AR-3): begin()
/// starts the reset/startup sequence and returns immediately; service() is
/// pumped by the consumer (after a HINTN wake, or polled) to advance the state
/// machine, drain available transfers, and deliver events. Every IHalTransport
/// call happens inside begin()/service(), never from an interrupt.
///
/// A Session is single-context (AR-3). Every call on it -- begin(), service(),
/// and every host->hub request -- must come from the one context that drives
/// service(). Not a second core, not an interrupt handler. The outbound queue,
/// the counters and the session state carry no synchronisation, deliberately:
/// nothing here allocates or locks, and a request staged from one context while
/// service() drains from another races with no diagnostic. A consumer's
/// interrupt handler latches the HINTN flag and its timestamp, and nothing
/// else.
///
/// Startup ordering (SR-1) is sequenced by detail::StartupSequencer; the
/// transaction rules for commands (SR-2) and FRS (SR-3) live in their own
/// engines. What stays here is the wiring: the counters (OR-1), the consumer's
/// sinks, and the decision of which collaborator a cargo belongs to.

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

/// Where the session is in its lifecycle. The detail of how far startup has got
/// is a StartupPhase, reported separately by startup_phase().
enum class SessionState : std::uint8_t {
    IDLE = 0,  // constructed; begin() not yet called
    STARTING,  // running the reset and handshake sequence
    READY,     // handshake complete; routing sensor traffic
    FAULTED,   // startup failed (deadline or protocol)
};

/// Notable session events surfaced through the optional diagnostics hook
/// (OR-2), in addition to the always-updated counters. The hook is
/// allocation-free and costs nothing when unset.
enum class DiagEvent : std::uint8_t {
    STARTUP_FAULT,    // a startup phase failed (deadline, bus error, or status)
    HUB_RESET,        // an unexpected hub reset was recognized (SR-5)
    COMMAND_TIMEOUT,  // an outstanding command passed its deadline
    FRS_TIMEOUT,      // an FRS operation passed its deadline
    WRITE_ERROR,      // the transport refused a frame; it stays queued
};

/// Normative diagnostic counters (OR-1). The set, and what each counter means,
/// is defined once in session_counters.def; both this struct and
/// for_each_counter() below are generated from it.
/// NOLINTBEGIN(cppcoreguidelines-macro-usage,bugprone-macro-parentheses)
/// The counter list is an X-macro: one definition has to expand into a struct,
/// a visitor body and a count, which no template or constexpr function can do.
/// The expansions are declarations and statements, not expressions, so the
/// parenthesising the second check asks for would not compile.
struct SessionCounters {
/// One counter from session_counters.def; undefined again after use. The
/// comment in its body documents each member it generates.
#define PICO_SH2_COUNTER(name, description)                       \
    /** One counter; session_counters.def says what it counts. */ \
    std::uint32_t name = 0;
#include "pico_sh2/sh2/session_counters.def"
#undef PICO_SH2_COUNTER
};

/// Visit every counter as (name, description, value), in declaration order.
/// Lets a consumer report the whole set without naming individual counters, so
/// adding one here reaches every reporter without editing it.
///
/// The visitor is taken by value, as std::for_each takes its own: it is invoked
/// once per counter, so a forwarding reference would have to not forward -- the
/// first call would move from it and every later call would see a moved-from
/// object. Visitors here are small callables, and the copy is elided at the
/// call site.
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

/// The profile values a caller's translation units were compiled with.
///
/// Every buffer in a Session is sized from ActiveProfile, which comes from the
/// PICO_SH2_ACTIVE_PROFILE macro. A consumer that compiles its own translation
/// units with a different value than the installed archive was built with
/// therefore disagrees with it about the layout of Session and of everything
/// crossing this boundary. CMake propagates the definition as a usage
/// requirement so this cannot normally happen; this is the check for when it
/// does anyway -- a hand-rolled build, a stale prebuilt archive -- because the
/// failure mode is memory corruption with no other symptom, and the consumer
/// smoke's static_assert cannot see it: it proves the macro *propagates*, not
/// that a consumer who overrides it is safe.
struct SessionBinding {
    /// ActiveProfile::MAX_RX_TRANSFER_LEN, the staging buffer extent.
    std::size_t max_rx_transfer_len;
    /// ActiveProfile::MAX_CARGO_LEN, the reassembly buffer extent and the span
    /// ShtpEngine is constructed over.
    std::size_t max_cargo_len;
    /// ActiveProfile::MAX_TX_CARGO, the width of one queued outbound cargo.
    std::size_t max_tx_cargo;
    /// ActiveProfile::TX_QUEUE_DEPTH, the outbound queue's element count.
    std::size_t tx_queue_depth;
    /// ActiveProfile::FRS_MAX_WORDS, the FRS record buffer extent.
    std::size_t frs_max_words;
    /// ActiveProfile::MAX_ENABLED_SENSORS. Reported separately because nothing
    /// in Session is sized by it: a driver built on this library sizes its own
    /// desired-sensor table from the forwarded profile, so a disagreement here
    /// moves that driver's layout and not this one's, and the size comparison
    /// below cannot see it.
    std::size_t max_enabled_sensors;
    /// Size of Session. The knobs above are the ones worth reporting
    /// individually; comparing the object itself covers every member they move
    /// and any a future profile knob adds.
    std::size_t session_size;
    /// Alignment of Session, paired with session_size.
    std::size_t session_align;

    /// Member-wise equality. A mismatch is the layout disagreement this struct
    /// exists to detect.
    bool operator==(const SessionBinding&) const = default;
};

/// An SH-2 session over one transport: reset, handshake, sensor configuration
/// and the running event stream. Drive begin() once, then service() from the
/// consumer's loop; every transport call happens inside those two.
class Session {
   public:
    /// Notification when the hub resets unexpectedly (an advertisement or
    /// reset-complete arrives after the session was READY). The session has
    /// already restarted its handshake by the time this fires.
    using ResetSink = pico_util::Callback<void()>;
    /// A Get Feature response carrying the hub's granted configuration.
    using FeatureSink = pico_util::Callback<void(const FeatureConfig&)>;
    /// A command response (solicited or unsolicited).
    using CommandSink = pico_util::Callback<void(const CommandResponse&)>;
    /// An FRS read/write completion: the status (NONE on success), the record
    /// type, and — for a read — the record words (empty for a write). The span
    /// is valid for the whole call: while it carries words, an frs_read() made
    /// from inside this sink is held and sent once it returns (ending here with
    /// the error if it cannot be queued), and an frs_write() from inside it is
    /// BUSY, since it could only be sent after the sink's own frame is gone.
    /// With no words there is nothing to protect, and either is handled as one
    /// made outside the sink.
    /// Every operation frs_read()
    /// or frs_write() accepted ends here exactly once: TIMEOUT past its
    /// deadline, HUB_RESET when an unexpected hub reset cut it off, TABLE_FULL
    /// for a record longer than the profile's FRS buffer. begin() is the
    /// exception: it discards a running operation without a completion.
    ///
    /// The fourth argument is the hub's status byte when the hub's own
    /// response ended the operation (1000-3625 §6.3.5, §6.3.7) -- which of a
    /// full memory, a read-only record or an invalid length refused a write --
    /// and empty when the session ended it: a deadline, a hub reset, a full
    /// buffer.
    using FrsSink = pico_util::Callback<void(Sh2Error, std::uint16_t,
                                             std::span<const std::uint32_t>,
                                             std::optional<std::uint8_t>)>;
    /// A sensor whose outstanding batched data the hub has finished sending, in
    /// answer to a force_flush() (1000-3625 §6.5.7).
    using FlushSink = pico_util::Callback<void(reports::SensorId)>;
    /// Optional real-time diagnostics hook (OR-2). Allocation-free; no-op when
    /// unset.
    using DiagSink = pico_util::Callback<void(DiagEvent)>;

    /// `timing` has no defaults to fall back on (see session_timing.hpp) and
    /// `channels` seeds the role-to-number map for the operations that must
    /// work before the hub has advertised — a host-requested reset (SR-7). Both
    /// are properties of the part, so both come from the consumer; the
    /// advertisement overrides whatever the seed claimed.
    ///
    /// `binding` is defaulted and is not for callers to supply. A default
    /// argument is evaluated at the call site, so it carries the *consumer's*
    /// profile values into a constructor compiled with the *library's*; the
    /// two are compared and disagreement aborts.
    Session(IHalTransport& hal, SessionTiming timing,
            shtp::ChannelMap channels = {},
            SessionBinding binding = caller_binding());

    /// Neither copyable nor movable: the engine, the sinks bound to this
    /// Session and the TX path hold pointers into it, which a copy or a move
    /// would leave aimed at the original.
    Session(const Session&) = delete;
    Session& operator=(const Session&) = delete;
    Session(Session&&) = delete;
    Session& operator=(Session&&) = delete;
    ~Session() = default;

    /// The profile values of the translation unit that calls it.
    ///
    /// Out of line below: sizeof(Session) needs Session complete.
    [[nodiscard]] static constexpr SessionBinding caller_binding() noexcept;

    /// Sinks. Setting a sink is optional; an unset sink is a no-op.
    ///
    /// Every sink runs inside service() and may call back into this Session:
    /// the host->hub requests, and begin(). A sink that calls begin() ends the
    /// pass it was called from -- nothing further from the traffic in flight
    /// reaches a sink, because it belongs to the hub incarnation the call
    /// restarted. service() called from a sink returns at once.
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

    // --- host->hub requests (READY only) -------------------------------------
    // Each enqueues a cargo flushed to the hub during the next service(). They
    // return NOT_READY before the session is READY and QUEUE_FULL if the TX
    // queue has no free slot (BR-2); the cargo is not sent in either case.

    /// Enable/configure a sensor (Set Feature, 0xFD).
    Result<void> set_feature(const FeatureConfig& config);

    /// Request a sensor's current configuration (Get Feature, 0xFE); the
    /// granted config arrives at the feature sink.
    Result<void> get_feature(reports::SensorId sensor);

    /// Issue a command (0xF2). Exactly one command may be outstanding: BUSY if
    /// one already is (SR-2); INVALID_ARGUMENT if `command` is out of the 1-127
    /// range or `params` is longer than COMMAND_MAX_PARAMS. The response
    /// arrives at the command sink; no response within the command deadline
    /// times it out (TR-1).
    Result<void> send_command(std::uint8_t command,
                              std::span<const std::uint8_t> params = {});

    /// Whether a command is outstanding, so a further send_command() would be
    /// refused (SR-2).
    [[nodiscard]] bool command_pending() const { return commands_.pending(); }

    /// Read an FRS record (0xF4). Streams responses into an internal buffer;
    /// the completion (words, or an error) arrives at the FRS sink. BUSY if an
    /// FRS operation is already in progress; only one runs at a time. The sink
    /// receives BUSY when the hub itself reports one running (SR-3).
    Result<void> frs_read(std::uint16_t frs_type);

    /// Reset the hub without disturbing anything it has learned: a one-byte
    /// reset on the executable channel (1000-3600 §2.2). The hub restarts and
    /// re-announces itself, which the session recognises as a reset (SR-5).
    ///
    /// Distinct from the Clear DCD and Reset command, which also discards the
    /// dynamic calibration the hub has accumulated. Use this when a reset is
    /// what is wanted and the calibration is not the point.
    ///
    /// Unlike the other requests this does not need READY: the executable
    /// channel is available as soon as framing is, which is what makes it a way
    /// out of a session that is stuck short of READY.
    Result<void> request_reset();

    /// Ask the hub to send `sensor`'s outstanding batched data now, rather than
    /// waiting for its batch interval to elapse (0xF0, 1000-3625 §6.5.6). The
    /// hub may flush other sensors too; each flushed sensor produces a Flush
    /// Completed report at the flush sink.
    Result<void> force_flush(reports::SensorId sensor);

    /// Write an FRS record (0xF7 + paced 0xF6 data). `words` (empty erases the
    /// record) must not exceed the profile's FRS buffer. BUSY if an FRS
    /// operation is in progress; INVALID_ARGUMENT if the record is too large.
    /// The sink receives BUSY when the hub reports itself busy (SR-3).
    Result<void> frs_write(std::uint16_t frs_type,
                           std::span<const std::uint32_t> words);

    /// Whether an FRS operation is in progress -- here, or still on the hub
    /// after the session ended it -- so a further one would be refused (SR-3).
    [[nodiscard]] bool frs_busy() const {
        return frs_.busy() || frs_.holding();
    }

    /// Begin the reset + startup sequence. Non-blocking: asserts NRST and
    /// returns with the session STARTING. Drive service() to reach READY.
    void begin();

    /// Advance the state machine: apply any due timed transition, drain all
    /// available transfers from the HAL into the framing engine (delivering
    /// routed cargoes as a side effect), then enforce the current phase
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

    /// How far startup has got. While FAULTED this is the phase the fault
    /// happened in, which is not advanced by the fault itself.
    [[nodiscard]] StartupPhase startup_phase() const {
        return startup_.phase();
    }

    /// The normative diagnostic counters (OR-1), updated as the session runs.
    [[nodiscard]] const SessionCounters& counters() const { return counters_; }

    /// The report lengths the hub advertised, as the decoders see them.
    [[nodiscard]] const reports::ReportLengthTable& report_lengths() const {
        return input_.lengths();
    }

    /// DR-2: install or replace the decode-time Q points for a report ID; the
    /// input decoders use them in place of the compiled defaults.
    /// INVALID_ARGUMENT if either Q point exceeds reports::MAX_Q_POINT (which
    /// would make the decoder's 2^Q shift undefined); TABLE_FULL if the
    /// override table has no room for a new entry. Overrides describe the part,
    /// not session state, so they persist across a hub reset (unlike the report
    /// lengths, which the advertisement re-establishes).
    Result<void> set_report_q(std::uint8_t report_id,
                              reports::ReportQ q_points) {
        return input_.set_report_q(report_id, q_points);
    }

    /// DR-2: install the authoritative Q points a sensor-metadata FRS record
    /// publishes (SH-2 Reference Manual §5.1) as the decode-time override for
    /// that sensor's input report. `record_type` is the metadata record's FRS
    /// type (0xE301-0xE324) and `words` its word span, as delivered to the FRS
    /// sink after a frs_read(). UNSUPPORTED if the record type is not a
    /// metadata record this library decodes; INVALID_ARGUMENT if the record is
    /// malformed (too short, or an out-of-range Q); TABLE_FULL if the override
    /// table is full.
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

    /// Copy the counters the framing engine and the cargo walk own into the
    /// published set. Called on every service() pass after draining, including
    /// the paths where draining stopped early, so a transport error cannot
    /// leave the published counters behind the ones they mirror.
    void mirror_counters();

    void fail(Sh2Error error);

    /// What a restart does with the channel numbers the last advertisement
    /// taught.
    enum class LearnedChannels : std::uint8_t {
        FORGET,  ///< The hub may come back numbered differently.
        KEEP,    ///< The hub has just spoken on its learned device channel.
    };

    /// SR-5: the hub reset underneath us. Drops everything tied to the previous
    /// incarnation and re-runs the handshake. False when a sink it called
    /// restarted the session with begin(), so the caller stops.
    bool restart_handshake(LearnedChannels learned = LearnedChannels::FORGET);

    /// One service() pass, behind its re-entry guard.
    void service_pass();

    /// Input router event sink trampoline target: forwards to the consumer's
    /// sink unless a sink restarted the session mid-cargo.
    void on_event(const reports::SensorEvent& event);

    /// Startup-phase cargo handlers. handle_command_channel() reads the SHTP
    /// response code and passes an advertisement's body on.
    void handle_command_channel(std::span<const std::uint8_t> payload);
    void handle_advertisement(std::span<const std::uint8_t> payload);
    void handle_reset_complete(std::span<const std::uint8_t> payload);
    void handle_initialize(std::span<const std::uint8_t> payload);

    /// Split a control cargo into its reports by the report-length table and
    /// hand each to handle_initialize() or route_control() by the state.
    void walk_control(std::span<const std::uint8_t> cargo);

    /// READY-state control routing: one report from the control channel.
    void route_control(std::span<const std::uint8_t> payload);

    /// A Flush Completed report (0xEF), from whichever channel carried it.
    void handle_flush_completed(std::span<const std::uint8_t> report);
    void handle_command_response(std::span<const std::uint8_t> payload);
    void handle_feature_response(std::span<const std::uint8_t> payload);
    /// Count an FRS response the engine could not use, and deliver the one that
    /// ended the operation.
    void account_frs(const detail::FrsEngine::Completion& completion,
                     detail::FrsEngine::Outcome outcome);
    void deliver_frs(const detail::FrsEngine::Completion& completion);

    /// The transport. Spelt as a call so hal_ is dereferenced in one place.
    [[nodiscard]] IHalTransport& hal() const noexcept { return hal_.get(); }

    /// The transport, which outlives this Session and is supplied by the
    /// consumer. A reference_wrapper rather than a pointer so that the "never
    /// null" invariant is carried by the type instead of by a comment: the
    /// constructor takes a reference, so no null value is reachable.
    std::reference_wrapper<IHalTransport> hal_;
    SessionTiming timing_;
    shtp::ChannelMap channels_;

    std::array<std::uint8_t, ActiveProfile::MAX_RX_TRANSFER_LEN> rx_staging_{};

    // reassembly_ MUST be declared before engine_. ShtpEngine stores a span
    // over the buffer it is constructed with, and members are initialised in
    // declaration order -- so swapping these two lines compiles cleanly and
    // hands the engine a span over storage that does not exist yet. Nothing
    // else guards this: the constructor's initialiser list order is ignored,
    // and no diagnostic fires. Keep them adjacent and in this order.
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
    /// Set while the FRS sink is handed a record's words, and the read it
    /// asked for in that time (SR-3): issued once it returns, so the record
    /// it holds stays valid.
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
    /// Advanced by begin(). A sink that restarts the session moves it, and
    /// every step of service() that follows a sink compares against it.
    std::uint32_t generation_ = 0;
    /// generation_ as the current drain began.
    std::uint32_t drain_generation_ = 0;
    /// Set for the duration of service(), so a sink's call to it is a no-op.
    bool in_service_ = false;
};

/// The session holds every buffer the library needs, so its size is the
/// library's static footprint (AR-1). Bound it to the profile's ceiling: adding
/// a member that does not fit is a compile error rather than silent growth
/// discovered in a firmware image.
static_assert(sizeof(Session) <= ActiveProfile::SESSION_SIZE_CEILING,
              "Session exceeds the active profile's static footprint ceiling");

// Out of line: Session must be complete before sizeof/alignof can be taken, and
// a default argument is only evaluated at the call site, so the constructor's
// `binding = caller_binding()` still resolves to the caller's values.
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

/// The profile values the installed library archive was compiled with.
///
/// Deliberately neither constexpr nor inline: it is compiled once, inside the
/// library, so it reports the library's view rather than the caller's.
/// Session::caller_binding() reports the caller's, and the two agreeing is what
/// the Session constructor requires.
[[nodiscard]] SessionBinding library_binding() noexcept;

/// Whether the calling translation unit's profile agrees with the library's.
///
/// The constructor aborts on a disagreement, because every buffer it is about
/// to lay out would be the wrong size. On a nosys cross target that abort is a
/// board that is dead at boot with nothing said, so a consumer that can log
/// tests this first and reports the mismatch itself.
[[nodiscard]] inline bool binding_matches() noexcept {
    return library_binding() == Session::caller_binding();
}

}  // namespace pico_sh2
