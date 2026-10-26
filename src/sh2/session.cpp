// SPDX-License-Identifier: MIT
// SH-2 session implementation: the drain loop, cargo routing, and the wiring
// between the startup sequencer (SR-1), the command and FRS engines (SR-2,
// SR-3), the input router (SR-4), and the outbound path. Unexpected-reset
// recovery (SR-5) and the normative counter set (OR-1) live here because both
// span every collaborator.
#include "pico_sh2/sh2/session.hpp"

#include "pico_sh2/pico_sh2.hpp"
#include "pico_sh2/sh2/profile.hpp"
#include "pico_sh2/sh2/startup_phase.hpp"

// The advertised version and project(pico_sh2 VERSION ...) are two records of
// one fact. Bumping either alone stops the build here.
//
// The constants live in the umbrella header, whose target is INTERFACE and has
// no translation unit of its own; this is the nearest compiled one. The CMake
// values arrive PRIVATE on this target, so they reach no installed consumer.
// A host test asserts the same pair, but that test is skipped on every
// configure with tests off -- which is every firmware and every consumer build.
static_assert(pico_sh2::VERSION_MAJOR == PICO_SH2_CMAKE_VERSION_MAJOR);
static_assert(pico_sh2::VERSION_MINOR == PICO_SH2_CMAKE_VERSION_MINOR);
static_assert(pico_sh2::VERSION_PATCH == PICO_SH2_CMAKE_VERSION_PATCH);

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <optional>
#include <span>

#include "pico_sh2/reports/decoders.hpp"
#include "pico_sh2/reports/report_length_table.hpp"
#include "pico_sh2/reports/sensor_event.hpp"
#include "pico_sh2/reports/sensor_id.hpp"
#include "pico_sh2/sh2/channel_binding.hpp"
#include "pico_sh2/sh2/command.hpp"
#include "pico_sh2/sh2/feature.hpp"
#include "pico_sh2/sh2/flush.hpp"
#include "pico_sh2/sh2/frs.hpp"
#include "pico_sh2/sh2/hal_transport.hpp"
#include "pico_sh2/sh2/session_timing.hpp"
#include "pico_sh2/shtp/advertisement.hpp"
#include "pico_sh2/shtp/channels.hpp"
#include "pico_sh2/util/result.hpp"

namespace pico_sh2 {

namespace {

// Device-channel response: a 1-byte cargo whose value is the event
// (1000-3600 §2.2). 0 is reserved and 1 is reset complete; 2-255 are reserved.
constexpr std::uint8_t RESET_COMPLETE = 0x01;
// SHTP Command 0 (Get Advertisement) with P0 = 1, the entire hub
// (1000-3535 §5.1.1).
constexpr std::array<std::uint8_t, 2> GET_ADVERTISEMENT_REQUEST{0x00, 0x01};

// Command-channel read cargo: byte 0 is the SHTP response code (1000-3535
// §5.1). 0 is the advertisement (§5.1.1.1) and 1 the Error List (§5.1.2.1).
constexpr std::uint8_t SHTP_RESPONSE_ADVERTISEMENT = 0x00;
constexpr std::uint8_t SHTP_RESPONSE_ERROR_LIST = 0x01;

// Device-channel command, same one-byte cargo in the write direction: 1 is
// reset (1000-3600 §2.2). "on" and "sleep" are defined there too and are not
// used here.
constexpr std::uint8_t EXECUTABLE_RESET = 0x01;

// The post-startup Initialize response is a command response (0xF1) whose
// command byte is the Initialize command with the unsolicited flag set, and
// whose R0 result is the status (SH-2 Reference Manual §6.4.4).
constexpr std::uint8_t INITIALIZE_UNSOLICITED =
    command::INITIALIZE | COMMAND_UNSOLICITED_FLAG;  // 0x84
constexpr std::uint8_t INITIALIZE_OK = 0x00;         // R0 status: success

}  // namespace

Session::Session(IHalTransport& hal, SessionTiming timing,
                 shtp::ChannelMap channels, SessionBinding binding)
    : hal_(hal),
      timing_(timing),
      channels_(channels),
      engine_(reassembly_),
      startup_(timing) {
    // caller_binding() here is the library's own view; `binding` is the
    // caller's, evaluated in the caller's translation unit. They differ only if
    // the two were compiled with different profile macros, in which case they
    // disagree about the layout of every type crossing this boundary. Nothing
    // after this point would be well defined.
    //
    // A consumer that wants to see this coming rather than meet it as a bare
    // abort() on a nosys target tests binding_matches() and logs first.
    if (binding != caller_binding()) {
        std::abort();
    }
    engine_.set_cargo_sink(
        shtp::ShtpEngine::CargoSink::bind<Session, &Session::on_cargo>(this));
    input_.set_event_sink(
        reports::EventSink::bind<Session, &Session::on_event>(this));
    tx_.bind_channels(channels_);
}

void Session::begin() {
    held_initialize_.reset();
    // Timing has no defaults, so an unset SessionTiming arrives here as zeros.
    // Releasing reset immediately and arming deadlines that expire on the same
    // tick would present as a hub that never answers, so name the real fault
    // instead of hanging in a phase that cannot complete.
    if (!timing_.valid()) {
        fail(Sh2Error::INVALID_ARGUMENT);
        return;
    }
    // A sink that called this is inside service(), which checks the
    // generation after every sink it calls and abandons the traffic it was
    // processing when it moves.
    ++generation_;
    // Every collaborator is reset, so nothing from a previous run — a queued
    // request, an outstanding command, a half-read FRS record — survives into
    // the new one. An FRS operation dropped here reports nothing: the caller
    // asked for the restart and already knows what it abandons.
    channels_.forget_learned();
    engine_.reset();
    commands_.reset();
    frs_.reset();
    tx_.reset();
    input_.reseed_lengths();
    fault_ = Sh2Error::NONE;
    hal().set_reset(true);
    startup_.begin(hal().now_us());
    state_ = SessionState::STARTING;
}

void Session::fail(Sh2Error error) {
    state_ = SessionState::FAULTED;
    fault_ = error;
    diag_sink_(DiagEvent::STARTUP_FAULT);
}

bool Session::restart_handshake(LearnedChannels learned) {
    const std::uint32_t generation = generation_;
    ++counters_.resets;
    // A hub that reset may come back with a different channel base (1000-3600
    // §2.3 lets N be chosen at run time), so what the last advertisement taught
    // is not carried across one -- unless the caller has just seen the hub
    // speak on its learned device channel, which is N itself. The consumer's
    // seed survives either way, which is what keeps a reset requestable (SR-7).
    if (learned == LearnedChannels::FORGET) {
        channels_.forget_learned();
    }
    engine_.reset();
    commands_.reset();
    // An Initialize held for a reset belongs to the reset that follows it;
    // this restart either is that reset, which took it, or is another (SR-5).
    held_initialize_.reset();
    // An FRS operation ends exactly once, through the FRS sink (SR-3), so one
    // the reset cut off is reported rather than dropped. An outstanding
    // command has no failure path through its sink; command_pending() falls
    // and the reset sink below is its notice, as the diagnostics hook is for
    // its timeout.
    const detail::FrsEngine::Completion abandoned =
        frs_.abandon(Sh2Error::HUB_RESET);
    tx_.reset();
    input_.reseed_lengths();
    startup_.restart(hal().now_us());
    state_ = SessionState::STARTING;
    diag_sink_(DiagEvent::HUB_RESET);
    if (abandoned.finished && generation_ == generation) {
        deliver_frs(abandoned);
    }
    if (generation_ == generation) {
        reset_sink_();
    }
    return generation_ == generation;
}

void Session::service() {
    // A sink runs inside service(). Draining again from one would overwrite
    // the transfer the outer pass is still delivering from.
    if (in_service_ || state_ == SessionState::IDLE ||
        state_ == SessionState::FAULTED) {
        return;
    }
    in_service_ = true;
    service_pass();
    in_service_ = false;
}

void Session::service_pass() {
    const std::uint32_t generation = generation_;

    // The timed reset and boot phases involve no hub traffic.
    if (startup_.in_reset_window()) {
        if (startup_.tick(hal().now_us()).release_reset) {
            hal().set_reset(false);
        }
        return;
    }

    drain_transfers();
    mirror_counters();

    // A bus error during drain_transfers() already faulted the session. Do not
    // write to a transport that has just failed, and do not re-fault it (which
    // would overwrite fault() and miscount a startup timeout) on a
    // coincidentally-elapsed phase deadline. A sink that called begin() has
    // started a run this pass's timings do not belong to.
    if (state_ == SessionState::FAULTED || generation_ != generation) {
        return;
    }

    // request_reset() is the one request allowed before READY (SR-7), so the
    // flush cannot sit inside the READY branch below: its cargo would be staged
    // and never written, and the next begin() would discard it.
    const detail::TxPath::Flushed flushed = tx_.flush(engine_, hal());
    counters_.cargoes_out += flushed.written;
    if (flushed.write_refused) {
        ++counters_.write_errors;
        diag_sink_(DiagEvent::WRITE_ERROR);
        if (generation_ != generation) {
            return;
        }
    }

    if (state_ == SessionState::READY) {
        const std::uint64_t now = hal().now_us();
        if (commands_.check_deadline(now)) {
            ++counters_.command_timeouts;
            diag_sink_(DiagEvent::COMMAND_TIMEOUT);
            if (generation_ != generation) {
                return;
            }
        }
        auto expired = frs_.check_deadline(now);
        if (expired.finished) {
            ++counters_.frs_timeouts;
            diag_sink_(DiagEvent::FRS_TIMEOUT);
            deliver_frs(expired);
        }
        return;
    }

    if (startup_.tick(hal().now_us()).timed_out) {
        ++counters_.startup_timeouts;
        fail(Sh2Error::TIMEOUT);
    }
}

void Session::drain_transfers() {
    drain_generation_ = generation_;
    std::size_t drained = 0;
    // The bound is the profile's (AR-3): it guarantees termination even if a
    // HAL reports data_ready() on a stuck-low HINTN, and it is a knob rather
    // than a constant here because it is the caller's latency it bounds.
    while (hal().data_ready() &&
           drained < ActiveProfile::MAX_TRANSFERS_PER_SERVICE) {
        ++drained;
        auto read = hal().read_transfer(rx_staging_);
        if (!read.ok()) {
            // A read only issues behind a data-ready indication, so a transport
            // error in READY is the hub having left the bus between indicating
            // and answering — what a reset looks like to a transport able to
            // report one at all (SR-5). Restart the handshake rather than
            // faulting: the hub re-advertises when it returns, and a transport
            // that is genuinely dead never does, so the advertisement deadline
            // faults it as the TIMEOUT it is. During startup the hub is already
            // expected to be silent, so an error there is the fault it appears
            // to be.
            if (state_ == SessionState::READY) {
                ++counters_.transport_restarts;
                restart_handshake();
                return;
            }
            fail(read.error());
            return;
        }
        const std::span<const std::uint8_t> transfer(rx_staging_.data(),
                                                     read.value().length);
        // The engine keeps the lead transfer's edge and returns it with the
        // completed cargo, so the framing rules stay in one place. A malformed
        // header or a dropped cargo is already counted in the engine's stats,
        // which the mirror picks up.
        (void)engine_.ingest(transfer, read.value().intn_edge_us);
        // Routing happens in on_cargo as the engine completes cargoes. A
        // startup phase may have advanced or a reset may have restarted the
        // handshake; either way, keep draining what the hub already offered --
        // unless a sink called begin(), which asserted reset on the hub whose
        // traffic this is.
        if (generation_ != drain_generation_) {
            return;
        }
        // SR-1: a fault stays observable. A later transfer in the same pass
        // must not carry the handshake on past it and set READY over it.
        if (state_ == SessionState::FAULTED) {
            return;
        }
    }
}

void Session::mirror_counters() {
    // Transport counters the framing engine owns.
    const shtp::ShtpEngine::Stats& stats = engine_.stats();
    counters_.cargoes_in = stats.cargoes_delivered;
    counters_.empty_transfers = stats.empty_transfers;
    counters_.bad_headers = stats.bad_headers;
    counters_.reassembly_drops = stats.reassembly_drops;
    counters_.sequence_gaps = stats.sequence_gaps;

    // Input-decode counters accumulated across cargoes.
    const reports::WalkStats& walk = input_.stats();
    counters_.events_decoded = walk.events_decoded;
    counters_.reports_skipped = walk.reports_skipped;
    counters_.unknown_reports = walk.unknown_reports;
    counters_.truncated_reports = walk.truncated_reports;
}

void Session::on_cargo(const shtp::ShtpEngine::Cargo& cargo) {
    // A cargo is routed by what its channel is *for*, which the hub told us in
    // its advertisement. Before that — or on a channel no application claimed —
    // there is no role to route to, so the cargo is counted rather than guessed
    // at. The command channel is the exception SHTP fixes (1000-3535 §5.1), so
    // the advertisement itself always has somewhere to arrive.
    const std::optional<shtp::ChannelRole> role =
        channels_.role_of(static_cast<std::uint8_t>(cargo.channel));
    if (!role.has_value()) {
        ++counters_.unrouted_cargoes;
        return;
    }

    switch (*role) {
        case shtp::ChannelRole::COMMAND:
            handle_command_channel(cargo.payload);
            break;
        case shtp::ChannelRole::DEVICE:
            handle_reset_complete(cargo.payload);
            break;
        case shtp::ChannelRole::CONTROL:
            walk_control(cargo.payload);
            break;
        case shtp::ChannelRole::INPUT_NORMAL:
        case shtp::ChannelRole::INPUT_WAKE:
            if (state_ == SessionState::READY) {
                input_.route_batched(cargo.payload, cargo.lead_edge_us);
            } else {
                ++counters_.unrouted_cargoes;
            }
            break;
        case shtp::ChannelRole::INPUT_GYRO_RV:
            if (state_ == SessionState::READY) {
                input_.route_gyro_integrated(cargo.payload, cargo.lead_edge_us);
            } else {
                ++counters_.unrouted_cargoes;
            }
            break;
    }
}

// --- startup handshake -------------------------------------------------------

void Session::handle_command_channel(std::span<const std::uint8_t> payload) {
    // Byte 0 of a channel-0 read cargo is the SHTP response code (1000-3535
    // §5.1). Response 0 is the advertisement; Response 1 is the hub's Error
    // List, sent on its own for any recoverable SHTP error (§5.1.2.1), which is
    // a report and not a re-advertisement; 2-255 are reserved (§5.1.3).
    if (payload.empty()) {
        ++counters_.protocol_errors;
        return;
    }
    switch (payload[0]) {
        case SHTP_RESPONSE_ADVERTISEMENT:
            handle_advertisement(payload.subspan(1));
            break;
        case SHTP_RESPONSE_ERROR_LIST:
            ++counters_.hub_error_lists;
            break;
        default:
            ++counters_.protocol_errors;
            break;
    }
}

void Session::handle_advertisement(std::span<const std::uint8_t> payload) {
    // An empty command-channel cargo is not an advertisement, which is a
    // multi-entry TLV list. Never let it advance the handshake or be mistaken
    // for a re-advertisement; a real advertisement follows.
    if (payload.empty()) {
        ++counters_.protocol_errors;
        return;
    }
    // An advertisement after READY means the hub reset (it re-advertises first,
    // 1000-3535 §2.3.1). Restart before consuming it, so a malformed one leaves
    // the session waiting for a clean advertisement rather than claiming to be
    // READY over a table that has already been re-seeded.
    if (state_ == SessionState::READY) {
        if (!restart_handshake()) {
            return;
        }
    } else if (startup_.phase() != StartupPhase::WAIT_ADVERTISEMENT) {
        // Advertisement arriving out of its startup phase: treat it as the
        // start of a (re)advertisement rather than dropping it.
        engine_.reset();
        input_.reseed_lengths();
    }

    auto applied = shtp::parse_advertisement(
        payload,
        shtp::ReportLengthSink::bind<Session, &Session::on_report_length>(this),
        shtp::ChannelSink::bind<Session, &Session::on_advertised_channel>(
            this));
    if (!applied.ok()) {
        // A malformed advertisement is not fatal; wait for a clean one. The
        // parse applies each entry as it reads it and rejects a truncated one
        // afterwards, so what it already wrote has to be discarded here
        // (advertisement.hpp) -- otherwise a corrupt length pair emitted before
        // the truncation outlives the cargo that carried it. forget_learned()
        // keeps the consumer's seed, which SR-7 depends on.
        ++counters_.protocol_errors;
        channels_.forget_learned();
        input_.reseed_lengths();
        return;
    }
    // Without every role bound there is no way to route control or input, so
    // an advertisement that names fewer channels than SH-2 defines leaves the
    // handshake where it is rather than advancing into a session that cannot
    // carry traffic.
    if (!channels_.complete()) {
        ++counters_.protocol_errors;
        channels_.forget_learned();
        input_.reseed_lengths();
        return;
    }
    startup_.on_advertisement(hal().now_us());
    // A requested advertisement can be the last of the three to arrive.
    if (startup_.complete()) {
        state_ = SessionState::READY;
    }
}

void Session::on_advertised_channel(const shtp::AdvertisedChannel& channel) {
    const std::optional<shtp::ChannelRole> role = role_of_advertised(channel);
    if (role.has_value()) {
        channels_.learn(*role, channel.number);
    }
}

void Session::handle_reset_complete(std::span<const std::uint8_t> payload) {
    if (payload.size() != 1 || payload[0] != RESET_COMPLETE) {
        ++counters_.protocol_errors;
        return;
    }
    // Reset-complete without a preceding re-advertisement: still a reset, and
    // the advertisement it should have followed was missed. What that
    // advertisement teaches -- the channel map, the report lengths -- is not
    // recoverable any other way, so the session asks for it: SHTP Command 0
    // with P0 = 1, the whole hub (1000-3535 §5.1.1).
    // It arrived on the learned device channel, so the hub is numbered as it
    // was and the learned map still routes the Initialize response that
    // follows; the requested advertisement replaces the map when it lands.
    if (state_ == SessionState::READY) {
        // Taken before the restart, which forgets it (SR-5).
        const std::optional<HeldInitialize> held = held_initialize_;
        if (!restart_handshake(LearnedChannels::KEEP)) {
            return;
        }
        startup_.on_reset_complete_without_advertisement();
        // The restart emptied the TX queue, so the request fits; were it
        // refused, the advertisement deadline would end the wait.
        static_cast<void>(
            tx_.enqueue(shtp::COMMAND_CHANNEL, GET_ADVERTISEMENT_REQUEST));
        // SR-5: the hub's two announcements are not ordered, so its
        // Initialize response may already have arrived while the session was
        // READY. One recent enough to belong to this reset -- inside the
        // Initialize phase's own timeout -- is taken rather than waited for.
        if (held.has_value() &&
            hal().now_us() - held->at_us <= timing_.initialize_timeout_us) {
            if (!held->ok) {
                fail(Sh2Error::PROTOCOL);
                return;
            }
            startup_.on_initialize(hal().now_us());
        }
        return;
    }
    if (!startup_.accepts_reset_complete()) {
        ++counters_.unrouted_cargoes;
        return;
    }
    startup_.on_reset_complete(hal().now_us());
    if (startup_.complete()) {
        state_ = SessionState::READY;
    }
}

void Session::handle_initialize(std::span<const std::uint8_t> payload) {
    if (!startup_.accepts_initialize()) {
        // Control traffic outside the startup phases has no handler until the
        // session is READY.
        ++counters_.unrouted_cargoes;
        return;
    }
    // Expect the unsolicited Initialize response: a command response (0xF1)
    // whose command byte is 0x84 and whose R0 result is the status. Any other
    // control report keeps us waiting.
    if (payload.size() < COMMAND_RESPONSE_SIZE ||
        payload[0] != COMMAND_RESPONSE_REPORT ||
        payload[COMMAND_RESP_COMMAND] != INITIALIZE_UNSOLICITED) {
        ++counters_.unrouted_cargoes;
        return;
    }
    if (payload[COMMAND_RESP_RESULTS] != INITIALIZE_OK) {
        fail(Sh2Error::PROTOCOL);
        return;
    }
    startup_.on_initialize(hal().now_us());
    if (startup_.complete()) {
        state_ = SessionState::READY;
    }
}

// --- READY-state control routing ---------------------------------------------

// The seeded lengths walk_control() splits on are the sizes the handlers
// parse; two copies of one number in two modules are bound here.
static_assert(reports::ReportLengthTable::seeded()
                  .length_of(COMMAND_RESPONSE_REPORT)
                  .value() == COMMAND_RESPONSE_SIZE);
static_assert(reports::ReportLengthTable::seeded()
                  .length_of(FRS_READ_RESPONSE_REPORT)
                  .value() == FRS_READ_RESPONSE_SIZE);
static_assert(reports::ReportLengthTable::seeded()
                  .length_of(FRS_WRITE_RESPONSE_REPORT)
                  .value() == FRS_WRITE_RESPONSE_SIZE);
static_assert(reports::ReportLengthTable::seeded()
                  .length_of(GET_FEATURE_RESPONSE)
                  .value() == FEATURE_REPORT_SIZE);
static_assert(reports::ReportLengthTable::seeded()
                  .length_of(FLUSH_COMPLETED_REPORT)
                  .value() == FLUSH_REPORT_SIZE);

void Session::walk_control(std::span<const std::uint8_t> cargo) {
    // A cargo from the hub may hold several reports, each as long as the
    // report-length table says (1000-3600 §2.3). A report whose ID has no
    // length leaves no way to find the next one, so it and the rest of the
    // cargo go to the handler as one, which counts it; so does a last report
    // shorter than its length.
    const std::uint32_t generation = generation_;
    while (!cargo.empty()) {
        const Result<std::uint8_t> length =
            input_.lengths().length_of(cargo.front());
        const std::size_t take =
            length.ok() ? std::min<std::size_t>(length.value(), cargo.size())
                        : cargo.size();
        const std::span<const std::uint8_t> report = cargo.first(take);
        if (state_ == SessionState::READY) {
            route_control(report);
        } else {
            handle_initialize(report);
        }
        // A sink that called begin(), or an Initialize that failed the
        // handshake, ends the cargo with the session it belonged to.
        if (generation_ != generation || state_ == SessionState::FAULTED) {
            return;
        }
        cargo = cargo.subspan(take);
    }
}

void Session::route_control(std::span<const std::uint8_t> payload) {
    if (payload.empty()) {
        ++counters_.unrouted_cargoes;
        return;
    }
    switch (payload[0]) {
        case COMMAND_RESPONSE_REPORT:
            handle_command_response(payload);
            break;
        case GET_FEATURE_RESPONSE:
            handle_feature_response(payload);
            break;
        case FLUSH_COMPLETED_REPORT:
            handle_flush_completed(payload);
            break;
        case FRS_READ_RESPONSE_REPORT: {
            auto outcome = detail::FrsEngine::Outcome::APPLIED;
            auto completion =
                frs_.on_read_response(payload, hal().now_us(), outcome);
            account_frs(completion, outcome);
            break;
        }
        case FRS_WRITE_RESPONSE_REPORT: {
            auto outcome = detail::FrsEngine::Outcome::APPLIED;
            auto completion =
                frs_.on_write_response(payload, hal().now_us(), outcome, tx_);
            account_frs(completion, outcome);
            break;
        }
        default:
            ++counters_.unrouted_cargoes;
            break;
    }
}

void Session::handle_flush_completed(std::span<const std::uint8_t> report) {
    const std::optional<reports::SensorId> sensor =
        parse_flush_completed(report);
    if (!sensor.has_value()) {
        ++counters_.protocol_errors;
        return;
    }
    ++counters_.flushes_completed;
    flush_sink_(*sensor);
}

void Session::account_frs(const detail::FrsEngine::Completion& completion,
                          detail::FrsEngine::Outcome outcome) {
    switch (outcome) {
        case detail::FrsEngine::Outcome::UNROUTED:
            ++counters_.unrouted_cargoes;
            break;
        case detail::FrsEngine::Outcome::MALFORMED:
            ++counters_.protocol_errors;
            break;
        case detail::FrsEngine::Outcome::STRAY:
            ++counters_.frs_stray_responses;
            break;
        case detail::FrsEngine::Outcome::APPLIED:
            if (completion.finished) {
                deliver_frs(completion);
            }
            break;
    }
}

void Session::on_event(const reports::SensorEvent& event) {
    // A cargo's events are delivered one after another from inside the walk.
    // Once a sink has called begin(), the rest of that cargo belongs to the
    // hub incarnation it restarted.
    if (generation_ == drain_generation_) {
        event_sink_(event);
    }
}

void Session::handle_command_response(std::span<const std::uint8_t> payload) {
    if (payload.size() < COMMAND_RESPONSE_SIZE) {
        ++counters_.protocol_errors;
        return;
    }
    const std::uint8_t command_byte = payload[COMMAND_RESP_COMMAND];
    const bool unsolicited = (command_byte & COMMAND_UNSOLICITED_FLAG) != 0;

    CommandResponse response;
    response.command =
        command_byte & static_cast<std::uint8_t>(~COMMAND_UNSOLICITED_FLAG);
    response.command_sequence = payload[COMMAND_RESP_CMD_SEQUENCE];
    response.response_sequence = payload[COMMAND_RESP_RESP_SEQUENCE];
    for (std::size_t i = 0; i < COMMAND_MAX_RESULTS; ++i) {
        response.results[i] = payload[COMMAND_RESP_RESULTS + i];
    }

    // An unsolicited Initialize while READY may be the first of a reset's two
    // announcements, ahead of the reset-complete (SR-5); remember it for the
    // handshake that reset-complete would restart. It still reaches the
    // command sink, as any unsolicited response does.
    if (state_ == SessionState::READY &&
        command_byte == INITIALIZE_UNSOLICITED) {
        held_initialize_ = HeldInitialize{
            .at_us = hal().now_us(),
            .ok = payload[COMMAND_RESP_RESULTS] == INITIALIZE_OK};
    }

    // A solicited response matching the outstanding command's sequence
    // completes it; an unsolicited response never does (SR-2).
    if (commands_.on_response(response, unsolicited)) {
        ++counters_.command_completions;
    }
    command_sink_(response);
}

void Session::handle_feature_response(std::span<const std::uint8_t> payload) {
    if (payload.size() < FEATURE_REPORT_SIZE) {
        ++counters_.protocol_errors;
        return;
    }
    ++counters_.feature_responses;
    feature_sink_(parse_feature_response(payload.first<FEATURE_REPORT_SIZE>()));
}

void Session::deliver_frs(const detail::FrsEngine::Completion& completion) {
    // SR-3: the words are a view of the engine's one record buffer, which a
    // new request would clear. A read the sink asks for is held until it
    // returns, so the view it was handed stays valid for the whole call. A
    // held read that cannot be sent was still accepted, so it ends at the
    // sink like every other read; that completion carries no words, so it
    // holds nothing more and the loop runs at most twice.
    detail::FrsEngine::Completion current = completion;
    for (;;) {
        if (current.status == Sh2Error::NONE) {
            ++counters_.frs_completions;
        }
        // Only a completion carrying words has a record to protect; a read
        // asked for from any other is sent at once, so a sink retrying on
        // failure gets its error back rather than another call.
        delivering_frs_record_ = !current.words.empty();
        frs_sink_(current.status, current.type, current.words,
                  current.device_status);
        delivering_frs_record_ = false;
        if (!deferred_frs_read_.has_value()) {
            return;
        }
        const std::uint16_t type = *deferred_frs_read_;
        deferred_frs_read_.reset();
        // A sink that called begin() restarted the hub, so the session is no
        // longer READY; like any running operation, the held read is dropped.
        if (!ready()) {
            return;
        }
        auto started =
            frs_.begin_read(type, hal().now_us(), timing_.frs_timeout_us, tx_);
        if (started.ok()) {
            return;
        }
        current = detail::FrsEngine::Completion{.finished = true,
                                                .status = started.error(),
                                                .type = type,
                                                .words = {},
                                                .device_status = {}};
    }
}

// --- host->hub requests ------------------------------------------------------

SessionBinding library_binding() noexcept {
    // Compiled here, in the library's own translation unit, so it reports the
    // archive's profile rather than the caller's. binding_matches() compares
    // the two and is the check a consumer can make before the constructor's
    // abort() makes it for them.
    return Session::caller_binding();
}

Result<void> Session::set_feature(const FeatureConfig& config) {
    if (!ready()) {
        return Sh2Error::NOT_READY;
    }
    std::array<std::uint8_t, FEATURE_REPORT_SIZE> body{};
    const std::size_t length = serialize_set_feature(config, body);
    return tx_.enqueue(shtp::ChannelRole::CONTROL,
                       std::span<const std::uint8_t>(body.data(), length));
}

Result<void> Session::get_feature(reports::SensorId sensor) {
    if (!ready()) {
        return Sh2Error::NOT_READY;
    }
    std::array<std::uint8_t, 2> body{GET_FEATURE_REQUEST,
                                     static_cast<std::uint8_t>(sensor)};
    return tx_.enqueue(shtp::ChannelRole::CONTROL, body);
}

Result<void> Session::request_reset() {
    std::array<std::uint8_t, 1> body{EXECUTABLE_RESET};
    return tx_.enqueue(shtp::ChannelRole::DEVICE, body);
}

Result<void> Session::force_flush(reports::SensorId sensor) {
    if (!ready()) {
        return Sh2Error::NOT_READY;
    }
    auto body = serialize_force_flush(sensor);
    auto sent = tx_.enqueue(shtp::ChannelRole::CONTROL, body);
    if (sent.ok()) {
        ++counters_.flushes_requested;
    }
    return sent;
}

Result<void> Session::send_command(std::uint8_t command,
                                   std::span<const std::uint8_t> params) {
    if (!ready()) {
        return Sh2Error::NOT_READY;
    }
    auto sent = commands_.send(command, params, hal().now_us(),
                               timing_.command_timeout_us, tx_);
    if (sent.ok()) {
        ++counters_.commands_sent;
    }
    return sent;
}

Result<void> Session::frs_read(std::uint16_t frs_type) {
    if (!ready()) {
        return Sh2Error::NOT_READY;
    }
    // From inside the FRS sink while it holds a record: held until the sink
    // returns (deliver_frs).
    if (delivering_frs_record_) {
        if (deferred_frs_read_.has_value()) {
            return Sh2Error::BUSY;
        }
        deferred_frs_read_ = frs_type;
        return Result<void>{};
    }
    return frs_.begin_read(frs_type, hal().now_us(), timing_.frs_timeout_us,
                           tx_);
}

Result<void> Session::frs_write(std::uint16_t frs_type,
                                std::span<const std::uint32_t> words) {
    if (!ready()) {
        return Sh2Error::NOT_READY;
    }
    // From inside the FRS sink while it holds a record a write is refused:
    // it could only be sent
    // after the sink returns, by which time a record in the sink's own frame
    // is gone, and the buffer it would be copied into holds the record being
    // delivered.
    if (delivering_frs_record_) {
        return Sh2Error::BUSY;
    }
    return frs_.begin_write(frs_type, words, hal().now_us(),
                            timing_.frs_timeout_us, tx_);
}

}  // namespace pico_sh2
