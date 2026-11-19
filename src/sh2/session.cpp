// SPDX-License-Identifier: MIT
// Session wiring for startup, commands, FRS, input, reset recovery and counters
// (SR-1, SR-2, SR-3, SR-4, SR-5, OR-1).
#include "pico_sh2/sh2/session.hpp"

#include "pico_sh2/pico_sh2.hpp"
#include "pico_sh2/sh2/profile.hpp"
#include "pico_sh2/sh2/startup_phase.hpp"

// Bind the public version to CMake, including builds with tests disabled.
// The umbrella target has no translation unit; private definitions reach this
// one.
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
    // The default argument carries the caller's profile. A mismatch can corrupt
    // memory during member initialisation, before this check.
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
    // Zero timing would release reset or expire a deadline immediately.
    if (!timing_.valid()) {
        fail(Sh2Error::INVALID_ARGUMENT);
        return;
    }
    // service() checks this after each sink to stop traffic from the previous
    // run.
    ++generation_;
    // Explicit restart discards FRS work without completion; the caller
    // requested it.
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
    // Channel bases can change at reset (1000-3600 §2.3), unless the device
    // channel just received confirms the learned base. Consumer seeds survive
    // (SR-7).
    if (learned == LearnedChannels::FORGET) {
        channels_.forget_learned();
    }
    engine_.reset();
    commands_.reset();
    // A held Initialize belongs only to the reset that follows it (SR-5).
    held_initialize_.reset();
    // Complete interrupted FRS work once (SR-3).
    // Commands have no error completion; the reset sink signals their
    // abandonment.
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
    // A nested drain would overwrite the transfer still being delivered.
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

    // Stop after a fault to preserve its error and phase; stop after begin()
    // because this pass's traffic and timings belong to the previous run.
    if (state_ == SessionState::FAULTED || generation_ != generation) {
        return;
    }

    // Flush before READY too, so an early reset request reaches the hub (SR-7).
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
    // The transfer limit bounds drain latency with a stuck-low HINTN (AR-3).
    while (hal().data_ready() &&
           drained < ActiveProfile::MAX_TRANSFERS_PER_SERVICE) {
        ++drained;
        auto read = hal().read_transfer(rx_staging_);
        if (!read.ok()) {
            // A read error in READY starts reset recovery (SR-5). An absent
            // advertisement then times out. During startup, the read error
            // faults directly.
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
        // Framing retains the lead edge and counts malformed or dropped input;
        // mirror_counters() publishes those counts.
        (void)engine_.ingest(transfer, read.value().intn_edge_us);
        // Keep draining after phase changes, unless a sink called begin().
        if (generation_ != drain_generation_) {
            return;
        }
        // A later transfer must not overwrite a fault with READY (SR-1).
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
    // Routing uses advertised roles and counts unknown channels. Channel 0 is
    // fixed, so advertisements can arrive before roles are learned (1000-3535
    // §5.1).
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

void Session::handle_command_channel(std::span<const std::uint8_t> payload) {
    // Channel 0 starts with a response code; Error List is not an advertisement
    // (1000-3535 §5.1, §5.1.2.1, §5.1.3).
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
    // An advertisement needs TLV entries; an empty body cannot advance startup.
    if (payload.empty()) {
        ++counters_.protocol_errors;
        return;
    }
    // Restart before parsing a re-advertisement (1000-3535 §2.3.1),
    // so a malformed one cannot leave READY over a reseeded table.
    if (state_ == SessionState::READY) {
        if (!restart_handshake()) {
            return;
        }
    } else if (startup_.phase() != StartupPhase::WAIT_ADVERTISEMENT) {
        // An out-of-phase advertisement starts fresh tables.
        engine_.reset();
        // A held frame carries a number from before the reset (NR-7).
        tx_.discard_framed();
        input_.reseed_lengths();
    }

    auto applied = shtp::parse_advertisement(
        payload,
        shtp::ReportLengthSink::bind<Session, &Session::on_report_length>(this),
        shtp::ChannelSink::bind<Session, &Session::on_advertised_channel>(
            this));
    if (!applied.ok()) {
        // Parsing can apply entries before finding truncation; discard those
        // changes. Keep consumer seeds so early reset requests still work
        // (SR-7).
        ++counters_.protocol_errors;
        channels_.forget_learned();
        input_.reseed_lengths();
        return;
    }
    // Incomplete channel roles cannot route SH-2 traffic.
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
    // A reset-complete without the advertisement it should have followed
    // triggers a request for channel and length tables (1000-3535 §5.1.1). The
    // learned device channel confirms the base, so keep the map to route
    // Initialize until the advertisement replaces it.
    if (state_ == SessionState::READY) {
        // Taken before the restart, which forgets it (SR-5).
        const std::optional<HeldInitialize> held = held_initialize_;
        if (!restart_handshake(LearnedChannels::KEEP)) {
            return;
        }
        startup_.on_reset_complete_without_advertisement();
        // The restart emptied the queue; an enqueue error would leave the
        // advertisement deadline to end the wait.
        static_cast<void>(
            tx_.enqueue(shtp::COMMAND_CHANNEL, GET_ADVERTISEMENT_REQUEST));
        // Initialize can precede reset-complete while READY. Reuse it only
        // within its phase timeout (SR-5).
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
    // Only unsolicited Initialize advances this phase; other reports leave it
    // waiting.
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

// Bind seeded report lengths to the sizes their handlers parse.
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
    // A cargo can contain several reports (1000-3600 §2.3). Without a length,
    // no next boundary is known; pass the remainder to the handler to count it.
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
    // A sink's begin() invalidates the rest of the cargo being walked.
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

    // An Initialize can precede reset-complete and is retained (SR-5).
    // It still reaches the command sink like other unsolicited responses.
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
    // Deferring re-entrant reads keeps the sink's borrowed words valid (SR-3).
    // A deferred enqueue error completes without words, so this loop runs at
    // most twice.
    detail::FrsEngine::Completion current = completion;
    for (;;) {
        if (current.status == Sh2Error::NONE) {
            ++counters_.frs_completions;
        }
        // With no borrowed words, a read the sink requests is sent at once, so
        // a retry gets its error directly.
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

SessionBinding library_binding() noexcept {
    // This translation unit uses the library archive's profile.
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
    // The sink still borrows the record buffer; defer its read until return.
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
    // A write from inside the sink is refused: the destination buffer holds the
    // delivered record, and the write's source might not outlive the sink.
    if (delivering_frs_record_) {
        return Sh2Error::BUSY;
    }
    return frs_.begin_write(frs_type, words, hal().now_us(),
                            timing_.frs_timeout_us, tx_);
}

}  // namespace pico_sh2
