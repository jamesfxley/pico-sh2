# Using pico-sh2

This guide explains how to drive a session. The
[specification](technical-specification.md) defines guarantees; the
[architecture](architecture.md) explains the implementation.

For Raspberry Pi Pico hardware,
[pico-bno085](https://github.com/jamesfxley/pico-bno085) supplies transports,
pins and a driver façade. Use this guide when porting or working directly with
the protocol layer.

## Supplying a transport

`IHalTransport` contains platform-specific operations:

<!-- example: transport -->
```cpp
class MyTransport final : public pico_sh2::IHalTransport {
    bool data_ready() override;                  // an edge is pending, or the line is asserted
    // the bytes read, and when the edge this read answers arrived
    pico_sh2::Result<pico_sh2::ReceivedTransfer> read_transfer(std::span<std::uint8_t> dst) override;
    pico_sh2::Result<void> write_transfer(std::span<const std::uint8_t> frame) override;
    std::uint64_t now_us() override;             // monotonic
    void set_reset(bool asserted) override;      // drive the hub's reset line
};
```

An idle read succeeds with a four-byte zeroed header. SHTP treats length zero
as a null header with no cargo, the payload carried by SHTP transfers
(1000-3535 §2.3.1). A read error during startup faults the session. While READY,
it restarts the handshake; without a new advertisement, recovery then faults
with `TIMEOUT` (SR-5). Returning an error for an idle bus therefore breaks an
otherwise healthy session.

The timestamp must be latched in the interrupt. SH-2 deltas are relative to the
data-ready assertion; sampling at read time adds host scheduling delay to every
event time. Return the edge consumed by the read. Taking the edge time and
clearing its pending flag together keeps them tied to one assertion.
`data_ready()` can see an asserted line before the interrupt captures its edge;
reading the timestamp then can return the previous edge and time the cargo early
without a diagnostic.

A 64-bit interrupt timestamp needs careful publication on a 32-bit core: a
reader can see the value between its two stores. The reader needs a consistent
value from one write. pico-bno085's `EdgeLatch` uses a seqlock for this.

## Bringing a session up

`begin()` asserts reset and starts the non-blocking handshake. Drive its waits
and three startup announcements by calling `service()`:

<!-- example: session_bring_up -->
```cpp
// Timing is part-specific; SH-2 and SHTP define no defaults.
// Zero timing faults with INVALID_ARGUMENT.
const pico_sh2::SessionTiming timing{
    .reset_hold_us = 2'000,
    .boot_delay_us = 100'000,
    .advertisement_timeout_us = 200'000,
    .reset_complete_timeout_us = 200'000,
    .initialize_timeout_us = 200'000,
    .command_timeout_us = 500'000,
    .frs_timeout_us = 500'000,
};

pico_sh2::Session session(transport, timing);
session.set_event_sink(pico_sh2::reports::EventSink(&on_event, &ctx));
session.begin();

while (true) {
    session.service();
    if (session.ready()) { /* the hub is configurable */ }
}
```

The session matches advertised channel names within their application GUIDs
(SR-10). To recover a session stuck short of ready, `request_reset()` needs
the device channel before the first advertisement, so seed that number:

<!-- example: channel_map -->
```cpp
pico_sh2::shtp::ChannelMap channels;
channels.seed(pico_sh2::shtp::ChannelRole::DEVICE, 1);  // your part's layout
pico_sh2::Session session(transport, timing, channels);
```

Advertisement overrides seeds. Once it arrives, a wrong seed no longer affects
routing. Without a device seed, `request_reset()` returns `NOT_READY` until the
channel is learned.

Each announcement wait has a deadline. On expiry, `fault()` reports the error
and `startup_phase()` identifies the failed step. Reset hold and boot delay are
timed transitions before those waits.

## Configuring sensors

`set_feature()` requests an interval; the hub's granted interval can differ:

<!-- example: set_feature -->
```cpp
pico_sh2::FeatureConfig config;
config.sensor = pico_sh2::reports::SensorId::ROTATION_VECTOR;
config.report_interval_us = 10'000;         // 100 Hz
config.flags = pico_sh2::feature_flag::WAKE;  // deliver on the wake channel
if (auto sent = session.set_feature(config); !sent.ok()) {
    // NOT_READY before the handshake, QUEUE_FULL if the outbound queue is full
}
```

Requests return `[[nodiscard]]` results. Ignoring one produces a warning and
fails a `-Werror` build. A failed request sends nothing, so its result is the
caller's only record of the error.

A non-zero interval requests that rate or faster. A sensor unable to run that
fast uses its minimum interval (1000-3625 §5.4.1). Treat the request as a target
and read Get Feature responses for the granted configuration.

The session keeps no requested-configuration table. After reset it restarts the
handshake and notifies the consumer, which reapplies configuration.
pico-bno085's driver implements that recovery.

## Receiving events

The event sink runs during `service()`. Events are decoded and unit-converted;
`event_us` combines the transport timestamp with report deltas. The sample is a
`std::variant`. `std::get_if` returns null when its type differs:

<!-- example: on_event -->
```cpp
void on_event(void* /*ctx*/, const pico_sh2::reports::SensorEvent& event) {
    using pico_sh2::reports::RotationSample;
    if (const auto* rotation = std::get_if<RotationSample>(&event.payload)) {
        use(rotation->unit, rotation->accuracy_rad, event.event_us);
    }
}
```

`event.kind()` names the held sample type for switching between alternatives.

Event times can move backwards. Base deltas and report delays use 100 µs ticks,
while the edge uses microseconds. Independently quantised cargo bases can put
consecutive events a tick or two out of order. Order events when needed; do not
assert monotonicity.

## Transactions

Commands and Flash Record System (FRS) operations are asynchronous, with one
command and one FRS operation allowed at a time. Calls enqueue requests,
`service()` sends them, and responses reach sinks. Another request of the same
kind returns `BUSY` while one is outstanding.

Use `send_command_without_response()` for a command the hub sends no response
to, such as Configure Periodic DCD Save (0x09), Clear DCD and Reset (0x0B)
or Wheel Request (0x0F) (1000-3625 §6.4.7, §6.4.9.1, §6.4.13.1).
The library keeps no list of such commands; the caller chooses the call.

<!-- example: transactions -->
```cpp
session.set_command_sink(pico_sh2::Session::CommandSink(&on_command, &ctx));
if (!session.send_command(pico_sh2::command::SAVE_DCD).ok()) { /* ... */ }

session.set_frs_sink(pico_sh2::Session::FrsSink(&on_frs, &ctx));
if (!session.frs_read(0x4D4D).ok()) { /* ... */ }  // completes at the FRS sink
```

Each accepted FRS operation completes at its sink exactly once, except when
`begin()` discards it for an explicit restart. Completion carries the record or
an error:

- Hub errors retain the hub's status byte, including `BUSY` when the hub was
  already running another FRS operation.
- `PROTOCOL` means a missing word or an undefined read status prevented a
  complete record.
- `TABLE_FULL` means the record exceeded the profile's FRS capacity.
- `TIMEOUT` means its deadline expired; `HUB_RESET` means reset interrupted it.

An operation ended locally can still run on the hub: a timeout, a full read
buffer before the final response, or a write unable to queue its next data.
`frs_busy()` stays true and new FRS requests return `BUSY` until the hub ends
that
operation or one more `frs_timeout_us` expires (SR-3). Retry after it falls.
Commands have no failure completion: `command_pending()` falls on timeout or
reset, reported by the diagnostics or reset sink.

Sinks are synchronous and may call requests or `begin()`. A diagnostics sink
that calls `begin()` on the `STARTUP_FAULT` that `begin()` raises for invalid
timing recurses without bound. While an
FRS sink borrows record words, a read requested from it waits until return so
the
words stay valid. A write returns `BUSY`; issue it from the consumer loop.
If the deferred read cannot then be queued, its error completes at the sink.

Without borrowed words, a sink request runs normally. It succeeds or returns an
error: `BUSY` during the hold after a locally ended operation, or `NOT_READY`
after `HUB_RESET` while startup restarts. A sink's `begin()` ends that service
pass and prevents further delivery from its traffic. A sink's `service()` call
does nothing.

## Resetting

`request_reset()` is the usual choice; clear calibration only when that is the
aim.

- `request_reset()` restarts the hub without clearing learned calibration.
- `send_command(command::CLEAR_DCD_AND_RESET)` also clears dynamic calibration;
  the hub rebuilds it through motion.

The hub re-announces itself and the session recognises the reset. Reapply sensor
configuration afterwards. Pending FRS work ends with `HUB_RESET`, and a pending
command is no longer pending.

## Reading counters

`counters()` returns the set. The generated `for_each_counter` visits names and
values together, so reporters include added counters without maintaining a list:

<!-- example: counters -->
```cpp
pico_sh2::for_each_counter(session.counters(),
    [](const char* name, const char* /*description*/, std::uint32_t value) {
        report_counter(name, value);  // however you report status
    });
```

Counters wrap at 2^32. Subtract successive readings to measure increments across
one wrap (OR-1).

`set_diag_sink()` reports startup faults, hub resets, command and FRS timeouts,
and transport write errors. The callback runs inside `service()`, or inside
`begin()` when invalid timing faults startup. Keep it short; log the first
occurrence and count repeats to limit output.
