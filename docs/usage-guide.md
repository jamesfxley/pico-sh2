# Using pico-sh2

How to drive the library from a consumer. The reference for what it guarantees
is [technical-specification.md](technical-specification.md); how it is built is
[architecture.md](architecture.md).

If you are targeting a Raspberry Pi Pico, you probably want
[pico-bno085](https://github.com/jamesfxley/pico-bno085) instead — it supplies
the transports, the pins, and a driver façade over this. Read on if you are
porting to another host or working on the protocol layer itself.

## Supplying a transport

Everything platform-specific goes behind `IHalTransport`:

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

Four things about it are easy to get wrong:

**A read that found nothing is a success, not an error.** `service()` drains
while `data_ready()` is true and treats a read error as the hub having gone
away. A transport that returns an error for an idle bus faults a session
still starting with that error; on a ready one it restarts the handshake, which
then faults with `TIMEOUT`, because a hub that did not reset sends no new
advertisement (SR-5). Write a zeroed header — an
SHTP header whose length is 0 — and return its four bytes instead; the framing
layer reads that as a null header, a transfer with no cargo (1000-3535 §2.3.1).

**The timestamp must be latched in the interrupt, not sampled when the read
runs.** SH-2 reports
carry deltas relative to the moment the data-ready line asserted. Sampling the
clock when the transfer runs folds the host's scheduling delay into every event
time.

**Return the edge the read consumed.** Take the edge time in the same step that
clears the pending flag, so the two describe the same assertion. `data_ready()`
can see the line asserted before the interrupt has latched the edge behind it,
and a time taken then is the previous edge's: the cargo is timed early by the
gap back to the previous assertion, and nothing counts it.

**A 64-bit timestamp written by an interrupt needs publishing carefully.** On a
32-bit core it is two stores, and a reader can land between them. pico-bno085's
`EdgeLatch` shows one way (a seqlock); the requirement is only that a reader
never observes half of one value and half of another.

## Bringing a session up

`begin()` starts a non-blocking sequence — assert reset, wait, then the three
announcements the hub makes on its way up — and returns immediately. Drive it
by calling `service()`:

<!-- example: session_bring_up -->
```cpp
// Timing is yours to supply: how long the hub is held in reset, how long it
// takes to boot, and how quickly it answers are properties of your part, and
// no SH-2 or SHTP manual states them. There are no defaults, and begin()
// faults with INVALID_ARGUMENT rather than starting on a zeroed struct.
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

Channel numbers are not supplied: the session reads them from the hub's
advertisement, matching each role by its advertised name (SR-10). The one
exception is `request_reset()`, which you may want before the hub has ever
advertised — to recover a session stuck short of `ready`. That needs a channel
number in advance, so seed one:

<!-- example: channel_map -->
```cpp
pico_sh2::shtp::ChannelMap channels;
channels.seed(pico_sh2::shtp::ChannelRole::DEVICE, 1);  // your part's layout
pico_sh2::Session session(transport, timing, channels);
```

A seed is a starting guess, not a claim. The advertisement overrides it, so a
wrong seed costs nothing once the hub has spoken; without one, `request_reset()`
is refused with `NOT_READY` until it has.

Each phase is deadline-bounded. If one expires the session faults, and
`fault()` plus `startup_phase()` say which deadline and which phase — enough to
tell a hub that never came out of reset from one that came up but never
announced itself.

## Configuring sensors

`set_feature` asks for a sensor at a report interval; the hub answers with what
it granted, which may differ:

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

Every request returns a `Result`, and it is `[[nodiscard]]`: a bare
`session.set_feature(config);` draws a warning, and fails a `-Werror` build,
because a refused request sends nothing and the only record of it is the value
returned.

A non-zero report interval asks the hub to generate reports at that interval
"or more frequently"; a sensor that cannot run that fast operates at its own
minimum interval instead (1000-3625 §5.4.1). So treat the request as a target
and read the Get Feature response for what you actually got.

**The session keeps no record of what you asked for.** After a hub reset it
restarts the handshake and tells you, but it does not re-send your
configuration, because it does not know it. That is the consumer's, and
pico-bno085's driver is one implementation of it.

## Receiving events

Events arrive at the event sink during `service()`, already decoded and
unit-converted, with `event_us` reconstructed from the transport timestamp and
the report's own deltas. The sample is a `std::variant`; read it by the type you
expect, and a report of any other kind gives you null rather than the wrong
fields:

<!-- example: on_event -->
```cpp
void on_event(void* /*ctx*/, const pico_sh2::reports::SensorEvent& event) {
    using pico_sh2::reports::RotationSample;
    if (const auto* rotation = std::get_if<RotationSample>(&event.payload)) {
        use(rotation->unit, rotation->accuracy_rad, event.event_us);
    }
}
```

`event.kind()` names which sample is held, for a `switch` over several.

Event times are *not* strictly monotonic. The base delta and report delay are
100 µs ticks while the edge anchoring them is microseconds, so consecutive
events from different cargoes are built from independently quantised bases and
can step backwards by a tick or two. Order events if you need order; do not
assert monotonicity.

## Transactions

Commands and FRS operations are one-at-a-time and asynchronous: the call
enqueues, `service()` sends, and the answer arrives at a sink. A second request
while one is outstanding returns `BUSY`.

<!-- example: transactions -->
```cpp
session.set_command_sink(pico_sh2::Session::CommandSink(&on_command, &ctx));
if (!session.send_command(pico_sh2::command::SAVE_DCD).ok()) { /* ... */ }

session.set_frs_sink(pico_sh2::Session::FrsSink(&on_frs, &ctx));
if (!session.frs_read(0x4D4D).ok()) { /* ... */ }  // completes at the FRS sink
```

An FRS operation the session accepted always ends at the FRS sink, exactly
once: with the record, with the hub's error and its status byte, with `BUSY`
and its status byte if the hub was already running another FRS operation, with
`PROTOCOL` if the hub's answer cannot be a complete record (a word no response
covered, or a status the manual does not define for a read), with `TIMEOUT`, or
with `HUB_RESET` if the hub reset underneath it. A record longer than the
profile's FRS buffer ends with `TABLE_FULL`. `begin()` is the one exception —
it discards a running operation, since you asked for the restart.

An operation the session ended itself — `TIMEOUT`, `TABLE_FULL` on a record
still arriving, or a write it could not keep feeding — may still be running on
the hub, which runs one FRS operation at a time. So `frs_busy()` stays true and
a new FRS request returns `BUSY` until the hub ends it, or for at most one more
`frs_timeout_us` (SR-3). Retry once `frs_busy()` falls. A command has no failure
path through its sink: `command_pending()` falls when it times out or the hub
resets, and the diagnostics hook or the reset sink says which.

Sinks run inside `service()` and may call back into the session: queue a
request, or call `begin()` to start over. While the FRS sink is handed a
record's words, an `frs_read()` from it is held and sent once the sink returns,
so the record stays intact, and an `frs_write()` from it is `BUSY`, so write
from your own loop. If the held read then cannot be queued, it ends at the FRS
sink with that error. A completion with no words (a write's own, a timeout, a
failure) has nothing to protect, so a request from its sink is handled as one
from your own loop: sent, or refused with its error — `BUSY` after an operation
the session ended itself, and `NOT_READY` after `HUB_RESET`, since the session
is restarting. A `begin()` from a sink ends that
`service()` pass, and nothing more from the traffic it interrupted is
delivered. Calling `service()` itself from a sink does nothing.

## Resetting

Two resets, and the difference matters:

- `request_reset()` restarts the hub and keeps what it has learned. This is the
  one you want.
- `send_command(command::CLEAR_DCD_AND_RESET)` also discards the accumulated
  dynamic calibration, which the hub then has to rebuild through motion.

Either way the hub re-announces itself and the session recognises it, so no
special handling is needed on the consumer's side beyond re-applying
configuration — and noticing that a pending FRS operation completed with
`HUB_RESET` and a pending command is no longer pending.

## Reading counters

`counters()` returns the whole set, and `for_each_counter` walks it without
naming any member — which is how to report them without a list that drifts out
of step with the library:

<!-- example: counters -->
```cpp
pico_sh2::for_each_counter(session.counters(),
    [](const char* name, const char* /*description*/, std::uint32_t value) {
        report_counter(name, value);  // however you report status
    });
```

Each counter wraps at 2^32, so compare two readings by their difference, which
stays right across one wrap, rather than reading a total (OR-1).

`set_diag_sink` reports the events worth acting on — a startup fault, a hub
reset, a command or FRS timeout, a write the transport refused — as they
happen. It runs inside `service()`,
so keep it short; log a first occurrence and count the rest rather than
printing every one.
