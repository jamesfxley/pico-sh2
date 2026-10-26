# pico-sh2 Architecture

How the library is put together, and why. The normative requirements it
satisfies live in [technical-specification.md](technical-specification.md).

## Layers

Each layer is a CMake target under `pico_sh2::`, and each is testable without
the ones above it:

```
        consumer (e.g. pico-bno085)
                  │  implements
                  ▼
          IHalTransport  ── the single hardware seam (sh2/hal_transport.hpp)
                  │
   ┌──────────────┴───────────────┐
   │           Session            │  sh2/  — startup, feature control,
   │        (see below)           │         commands, FRS, flush, routing
   └──────┬───────────────┬───────┘
          │               │
     ShtpEngine       decoders +      shtp/    framing and reassembly
     (shtp/)          length table    reports/ typed events, report lengths
          │               │
        util/ — Result<T>; Callback and the fixed-capacity containers are pico-util's
```

`shtp/` and `reports/` are siblings: neither includes the other. The
report-length table lives in `reports/` because what it holds is a map of SH-2
report IDs to their wire lengths — SH-2 knowledge that the framing layer has no
use for, even though the advertisement that populates it arrives over SHTP.

- **`IHalTransport`** is the whole platform boundary: `data_ready()`,
  `read_transfer()`, `write_transfer()`, `now_us()`, `set_reset()`. A read
  returns the HINTN edge it answers with the bytes, because only the transport
  knows which edge it consumed. It is synchronous. Every bus transaction on it
  happens in `begin()` or `service()`; a request reads `now_us()` to start its
  deadline when it is made.
- **`ShtpEngine`** turns the raw transfer stream into complete cargoes,
  reassembling continuation fragments and tracking per-channel sequence
  numbers. It also carries each cargo's lead-transfer timestamp forward, so the
  layer that reconstructs event times does not have to re-parse the header to
  find out when the cargo started.
- **`reports/`** holds the `constexpr` decoder table — rows of
  `{SensorId, wire_length, default_q, DecodeFn}` — which turns a raw input
  report into a typed `SensorEvent` with its fixed-point fields converted. A
  new sensor is a struct, a decode function, a row, and a golden vector.

## Inside Session

`Session` is one public façade over five engines, each owning its own state;
the startup, command and FRS engines also own a deadline. They live in `sh2/detail/` and are not API; the split is so
that each piece has one thing to be right about, not so consumers can reach
into them.

| Engine | Owns |
|---|---|
| `StartupSequencer` | The reset pulse, the boot wait, and the three announcements the hub makes on its way up — each with its own phase and deadline |
| `CommandEngine` | Sequence numbers and the single command allowed in flight, with its timeout |
| `FrsEngine` | One FRS read or write at a time: the record buffer, the write pacing, the deadlines |
| `InputRouter` | Splitting batched cargoes into events, reconstructing their times, applying Q overrides |
| `TxPath` | The outbound cargo queue and the framing it is flushed through |

Each engine has an entry point that returns it to its starting state —
`reset()` on most, `reseed_lengths()` on the input router — and two paths call
them all: `begin()`, and `restart_handshake()` when the hub resets. The two
differ in two engines. The startup sequencer is begun on one path and restarted
on the other, and a transfer in flight on the FRS engine is reset by `begin()`
but abandoned by recovery, so its caller hears it failed (SR-3). With session
state divided among the engines, "reset everything" is a short list of calls on
each path rather than a sequence of assignments; nothing makes the compiler
check that a list is complete, so an engine added later must be added to both.

## Service model

There is no RTOS assumption and no callback from an interrupt. The consumer's
interrupt handler latches only the HINTN edge — a flag and a microsecond
timestamp — and returns; the consumer calls `Session::service()` from its main
loop, and every bus transaction happens there.

That edge timestamp is the anchor the whole time base hangs off: SH-2 reports
carry deltas relative to the moment HINTN asserted, not to the moment the host
got around to reading. Capturing it in the handler is what makes event times
independent of when the transfer actually ran.

`service()` bounds how many transfers it will drain in one pass, so a hub with
a stuck-low interrupt line cannot hold the loop.

## Errors

One rule: a fallible mutator returns `Result<void>` (or `Result<T>`), and a
query that may have no answer returns `std::optional`. Nothing returns a bare
`bool` to mean "it didn't work", because that collapses distinct failures — a
full table, a malformed argument, a busy engine — into one indistinguishable
answer, and the caller's recovery differs for each.

Errors that belong to the hub rather than to the caller are counted rather than
returned: a malformed cargo is not a failure of the call that happened to drain
it. The counters are the API for those, and the diagnostics hook reports the
ones worth acting on as they happen. The hook is a bound callback rather than a
`std::function`, so an unset one costs a null check on the call path and no
allocation anywhere.

## Allocation

Every buffer is fixed-capacity and sized from the active profile
(`sh2/profile.hpp`). After `begin()` the library allocates nothing; capacity
limits surface as errors (`CARGO_TOO_LARGE`, `QUEUE_FULL`, `TABLE_FULL`) or as
drop counters, never as silent truncation. `sizeof(Session)` is bounded by a
`static_assert` against a profile knob, so a new buffer cannot quietly grow the
object.

The host-only `FakeHub` in `src/testing/` is exempt: it is test-support code
and uses the heap freely.

## Counters

The counter set is defined once, in `sh2/session_counters.def`, as a list of
name-and-description pairs. `SessionCounters` is generated from it, and so is
`for_each_counter`, which visits every counter without naming any.

A consumer walks the set rather than naming members, so a reporter cannot fall
out of step with the struct: a printer whose field order disagreed with its
labels would report every counter under its neighbour's name. Adding a counter
reaches every reporter without touching one.

## Bootloader and DFU are not this library's

The bootloader commands of 1000-3625 §6.4.11 are SH-2 command requests,
command 13 with a sub-command, and `Session::send_command` can issue them like
any other. Transferring a firmware image to the hub is out of scope: none of
the three CEVA manuals cited here describes that transfer (UNCONFIRMED), and
getting it wrong bricks the part. A consumer that needs it owns it.

