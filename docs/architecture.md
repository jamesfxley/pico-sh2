# pico-sh2 Architecture

This document describes the implementation and its design reasons. FRS means
Flash Record System.
[technical-specification.md](technical-specification.md) owns the requirements.

## Layers

Each layer has a CMake target under `pico_sh2::` and can be tested without the
layers above it:

```
        consumer (e.g. pico-bno085)
                  │  implements
                  ▼
          IHalTransport  ── the only hardware interface (sh2/hal_transport.hpp)
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

`shtp/` and `reports/` are independent siblings. The report-length table belongs
to `reports/` because it maps SH-2 report IDs to wire lengths. SHTP delivers the
advertisement that supplies those lengths but does not interpret the reports.

- `IHalTransport` supplies the bus, clock and reset operations. Reads return
  their HINTN edge timestamps because only the transport knows which edge a
  read consumed. Bus operations run in `begin()` or `service()`; requests read
  `now_us()` to start deadlines. The interface is synchronous: read and write
  buffers are borrowed only during the call. An SPI implementation owns the
  wake handshake: assert WAKE, wait for HINTN, assert CSN and clock the bytes.
- `ShtpEngine` frames and reassembles cargo, the payload carried by one or more
  SHTP transfers. It retains the lead edge for timestamp reconstruction.
  Sequence and drop tracking cover `MAX_TRACKED_CHANNELS` to bound memory;
  higher channels still route without sequence-gap accounting. The limit
  covers SH-2 bases through 11 without tracking all 256 wire channel numbers.
- `reports/` holds the `constexpr` decoder table, mapping sensor IDs to
  lengths, default fixed-point scaling exponents (Q points) and decode
  functions.
  A new sensor needs a sample type, a decode function, a decoder table row and a
  golden vector. Names are spelt out to avoid collisions with generic SDK macros
  such as `STATIC` and `ERROR`.

## Inside Session

`Session` coordinates five internal engines in `sh2/detail/`. Each owns its
state; startup, commands and FRS also own deadlines.

| Engine | Owns |
|---|---|
| `StartupSequencer` | Reset, boot and announcement phases with their deadlines |
| `CommandEngine` | Sequence numbers and one outstanding command |
| `FrsEngine` | One FRS operation, record storage, write pacing and deadlines |
| `InputRouter` | Report splitting, timestamp reconstruction and Q overrides |
| `TxPath` | Queued outbound payloads and their framing |

`begin()` and `restart_handshake()` reset the engines. Startup begins on the
first path and restarts on the second. `begin()` discards FRS work; recovery
reports its failure to the consumer (SR-3). An added engine must be reset on
both paths because the compiler cannot check that either list is complete.

Q overrides describe the part and survive reset. Advertised lengths describe
the hub's advertisement and are reseeded during recovery.
Announcement ordering is documented in
[`startup.hpp`](../src/sh2/include/pico_sh2/sh2/detail/startup.hpp).

## Service model

The consumer's interrupt handler latches a HINTN flag and microsecond timestamp.
The main loop calls `Session::service()`; no RTOS or interrupt callback is
required. The timestamp anchors event reconstruction independently of read
scheduling. The transport requirements are in the
[usage guide](usage-guide.md#supplying-a-transport).

`MAX_TRANSFERS_PER_SERVICE` bounds each service pass even if HINTN stays low.
The callback and restart rules are in the
[usage guide](usage-guide.md#transactions).

## Errors

Public fallible mutators return `Result<T>` or `Result<void>`. Queries that can
have no answer return `std::optional`. Distinct errors let callers choose
recovery for a full table, malformed argument or busy engine.

Malformed hub input is counted because it is not an error in the call draining
it. The diagnostics callback reports selected events as they occur. It uses a
bound `Callback`, so an unset sink needs only a null check and no allocation.

## Allocation

The active profile in `sh2/profile.hpp` sizes fixed buffers. After `begin()`,
there is no allocation in the protocol library. Capacity failures return errors
or increment drop counters; no payload is silently truncated. The profile's
ceiling bounds `sizeof(Session)`. Host-only `FakeHub` uses the heap.

The default receive buffer must hold the header plus a typical hub transfer;
partial reads allow larger cargoes to arrive in several transfers. The
Q-override table has room for every decoder table entry and the
gyro-integrated rotation vector.

Read staging is reused for each transfer, and TX framing scratch for each
queued cargo. A transport that retains either buffer past its call reads or
writes storage the library reuses for other transfers.

The profile reaches consumers as a public compile definition. A different
consumer profile changes layout and can corrupt memory. The constructor
compares caller and archive bindings, including capacities, size and
alignment, before any member is initialised. It aborts on disagreement, so a
mismatch leaves a caller object sized by a smaller profile untouched.
`MAX_ENABLED_SENSORS` is checked separately because it sizes a driver's table
without changing `Session`. A hand-written build or stale archive can bypass
CMake propagation; a consumer can call `binding_matches()` before construction
to log the mismatch. On a nosys target, the constructor's abort alone can leave
a silent boot failure.

## Counters

`sh2/session_counters.def` owns counter names and descriptions. It generates
`SessionCounters` and `for_each_counter`. Iterating the generated visitor keeps
labels and values aligned and includes added counters without changing
reporters.

## Bootloader and DFU

`Session::send_command` can issue bootloader command 13 and its subcommands
(1000-3625 §6.4.11). Firmware-image transfer belongs to the consumer. The three
cited manuals do not describe that transfer (`UNCONFIRMED`); an incorrect
transfer risks making the part unusable.
