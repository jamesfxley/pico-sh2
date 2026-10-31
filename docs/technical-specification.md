# pico-sh2 Technical Specification

**Status:** Normative.

This specification defines the architecture, framing, session, decoding,
boundary-safety, timing, and observability requirements for the pico-sh2 SH-2 /
SHTP protocol library. Every requirement has an ID
(`AR`/`NR`/`SR`/`DR`/`BR`/`TR`/`OR`) and is mapped to an acceptance test in the
traceability matrix. [architecture.md](architecture.md) describes the
implementation, and work deliberately not built is an entry in
[future-work.md](future-work.md) (`EN-03.7`) — so this document can describe
what is, rather than a mixture of what is and what is intended.

---

## 1. Scope and targets

| | |
|---|---|
| Protocol | SH-2 sensor protocol over SHTP, as used by the CEVA/Hillcrest BNO08x (BNO085/BNO086). |
| Platform | **Independent.** No Pico SDK, no OS. All hardware access is through `IHalTransport` (`pico_sh2/sh2/hal_transport.hpp`). The library also holds no part-specific *values*: reset and boot timing, transaction deadlines and channel numbers are supplied by the consumer or read from the hub, never compiled in. |
| Transports | Any byte transport a host implements behind `IHalTransport`; I2C and SPI are provided by the pico-bno085 consumer, not here. |
| Out of scope | The physical bus drivers, pin electrical handling and the interrupt latch, UART-RVC/UART-SHTP modes (FW-3), and DFU/bootloader — all owned by the platform consumer. The library drives the *timing* of the reset pulse through `IHalTransport::set_reset` (SR-1), because that is protocol knowledge; which pin it is, and how it is driven, is the consumer's. |

Two named sizing profiles parameterize every static buffer (`profile.hpp`):

- **default**: single-IMU host, a handful of enabled sensors.
- **large**: wider reassembly buffer, TX queue, and enabled-sensor count.

Profiles are selected by the `PICO_SH2_PROFILE` build option, which sets the
`PICO_SH2_ACTIVE_PROFILE` type macro. Each buffer knob is bound to the protocol
invariant it must satisfy by a `static_assert`, so a knob cannot drift from the
wire fact that sizes it. Total session state is bounded separately by the
profile's `SESSION_SIZE_CEILING`, asserted against `sizeof(Session)` — a
footprint gate rather than a protocol invariant, so a new member cannot grow the
session unnoticed.

**API stability:** the public API (`namespace pico_sh2`, headers under
`pico_sh2/`) follows semantic versioning (`EN-06.1`): a breaking change lands at a minor version
bump while the major version is 0, and at a major version bump from 1.0.

---

## 2. Architecture requirements (AR)

- **AR-1 — Zero post-init heap.** The library performs no dynamic allocation
  after `Session::begin()` returns, on any code path. Fixed-capacity types only;
  capacity is a compile-time property of the active profile.
- **AR-2 — Bounded inbound copy.** A cargo delivered in a single transfer is
  parsed in place from the staging buffer; only a continuation-fragmented cargo
  incurs a single copy into the reassembly buffer.
- **AR-3 — Service-context confinement.** Every bus transaction — an
  `IHalTransport` read, write or reset — happens in `Session::service()` /
  `begin()` context; a host->hub request reads only the clock, to start its
  deadline when it is made. Every call *on* a `Session` —
  `begin()`, `service()`, and each host->hub request — is made from that one
  context. Nothing in a `Session` is synchronised: the outbound queue, the
  counters and the session state are single-context by construction, so a
  request staged from a second core or an interrupt while `service()` drains
  races with no diagnostic and fails only on target. The library is correct when
  the consumer's interrupt handler does nothing but latch the HINTN flag and
  timestamp. A single `service()` call drains a bounded number of transfers, so
  worst-case call latency is bounded and a HAL that reports `data_ready()`
  indefinitely (e.g. a stuck-low line under a level fallback) cannot spin the
  drain loop; the remainder is drained on subsequent calls. Every sink runs
  inside `service()` and may call back into its session — a host->hub request,
  or `begin()`. A `begin()` from a sink ends the pass it was made in: nothing
  further from the traffic then in flight reaches a sink, because it belongs to
  the hub incarnation the call restarted. `service()` called from a sink returns
  without doing anything, since a nested drain would overwrite the transfer the
  outer pass is still delivering from.
- **AR-4 — Platform independence.** No Pico SDK or OS headers; time and bus I/O
  reach the library only through `IHalTransport`. Enforced by the cross build,
  whose probe links no SDK library (exceptions/RTTI off). That probe is bounded
  using pico-tooling:AR-11 (ALLOC except equal-VMA/LMA read-only LOAD, heap
  and stack reservations), at <= 8 KiB of static RAM and <= 48 KiB of flash
  (every `LOAD` section), gated
  by the footprint gate. Both halves, because the profile's fixed-capacity
  buffers land in RAM and the report decoder table lands in flash.

- **AR-5 — The sizing profile is a stated interface.** A consumer may supply
  its own profile: the CMake cache variable `PICO_SH2_PROFILE_HEADER` names, by
  absolute path, a header that `profile.hpp` includes after `DefaultProfile`,
  and that header defines the profile type and `PICO_SH2_ACTIVE_PROFILE` to
  name it. The path is a PUBLIC definition, so the library and every
  translation unit linking it compile the same profile; setting it together
  with a `PICO_SH2_PROFILE` other than `default` is refused at configure. The
  knobs a profile must declare and the range each may take are therefore part
  of this library's contract rather than of its assert expressions.
  `Sh2Profile` is the concept a profile is checked against, at the point the
  active profile is named; every knob is `std::size_t`, and a knob declared as
  any other type does not satisfy it, so no conversion can carry a negative
  value past a range. `ProfileBounds<P>` states each range below as a named
  predicate over a profile `P`, and `ALL` as all of them together; the active
  profile's asserts read them, so a consumer can check a profile of its own
  against the same ranges the library holds it to.

  | Knob | Range | What it sizes |
  |---|---|---|
  | `MAX_RX_TRANSFER_LEN` | `>= 32`, and `<= MAX_CARGO_LEN + 4` | The staging buffer one HAL transfer is read into. A transfer cannot exceed one cargo plus an SHTP header |
  | `MAX_CARGO_LEN` | `>= 21` | The reassembly buffer. Must hold the hub's advertisement, or its channels are never learned |
  | `MAX_TX_CARGO` | `>= 21` | One outbound cargo payload. 21 is a Set Feature cargo, the largest this library builds |
  | `TX_QUEUE_DEPTH` | `>= 2` | The outbound queue's element count. The ring counts its elements rather than masking an index, so any depth works |
  | `MAX_ENABLED_SENSORS` | `[4, 64]` | Nothing here. A driver built on this library sizes its desired-sensor table from the forwarded profile |
  | `FRS_MAX_WORDS` | `[64, 65535]` | The FRS record buffer, in 32-bit words. 64 is the User Record, the largest the manual bounds; 65535 is the largest a 16-bit write length and word offset address (1000-3625 §6.3.3, §6.3.4) |
  | `MAX_TRANSFERS_PER_SERVICE` | `>= 1` | Transfers drained from the HAL in one `service()` call. Bounds the caller's latency, not throughput: the remainder drains on the next call |
  | `SESSION_SIZE_CEILING` | `>= MAX_RX_TRANSFER_LEN + MAX_CARGO_LEN + 4 * FRS_MAX_WORDS + 4 * ceil(FRS_MAX_WORDS / 32)` | Nothing. It bounds `sizeof(Session)`, so a new member cannot grow the library's static footprint unnoticed |

  A profile that disagrees with the archive a consumer links is refused at
  construction rather than corrupting memory, where the disagreement is in a
  knob the binding carries — every knob but `MAX_TRANSFERS_PER_SERVICE` and
  `SESSION_SIZE_CEILING`, which size no member — or in `sizeof(Session)` or its
  alignment: `Session::caller_binding()` is evaluated in the caller's
  translation unit and compared with the library's.

## 3. Protocol conformance (NR) — SHTP framing

- **NR-1 — Reserved length rejected.** A header whose length field is `0xFFFF`
  is rejected as `BAD_HEADER`. 1000-3535 §2.2.1 reserves that value "because a
  failed peripheral can too easily produce 0xFFFF", which is what makes it the
  dead-bus signature rather than a length; the transfer
  is discarded and counted, never parsed. A transfer shorter than the 4-byte
  minimum is likewise rejected, as is a nonzero length field below the 4-byte
  header size that field must count (impossible; a length of 0 is a valid null
  header — no cargo).
- **NR-2 — Fragmentation asymmetry.** Inbound cargoes are reassembled across
  continuation transfers into a single buffer; outbound framing fails with
  `CARGO_TOO_LARGE` rather than fragmenting. 1000-3535 §2.4 permits a hub that
  cannot support fragmented writes to advertise identical maximum transfer and
  cargo lengths, in which case "all legal writes must be performed in a single
  non-fragmented transfer", and a host that fragments anyway earns a distinct error from the hub (§5.1.2.1, Figure 15,
  codes 5 and 6). A cargo interrupted by a
  new lead transfer, a stray continuation, or a cross-channel continuation is
  dropped whole and counted once per cargo: the continuations of a dropped
  cargo are discarded without counting again until the next lead transfer on
  their channel.
- **NR-3 — Sequence gap accounting.** Per-channel receive sequence gaps are
  detected and counted; a gap is diagnostic, never fatal. The scope is the
  protocol's: 1000-3535 §2.2.1 gives each channel and each direction its own
  sequence number.
- **NR-4 — Advertisement report lengths.** Byte 0 of a channel-0 read cargo is
  the SHTP response code (1000-3535 §5.1), and only Response 0 carries an
  advertisement; its `(reportID, length)` entries (tag `0x81`) override the compiled report-length
  seed and drive splitting of concatenated reports. Tags `0x80`–`0xFF` are
  scoped to the nearest preceding GUID (1000-3535 §5.2); tag `0x81` is the
  report-length list only under the SH-2 sensorhub application (1000-3600 §2.3),
  whereas under the SHTP application (GUID 0) `0x81` is the UART-timeout value
  (1000-3535 §5.3) and is not applied, and under any other application it is
  that application's own tag and is not applied either. A truncated TLV entry,
  an odd-length report-length array, or a GUID longer than four bytes is
  rejected as `PROTOCOL`, never read out of bounds. The four-byte GUID width is
  this library's limit: UNCONFIRMED (FW-5) — 1000-3535 §5.2 does not state a GUID's
  width, and 1000-3600 is silent.
  A cargo from the hub may hold several reports, each as long as its entry
  says (1000-3600 §2.3), and control-channel cargoes are split by the same
  table as input cargoes; the seed carries the control responses the session
  routes. A report whose ID has no length ends its cargo's walk and is counted
  in `unrouted_cargoes`; a sink that calls `begin()`, or an Initialize response
  that faults the handshake, ends it too. A channel-0 read cargo may hold more
  than one response (1000-3535 §5.1), but neither the advertisement nor the
  Error List states its own length, so a channel-0 cargo is read as one
  response: where a response followed by another ends is UNCONFIRMED (FW-5).
- **NR-5 — Reassembly completion.** A continuation-fragmented cargo is complete
  when the payload accumulated across its transfers reaches the total length
  declared by the lead (continuation-flag-clear) transfer's header — not by
  short-transfer termination. The lead length field counts the whole cargo plus
  one header and may be spread over multiple messages; continuation length
  fields carry the *remaining* count and are not consulted for completion
  (1000-3535 §2.2.1, §2.3.1). Payload taken from each transfer is its physical
  size less the header, so a lead fragment that reports the whole cargo but
  carries only part contributes only what it carries. Bytes beyond the declared
  cargo length are ignored (duplex padding, 1000-3535 §2.3.1). The sink receives
  the reassembled cargo exactly once.

- **NR-6 — Arbitrary input reaches a verdict.** Any sequence of transfer bytes
  fed through a full session terminates in a verdict — delivered, dropped or
  counted — without a hang, an assertion failure, or undefined behaviour. The
  framing and decoder layers each carry their own fuzz target; this one covers
  the state machine that composes them, where a hang is reachable that neither
  layer alone can produce.

## 4. Session requirements (SR) — SH-2

- **SR-1 — Startup ordering.** `begin()` asserts the hub's reset line, holds it
  for the consumer's reset-hold interval (`RESET_ASSERTED`), releases it and
  waits out the consumer's boot interval (`BOOTING`), and only then listens for
  the advertisement. The session reaches `ready` only after the advertisement
  (command channel), followed by both post-advertisement announcements:
  reset-complete on the device channel (1-byte cargo `0x01`) and the unsolicited
  Initialize response (0xF1 command `0x84`, R0 status success) on the control
  channel. An Initialize response whose R0 reports failure faults the session
  with `PROTOCOL`. The two are issued by independent applications; **the session does
  not depend on which arrives first** and accepts either order, and the reported
  state names whichever is still outstanding. That is a property of this
  implementation, not a claim about any part's behaviour. Each of the three
  waiting phases is deadline-bounded, and a deadline expiry records the faulting
  phase (`startup_phase()`) so it is attributable to a distinct phase. The two
  timed phases carry no deadline and need none: they are driven by elapsed time
  against the monotonic clock `IHalTransport::now_us()` promises, so each one
  terminates by construction, and a deadline read from that same clock could
  never expire before the phase it bounds had already advanced.
  A fault ends the `service()` pass that raised it: no transfer after it is
  consumed in that pass, so the fault and its phase are what the consumer
  reads afterwards.
  How long a hub must be held in reset, how long it takes to boot, and how
  quickly it answers are properties of the part: `SessionTiming` carries no
  defaults, and `begin()` faults with `INVALID_ARGUMENT` rather than starting on
  a zeroed one.
- **SR-2 — One command in flight.** Exactly one command (0xF2) is outstanding at
  a time (a second request returns `BUSY`); responses (0xF1) are correlated by
  the request's sequence number echoed in the response's command-sequence field,
  and a response whose command byte has its MSB set is routed as unsolicited,
  never completing a pending command. A parameter block longer than the
  request's `COMMAND_MAX_PARAMS` bytes is refused with `INVALID_ARGUMENT` and
  nothing is sent.
- **SR-3 — FRS read/write.** Exactly one FRS operation runs at a time (`BUSY`
  otherwise). A read (0xF4) accumulates streamed 0xF3 responses (up to two words
  each, placed at their record offset) until a record-completed status, then
  delivers the record words; an unrecognized-type status surfaces `UNSUPPORTED`
  and a device-error status aborts the read as `PROTOCOL`; record-empty ends it
  with `NONE` and no words. Busy ends it with `BUSY`: the hub answers busy to a
  read made while another FRS operation runs, and then completes that one
  (1000-3625 §6.3.6). A deprecated (4, 6, 7) or reserved (9–15) status ends the
  read with `PROTOCOL`: only the statuses defined for a read in progress carry
  it forward. A completed read covers every word below its highest, or it ends
  with `PROTOCOL` and no words: whether the hub may answer a read non-densely is
  UNCONFIRMED (FW-5; 1000-3625 §6.3.7 does not say), and a gap is what a dropped
  response or the tail of an earlier read of the same record leaves. The buffer
  is cleared for each read, so no word is retained from a previous one. A
  response is applied only when its FRS Type names the record being read
  (1000-3625 §6.3.7, Figure 39); one naming another record is the tail of an
  earlier read that has already ended, and is counted (`frs_stray_responses`)
  rather than applied. A response placing a word past the profile's FRS buffer
  ends the read with `TABLE_FULL`, never a record completed short. Only one FRS
  operation may be in progress on the hub at a time (1000-3625 §6.3.7), and one
  the session ended itself — past its deadline, for a full buffer before the
  record's last response, or because a data report could not be queued — may
  still be running there. So the session holds: `frs_busy()` is true and a new
  operation is refused `BUSY`, and a response to the ended operation is stray,
  until the hub ends it with a status of its own, for one more FRS timeout if it
  never does, or until a hub reset or `begin()`. A write (0xF7) sends ordered
  2-word 0xF6 chunks paced by each 0xF5 status; write completed (3) finishes it
  with `NONE`, and any failure status, record invalid (9) among them, aborts
  with the device status surfaced. Record valid (8) reports the validation and
  not the write, so the write runs on to write completed, and a held write's
  hold does not end at it. 1000-3625 §6.3.5 defines the codes without ordering
  them; their ordering is UNCONFIRMED (FW-5) by the documents. The order follows CEVA's
  reference host driver (`ceva-dsp/sh2`
  `b514b1e`, `sh2.c` `setFrsRx`), which waits past record valid and ends a write
  only at write completed or a failure status. That is vendor code, not a
  vendor document. Busy ends it with
  `BUSY`; what the hub does after a busy write status is UNCONFIRMED (FW-5; 1000-3625
  §6.3.5 lists the code only). Both carry a deadline (TR-1). The FRS sink
  receives the hub's status byte whenever the hub's own response ended the
  operation, and none when the session ended it. Every operation `frs_read()` or
  `frs_write()` accepts ends exactly once, at the FRS sink: with its outcome,
  `TIMEOUT` past its deadline, or `HUB_RESET` when an unexpected hub reset
  (SR-5) cuts it off. `begin()` is the exception: it discards a running
  operation without a completion, because the caller asked for the restart. The
  record words delivered to the FRS sink stay valid for the whole call: while
  the sink holds a record's words, a read it requests is held and sent once it
  returns, and a write requested is refused `BUSY`, since it could only be sent
  after the sink's own frame is gone. A held read that cannot then be sent ends
  at the FRS sink with the error that refused it, because `frs_read()` has
  already accepted it. A completion carrying no words has nothing to protect, so
  a read asked for from that sink is sent at once and its error returned, and a
  sink retrying on failure cannot hold `service()`.
- **SR-4 — Timestamp reconstruction.** For a batched input cargo,
  `event_us = intn_edge_us − 100·base_delta + 100·(Σ rebases + delay)`, where
  `base_delta`/rebases come from the cargo's `0xFB`/`0xFA` records and `delay`
  from each report's 14-bit delay field; a base delta of `0x7FFFFFFF` marks
  events `timebase_overflow` and falls back to the HINTN edge rather than
  producing a wrong time. A reconstructed time that would precede zero is
  clamped to zero, and one past the top of the 64-bit range to its maximum. A
  `0xFB`/`0xFA` record whose advertised length cannot carry its delta aborts the
  cargo walk and is counted (`truncated_reports`), rather than timing the events
  after it from bytes that are not a delta. Summing the rebases is UNCONFIRMED (FW-5):
  1000-3625 §7.2.2 defines each Rebase Delta relative to the Base Timestamp
  Reference, which read literally replaces rather than adds; the two agree for a
  cargo with one rebase. `intn_edge_us` is the edge of the cargo's lead transfer — a
  fragmented cargo is timed from its first HINT only (1000-3535 §2.6). The
  compact gyro-integrated report carries no delay and is timed at the HINTN edge
  directly. It arrives on the gyro-RV input channel, which SR-10 resolves by
  role: the number depends on the base the hub advertises, and naming one here
  would contradict the requirement that makes this library part-independent.
- **SR-5 — Reset recovery.** An advertisement (channel-0 Response 0) or
  reset-complete arriving after the session is `ready` is recognized as an
  unexpected hub reset; the hub's Error List (Response 1, 1000-3535 §5.1.2.1)
  is counted (`hub_error_lists`) and is not one. On a reset, SHTP framing
  state and the report-length table are reset, the handshake restarts, and the
  consumer is notified. An FRS operation in progress ends at the FRS sink with
  `HUB_RESET` (SR-3). An outstanding command is dropped — `command_pending()`
  falls — and the reset notification is its notice, as the diagnostics hook is
  for its timeout; the command sink carries responses, not failures. Replaying
  desired sensor configurations afterwards is the consumer's, since the session
  holds no record of what was asked for.
  A reset-complete while `ready` means the advertisement before it was missed,
  and what that advertisement teaches is not recoverable otherwise: the
  session requests it (SHTP Command 0 with P0 = 1, 1000-3535 §5.1.1) and waits
  in `WAIT_ADVERTISEMENT`, holding the reset-complete and an Initialize
  response that arrives meanwhile, so the requested advertisement completes
  the handshake when both have been seen. An Initialize response that arrived
  first, while the session was still `ready`, is held too, as long as it is no
  older than the Initialize phase's own timeout when the reset-complete comes,
  and no restart has come between; it still reaches the command sink as any
  unsolicited response does. Whether
  a hub can send the two in that order is UNCONFIRMED (FW-5): the manuals do not
  order them, and SR-1 does not depend on it. The learned channel numbers are kept
  meanwhile (SR-10), so the Initialize response routes whether or not the
  consumer seeded the control channel. `begin()` abandons the request.
  A transport error on a read issued while `ready` is recognized the same way
  and counted in `transport_restarts`. A read is issued only behind a data-ready
  indication, so an error there is the hub leaving the bus between indicating
  and answering — the signature of a reset on any transport able to report one,
  and indistinguishable from an idle transfer on one that cannot. The session
  restarts the handshake and waits for the re-advertisement rather than
  faulting; a transport that is dead rather than resetting never re-advertises,
  so the advertisement deadline surfaces it as `TIMEOUT` in
  `WAIT_ADVERTISEMENT`. Outside `ready` the hub is already expected to be
  silent, so a transport error during a startup phase faults that phase.

- **SR-6 — Force flush.** `force_flush(sensor)` sends a Force Sensor Flush
  (0xF0, 1000-3625 §6.5.6) so a batching sensor delivers its outstanding data without
  waiting for its batch interval. The hub may flush other sensors as well, and
  answers each flushed sensor with its own Flush Completed report (0xEF,
  1000-3625 §6.5.7), so every such report is delivered to the flush sink rather than only
  the one matching the request. A Flush Completed too short to name a sensor is
  a protocol error and is not delivered.
  Both reports travel on the SH-2 control channel: 1000-3600 §2.2 (Figure 2)
  defines its write direction as carrying force-flush reports and its read
  direction as "all reports other than sensor data reports", while the input
  channels carry only sensor data. 0xEF's length is nonetheless seeded into the
  report-length table, so a hub that batched it with input reports would cost
  one skipped report rather than the remainder of the cargo.

- **SR-7 — Host-requested reset.** `request_reset()` writes the one-byte reset
  command on the device channel (1000-3600 §2.2). The hub restarts and
  re-announces itself, which SR-5 already recognises, so recovery needs nothing
  further. It is distinct from the Clear DCD and Reset command (1000-3625 §6.4.9), which
  additionally discards the hub's accumulated dynamic calibration. Unlike the
  control-channel requests it does not require `ready` — but it does require the
  device channel to have a number (SR-10). A consumer that seeded one has a way
  out of a session stuck short of `ready`; one that did not is refused with
  `NOT_READY` until the hub advertises, because the library will not guess a
  channel number.

- **SR-8 — Feature control.** `set_feature(config)` enqueues a Set Feature
  request (0xFD) and `get_feature(sensor)` a Get Feature request (0xFE), both on
  the control channel and both refused with `NOT_READY` before the handshake
  completes. A Get Feature Response (0xFC) is parsed and delivered to the
  feature sink; one shorter than the report is a protocol error and is not
  delivered. The session holds no record of what was requested — the granted
  configuration is whatever the response reports, and replaying configurations
  after a hub reset is the consumer's (SR-5).

  1000-3600 Figure 2 gives Get Feature Response as 0xF2, conflicting with
  1000-3625 §6.5.5's 0xFC. SR-8 follows the latter.

- **SR-9 — Wake and non-wake input channels.** Batched sensor input arrives on
  both the non-wake and wake input channels, and both are routed through the
  same decode path: a report's channel selects neither its decoder nor its
  timestamp reconstruction. Input on either is only routed once `ready`; outside
  `ready` it is counted as unrouted rather than decoded. The gyro-integrated
  channel is separate and carries its own report format (DR-1, SR-4).

- **SR-10 — Channel binding.** A cargo is routed by the *role* its channel
  plays, not by a channel number compiled into the library. SHTP fixes one
  number — the command channel is channel 0 (1000-3535 §5.1) — and SH-2's five
  sit at `N`..`N+4` for a base the hub selects "at compile time or run time"
  (1000-3600 §2.3), so the rest are read from the channel-0 advertisement.
  Each is matched by its advertised `ChannelName` (tag 9) — `device` under the
  executable application (GUID 1); `control`, `inputNormal`, `inputWake` and
  `inputGyroRv` under the sensorhub (GUID 2). **Matching is scoped to the
  advertising GUID**: the SHTP application also advertises a channel named
  `control`, and a name-only match binds SH-2's control role to the command
  channel. A cargo on a channel no role claims is counted as unrouted rather
  than guessed at. An advertisement that leaves a role unbound does not advance
  an unseeded session, and advances a seeded one, whose seed supplies the role.
  A consumer may seed the map for the operations that must work
  before a hub has advertised (SR-7); the advertisement overrides a seed, and a
  hub reset discards what was learned but keeps the seed, because a hub that
  reset may return at a different base (1000-3600 §2.3 lets N be chosen at run
  time). The exception is a reset announced by a reset-complete while
  `ready` (SR-5): it arrived on the learned device channel, which is N itself,
  so the learned numbers are kept until the requested advertisement replaces
  them.

  1000-3600 Figure 3 places `inputNormal` under tag 8, but 1000-3535 §5.2
  defines ChannelName as 9. SR-10 follows tag 9. The part's actual advertisement
  is UNCONFIRMED (FW-5) until device bring-up records it.

## 5. Decoder requirements (DR)

- **DR-1 — Typed, unit-converted events.** Each in-scope report ID decodes to a
  typed `SensorEvent` with fixed-point fields converted by the report's Q-point.
  The sample is a `std::variant` whose alternative is the event's `kind()`, so a
  read of the wrong sample type returns null from `std::get_if` rather than
  another sample's bytes.
  The compact gyro-integrated rotation vector (0x2A), which carries no report
  prefix and arrives one-per-transfer on the gyro-RV input channel (SR-10, not a
  fixed number), is decoded separately.
- **DR-2 — Metadata Q override.** The input decoders resolve each report's Q
  point(s) from an override table at decode time, falling back to the compiled
  default (1000-3625 §6.5) when no override is installed. A report uses at most
  a primary Q, for its data fields, and a secondary Q, for a bias or accuracy
  field — the two roles 1000-3625 §5.1 gives a sensor's Q point 1 and Q point
  2. The secondary Q scales the uncalibrated reports' bias and the rotation
  vectors' heading accuracy, and nothing else; an installed override replaces
  either. The gyro-integrated rotation vector has no bias or accuracy field, so
  Q point 2 applicability is UNCONFIRMED (FW-5); the library does not apply it to its
  angular velocity: that stays at the Q10
  §6.5.44 fixes, and no override moves it. A Q point beyond the representable
  maximum (which would make the decoder's `2^Q` scaling undefined) is refused on
  both the metadata and direct-install paths, and the override table refuses a
  new report ID past capacity rather than dropping it (AR-1). Overrides are
  part-intrinsic and persist across a hub reset (unlike the report lengths,
  which the advertisement re-establishes). The authoritative Q points a part
  publishes in its sensor-metadata FRS records (0xE301–0xE325) are obtained with
  an FRS read (SR-3): the record's word 7 packs Q point 1 (data fields) and
  Q point 2 (bias/accuracy fields) in a layout common to record revisions 3 and
  4 (1000-3625 §5.1, Figures 28–29), and Figure 27 fixes which
  sensor each record type describes. A completed metadata record is installed as
  the override for its sensor's input report in one call; a record too short to
  reach word 7, a Q point outside the representable range, or a record type
  outside the decodable set is rejected rather than installed.
- **DR-3 — Unknown-report handling.** A report with a known advertised length
  but no decoder, or with a length shorter than its decoder needs, is skipped
  and counted; an unknown length, or a report whose
  length overruns the cargo, aborts the cargo walk (it cannot resynchronize) and
  is counted — as does a timestamp record too short for its delta (SR-4).

## 6. Boundary requirements (BR)

- **BR-1 — Oversize cargo dropped whole.** A cargo exceeding `MAX_CARGO_LEN` is
  dropped in full and counted once (`reassembly_drops`), however many transfers
  carry it, never truncated into a partial event.
- **BR-2 — TX queue overflow surfaced.** A full TX queue returns `QUEUE_FULL` to
  the caller: a write is never dropped *at enqueue time*, and the refusal is
  returned rather than counted. The exception is a read held for the FRS sink
  (SR-3): `frs_read()` accepted it before it was enqueued, so its refusal ends
  it at the FRS sink instead. Host->hub requests are enqueued and flushed to
  the HAL during `service()` (AR-3); a payload larger than a queue slot is
  likewise refused, with `QUEUE_FULL` — `TxPath::enqueue` rejects it before
  framing runs, so NR-2's `CARGO_TOO_LARGE` is the framing layer's contract and
  reaches a consumer only through `ShtpEngine::frame_outbound` directly.

  Two later drops are deliberate and outside that guarantee. A staged cargo
  whose framing fails is discarded rather than left to wedge the queue behind
  it, and `begin()` discards everything staged before it, because a previous
  incarnation's requests have no meaning to a hub that has just reset — which is
  what `BeginDropsCargoesQueuedBeforeIt` asserts. Neither is counted, and
  neither is reachable through the session's own enqueue path.

## 7. Timing requirements (TR)

- **TR-1 — Deadline-bounded operations.** Startup phases, commands, and FRS
  operations each carry a deadline; expiry surfaces `TIMEOUT` (startup faults the
  session with the faulting phase recorded; a command or FRS deadline clears the
  outstanding operation and counts a timeout). A deadline saturates at the
  clock's end, so the longest timeout `SessionTiming` accepts never wraps into
  the past.

## 8. Observability requirements (OR)

- **OR-1 — Diagnostic counters.** The session maintains the counter set defined
  in `session_counters.def`, which is the definition rather than a copy of it.
  Every counter is a 32-bit count that wraps modulo 2^32, so a consumer takes
  the difference between two readings rather than reading a total, which is
  correct across one wrap; it never decreases otherwise, is cleared only by
  constructing a new `Session`, and is reachable by a visitor that walks the
  set without naming a member — so a counter added to the `.def` reaches a
  consumer's reporting without a change there.
- **OR-2 — Allocation-free diagnostics hook.** An optional diagnostics hook
  surfaces notable real-time events (startup fault, hub reset, command/FRS
  timeout, a frame write the transport refused). It is allocation-free and a
  no-op when unset. A refused write is also counted (`write_errors`), and its
  frame stays queued for the next pass.

---

## 9. Traceability matrix

| Req | Test(s) | Pass criterion |
|-----|---------|----------------|
| AR-1 | `AllocationCounterIsLive`; `symbols` c-heap-alloc rows over sh2/shtp/reports objects and the refused allocating fixture; `session_heap_test` no-heap-after-begin over the input drain and `NoHeapAllocationHandlingControlReplies` over the control replies (armed allocation counter); host ASan/UBSan; cross build | Zero allocations recorded inside either armed region. The sanitizers show memory safety, not the absence of allocation |
| AR-2 | `engine_test` single-transfer in-place delivery / fragmented delivery from the reassembly buffer | A single-transfer cargo is delivered without a copy; a fragmented one from the reassembly buffer |
| AR-3 | `tx_path_test` `YieldsForDuplexInputBeforeWritingAgain`; `session_reset_test` bounded drain per `service()`, `BeginDropsCargoesQueuedBeforeIt`, `RequestResetDoesNotNeedReady`, `BeginFromTheResetSinkRestartsCleanly`, `BeginFromTheEventSinkDropsTheRestOfTheCargo`, `ServiceFromASinkIsANoOp` | A queued write yields without a write error when duplex input becomes ready; the next pass drains it before the next queued write. No more than `MAX_TRANSFERS_PER_SERVICE` transfers are drained in one call, asserted against the knob rather than against the flood size. The service-context confinement half is what `BeginDropsCargoesQueuedBeforeIt` and `RequestResetDoesNotNeedReady` establish: a request reaches the HAL only after `service()` runs, never at the call that enqueued it. That confinement is what makes the single-context counter policy sound. A `begin()` from a sink holds the hub in reset for the full interval and reaches `ready` again, and delivers none of the cargo it interrupted; a `service()` from a sink leaves event order unchanged |
| AR-4 | `firmware` CI job — `pico_sh2_cross_probe`; the footprint gate over the same image (`pico.toml`) | Compiles and links for Cortex-M33 against nosys alone, reaching no SDK or OS symbol, and stays inside the 8 KiB static-RAM and 48 KiB flash budgets |
| AR-5 | the consumer case `custom` in `house.toml`, which installs the library with a profile header of the consumer's own and asserts its knobs in the consumer's translation unit; the case `refused`, where a consumer profile with `FRS_MAX_WORDS` past its range fails the library's build on that bound's message; the case `conflict`, where a profile header with `PICO_SH2_PROFILE=large` is refused at configure naming both; `profile_test` — `EachBoundHoldsAtItsLimitAndRefusesOnePast`, `AllIsEveryBound`, `ConceptRejectsAProfileMissingAKnob`, `ConceptRejectsAKnobThatIsNotASize`, `LargeInheritsTheKnobsItDoesNotChange`, `CeilingClearsTheBuffersTheProfileAsksFor`; `session_control_test` — `MismatchedProfileBindingAborts`, `ProfileBindingDetectsLayoutDrift`, `ProfileBindingCatchesAKnobThatSizesNothingHere`, `MatchingProfileBindingConstructs` | Each of the table's eight ranges, as a `ProfileBounds` bound the active profile's static_asserts read, holds at the table's number and refuses one past it (the ceiling's counting the record's coverage bits), and `ALL` is every bound; both built-in profiles sit inside them; a profile outside a range does not compile; a profile header with another named profile is refused at configure; a profile missing a knob, or declaring one as another type, does not satisfy `Sh2Profile`; a knob `LargeProfile` does not override reads through to `DefaultProfile`; the ceiling clears the buffers the same profile asks for; a consumer's own profile header reaches the library and the consumer alike; a binding disagreeing in a named knob, in `sizeof(Session)`, in its alignment, or in a knob that sizes nothing here aborts, and a matching one constructs |
| NR-6 | `session_fuzz` (`pico_sh2_session_fuzz`), run by CTest over its seed corpus for 100000 inputs in the fuzz tree | Arbitrary transfer bytes fed through a full session reach a verdict without a hang, an assertion failure or a sanitizer report |
| NR-1 | `header_test` reserved/short-length/sub-header-length reject, `engine_test` bad-header counter | Every malformed header rejected and counted; none decoded |
| NR-2 | `engine_test` reassembly, interrupt/stray/cross-channel drop, `StrayContinuationsCountOneDropPerCargo`, outbound-frame reject; `header_test` serialize | Fragments reassembled in order; any interruption drops the whole cargo, never a partial one, and a run of continuations from one lost cargo counts one drop; an outbound cargo too large is refused rather than split |
| NR-3 | `engine_test` sequence-gap; `session_counters_test` gap moves only its own counter | A skipped sequence number is counted once, and the cargo is still delivered |
| NR-4 | `advertisement_test` report-length merge + GUID scoping + `Tag81OutsideTheSensorhubEmitsNothing` + malformed-TLV rejection + `RejectsAGuidLongerThanFourBytes`; `session_startup_test` `StartupReachesReady` (a length differing from the seed); `session_reset_test` `AnErrorListWhileReadyIsNotAReset`, `AReservedChannelZeroResponseIsAProtocolError`, `AnAdvertisementIsReadAfterItsResponseByte`, `ABodilessResponseZeroWhileReadyIsNotAReset`; `session_control_test` `EveryReportInAControlCargoIsRouted`, `AnUnsizedReportEndsTheControlCargo`, `AReportBatchedAfterTheInitializeResponseIsRouted`, `BeginFromASinkEndsTheControlCargo`, `AFailedInitializeEndsTheControlCargo`; `shtp_fuzz` advertisement parse | Advertised lengths override the seed; every report in a control cargo is routed, including one batched after the Initialize response; an unsized report, a restarting sink or a failed Initialize ends the walk; tag 0x81 is applied only under the sensorhub GUID; a malformed TLV or an over-long GUID is rejected as `PROTOCOL`; only Response 0 on channel 0 is read as an advertisement, from the byte after the response code, and one with no body is refused |
| NR-5 | `ReassemblyIsIndependentOfEverySplit`; `engine_test` fragmented reassembly, `BytesBeyondTheDeclaredLengthAreIgnored`; `shtp_fuzz` | Reassembly completes at the declared total, and bytes past it are not cargo in a single transfer or a continuation; the fuzzer shows memory safety across arbitrary fragment boundaries, not the reassembled bytes |
| SR-1 | `session_startup_test` `ResetIsHeldForTheWholeHoldInterval`, `TheBootIntervalIsWaitedOutInFull`, startup ordering + per-phase timeout; empty command-channel cargo does not advance; `session_reset_test` `BusFaultDuringDrainDoesNotDoubleFault`; `session_control_test` `AFaultEndsTheServicePass`; `begin()` rejects unset and part-zeroed timing | Reset stays asserted and the phase stays `RESET_ASSERTED` until the hold interval has elapsed, and `BOOTING` until the boot interval has, with no hub traffic consumed during it; ready is reached from either announcement order; each waiting phase's deadline faults with that phase recorded; a fault ends the pass that raised it, so later transfers in it leave the fault standing; a zeroed timing field faults `INVALID_ARGUMENT` rather than starting |
| SR-2 | `session_control_test` command round-trip / one-in-flight / unsolicited-no-complete / `AResponseToAnotherSequenceDoesNotCompleteTheCommand` / `SendCommandRefusesMoreParametersThanARequestCarries` | One command in flight; a response completes it only when it echoes the request's sequence; an unsolicited response completes nothing; an over-long parameter block is refused and nothing is sent |
| SR-3 | `session_frs_test` FRS read streaming / unrecognized-type / device-error / `FrsReadCompletingWithAGapIsProtocol` / `ARetryAfterATimeoutIsHeldUntilTheTailEnds` / `AnUnfinishedTailHoldsForOneTimeout` / `OnlyTheHeldRecordsLastStatusEndsTheHold` / `AHubResetEndsTheHold` / `ABeginEndsTheHold` / `ATableFullReadHoldsBehindItsTail` / `ATimedOutWriteHoldsUntilTheHubEndsIt` / `AWriteThatCannotBeFedIsHeld` / write pacing / busy / timeout / `FrsReadLongerThanTheBufferEndsWithTableFull` / `AStrayResponseFromAnEarlierReadIsCountedNotApplied`; `session_reset_test` `HubResetEndsAnInProgressFrsOperationThroughItsSink`; `session_frs_test` `FrsReadDeprecatedOrReservedStatusSurfacesProtocol`, `FrsReadBusyEndsTheReadWithBusy`, `FrsWriteBusyEndsTheWriteWithBusy`, `AReadChainedFromTheSinkLeavesTheRecordIntact`, `ARestartFromTheSinkDropsTheChainedRead`, `ARetryOnAFailedReadDoesNotLoop`, `FrsReadOfAnEmptyRecordCompletesWithNoWords`, `FrsWriteFailureSurfacesTheDeviceStatus`, `ACompletedFrsReadCarriesTheDeviceStatus`, `AnFrsTimeoutCarriesNoDeviceStatus`, `ARecordValidWriteWaitsForTheWriteToComplete`, `ARecordValidWriteThatNeverCompletesTimesOut`, `AWriteAnsweredRecordInvalidFails`, `ARecordValidStatusDoesNotEndAHeldWrite` | A read streams into the buffer, and one completing with an uncovered word ends `PROTOCOL` with no words, a dense earlier read's words and coverage carrying into it none; after a timeout, a full buffer before the last response, a timed-out write, or a write whose data report the queue refuses, a new operation is `BUSY` and the ended one's responses are counted stray until the hub ends it with its own last status — a response saying more follows, or naming another record, does not — for at most one more timeout, and not past a hub reset or `begin()`; a write is paced; a second operation is refused; a read chained from the FRS sink leaves the delivered record intact and is sent once the sink returns, a write from it is `BUSY`, and a restart from it drops the chained read; a retry from the sink of a failed read is answered there, not held, so the sink is called twice, and a held read the queue refuses ends at the sink with `QUEUE_FULL`; a record past the buffer ends with `TABLE_FULL`; a response for another record is counted and not applied; an operation a hub reset cuts off completes once with `HUB_RESET`; a deprecated or reserved read status ends the read with `PROTOCOL`, a busy read or write status ends it with `BUSY` and the status byte, and record-empty with `NONE` and no words; a write answered record valid (8) waits, completes `NONE` with status 3 at write completed and times out without it, a held write is not released at 8, and a write answered record invalid (9) fails `PROTOCOL`; a failed write and a completed read reach the sink with the hub's status byte, and a timeout, a full buffer and a hub reset with none |
| SR-4 | `timestamp_test` reconstruction, `ClampsBelowZeroToZero`, `SaturatesAtTheTopOfTheRange`; `decoders_test` walk reconstruction/overflow, `AShortTimestampRecordAbortsAndCounts`; `engine_test` `FragmentedCargoCarriesTheLeadEdge`; `session_input_test` end-to-end input + gyro-integrated timing | Event times equal the edge minus the base delta plus the report delay; a fragmented cargo carries its lead transfer's edge; an overflowed base falls back to the edge; a time before zero reads zero and one past the range its maximum; a gyro-integrated event is timed at its edge; a timestamp record too short for its delta aborts the walk |
| SR-5 | `session_reset_test` unexpected-reset restart + notify; transport error in READY restarts instead of faulting, and a transport that never re-advertises times out in `WAIT_ADVERTISEMENT`; `HubResetClearsAnOutstandingCommand`; `HubResetEndsAnInProgressFrsOperationThroughItsSink`; `AResetCompleteWhileReadyIsAHubReset`, `AResetCompleteWithoutAnAdvertisementRequestsOne`, `RecoveryTakesAnInitializeThatArrivedBeforeTheResetComplete`, `ARememberedFailedInitializeFaults`, `AnInitializeTooOldForTheResetIsNotTaken`, `ARestartForgetsAnInitializeItDidNotTake`, `BeginForgetsARememberedInitialize`, `ARememberedInitializeStillReachesTheCommandSink`, `RecoveryFromAMissedAdvertisementCompletesOnTheRequestedOne`, `RecoveryFromAMissedAdvertisementStillWaitsForInitialize`, `RecoveryFromAMissedAdvertisementNeedsNoSeededControlChannel`, `TheRequestedAdvertisementReplacesTheKeptChannelNumbers`, `ATransportRestartForgetsTheLearnedChannelNumbers`, `BeginAbandonsARequestedAdvertisement`, `AnErrorListWhileReadyIsNotAReset` | A re-advertisement or a transport error while ready restarts the handshake without faulting and notifies the consumer; a transport that never re-advertises times out in `WAIT_ADVERTISEMENT`; an outstanding command is no longer pending and an FRS operation completes with `HUB_RESET`; a reset-complete while ready is a reset too, waits in `WAIT_ADVERTISEMENT` and sends `00 01` on channel 0, and the requested advertisement completes the handshake with the held announcements, with or without a seeded control channel, replacing the kept channel numbers; a transport-error restart forgets them; after `begin()` an early Initialize is unrouted; an Error List is not a reset; an Initialize response arriving before the reset-complete while `ready` completes the recovery, faults it when it reported failure, and is not taken when older than the Initialize timeout or after an intervening restart or `begin()`; it reaches the command sink either way |
| SR-6 | `session_control_test` force-flush request emitted, completion routed to the sink, every completion delivered, short completion rejected; `decoders_test` `AFlushCompletedInAnInputCargoIsSteppedOver` | The request is emitted on the control channel; every completion reaches the sink; a short one is a protocol error; the seeded length table steps over a Flush Completed inside an input cargo |
| SR-7 | `session_reset_test` device-channel command emitted, allowed before ready with a seed, refused without one, hub's re-announcement recognised | The one-byte reset is emitted on the device channel; with a seeded map it is emitted before ready — accepted *and* written to the HAL by the next `service()` — and without one it is refused `NOT_READY`; the hub's re-announcement is recognised |
| SR-8 | `session_control_test` set feature emitted, `GetFeatureEmitsCargo`, `AShortGetFeatureResponseIsNotDelivered`, both refused before ready; `control_reports_test` feature round-trip; `session_counters_test` feature response moves only its own counter | Each request is emitted on the control channel with its report ID and sensor, and refused with `NOT_READY` before the handshake; a Get Feature Response reaches the sink, and a short one does not |
| SR-9 | `session_input_test` wake-channel input delivered and timed; wake input before ready not decoded | A batched cargo on the wake channel decodes through the same path as the non-wake one, with the base-delta and delay timing of SR-4 applied; before `ready` it is counted unrouted and no event is delivered |
| SR-10 | `channels_test` map seed/learn/forget/complete; `advertisement_test` channel emission + GUID-scoped duplicate names + name-must-follow-tag; `session_startup_test` ready at a shifted base, follows a hub that moves base across a reset, incomplete advertisement does not advance; `session_reset_test` seed required before the first advertisement, advertisement overrides a wrong seed; `channel_binding_test` role per GUID; `session_startup_test` incomplete advertisement advances a seeded session; `session_counters_test` `ACargoOnAnUnclaimedChannelMovesCargoesInAndUnrouted` | The handshake completes for any advertised base; a role is bound only from its own application's GUID; a channel no role claims is counted unrouted; learned numbers are dropped across a hub reset other than a reset-complete while ready (SR-5), and the seed is not; a sensorhub name under another GUID binds no role; an incomplete advertisement advances a seeded session |
| DR-1 | `decoders_test` golden vectors; `q_point_test`; `report_layout_test`; `session_input_test` `DeliversGyroIntegratedFromTheGyroRvChannel`; the `sensor_event.hpp` static_asserts | Each decoder's golden vector round-trips to the documented units and Q point, as the sample type its `kind()` names; the variant's alternatives match `EventKind` in order, and `SensorEvent` is trivially copyable |
| DR-2 | `decoders_test` primary/secondary override, `GyroIntegratedOverridesOnlyTheQuaternion`, `UncalibratedBiasTakesTheSecondaryQ`, capacity; `metadata_q_test` record word-7 parse + type→report-ID map; `session_input_test` override applied and reset-persistent, metadata record installed, out-of-range Q refused | An override replaces the default Q and survives a hub reset; the secondary Q scales the bias and accuracy fields only, and the gyro-integrated angular velocity stays at Q10; an out-of-range Q is refused |
| DR-3 | `decoders_test` unknown/skipped/truncated walk, `ALengthShorterThanItsDecoderNeedsIsSkipped`; `report_fuzz`, including walks under input-supplied report lengths | A known-length report with no decoder, or one shorter than its decoder needs, is skipped and counted and the walk continues; an unknown length or a report overrunning the cargo aborts the walk and is counted |
| BR-1 | `engine_test` oversize drop, `AnOversizeCargoCountsOneDropAcrossItsTransfers` | A cargo larger than the profile's buffer is dropped whole and counted once across all its transfers |
| BR-2 | `session_control_test` TX queue overflow; `tx_path_test` | A full queue returns `QUEUE_FULL` and sends nothing, and so does a payload larger than a slot |
| TR-1 | `session_control_test` `CommandTimesOut`, `TheLongestCommandTimeoutDoesNotWrap`, `TheLongestStartupTimeoutDoesNotWrap`; `session_frs_test` `FrsReadTimesOut`, `TheLongestFrsReadTimeoutDoesNotWrap`, `TheLongestFrsWriteTimeoutDoesNotWrap`; `session_startup_test` per-phase deadlines | A command or FRS deadline clears its operation; a startup deadline faults the session with its phase recorded; the longest timeout of each kind does not expire at the next `service()`. An extra service call after command or FRS expiry leaves its timeout count at one |
| OR-1 | `session_counters_test` per-event moved-set cases and `VisitorCoversTheWholeSet` and `EveryEnumeratedCounterIsUnsigned32Bit`; `session_reset_test` `MirroredCountersSurviveATransportError`, `CountersSurviveBegin` | Every counter in the set moves only on the event it names: each case asserts the whole set of counters that moved, not just the one it is about; `begin()` clears none of them, the timeout counters made non-zero first; the visitor walks the whole generated set. Visitor-instantiated static assertions bind every enumerated counter to std::uint32_t; review checks moved-set coverage |
| OR-2 | `session_control_test` diagnostics hook events, `DiagnosticsHookReportsCommandTimeout`, `DiagnosticsHookReportsAHubReset`, `DiagnosticsHookReportsAnFrsTimeout`, `ARefusedWriteIsCountedReportedAndRetried`; `session_heap_test` `NoHeapAllocationReportingADiagnostic` | Each diagnostic event fires once per occurrence; an unset hook allocates nothing inside the armed regions |
