# Future work

Deferred work follows
[EN-03.7](../infra/house/standard/engineering/03-documentation.md).

## FW-1 — Three fixture calibration commands

### What is absent

The command layer supports tare, motion-engine calibration configuration,
dynamic calibration saving and clearing, reset, oscillator queries and the error
queue. Three commands in 1000-3625 have no constant or documented use:

- Simple and turntable calibration (§6.4.10) needs a known-rate turntable.
- Interactive calibration (§6.4.12) needs a defined motion sequence and
  reference.
- Wheel requests (§6.4.13) need a wheeled platform with encoder input.

### Why it is not built

The requests and responses are small additions, but each needs a fixture to
validate its effect. Adding an unverified command would imply working support.

### What would have to be true to build it

Each fixture is available, with a recorded run and its conditions (EN-06.7).

### Version cost

Each command is additive within the existing API, so it is not breaking
(EN-06.1).

### Affected repositories

pico-sh2.

## FW-2 — Decoders beyond the core set

### What is absent

The decoder table covers inertial, tap, step and stability reports. Other BNO08x
reports include significant motion, shake, flip, pickup, sleep, tilt, pocket,
circle, heart rate and personal activity. Each needs a sample type, decoder,
decoder table row and golden vector.

### Why it is not built

None has been exercised on hardware. A decoder and constructed golden vector
based on the same manual reading could agree and still misread real output.

### What would have to be true to build it

Each decoder runs on hardware with a captured report as its golden vector.
The capture supplies evidence independent of the implementation.

### Version cost

Each decoder adds a report type and decoder table row, so it is not breaking
(EN-06.1). Compatibility with consumers that exhaustively visit the growing
event variant is UNCONFIRMED.

### Affected repositories

pico-sh2 would hold decoder additions and golden vectors. pico-bno085 would
supply bench captures; its partner entry cites this one.

## FW-3 — UART-SHTP and UART-RVC transports

### What is absent

Neither UART-SHTP nor fixed-format UART-RVC is implemented. `IHalTransport`
cannot express those BNO08x modes.

### Why it is not built

`IHalTransport` uses a data-ready signal and latched edge timestamp to determine
when to read and anchor event times. UART provides neither in that form, so
support changes the interface.

### What would have to be true to build it

Readiness covers a signal or byte stream without adding UART concepts to I2C
and SPI transports. The specification defines the UART event-time reference.

### Version cost

Changing the transport interface breaks consumers; pico-bno085 implements it
twice (EN-06.1).

### Affected repositories

pico-sh2 owns the transport interface. pico-bno085 would implement the revised
interface; its partner entry cites this one.

## FW-4 — Streaming reassembly

### What is absent

Cargo, the payload carried by SHTP transfers, is reassembled in a profile-sized
buffer. Oversize cargoes are dropped whole and counted. Decoding fragments as
they arrive would remove that buffer and its capacity limit.

### Why it is not built

The fixed buffer has a known cost, and its limit matters when the hub batches
many sensors. A streaming parser must retain partial reports across transfers,
adding complexity to a path that drops oversize input safely. Truncating a
report could produce plausible wrong values; dropping a cargo is counted.

### What would have to be true to build it

A profile needs a batch larger than its buffer can hold.

### Version cost

Reassembly is internal, so this is not breaking (EN-06.1).

### Affected repositories

pico-sh2.

## FW-5 — Hub behaviour the CEVA manuals leave UNCONFIRMED

### What is absent

The specification marks these manual omissions `UNCONFIRMED`:

- The GUID width limit (NR-4).
- Boundaries between channel-0 responses.
- Non-dense Flash Record System (FRS) reads, status ordering and responses after
  a busy write.
- Whether rebase deltas sum.
- Initialize/reset ordering and the actual channel advertisement.
- Q point 2's applicability to gyro-integrated rotation vectors.

No bench observations are recorded.

### Why it is not built

The library uses stated fallbacks where its authoritative manuals are silent.
Each unresolved case needs observation on a real hub.

### What would have to be true to build it

A recorded observation settles each case, or `UNCONFIRMED` remains with a reason
it cannot be observed. The specification then describes the observed behaviour.

### Version cost

Added specification wording alone is not breaking (EN-06.1). A contradictory
observation needs a compatibility assessment when behaviour changes.

### Affected repositories

pico-sh2 owns the unresolved protocol questions. pico-bno085 hosts the bench;
its partner entry cites this one.
