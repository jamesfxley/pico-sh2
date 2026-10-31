# Future work

Work deliberately not done, one entry per deferral, in the form `EN-03.7`
defines.

## FW-1 — Three fixture calibration commands

### What is absent

The command layer covers what a bench can exercise: tare, motion-engine
calibration configuration, saving the dynamic calibration data (once and
periodically), clearing it and resetting, the oscillator type and the error
queue. Three commands that 1000-3625 defines have no constant and no
documented use:

- simple and turntable calibration (§6.4.10), which needs a turntable at a
  known rate;
- interactive calibration (§6.4.12), which needs a defined motion sequence
  against a known reference;
- the wheel request (§6.4.13), which applies to a wheeled platform with encoder
  input.

### Why it is not built

Each is a request and a response like the commands that exist, so writing one
is small. What is missing is a way to tell whether it worked: none can be
validated without its fixture, and a command that has never been confirmed to
do anything looks supported when it is not.

### What would have to be true to build it

Access to the fixture each command needs, and a run against it recorded with
its conditions (`EN-06.7`).

### Version cost

Not breaking under `EN-06.1`: each command is additive, behind the existing
command surface.

### Affected repositories

pico-sh2.

## FW-2 — Decoders beyond the core set

### What is absent

The decoder table covers the core inertial set and the tap, step and stability
reports. The BNO08x defines more: significant motion, shake, flip, pickup,
sleep, tilt, pocket, circle, heart rate and personal activity. Each would need a
struct, a decode function, a table row and a golden vector.

### Why it is not built

None has been exercised against the part. A decoder written from the manual
alone, with a golden vector constructed from the same reading, would pass every
host test and could still be wrong against real output.

### What would have to be true to build it

Each decoder driven on hardware, with a captured report as its golden vector
rather than a constructed one. The code is mechanical; the evidence is the
cost.

### Version cost

Not breaking under `EN-06.1`: each decoder adds a report type and a table row.
Whether growth of the event variant breaks a consumer that visits it
exhaustively is UNCONFIRMED.

### Affected repositories

pico-sh2, where the table and the golden vectors would land; pico-bno085,
whose partner entry cites this one and where the captures come from the bench.

## FW-3 — UART-SHTP and UART-RVC transports

### What is absent

The BNO08x also offers UART-SHTP and a fixed-format UART-RVC mode. Neither is
implemented, and `IHalTransport` has no way to express them.

### Why it is not built

The transport seam is shaped for a bus with a data-ready signal:
`data_ready()` and a latched edge timestamp tell the session when to read and
what to anchor event times to. A UART has neither in that form, so supporting
it changes the seam rather than adding to it.

### What would have to be true to build it

`IHalTransport` expresses readiness in a way that covers both a signal and a
byte stream, without making the I2C and SPI transports carry a concept they do
not need. Event timestamps have a defined anchor on UART, and the specification
says which.

### Version cost

Breaking under `EN-06.1`: `IHalTransport` is the seam every consumer
implements, and pico-bno085 implements it twice.

### Affected repositories

pico-sh2, which owns the seam; pico-bno085, whose partner entry cites this one
and which would implement it.

## FW-4 — Streaming reassembly

### What is absent

A fragmented cargo is reassembled into one buffer sized by the profile, and a
cargo that does not fit is dropped whole and counted. Decoding reports out of
fragments as they arrive would remove the buffer and its ceiling.

### Why it is not built

It is not clearly worth it. The ceiling binds only when a hub batches many
sensors at once, the buffer is a fixed, known cost, and a streaming parser must
hold partial-report state across transfers: more intricate, on a path that
cannot currently get a report wrong. Dropping a cargo whole rather than
truncating it is deliberate: a truncated report would decode into plausible
wrong numbers, and a dropped one is counted.

### What would have to be true to build it

A profile that has to fit a batch the buffer cannot.

### Version cost

Not breaking under `EN-06.1`: the reassembly path is internal.

### Affected repositories

pico-sh2.

## FW-5 — Hub behaviour the CEVA manuals leave UNCONFIRMED

### What is absent

Behaviour the CEVA manuals do not state, which the specification marks
`UNCONFIRMED`: the GUID width limit (NR-4), where one channel-0 response ends
and the next begins, whether a hub answers an FRS read non-densely, the order
of the FRS status codes, what the hub does after a busy write status, whether
rebase deltas sum, the order of an Initialize response and a reset, the actual
channel advertisement, and whether Q point 2 applies to the gyro-integrated
rotation vector. No bench observation of any of them is recorded.

### Why it is not built

Only an observation on a real hub can close each one, and the manuals are the
authority this library cites. Where the manuals are silent, the library follows
the stated fallback and marks the point.

### What would have to be true to build it

Each item closed by a recorded bench observation, or left `UNCONFIRMED` with
the reason it cannot be observed. The specification is then updated to the
observed behaviour.

### Version cost

Depends on what an observation shows. Not breaking under `EN-06.1` while an
observation only adds wording to the specification; a library behaviour that
an observation contradicts is judged when it is changed.

### Affected repositories

pico-sh2; pico-bno085, whose partner entry cites this one and where the bench
is.
