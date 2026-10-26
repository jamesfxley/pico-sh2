# pico-sh2

[![CI](https://github.com/jamesfxley/pico-sh2/actions/workflows/ci.yml/badge.svg)](https://github.com/jamesfxley/pico-sh2/actions/workflows/ci.yml)

`pico-sh2` is a C++20 implementation of the **SH-2** sensor protocol and its
**SHTP** transport, as used by the CEVA/Hillcrest BNO08x family of IMUs
(BNO085/BNO086).

The library **code** is platform-independent: it has no Pico SDK dependency and
no operating-system dependency, and reaches hardware only through a single
transport interface (`IHalTransport`). The same code is host-tested against a
fake hub and runs on a microcontroller behind an I2C or SPI driver.

The **repository**, however, belongs to the `pico-*` family and is oriented
around the Raspberry Pi Pico: the build helpers, shared tooling
(`pico-tooling`, the `infra/` submodule), and CI are the family's, and the library is developed and
validated against the Pico 2 (RP2350) through its sibling
[pico-bno085](https://github.com/jamesfxley/pico-bno085), which implements
`IHalTransport` over RP2350 I2C/SPI and owns the board-specific pin, reset, and
interrupt handling. Porting the *code* to another host means supplying an
`IHalTransport` and a build; nothing in the protocol layer stands in the way,
but the surrounding scaffolding assumes the Pico workflow.

## Quick start

```cmake
include(FetchContent)
FetchContent_Declare(pico_sh2
    GIT_REPOSITORY https://github.com/jamesfxley/pico-sh2.git
    GIT_TAG v0.1.0)  # pin the release you build against
FetchContent_MakeAvailable(pico_sh2)

target_link_libraries(app PRIVATE pico_sh2::core)
```

<!-- example: readme_include -->
```cpp
#include <pico_sh2/pico_sh2.hpp>
```

All public symbols live in `namespace pico_sh2` with headers under the
`pico_sh2/` prefix. The one dependency is
[pico-util](https://github.com/jamesfxley/pico-util), the family's base layer:
`Result`, `Callback`, the endian accessors and the outbound ring are its, so a
consumer sees `pico_util::` types on this library's signatures.

## Layout

The source layout, module by module, is in
[docs/orientation.md](docs/orientation.md). The CEVA manuals every protocol
claim rests on are listed in [docs/README.md](docs/README.md), cited by document
number rather than redistributed.

## Building & tests

Every tool the gates need, the Pico SDK included, is in the pico family
development image that `.devcontainer/devcontainer.json` builds from `infra/`;
open the repository in its dev container to use that image. The `infra/` git
submodule ([pico-tooling](https://github.com/jamesfxley/pico-tooling)) carries
the gates, the CMake helpers and the pinned guides; clone with
`git clone --recurse-submodules` or run
`git submodule update --init --recursive` after a plain clone.
`python3 dev.py build --tree firmware` builds the firmware tree where
`PICO_SDK_PATH` is set.

```sh
python3 dev.py check          # every gate; a change is done when this is green
python3 dev.py build          # clean build and test of every enabled tree
python3 dev.py format --check # clang-format at the pinned major version
python3 dev.py lint           # clang-tidy over the compile database
```

`-DPICO_SH2_PROFILE=large` selects the wider sizing profile, and
`-DPICO_SH2_PROFILE_HEADER=/abs/path/profile.hpp` a profile of your own (AR-5 in
the specification lists the knobs and their ranges); `-DPICO_SH2_SANITIZE=ON`
builds the host suite under ASan+UBSan. A local check selects the profile with
`HOUSE_MATRIX_PROFILE=large python3 dev.py check` (the CI legs do the same), and
the orientation guide lists the rest. A firmware consumer pulls this library in
via `FetchContent`, or declares its pin directly in CMake with the family helpers,
and gets the `pico_sh2::core` target; there is no firmware or networking code here to configure.

## What it does

- **SHTP framing**: the 4-byte transfer header, per-channel/per-direction
  sequence tracking, continuation-fragment reassembly, and the channel-0
  advertisement (including the report-length table).
- **SH-2 session**: the non-blocking startup state machine, Set/Get Feature
  sensor configuration, command/response correlation, the FRS read/write
  engines, force-flush, and batched-report timestamp reconstruction.
- **Report decoding**: a constexpr decoder table turning raw input reports into
  typed, unit-converted sensor events (accelerometer, gyroscope, magnetometer,
  the rotation-vector variants, the gyro-integrated rotation vector, raw
  sensors, and the tap/step/stability reports), extensible in a few lines per
  new report.

The design is allocation-free after start-up: every buffer is fixed-capacity,
sized from a compile-time profile, and capacity failures are surfaced as errors
rather than truncated.

## Documentation

- [Documentation index](docs/README.md) — what each document is for, and the
  CEVA protocol manuals every claim here rests on
- [Orientation](docs/orientation.md) — how the repository is laid out, and the
  traps it has already paid for
- [Contributing](CONTRIBUTING.md) — how a change is made and when it is done
- [Technical specification](docs/technical-specification.md) — normative;
  requirement IDs `AR/NR/SR/DR/BR/TR/OR` with their pass criteria
- [Architecture](docs/architecture.md) — the engines, the service model,
  allocation, the counter set
- [Usage guide](docs/usage-guide.md) — starting a session, configuring sensors,
  receiving events, reading diagnostics
- [Standing deviations](docs/deviations.md) — scope-wide rule suppressions and
  their review dates
- [Releases](https://github.com/jamesfxley/pico-sh2/releases) — what changed per
  release, and what to change in your code when an API broke

## Releases

Versioning is semver: breaking changes only at a major version. The notes on
each [release](https://github.com/jamesfxley/pico-sh2/releases) record what
changed and, for a break, what to change in your own code (`EN-06.3`).

## AI-assisted development

This project is developed with AI assistance. Design decisions, review and
acceptance are human; an AI agent writes and refactors code under direction and
against a written engineering standard. Every change passes the repository's own
gates — build, tests, formatter, linter, deviation counts — before it lands.

This section is disclosure, not a warranty. The terms in [LICENSE](LICENSE)
apply unchanged, including its disclaimer of warranties.

## Development configuration

`house.toml` refers to `infra/family.toml`; the pinned family supplies gates,
trees, CI preparation and synced file selections. Product dependency pins live
in `CMakeLists.txt`. `AGENTS.md` routes to `infra/agents/AGENTS.md`.
House renders root tool files, the workflow and `.devcontainer/devcontainer.json`;
`python3 -m infra.pico_tooling sync` writes the bootstrap. Tests own their
registered `.clang-tidy` overlay. Infrastructure freshness reports adoption as a
warning and passes; unreadable release tags are unavailable evidence.
