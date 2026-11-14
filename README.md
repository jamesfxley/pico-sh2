# pico-sh2

[![CI](https://github.com/jamesfxley/pico-sh2/actions/workflows/ci.yml/badge.svg)](https://github.com/jamesfxley/pico-sh2/actions/workflows/ci.yml)

`pico-sh2` implements the SH-2 sensor protocol and SHTP transport in C++20 for
CEVA/Hillcrest BNO08x IMUs (BNO085/BNO086).

The library has no Pico SDK or operating-system dependency. It reaches hardware
through `IHalTransport`, which can connect it to a host fake or an I2C/SPI
driver.
Porting requires a transport implementation and a build.

The repository uses the pico family's shared tooling and CI. Development and
validation target the Pico 2 (RP2350) through
[pico-bno085](https://github.com/jamesfxley/pico-bno085). That driver supplies
RP2350 I2C/SPI transports, pins, reset and interrupt handling. The repository's
build helpers assume the Pico workflow.

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

Public symbols use `namespace pico_sh2` and headers under `pico_sh2/`.
The dependency [pico-util](https://github.com/jamesfxley/pico-util) supplies
`Result`, `Callback`, byte accessors and the outbound ring. Its `pico_util::`
types appear in this library's signatures.

## Layout

[docs/orientation.md](docs/orientation.md) maps the source modules.
[docs/README.md](docs/README.md) lists the CEVA manuals cited by document
number.

## Building & tests

The family's development image contains the tools and Pico SDK used by the
gates. Open this repository in its dev container to use the image built by
`.devcontainer/devcontainer.json`. The `infra/` submodule
([pico-tooling](https://github.com/jamesfxley/pico-tooling)) supplies the gates,
CMake helpers and pinned guides. Clone with `git clone --recurse-submodules`,
or run `git submodule update --init --recursive` after cloning.

```sh
python3 dev.py check          # every gate; a change is done when this is green
python3 dev.py build          # clean build and test of every enabled tree
python3 dev.py format --check # clang-format at the pinned major version
python3 dev.py lint           # clang-tidy over the compile database
```

`python3 dev.py build --tree firmware` builds firmware when `PICO_SDK_PATH` is
set. `tests/device/` checks the consumer's session binding and constructs a
session. Its output follows the [device-test
convention](infra/guides/device.md#device-tests).

Sizing, sanitiser and fuzz configurations are listed in the
[orientation guide](docs/orientation.md#commands). Consumers can fetch the
source with `FetchContent` or declare their pin with the family CMake helpers.
The public aggregate target is `pico_sh2::core`; supported module targets are
listed in the orientation guide. The library has no firmware or networking
configuration of its own.

## What it does

- SHTP framing, per-channel sequence tracking, continuation reassembly and
  channel-0 advertisement parsing, including report lengths.
- Non-blocking SH-2 startup, Set/Get Feature configuration, command correlation,
  Flash Record System (FRS) reads and writes, force-flush and batched timestamp
  reconstruction.
- Typed report decoding for inertial sensors, rotation vectors, raw sensors and
  tap, step and stability reports. New reports extend the decoder table.

Buffers have fixed compile-time capacities. Capacity failures return errors or
increment drop counters; the library allocates nothing after startup.

## Documentation

- [Documentation index](docs/README.md): document routing and protocol manuals.
- [Orientation](docs/orientation.md): modules, builds and local invariants.
- [Contributing](CONTRIBUTING.md): change and verification process.
- [Technical specification](docs/technical-specification.md): requirements and
  pass criteria.
- [Architecture](docs/architecture.md): engines, service model and storage.
- [Usage guide](docs/usage-guide.md): sessions, sensors, events and diagnostics.
- [Standing deviations](docs/deviations.md): approved suppressions and review
  dates.
- [Releases](https://github.com/jamesfxley/pico-sh2/releases): changes and
  migration notes.

## Releases

Versioning follows
[EN-06.1](infra/house/standard/engineering/06-release-and-versioning.md).
Each [release](https://github.com/jamesfxley/pico-sh2/releases) records changes
and migration instructions under
[EN-06.3](infra/house/standard/engineering/06-release-and-versioning.md).

## AI-assisted development

This project uses AI assistance. Humans direct design, review and acceptance;
AI agents write and refactor code against the engineering standard. Changes
pass the repository's build, test, format, lint and deviation gates before
merge.

The terms and warranty disclaimer in [LICENSE](LICENSE) apply unchanged.

## Development configuration

`house.toml` selects `infra/family.toml` for family gates, build trees, CI
preparation and synced files. Product dependency pins live in `CMakeLists.txt`.
`AGENTS.md` routes agent instructions. House renders the tool configuration,
workflow and dev container; `python3 -m infra.pico_tooling sync` writes the
bootstrap. Test-specific analyser overlays are registered in
[docs/deviations.md](docs/deviations.md). Infrastructure freshness and
unavailable
release-tag evidence follow the
[family build guide](infra/docs/build.md).
