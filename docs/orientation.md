# pico-sh2

C++20 implementation of the CEVA/Hillcrest **SH-2** sensor protocol and its
**SHTP** transport (BNO08x family). The library code is platform-independent —
no Pico SDK, no OS — and reaches hardware only through `IHalTransport`. The
repository belongs to the pico-* family and uses its shared tooling (the
`infra/` submodule) and CI. Consumed by the sibling driver, pico-bno085.

Orientation for anyone working in this repository: how to build and gate it,
how it is laid out, and the repository's invariants and traps.

Engineering standards, cited by rule ID (`EN-`/`CXX-`), resolve in the pinned
standard, `infra/house/standard/`.

## Commands

```sh
python3 dev.py check            # every gate that applies; the one to run
python3 dev.py check --list     # name what would run, and why anything would not
python3 dev.py check --group lint   # one group
```

`dev.py check` is the entry point in every repository of the family, and its
report is the answer to "have the gates run?" (`EN-01.8`).

Individual gates:

```sh
python3 dev.py build [--tree NAME]   # clean build + GoogleTest suite (build/, build-NAME/)
python3 dev.py format [--check]      # clang-format at the pinned major version
python3 dev.py lint                  # clang-tidy and ruff; BLOCKING
python3 dev.py gate <name>           # one gate, e.g. deviations (--ratchet), consumer
```

`python3 dev.py check --list` prints every gate with its group and whether it
applies. The footprint budget is declared in `pico.toml`, its one authoritative
home. The util dependency is fetched at the revision declared in
`CMakeLists.txt`.

- Profiles: `HOUSE_MATRIX_PROFILE=large` runs every tree at the wider sizing
  profile, which is what the `large` CI legs do (`[ci.axes]` in `house.toml`);
  `-DPICO_SH2_PROFILE_HEADER=<absolute path>` names a consumer's own profile
  header, which defines the type and `PICO_SH2_ACTIVE_PROFILE`. The declared
  consumer cases `default`, `large`, `custom`, `refused` and `conflict` in
  `house.toml` install the package under each profile with the headers in
  `tests/consumer/profiles/`.
- `HOUSE_MATRIX_SANITIZE=ON` adds the `sanitize` tree: the host suite under
  ASan+UBSan (`PICO_SH2_SANITIZE`, host only).
- The `fuzz` tree builds the libFuzzer targets with the clang the platform pins:
  `pico_sh2_shtp_fuzz`, `pico_sh2_report_fuzz`, `pico_sh2_session_fuzz`. Each is
  run by CTest over its tracked seed corpus (`tests/<module>/corpus/<name>/`)
  for 100000 inputs. Longer exploratory runs are not part of the checks.
- Tests live in `tests/<module>/`, mirroring `src/` (`CXX-13.2`), so `src/` is
  exactly the shippable surface and no install, lint or coverage rule over it
  needs a test exclusion. Which `.clang-tidy` files sit under `tests/`, and
  what each adds, is under [Repo specifics](#repo-specifics).
- There is no networking code here to configure, and the library links nothing
  from the Pico SDK. The firmware job cross-compiles the probe under
  `tests/firmware/`, links it and holds it to the footprint gate.

## Layout

- `src/util/` — the `Sh2Error` taxonomy and `Result<T>`, which is an alias for
  `pico_util::Result<T, Sh2Error>`. The container, the non-owning `Callback`,
  the byte accessors and the outbound ring are pico_util's; what is this
  library's is the vocabulary.
- `src/shtp/` — SHTP header, channel map, advertisement, framing engine.
- `src/reports/` — sensor/report IDs, typed `SensorEvent`, the constexpr decoder
  table, Q-point handling and overrides.
- `src/sh2/` — the `IHalTransport` seam, the sizing profile, and `Session`
  (startup state machine, feature control, commands, FRS, timestamps).
- `src/testing/` — `FakeHub`, a bus-level hub emulator driving the whole stack.
- `docs/` — the normative specification (requirement IDs
  `AR/NR/SR/DR/BR/TR/OR`), architecture notes and the usage guide;
  [docs/README.md](README.md) routes them and lists the CEVA manuals, which are
  cited rather than carried here.

## Conventions

### House policy

The rules shared by every repository that takes its gates from pico-tooling are
in the pinned guides, starting with the
[engineering standard](../infra/house/guides/engineering-standard/SKILL.md).

### Repo specifics

- **`-Werror` sits on the target here**, unlike the SDK-linking repos in the
  family: this project owns every source that compiles into its targets, and
  even the cross build, where `pico_sh2_cross_probe` links against nosys alone.
  `house_cxx_add_strict_target(... LANGUAGES CXX)` for the same reason — CXX
  is the only language to diagnose. `project()` itself declares `C CXX ASM`,
  which is load-bearing: `pico_family_init_sdk` brings the SDK's C and assembly
  sources into a firmware consumer's build.
- **`house_cxx_enable_sanitizers` must precede `house_cxx_enable_gtest`.**
  `add_compile_options` reaches only targets created after it, so the reverse
  order leaves the fetched GoogleTest uninstrumented while this project's code
  carries ASan. House refuses that order outright.
- **A test directory's `.clang-tidy` adds only what `tests/.clang-tidy` does
  not disable.** `tests/.clang-tidy` is a library-owned registered deviation; the four directories that carry their own file (the three
  module directories and `tests/docs`, the example harness) add what each needs
  on top of it. Nothing under `tests/` inherits
  `src/.clang-tidy` — they are siblings, and the magic-numbers disable reaches
  the test tree from the overlay, which is why it has its own register row.
- **The vendor manuals are the source of protocol truth**;
  [docs/README.md](README.md) lists them and states the one exception, CEVA's
  reference host driver where the manuals are silent. `pdftotext -layout`
  extracts them faithfully, tables included. Never confirm behavior against
  any other implementation.
- **`PICO_SH2_ACTIVE_PROFILE` is a `PUBLIC` compile definition on the
  `pico_sh2_sh2` target**, not a directory-scoped one. It must propagate through
  the link graph to every consumer that includes `profile.hpp`; a
  directory-scoped definition reaches only this repo's targets and produces a
  silent ABI mismatch (stack smashing) in consumers built with a non-default
  profile. The consumer case `large` asserts the profile reaches a
  consumer; run `python3 dev.py gate consumer` after changing propagation.
- **`Session` enqueues TX; the frame is flushed on the *next* `service()`.**
  Tests must call `service()` before inspecting `hub.writes()`.
- **`Session::drain_transfers()` is capped, and a read error in `READY`
  restarts the handshake.** The loop drains at most
  `MAX_TRANSFERS_PER_SERVICE` transfers (AR-3), so it cannot wedge on a
  stuck-low HINTN; a read error while ready counts `transport_restarts` and
  restarts the handshake rather than faulting at once (SR-5). The restart waits
  for an advertisement only a hub that really reset sends, so a spurious error
  ends in a `TIMEOUT` fault in `WAIT_ADVERTISEMENT`. What a HAL should return
  for an empty bus instead is in the usage guide's
  [Supplying a transport](usage-guide.md#supplying-a-transport).
- **Every sink runs inside `service()` and may re-enter the session.** A
  `begin()` from a sink advances `generation_`, and every step of `service()`
  that follows a sink call checks it and stops, so traffic from the hub
  incarnation the call restarted is never delivered. `service()` from a sink
  returns at once: a nested drain would overwrite `rx_staging_` while the outer
  pass is still delivering from it. A new sink call site needs the same check
  after it. A fault ends the pass the same way: each step that follows one that
  can fault checks for `FAULTED` and stops, so the fault and its phase are what
  the consumer reads (SR-1). A new step after such a call needs that check too.
- Q-point overrides are part-intrinsic and **persist across a hub reset**. The
  report-length table does not: `restart_handshake()` reseeds it from the
  compiled defaults, because the lengths a hub advertises belong to that
  advertisement (SR-5, DR-2).

- **`Sh2Error` is this library's taxonomy and must not become a consumer's.**
  Its own header says codes are appended and never renumbered — which is safe
  here and unsafe in a consumer that re-exports it, because each append then
  silently changes that consumer's contract. A library owns the taxonomy of the
  failures it can produce; a driver built on this one maps `Sh2Error` into its
`StartupPhase` is defined and documented in the public `startup_phase.hpp`.
