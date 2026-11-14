# pico-sh2

This guide maps the modules, build configurations and local invariants for
maintainers. [The README](../README.md) introduces the library;
[docs/README.md](README.md) routes the detailed documents.
Engineering rule IDs resolve in the pinned `infra/house/standard/`.

## Commands

```sh
python3 dev.py check            # every gate that applies; the one to run
python3 dev.py check --list     # name what would run, and why anything would not
python3 dev.py check --group lint   # one group
```

The check report records which gates ran (EN-01.8).

```sh
python3 dev.py build [--tree NAME]   # clean build + GoogleTest suite (build/, build-NAME/)
python3 dev.py format [--check]      # clang-format at the pinned major version
python3 dev.py lint                  # clang-tidy and Ruff; findings fail
python3 dev.py gate <name>           # one gate, e.g. deviations (--ratchet), consumer
```

`python3 dev.py check --list` lists applicable gates and groups. `pico.toml`
owns footprint budgets; `CMakeLists.txt` pins pico-util.

- `HOUSE_MATRIX_PROFILE=large` selects the larger profile across build trees,
  as in CI. `-DPICO_SH2_PROFILE=large` selects it for a CMake build.
  `-DPICO_SH2_PROFILE_HEADER=<absolute path>` names a custom header defining a
  profile type and `PICO_SH2_ACTIVE_PROFILE`. The capacity ranges are specified
  in [the specification](technical-specification.md).
  Consumer cases `default`, `large` and `custom` install and run each profile.
  `refused` checks an invalid profile's build failure; `conflict` checks a
  conflicting selection's configure failure. Their headers are in
  `tests/consumer/profiles/`, and `house.toml` declares the cases.
- `HOUSE_MATRIX_SANITIZE=ON` enables the host-only ASan+UBSan `sanitize` tree
  through `PICO_SH2_SANITIZE`.
- The `fuzz` tree uses the pinned Clang to build `pico_sh2_shtp_fuzz`,
  `pico_sh2_report_fuzz` and `pico_sh2_session_fuzz`. CTest runs each for 100000
  inputs from `tests/<module>/corpus/<name>/`. Longer runs are exploratory.
- Tests mirror `src/` under `tests/<module>/`, keeping installable code separate
  from tests (CXX-13.2). The analyser overlays are described below.
- Firmware cross-compiles and links the probe in `tests/firmware/` for footprint
  checks. The protocol library links no SDK or networking code.

## Layout

- `src/util/`: `Sh2Error` and `Result<T>`, an alias for
  `pico_util::Result<T, Sh2Error>`. pico-util supplies the containers,
  `Callback`, byte accessors and ring.
- `src/shtp/`: headers, channels, advertisement and framing.
- `src/reports/`: sensor IDs, events, decoders and Q-point handling.
- `src/sh2/`: transport interface, profile and session engines.
- `src/testing/`: heap-using host test support and `FakeHub`.
- `docs/`: specification, architecture and usage, routed by
  [docs/README.md](README.md).

The public aggregate target is `pico_sh2::core`. Public module targets are
`pico_sh2::util`, `pico_sh2::shtp`, `pico_sh2::reports` and `pico_sh2::sh2`;
`pico_sh2::testing` is available only on hosts. The `pico_sh2::strict` target is
installed because the library targets name it in their link interface; it is
not a supported consumer API.

## Conventions

### House policy

Shared rules start at the pinned
[engineering guide](../infra/house/guides/engineering-standard/SKILL.md).

### Repo specifics

- Host builds apply `-Werror` to targets. Firmware images include SDK sources,
  so `VENDOR_SOURCES` applies it per owned source. Strict targets select only
  C++. `project()` declares `C CXX ASM` for sources brought in by
  `pico_family_init_sdk`.
- `house_cxx_enable_sanitizers` must precede `house_cxx_enable_gtest`.
  Directory compile options reach only targets created later; reversing the
  calls leaves GoogleTest uninstrumented. House fails that configuration.
- `tests/.clang-tidy` owns the registered test deviations. Three module
  directories and `tests/docs` add their own checks. Tests inherit no settings
  from sibling `src/.clang-tidy`; their magic-number deviation has its own row
  in [deviations.md](deviations.md).
- Protocol sources and their limits are documented in
  [docs/README.md](README.md#the-protocol-manuals). Never confirm behaviour
  against another implementation. The cited CEVA vendor driver can resolve
  omissions in the manuals.
- The profile is a `PUBLIC` definition on `pico_sh2_sh2`. It must reach every
  consumer including `profile.hpp`; a directory definition leaves external
  consumers with the wrong layout. The `large` consumer case checks
  propagation. Run `python3 dev.py gate consumer` after changing it. The
  constructor's binding check is described in
  [the architecture](architecture.md#allocation).
- Requests enqueue TX for the next `service()` pass. Tests must call
  `service()` before inspecting `hub.writes()`.
- The drain loop is bounded by `MAX_TRANSFERS_PER_SERVICE` (AR-3).
  READY-state read errors count `transport_restarts` and restart the handshake
  (SR-5). What a HAL should return for an idle bus is explained in
  [the usage guide](usage-guide.md#supplying-a-transport).
- Sinks are synchronous and may re-enter the session. `begin()` advances
  `generation_`; each sink call site checks it afterwards and stops delivery
  from the interrupted run. A new sink call site needs that check.
  Re-entrant `service()` returns immediately to protect `rx_staging_`.
  Steps after a possible fault also check `FAULTED`, preserving the error and
  phase the consumer reads (SR-1). A new such step needs the same check.
- Reset handling of Q overrides and report lengths is described in
  [the architecture](architecture.md#inside-session) (SR-5, DR-2).

- `Sh2Error` is this library's taxonomy and must not become a consumer's.
  Its own header says codes are appended and never renumbered — which is safe
  here and unsafe in a consumer that re-exports it, because each append then
  silently changes that consumer's contract. A library owns the taxonomy of the
  failures it can produce; a driver built on this one maps `Sh2Error` into its
  own error taxonomy rather than re-exporting it.
