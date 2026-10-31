# pico-sh2 documentation

| Document | What it is | Read it when |
|---|---|---|
| [traceability-exceptions.txt](traceability-exceptions.txt) | Requirements no source file can cite (EN-02.7). | Reviewing uncitable requirements. |
| [orientation.md](orientation.md) | How this repository is built and gated, how it is laid out, and the repository's invariants and traps | Working in it for the first time, or before an edit |
| [technical-specification.md](technical-specification.md) | **Normative.** Requirement IDs (`AR`/`NR`/`SR`/`DR`/`BR`/`TR`/`OR`), their pass criteria, and the traceability matrix | Deciding whether a behaviour is required or incidental, or changing one |
| [architecture.md](architecture.md) | Descriptive. How the implementation is put together — the engines, the service model, allocation, the counter set | Working out where a change belongs |
| [usage-guide.md](usage-guide.md) | Practical use of the public surface: starting a session, configuring sensors, receiving events, reading diagnostics | Consuming the library |
| [future-work.md](future-work.md) | What is deliberately not built, and why | Asking why something is absent, or proposing it |
| [deviations.md](deviations.md) | Standing (scope-wide) deviations from the coding standard, each with a rationale, a risk assessment and a review date | Adding or reviewing a suppression |
| [deviation-budget.txt](deviation-budget.txt) | Per-language suppression ceilings. These only ever ratchet down | Same |
| [requirement-prefixes.txt](requirement-prefixes.txt) | The requirement prefix convention checked by review | Adding a requirement under a new prefix |
| [GitHub Releases](https://github.com/jamesfxley/pico-sh2/releases) | What changed in each release, and what to change in your code when an API broke | Upgrading across a release |

The specification is the only normative document here. `architecture.md`
describes what is, and work deliberately not done is an entry in
[future-work.md](future-work.md) (`EN-03.7`); the issue tracker holds defects.

A behaviour claimed in `README.md` or `docs/orientation.md` but absent from the
specification is not a commitment, and a requirement is not satisfied until the
matrix names what establishes it.

## The protocol manuals

Every protocol claim in this repository rests on CEVA's own documentation,
or, where that is silent, is marked `UNCONFIRMED` or cited to CEVA's
reference host driver as described below. The manuals are not
redistributed here — they are CEVA's, under CEVA's terms — so they are cited by
document number and obtained from the vendor.

| Document | What it covers |
|---|---|
| CEVA 1000-3625, *SH-2 Reference Manual*, r1.9 | The SH-2 sensor protocol: reports, commands, FRS records |
| CEVA 1000-3600, *SH-2 SHTP Reference Manual*, r1.6 | SH-2's use of SHTP: channel assignments and report routing |
| CEVA 1000-3535, *Sensor Hub Transport Protocol*, r1.10 | SHTP itself: headers, fragmentation, channel advertisement |
| CEVA 1000-3927, *BNO08X Datasheet*, v1.17 | BNO08X limits on SHTP operation |

Citations in the specification and in the source name the document and the
section, so a reader with the manual can follow one without needing this
repository to carry it. `pdftotext -layout` extracts them faithfully, tables
included.

Anything the manuals do not cover is marked `UNCONFIRMED` at the point it is
stated, rather than inferred from an implementation. The one exception is CEVA's
own reference host driver, [`ceva-dsp/sh2`](https://github.com/ceva-dsp/sh2):
where it settles what the manuals leave open, the specification cites it by
commit and names it as vendor code at each such citation.

## The API reference

The API reference is generated from the headers rather than written by hand, so
it cannot drift from the declarations it documents:

```sh
doxygen docs/Doxyfile      # -> docs/api/html/index.html (gitignored)
```

CI builds it on every push to `main` and every pull request, fails on an
undocumented public entity.

## Conventions

- Requirement IDs are stable and never renumbered. Code and tests cite the ID,
  never a section number: IDs survive a reorganisation of this document and
  section numbers do not. Review checks that specification IDs are cited by code or tests;
  `requirement-prefixes.txt` records their prefix convention.
- The specification states what is true now — no dates, no version history, no
  measured values. It may state a normative gate ("within a CI-enforced
  budget") but not a measurement; a measured figure is a run record and lives
  beside the rig that produced it (`EN-03.2`).
- A behaviour change lands as one change: the specification row, a host test,
  and the affected documents together.
