# pico-sh2 documentation

| Document | What it is | Read it when |
|---|---|---|
| [traceability-exceptions.txt](traceability-exceptions.txt) | Requirements without source citations | Reviewing uncitable requirements |
| [orientation.md](orientation.md) | Build commands, modules and local invariants | Starting work or preparing an edit |
| [technical-specification.md](technical-specification.md) | Normative requirements, pass criteria and traceability | Checking or changing a guarantee |
| [architecture.md](architecture.md) | Implementation and design reasons | Finding where a change belongs |
| [usage-guide.md](usage-guide.md) | Sessions, sensors, events and diagnostics | Using the public API |
| [future-work.md](future-work.md) | Deferred work and its reasons | Checking or proposing an omission |
| [deviations.md](deviations.md) | Standing deviations and their review dates | Reviewing a suppression |
| [deviation-budget.txt](deviation-budget.txt) | Per-language suppression ceilings | Checking suppression counts |
| [requirement-prefixes.txt](requirement-prefixes.txt) | Requirement prefix vocabulary | Adding a new prefix |
| [GitHub Releases](https://github.com/jamesfxley/pico-sh2/releases) | Release changes and migration instructions | Upgrading |

The specification owns commitments and maps them to acceptance evidence.
Architecture describes the implementation. A behaviour described only in the
README or orientation guide is not a commitment. Deferred work follows
[EN-03.7](../infra/house/standard/engineering/03-documentation.md);
the issue tracker holds defects.

## The protocol manuals

Protocol claims use CEVA documentation. The manuals remain private and are cited
by document number and section. Obtain them from the vendor.

| Document | What it covers |
|---|---|
| CEVA 1000-3625, *SH-2 Reference Manual*, r1.9 | Reports, commands and Flash Record System (FRS) records |
| CEVA 1000-3600, *SH-2 SHTP Reference Manual*, r1.6 | Channel assignments and report routing |
| CEVA 1000-3535, *Sensor Hub Transport Protocol*, r1.10 | Headers, fragmentation and advertisement |
| CEVA 1000-3927, *BNO08X Datasheet*, v1.17 | BNO08X SHTP limits |

Use `pdftotext -layout` to extract text with table layout preserved.
Claims the manuals do not settle are marked `UNCONFIRMED` where stated.
The repository also cites CEVA's reference host driver,
[`ceva-dsp/sh2`](https://github.com/ceva-dsp/sh2), by commit where it resolves
an
omission. Each such citation identifies it as vendor code.

## The API reference

Doxygen generates the reference from public declarations:

```sh
doxygen docs/Doxyfile      # -> docs/api/html/index.html (gitignored)
```

CI builds it on pushes to `main` and on pull requests. Undocumented public
entities produce warnings and fail the gate.

## Conventions

Requirement citations and stability follow
[the traceability
rules](../infra/house/standard/engineering/02-spec-and-traceability.md).
The specification's requirements may state normative gates; measurements belong
in run records under
[the documentation
rules](../infra/house/standard/engineering/03-documentation.md).
A behaviour change updates its requirement, test and affected documents
together.
