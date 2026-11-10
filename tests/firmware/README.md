# Cross-compiled probe

`cross_probe.cpp` holds a `Session` in static storage so the profile's
reassembly buffer and TX queue contribute to the footprint. Build it with
`python3 dev.py build --tree firmware`. The `footprint` gate reports the
static RAM and flash against `pico.toml`. Measurements and their conditions belong
in run records ([EN-06.7](../../infra/house/standard/engineering/06-release-and-versioning.md)).

For the large profile, use
`HOUSE_MATRIX_PROFILE=large python3 dev.py build --tree firmware`.

The allocating fixture checks that the `symbols` gate detects heap allocation.
