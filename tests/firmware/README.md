# Cross-compiled probe

`cross_probe.cpp` holds a `Session` in static storage so the profile's
reassembly buffer and TX queue contribute to the footprint. Build it with
`python3 dev.py build --tree firmware`. The `footprint` gate reports the
current figure against `pico.toml`; measurements belong in run records with
their conditions (EN-06.7).

For the large profile, use
`HOUSE_MATRIX_PROFILE=large python3 dev.py build --tree firmware`.

The allocating fixture is a negative control for the objects symbols check.
