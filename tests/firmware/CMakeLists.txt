# AR-4's cross-compiled probe, and the only static-RAM figure this repository
# has: the probe holds the Session in static storage, so the sizing profile's
# reassembly buffer and TX queue land in .bss where a change to them shows up.
#
# Declared here rather than in the root listfile: a target lives in the
# directory holding its sources, which is what lets a reader find it from the
# path.
# Freestanding inside the SDK tree: the probe links no SDK library, so it gets
# no linker script and no runtime from one. AR-4 is exactly this link
# succeeding; the helper supplies nosys, no exceptions and no RTTI.
pico_family_add_cross_probe(pico_sh2_cross_probe
    PACKAGE pico_sh2
    SOURCE cross_probe.cpp
    LIBRARIES pico_sh2::core
)

pico_family_add_cross_probe(pico_sh2_allocating_fixture
    PACKAGE pico_sh2
    SOURCE allocating_fixture.cpp
)
