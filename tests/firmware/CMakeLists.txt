# The footprint probe uses static storage (tests/firmware/README.md; AR-4).
# The helper links nosys and no SDK library, without exceptions or RTTI (AR-4).
pico_family_add_cross_probe(pico_sh2_cross_probe
    PACKAGE pico_sh2
    SOURCE cross_probe.cpp
    LIBRARIES pico_sh2::core
)

pico_family_add_cross_probe(pico_sh2_allocating_fixture
    PACKAGE pico_sh2
    SOURCE allocating_fixture.cpp
)
