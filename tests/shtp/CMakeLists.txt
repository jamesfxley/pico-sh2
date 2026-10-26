house_cxx_add_module_test(pico_sh2 shtp
    SOURCES
        channels_test.cpp
        header_test.cpp
        advertisement_test.cpp
        engine_test.cpp
)

# Drives the framing engine over arbitrary transfer streams: reassembly,
# sequence tracking, and the drop paths that must never truncate a cargo.
house_cxx_add_fuzzer(pico_sh2_shtp_fuzz
    PACKAGE pico_sh2
    SOURCE shtp_fuzz.cpp
    LIBRARIES pico_sh2::shtp
    CORPUS corpus/pico_sh2_shtp_fuzz
    RUNS 100000
)
