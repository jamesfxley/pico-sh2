house_cxx_add_module_test(pico_sh2 sh2
    SOURCES
        channel_binding_test.cpp
        control_reports_test.cpp
        profile_test.cpp
        session_control_test.cpp
        session_counters_test.cpp
        session_frs_test.cpp
        session_heap_test.cpp
        session_input_test.cpp
        session_reset_test.cpp
        session_startup_test.cpp
        tx_path_test.cpp
    LIBRARIES
        pico_sh2::testing
)

# Drives a whole session over a fuzz-backed HAL, so the startup sequence and
# the transaction engines see arbitrary hub behaviour.
house_cxx_add_fuzzer(pico_sh2_session_fuzz
    PACKAGE pico_sh2
    SOURCE session_fuzz.cpp
    LIBRARIES pico_sh2::sh2 pico_sh2::testing
    CORPUS corpus/pico_sh2_session_fuzz
    RUNS 100000
)
