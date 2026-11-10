house_cxx_add_module_test(pico_sh2 core
    SOURCES
        pico_sh2_test.cpp
)

# Version definitions stay on this test; pico_sh2::core is an INTERFACE target
# whose definitions would reach installed consumers.
house_cxx_version_definitions(pico_sh2_core_tests pico_sh2 PRIVATE)
