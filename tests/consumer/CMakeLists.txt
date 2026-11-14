cmake_minimum_required(VERSION 3.26)
project(pico_sh2_consumer_smoke LANGUAGES CXX)

# Checks installed exports that in-tree aliases can hide.
# This consumer must be skipped by lint (root .lintignore).
find_package(pico_sh2 REQUIRED)

add_executable(consumer_smoke consumer_smoke.cxx)

# Every public module is named, including testing, so a missing installed target
# fails here even if the aggregate no longer references it.
target_link_libraries(consumer_smoke PRIVATE
    pico_sh2::core
    pico_sh2::util
    pico_sh2::shtp
    pico_sh2::reports
    pico_sh2::sh2
    pico_sh2::testing
)

target_compile_features(consumer_smoke PRIVATE cxx_std_20)
