add_library(pico_sh2_testing STATIC
    fake_hub.cpp
)
add_library(pico_sh2::testing ALIAS pico_sh2_testing)

target_sources(pico_sh2_testing PUBLIC
    FILE_SET HEADERS
    BASE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
    FILES
        include/pico_sh2/testing/fake_hub.hpp
        include/pico_sh2/testing/frame_builder.hpp
)

target_include_directories(pico_sh2_testing PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

target_compile_features(pico_sh2_testing PUBLIC cxx_std_20)
target_link_libraries(pico_sh2_testing
    PUBLIC pico_sh2::sh2
    PRIVATE pico_sh2_strict)
