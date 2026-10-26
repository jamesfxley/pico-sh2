set(PICO_SH2_PUBLIC_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)

add_subdirectory(util)
add_subdirectory(shtp)
add_subdirectory(reports)
add_subdirectory(sh2)

# The host-side hub emulator is pointless in firmware images; build it for host
# consumers and for the test suite only.
if(NOT PICO_SH2_BUILD_FIRMWARE)
    add_subdirectory(testing)
endif()

add_library(pico_sh2_core INTERFACE)
add_library(pico_sh2::core ALIAS pico_sh2_core)

target_sources(pico_sh2_core INTERFACE
    FILE_SET HEADERS
    BASE_DIRS ${PICO_SH2_PUBLIC_INCLUDE_DIR}
    FILES
        ${PICO_SH2_PUBLIC_INCLUDE_DIR}/pico_sh2/pico_sh2.hpp
)

target_include_directories(pico_sh2_core INTERFACE
    $<BUILD_INTERFACE:${PICO_SH2_PUBLIC_INCLUDE_DIR}>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

target_compile_features(pico_sh2_core INTERFACE cxx_std_20)

target_link_libraries(pico_sh2_core INTERFACE
    pico_sh2::util
    pico_sh2::shtp
    pico_sh2::reports
    pico_sh2::sh2
)
