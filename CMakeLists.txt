cmake_minimum_required(VERSION 3.26)

include(cmake/pico_family_bootstrap.cmake)
pico_family_bootstrap(PICO_SH2)

project(pico_sh2 VERSION 0.1.0 LANGUAGES C CXX ASM)

# The firmware mode omits host-only testing; the library has no SDK or OS dependency.
house_cxx_common_settings(${PICO_SH2_BUILD_FIRMWARE})
pico_family_init_sdk(${PICO_SH2_BUILD_FIRMWARE})

# Host sources get target-wide -Werror; firmware device images include SDK
# sources, so firmware applies -Werror per owned source.
house_cxx_add_strict_target(pico_sh2_strict LANGUAGES CXX
    VENDOR_SOURCES ${PICO_SH2_BUILD_FIRMWARE})

# The named profile is validated here; src/sh2/CMakeLists.txt propagates its type.
pico_family_declare_profile(PICO_SH2 NAME PROFILE
    DEFAULT default VALUES default large
    DOC "pico-sh2 sizing profile")

# Instrumentation precedes GoogleTest: mixed container instrumentation can
# produce false ASan container-overflow reports.
house_cxx_enable_sanitizers(PICO_SH2)

# Options precede subdirectories; an undeclared option reads as an empty string.
house_cxx_declare_tests_option(PICO_SH2)

# Enabling fuzzers without tests fails configuration. Non-Clang hosts omit
# fuzz targets.
house_cxx_declare_fuzzers_option(PICO_SH2)

# Cross-compiled probe measured by the footprint gate (AR-4).
pico_family_declare_probe_option(PICO_SH2)

set(PICO_SH2_INSTALL_AVAILABLE ON)
if(PICO_SH2_BUILD_FIRMWARE)
    set(PICO_SH2_INSTALL_AVAILABLE OFF)
endif()
house_cxx_declare_install_option(PICO_SH2 AVAILABLE ${PICO_SH2_INSTALL_AVAILABLE})

# When this project's targets are exported, the dependency must export too.
# Set its install mode before fetching, so it sees the selected mode.
pico_family_propagate_install(PICO_SH2 PICO_UTIL)

# Public headers expose pico-util types, so installed packages depend on it too.
pico_family_dependency(pico_util
    REPOSITORY "https://github.com/jamesfxley/pico-util.git"
    REVISION 2afe3150a52429cb61c2330041cb4ec2e26c00ef)
pico_family_make_available(pico_util)

add_subdirectory(src)

if(PICO_SH2_ENABLE_TESTS)
    house_cxx_enable_gtest()
    add_subdirectory(tests)
endif()

if(PICO_SH2_ENABLE_FIRMWARE_PROBE)
    add_subdirectory(tests/firmware)
endif()

if(PROJECT_IS_TOP_LEVEL AND PICO_SH2_BUILD_FIRMWARE)
    add_subdirectory(tests/device)
endif()

if(PICO_SH2_ENABLE_INSTALL)
    set(PICO_SH2_EXPORT_TARGETS
        pico_sh2_strict
        pico_sh2_core
        pico_sh2_util
        pico_sh2_shtp
        pico_sh2_reports
        pico_sh2_sh2
    )

    # Firmware creates no fake-hub target to export.
    if(NOT PICO_SH2_BUILD_FIRMWARE)
        list(APPEND PICO_SH2_EXPORT_TARGETS pico_sh2_testing)
    endif()

    house_cxx_install_package(
        PACKAGE pico_sh2 VERSION ${PROJECT_VERSION}
        NAMESPACE pico_sh2
        TARGETS ${PICO_SH2_EXPORT_TARGETS}
        DEPENDS pico_util
    )
endif()
