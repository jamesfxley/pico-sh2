cmake_minimum_required(VERSION 3.26)

include(cmake/pico_family_bootstrap.cmake)
pico_family_bootstrap(PICO_SH2)

project(pico_sh2 VERSION 0.1.0 LANGUAGES C CXX ASM)

# pico-sh2 is platform-independent: it depends on no Pico SDK and no OS. The
# firmware flag only matters because a firmware consumer (pico-bno085) pulls
# this project in via FetchContent; when it does, PICO_SH2_BUILD_FIRMWARE is ON
# and the host-only testing library is skipped. Standalone, this is a pure host
# library with a GoogleTest suite.
house_cxx_common_settings(${PICO_SH2_BUILD_FIRMWARE})
pico_family_init_sdk(${PICO_SH2_BUILD_FIRMWARE})

# This project owns every source that compiles into its targets — no SDK
# attaches its own through an INTERFACE library here, not even in the cross
# build, where pico_sh2_cross_probe links against nosys alone. So -Werror can
# sit on the target rather than being applied per source, with no VENDOR_SOURCES
# keyword, and CXX is the only language to diagnose. FREESTANDING follows the
# toolchain, which is the helper's default.
house_cxx_add_strict_target(pico_sh2_strict LANGUAGES CXX)

# Named sizing profile. "default" targets a single-IMU host; "large" widens the
# reassembly buffer, TX queue, and enabled-sensor count. profile.hpp binds every
# static size to the selected struct with static_asserts. The macro selecting it
# is applied on the sh2 module (src/sh2/CMakeLists.txt); here only the value is
# validated.
pico_family_declare_profile(PICO_SH2 NAME PROFILE
    DEFAULT default VALUES default large
    DOC "pico-sh2 sizing profile")

# Host-only ASan+UBSan build. Directory-scoped, so the library, the tests and
# the fetched GoogleTest are all instrumented: ASan's container-overflow
# detector needs both sides of a std::string or std::vector boundary built with
# the same instrumentation, and partial coverage is the documented cause of
# false positives there.
house_cxx_enable_sanitizers(PICO_SH2)

# Every knob this build has, declared before anything is added: a subdirectory
# reading an option that has not been declared yet reads an empty string.
house_cxx_declare_tests_option(PICO_SH2)

# libFuzzer targets (Clang only), off by default; CI turns them on for the
# fuzz job. A non-Clang host configure is a clean no-op; a configure without
# the test tree, where the fuzz targets live, is refused.
house_cxx_declare_fuzzers_option(PICO_SH2)

# AR-4's cross-compiled probe, and the image the footprint gate measures. Built
# only when cross-compiling, which is the only time the question is meaningful.
pico_family_declare_probe_option(PICO_SH2)

set(PICO_SH2_INSTALL_AVAILABLE ON)
if(PICO_SH2_BUILD_FIRMWARE)
    set(PICO_SH2_INSTALL_AVAILABLE OFF)
endif()
house_cxx_declare_install_option(PICO_SH2 AVAILABLE ${PICO_SH2_INSTALL_AVAILABLE})

# When this project's targets are exported, the fetched dependency they link
# must be in an export set too. Before the fetch below, or it is configured
# with its default first.
pico_family_propagate_install(PICO_SH2 PICO_UTIL)

# pico_util is the family's base layer, and this library's Result, Callback,
# byte accessors and outbound ring are its primitives rather than second copies
# of them. Its headers reach a consumer through this library's own, so it is a
# PUBLIC link on the modules that include it and a DEPENDENCIES entry in the
# installed package.
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

if(PICO_SH2_ENABLE_INSTALL)
    set(PICO_SH2_EXPORT_TARGETS
        pico_sh2_strict
        pico_sh2_core
        pico_sh2_util
        pico_sh2_shtp
        pico_sh2_reports
        pico_sh2_sh2
    )

    # The fake hub is host-only: it stands in for a transport an image already
    # has, so a firmware build does not create it. Naming a target that was
    # never created fails the configure, which is what a firmware consumer
    # installing this package as a dependency hits.
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
