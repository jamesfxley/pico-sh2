add_library(pico_sh2_sh2 STATIC
    session.cpp
)
add_library(pico_sh2::sh2 ALIAS pico_sh2_sh2)

target_sources(pico_sh2_sh2 PUBLIC
    FILE_SET HEADERS
    BASE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
    FILES
        include/pico_sh2/sh2/channel_binding.hpp
        include/pico_sh2/sh2/command.hpp
        include/pico_sh2/sh2/feature.hpp
        include/pico_sh2/sh2/flush.hpp
        include/pico_sh2/sh2/frs.hpp
        include/pico_sh2/sh2/hal_transport.hpp
        include/pico_sh2/sh2/profile.hpp
        include/pico_sh2/sh2/startup_phase.hpp
        include/pico_sh2/sh2/session.hpp
        include/pico_sh2/sh2/session_timing.hpp
        include/pico_sh2/sh2/detail/deadline.hpp
        include/pico_sh2/sh2/detail/startup.hpp
        include/pico_sh2/sh2/detail/command_engine.hpp
        include/pico_sh2/sh2/detail/frs_engine.hpp
        include/pico_sh2/sh2/detail/input_router.hpp
        include/pico_sh2/sh2/detail/tx_path.hpp
        include/pico_sh2/sh2/session_counters.def
)

target_include_directories(pico_sh2_sh2 PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

# Library and consumers must agree on the profile for a consistent Session layout.
# A custom header defines its type and PICO_SH2_ACTIVE_PROFILE (AR-5).
# The installed package retains the configured path.
set(PICO_SH2_PROFILE_HEADER "" CACHE FILEPATH
    "Header defining a consumer's sizing profile (AR-5); empty for none")
if(PICO_SH2_PROFILE_HEADER)
    if(PICO_SH2_PROFILE AND NOT PICO_SH2_PROFILE STREQUAL "default")
        message(FATAL_ERROR
            "PICO_SH2_PROFILE_HEADER and PICO_SH2_PROFILE=${PICO_SH2_PROFILE} "
            "both select a profile; set one")
    endif()
    if(NOT IS_ABSOLUTE "${PICO_SH2_PROFILE_HEADER}"
       OR NOT EXISTS "${PICO_SH2_PROFILE_HEADER}")
        message(FATAL_ERROR
            "PICO_SH2_PROFILE_HEADER must be an absolute path to an existing "
            "header: '${PICO_SH2_PROFILE_HEADER}'")
    endif()
    target_compile_definitions(pico_sh2_sh2 PUBLIC
        "PICO_SH2_PROFILE_HEADER=\"${PICO_SH2_PROFILE_HEADER}\"")
elseif(PICO_SH2_PROFILE STREQUAL "large")
    target_compile_definitions(pico_sh2_sh2 PUBLIC
        PICO_SH2_ACTIVE_PROFILE=::pico_sh2::LargeProfile
        PICO_SH2_PROFILE_LARGE)
endif()

# Private includes expose umbrella version constants without linking back to core.
target_include_directories(pico_sh2_sh2 PRIVATE
    $<BUILD_INTERFACE:${PICO_SH2_PUBLIC_INCLUDE_DIR}>
)

# Version definitions are private to the archive's compile-time check.
house_cxx_version_definitions(pico_sh2_sh2 pico_sh2 PRIVATE)

target_compile_features(pico_sh2_sh2 PUBLIC cxx_std_20)
target_link_libraries(pico_sh2_sh2
    PUBLIC pico_sh2::util pico_sh2::shtp pico_sh2::reports
    PRIVATE pico_sh2_strict)
