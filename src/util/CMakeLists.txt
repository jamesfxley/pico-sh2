add_library(pico_sh2_util INTERFACE)
add_library(pico_sh2::util ALIAS pico_sh2_util)

target_sources(pico_sh2_util INTERFACE
    FILE_SET HEADERS
    BASE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
    FILES
        include/pico_sh2/util/result.hpp
)

target_include_directories(pico_sh2_util INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

target_compile_features(pico_sh2_util INTERFACE cxx_std_20)

# PUBLIC: Result is an alias for pico_util::Result and the byte accessors are
# pico_util's, so its headers reach a consumer through this library's own.
# Every other module links pico_sh2::util PUBLIC, so this is the one place the
# base layer has to be named.
target_link_libraries(pico_sh2_util INTERFACE pico_util::core)
