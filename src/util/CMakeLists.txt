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

# Public headers expose pico_util::Result; other modules inherit this dependency.
target_link_libraries(pico_sh2_util INTERFACE pico_util::core)
