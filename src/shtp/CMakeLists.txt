add_library(pico_sh2_shtp STATIC
    engine.cpp
    advertisement.cpp
)
add_library(pico_sh2::shtp ALIAS pico_sh2_shtp)

target_sources(pico_sh2_shtp PUBLIC
    FILE_SET HEADERS
    BASE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
    FILES
        include/pico_sh2/shtp/channels.hpp
        include/pico_sh2/shtp/header.hpp
        include/pico_sh2/shtp/advertisement.hpp
        include/pico_sh2/shtp/engine.hpp
)

target_include_directories(pico_sh2_shtp PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

target_compile_features(pico_sh2_shtp PUBLIC cxx_std_20)
target_link_libraries(pico_sh2_shtp PUBLIC pico_sh2::util
    PRIVATE pico_sh2_strict)
