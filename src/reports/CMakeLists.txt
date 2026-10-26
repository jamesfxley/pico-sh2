add_library(pico_sh2_reports STATIC
    decoders.cpp
)
add_library(pico_sh2::reports ALIAS pico_sh2_reports)

target_sources(pico_sh2_reports PUBLIC
    FILE_SET HEADERS
    BASE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
    FILES
        include/pico_sh2/reports/sensor_id.hpp
        include/pico_sh2/reports/q_point.hpp
        include/pico_sh2/reports/q_override.hpp
        include/pico_sh2/reports/metadata_q.hpp
        include/pico_sh2/reports/report_layout.hpp
        include/pico_sh2/reports/report_length_table.hpp
        include/pico_sh2/reports/sensor_event.hpp
        include/pico_sh2/reports/decoders.hpp
        include/pico_sh2/reports/timestamp.hpp
)

target_include_directories(pico_sh2_reports PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

target_compile_features(pico_sh2_reports PUBLIC cxx_std_20)
target_link_libraries(pico_sh2_reports PUBLIC pico_sh2::util
    PRIVATE pico_sh2_strict)
