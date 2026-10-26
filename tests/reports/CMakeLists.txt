house_cxx_add_module_test(pico_sh2 reports
    SOURCES
        sensor_id_test.cpp
        q_point_test.cpp
        report_layout_test.cpp
        report_length_table_test.cpp
        decoders_test.cpp
        metadata_q_test.cpp
        timestamp_test.cpp
)

# Walks arbitrary cargoes through the report splitter and decoders.
house_cxx_add_fuzzer(pico_sh2_report_fuzz
    PACKAGE pico_sh2
    SOURCE report_fuzz.cpp
    LIBRARIES pico_sh2::reports
    CORPUS corpus/pico_sh2_report_fuzz
    RUNS 100000
)
