# README and usage-guide examples are extracted at configure time and compiled.
# Harnesses supply only the surrounding declarations.
house_cxx_add_doc_examples(pico_sh2_doc_examples PACKAGE pico_sh2
    DOCUMENTS ${PROJECT_SOURCE_DIR}/README.md
              ${PROJECT_SOURCE_DIR}/docs/usage-guide.md
    HARNESS_DIR ${CMAKE_CURRENT_SOURCE_DIR}
    LIBRARIES pico_sh2::core)
