# EN-03.5: the C++ examples in the README and the usage guide, compiled from
# those documents. Each harness here supplies only what its example leaves out;
# the example itself is read out of the document at configure time, so an API
# change that breaks one fails this build (EN-03.5's requirement on
# compiled document examples).
house_cxx_add_doc_examples(pico_sh2_doc_examples PACKAGE pico_sh2
    DOCUMENTS ${PROJECT_SOURCE_DIR}/README.md
              ${PROJECT_SOURCE_DIR}/docs/usage-guide.md
    HARNESS_DIR ${CMAKE_CURRENT_SOURCE_DIR}
    LIBRARIES pico_sh2::core)
