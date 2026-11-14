# Tests mirror src/ (docs/orientation.md, Commands).
# src/ is added first so linked module targets already exist.

add_subdirectory(core)
add_subdirectory(shtp)
add_subdirectory(reports)
add_subdirectory(sh2)

# The host-side hub emulator is not built for firmware images, so neither are
# its tests. Mirrors the guard on src/testing/.
if(NOT PICO_SH2_BUILD_FIRMWARE)
    add_subdirectory(testing)
    # Document examples need no SDK and compile in the host tree.
    add_subdirectory(docs)
endif()
