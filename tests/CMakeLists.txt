# The test tree mirrors src/ (CXX-13.2), so src/ is exactly the shippable
# surface: an install, footprint, lint or coverage rule over it needs no test
# exclusion. One directory per module, added after src/ so every
# pico_sh2::<module> target the tests link already exists.

add_subdirectory(core)
add_subdirectory(shtp)
add_subdirectory(reports)
add_subdirectory(sh2)

# The host-side hub emulator is not built for firmware images, so neither are
# its tests. Mirrors the guard on src/testing/.
if(NOT PICO_SH2_BUILD_FIRMWARE)
    add_subdirectory(testing)
    # The documents' examples, compiled from the documents (EN-03.5). None
    # needs the SDK, so the host tree is the only one that builds them.
    add_subdirectory(docs)
endif()
