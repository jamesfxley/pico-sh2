cmake_minimum_required(VERSION 3.26)
project(pico_sh2_consumer_smoke LANGUAGES CXX)

# Stands in for a downstream project consuming an installed pico-sh2. Its
# purpose is to prove the package is reachable by find_package and that every
# component behind the pico_sh2::core aggregate is in the export set -- the
# contract that fails only outside the source tree, which is why an in-tree test
# cannot cover it. A component dropped from the export set still links in-tree,
# because the in-tree alias resolves regardless.
#
# This translation unit compiles only against an installed package, so it has no
# entry in the merged compile database and the repo-wide clang-tidy sweep must
# skip it. The root .lintignore is what does that; the .cxx extension does not,
# since the lint gate enumerates .cxx alongside .cpp and .cc.
find_package(pico_sh2 REQUIRED)

add_executable(consumer_smoke consumer_smoke.cxx)

# pico_sh2::testing is linked as well as pico_sh2::core: the hub emulator is
# part of the shipped surface -- pico-bno085's own driver tests link it -- and
# an export set that drops it fails only here.
# Every exported target is named, not only the aggregate. The aggregate pulls
# most of them in transitively, so a module dropped from the export set would
# still resolve here and the smoke would report nothing -- which is the one
# contract this executable exists to prove.
target_link_libraries(consumer_smoke PRIVATE
    pico_sh2::core
    pico_sh2::util
    pico_sh2::shtp
    pico_sh2::reports
    pico_sh2::sh2
    pico_sh2::testing
)

target_compile_features(consumer_smoke PRIVATE cxx_std_20)
