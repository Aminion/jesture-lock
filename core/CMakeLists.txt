# Recognition core: the core.h contract plus one implementation of it. Builds as part of
# the top-level project (host and MCU) and standalone (host/crates/gesture-core/build.rs
# via the `cmake` crate). Same compile options either way: cmake/gesture_options.cmake.
# Callers link `gesture_core` and see core.h only.
cmake_minimum_required(VERSION 3.25)
project(gesture_core LANGUAGES CXX)

include(${CMAKE_CURRENT_LIST_DIR}/../cmake/gesture_options.cmake)

add_subdirectory(contract) # gesture_core_abi: core.h

# The implementation: a directory under core/ whose CMakeLists.txt defines `gesture_core`,
# a static library (built here, or IMPORTED GLOBAL from another toolchain) that links
# `gesture_core_abi`. Its unit tests, if any, go in <dir>/tests (top-level CMakeLists.txt);
# the conformance vectors (contract/tests) run against whichever one is chosen.
set(GESTURE_CORE_IMPL cpp CACHE STRING "core.h implementation: a directory under core/")
if(GESTURE_CORE_IMPL STREQUAL "contract"
   OR NOT EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/${GESTURE_CORE_IMPL}/CMakeLists.txt)
    message(FATAL_ERROR "GESTURE_CORE_IMPL=${GESTURE_CORE_IMPL}: "
        "not an implementation directory under ${CMAKE_CURRENT_SOURCE_DIR}")
endif()
add_subdirectory(${GESTURE_CORE_IMPL})

if(NOT TARGET gesture_core)
    message(FATAL_ERROR "core/${GESTURE_CORE_IMPL} does not define `gesture_core`")
endif()
get_target_property(gesture_core_type gesture_core TYPE)
get_target_property(gesture_core_links gesture_core INTERFACE_LINK_LIBRARIES)
if(NOT gesture_core_type STREQUAL "STATIC_LIBRARY" OR NOT "gesture_core_abi" IN_LIST gesture_core_links)
    message(FATAL_ERROR "core/${GESTURE_CORE_IMPL}: `gesture_core` must be a static library "
        "linking gesture_core_abi (type ${gesture_core_type}, links ${gesture_core_links})")
endif()
message(STATUS "core.h implementation: core/${GESTURE_CORE_IMPL}")

# build.rs links `static=gesture_core` from lib/: the file name is part of the interface.
# install(FILES), not install(TARGETS), so an IMPORTED library installs too.
install(FILES $<TARGET_FILE:gesture_core> DESTINATION lib
    RENAME ${CMAKE_STATIC_LIBRARY_PREFIX}gesture_core${CMAKE_STATIC_LIBRARY_SUFFIX})
install(DIRECTORY contract/include/ DESTINATION include)
