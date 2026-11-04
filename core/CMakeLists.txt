# Recognition core: the core.h contract plus one implementation of it. Builds as part of
# the top-level project (host and MCU) and standalone (host/crates/gesture-core/build.rs
# via the `cmake` crate). Same compile options either way: cmake/gesture_options.cmake.
# Callers link `gesture_core` and see core.h only.
cmake_minimum_required(VERSION 3.25)
project(gesture_core LANGUAGES CXX)

include(${CMAKE_CURRENT_LIST_DIR}/../cmake/gesture_options.cmake)

add_subdirectory(contract) # gesture_core_abi: core.h
add_subdirectory(cpp)      # gesture_core: the C++ implementation

install(TARGETS gesture_core ARCHIVE DESTINATION lib)
install(DIRECTORY contract/include/ DESTINATION include)
