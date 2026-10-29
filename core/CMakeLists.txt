# Recognition core. Builds as part of the top-level project (host and MCU)
# and standalone (host/crates/gesture-core/build.rs via the `cmake` crate).
# Same compile options either way: cmake/gesture_options.cmake.
cmake_minimum_required(VERSION 3.25)
project(gesture_core LANGUAGES CXX)

include(${CMAKE_CURRENT_LIST_DIR}/../cmake/gesture_options.cmake)

add_library(gesture_core STATIC
    src/gesture_core.cpp
)

target_include_directories(gesture_core PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

gesture_target_options(gesture_core)

install(TARGETS gesture_core ARCHIVE DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)
