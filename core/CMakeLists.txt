# Recognition core. Builds as part of the top-level project (host and MCU)
# and standalone (host/crates/gesture-core/build.rs via the `cmake` crate).
# Compile options live here so every consumer gets the same build.
cmake_minimum_required(VERSION 3.25)
project(gesture_core LANGUAGES CXX)

add_library(gesture_core STATIC
    src/gesture_core.cpp
)

target_include_directories(gesture_core PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_compile_features(gesture_core PUBLIC cxx_std_26)
set_target_properties(gesture_core PROPERTIES CXX_EXTENSIONS OFF)
target_compile_options(gesture_core PRIVATE
    -fno-exceptions -fno-rtti
    -Wall -Wextra -Wpedantic -Wconversion -Wsign-conversion -Wshadow
    -Wcast-align -Wnull-dereference -Wdouble-promotion -Werror
)

install(TARGETS gesture_core ARCHIVE DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)
