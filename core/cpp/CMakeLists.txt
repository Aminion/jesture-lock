# The C++ implementation of core.h (docs/core.md). Internal headers in src/ stay private.
add_library(gesture_core STATIC
    src/gesture_core.cpp
)
target_link_libraries(gesture_core PUBLIC gesture_core_abi)
gesture_target_options(gesture_core)
