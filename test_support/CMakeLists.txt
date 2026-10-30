# Shared helpers for the host test executables (header-only).
add_library(gesture_test INTERFACE)
target_include_directories(gesture_test INTERFACE include)
target_compile_features(gesture_test INTERFACE cxx_std_26)
