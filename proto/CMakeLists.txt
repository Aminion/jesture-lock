# Protocol framing (COBS + CRC16). No hardware dependencies; builds for host and MCU.
# Header-only and constexpr, so most tests are static_asserts (proto/tests).
add_library(gesture_proto INTERFACE)
target_include_directories(gesture_proto INTERFACE include)
target_compile_features(gesture_proto INTERFACE cxx_std_26)
