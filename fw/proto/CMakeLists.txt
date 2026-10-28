# Protocol framing (COBS + CRC16). No hardware dependencies; builds for host and MCU.
add_library(gesture_proto STATIC
    src/cobs.cpp
    src/crc16.cpp
)
target_include_directories(gesture_proto PUBLIC include)
gesture_target_options(gesture_proto)
