# Register-free parts of firmware/board (header-only: ring buffer, baud/I2C timing math, WHO_AM_I map, IMU sample decode).
# The driver sources need CMSIS/LL and the MCU.
add_executable(board_test
    board_test.cpp
)
target_link_libraries(board_test PRIVATE gesture_board_headers gesture_test)
gesture_target_options(board_test)

add_test(NAME board_test COMMAND board_test)
