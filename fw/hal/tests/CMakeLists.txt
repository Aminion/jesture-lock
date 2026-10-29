# Register-free parts of fw/hal (header-only: ring buffer, baud/I2C timing math, WHO_AM_I map, IMU sample decode).
# The driver sources need CMSIS/LL and the MCU.
add_executable(hal_test
    hal_test.cpp
)
target_link_libraries(hal_test PRIVATE gesture_hal_headers gesture_test)
gesture_target_options(hal_test)

add_test(NAME hal_test COMMAND hal_test)
