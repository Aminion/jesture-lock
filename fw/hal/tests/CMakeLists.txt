# Register-free parts of fw/hal (header-only: ring buffer, baud/I2C timing math, WHO_AM_I map, IMU sample decode).
# Only the include directory: the driver sources need CMSIS/LL and the MCU.
add_executable(hal_test
    hal_test.cpp
)
target_include_directories(hal_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../include)
gesture_target_options(hal_test)

add_test(NAME hal_test COMMAND hal_test)
