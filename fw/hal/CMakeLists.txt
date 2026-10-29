# Board support: clocks, pins, I2C/UART, LSM6DS3 driver. CMSIS + ST LL, no HAL.

# Public headers alone: their register-free parts (ring buffer, baud/I2C timing math,
# WHO_AM_I map, IMU sample decode) are tested on the host (fw/hal/tests).
add_library(gesture_hal_headers INTERFACE)
target_include_directories(gesture_hal_headers INTERFACE include)
target_link_libraries(gesture_hal_headers INTERFACE gesture_proto)

if(NOT CMAKE_CROSSCOMPILING)
    return()
endif()

foreach(dir CMSIS_CORE_DIR CMSIS_DEVICE_G0_DIR STM32G0_LL_DIR)
    if(NOT ${dir} OR NOT IS_DIRECTORY "${${dir}}")
        message(FATAL_ERROR "${dir} is not set or missing ('${${dir}}'). Run inside `nix develop`.")
    endif()
endforeach()

# --- Vendor code (ST/ARM): built without project warnings ---------------------
add_library(stm32g0_vendor STATIC
    ${CMSIS_DEVICE_G0_DIR}/Source/Templates/gcc/startup_stm32g031xx.s
    ${CMSIS_DEVICE_G0_DIR}/Source/Templates/system_stm32g0xx.c
    ${STM32G0_LL_DIR}/Src/stm32g0xx_ll_dma.c
    ${STM32G0_LL_DIR}/Src/stm32g0xx_ll_exti.c
    ${STM32G0_LL_DIR}/Src/stm32g0xx_ll_gpio.c
    ${STM32G0_LL_DIR}/Src/stm32g0xx_ll_i2c.c
    ${STM32G0_LL_DIR}/Src/stm32g0xx_ll_rcc.c
    ${STM32G0_LL_DIR}/Src/stm32g0xx_ll_tim.c
    ${STM32G0_LL_DIR}/Src/stm32g0xx_ll_usart.c
    ${STM32G0_LL_DIR}/Src/stm32g0xx_ll_utils.c
)
target_include_directories(stm32g0_vendor SYSTEM PUBLIC
    ${CMSIS_CORE_DIR}/CMSIS/Core/Include
    ${CMSIS_DEVICE_G0_DIR}/Include
    ${STM32G0_LL_DIR}/Inc
)
target_compile_definitions(stm32g0_vendor PUBLIC
    STM32G031xx
    USE_FULL_LL_DRIVER
)

# --- First-party drivers ------------------------------------------------------
add_library(gesture_hal STATIC
    src/clock.cpp
    src/gpio.cpp
    src/i2c.cpp
    src/uart.cpp
    src/lsm6ds3.cpp
    src/imu_int.cpp
)
target_link_libraries(gesture_hal PUBLIC gesture_hal_headers stm32g0_vendor)
gesture_target_options(gesture_hal)

set(GESTURE_LINKER_SCRIPT ${CMAKE_CURRENT_SOURCE_DIR}/ld/stm32g031k8.ld
    CACHE FILEPATH "Linker script for the firmware images")
