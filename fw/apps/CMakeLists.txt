# The two firmware images. Each app directory holds only main.cpp.

function(gesture_firmware name)
    add_executable(${name} ${ARGN})
    target_link_libraries(${name} PRIVATE gesture_hal gesture_proto)
    gesture_target_options(${name})
    target_link_options(${name} PRIVATE
        -T${GESTURE_LINKER_SCRIPT}
        -Wl,-Map=$<TARGET_FILE_DIR:${name}>/${name}.map,--cref
    )
    set_target_properties(${name} PROPERTIES LINK_DEPENDS ${GESTURE_LINKER_SCRIPT})

    add_custom_command(TARGET ${name} POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${name}> $<TARGET_FILE_DIR:${name}>/${name}.bin
        COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${name}> $<TARGET_FILE_DIR:${name}>/${name}.hex
        COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${name}>
        VERBATIM
    )

    # cmake --build --preset firmware --target flash_<name>
    add_custom_target(flash_${name}
        COMMAND openocd -f interface/stlink.cfg -f target/stm32g0x.cfg
                -c "program $<TARGET_FILE:${name}> verify reset exit"
        DEPENDS ${name}
        USES_TERMINAL
        VERBATIM
    )
endfunction()

gesture_firmware(fw_stream stream/main.cpp)

gesture_firmware(fw_recognize recognize/main.cpp)
target_link_libraries(fw_recognize PRIVATE gesture_core)
