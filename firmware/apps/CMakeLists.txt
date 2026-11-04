# The two firmware images. Each app directory holds only main.cpp.

function(gesture_firmware name)
    add_executable(${name} ${ARGN})
    target_link_libraries(${name} PRIVATE gesture_board gesture_proto)
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

gesture_firmware(stream stream/main.cpp)

gesture_firmware(recognize recognize/main.cpp)
target_link_libraries(recognize PRIVATE gesture_core)
# The board self-test vector (gesture-cli vector), embedded with #embed. The compiler's
# depfile does not list it, so name it here to rebuild when it is regenerated.
set(GESTURE_BOARD_VECTOR ${PROJECT_SOURCE_DIR}/core/contract/vectors/v1/air-lines.gcv)
target_compile_definitions(recognize PRIVATE GESTURE_BOARD_VECTOR="${GESTURE_BOARD_VECTOR}")
set_source_files_properties(recognize/main.cpp PROPERTIES OBJECT_DEPENDS ${GESTURE_BOARD_VECTOR})
