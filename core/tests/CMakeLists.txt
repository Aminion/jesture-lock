add_executable(core_test
    core_test.cpp
)
target_link_libraries(core_test PRIVATE gesture_core)
gesture_target_options(core_test)

add_test(NAME core_test COMMAND core_test)
