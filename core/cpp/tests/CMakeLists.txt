add_executable(core_test
    core_test.cpp
)
target_link_libraries(core_test PRIVATE gesture_core gesture_test)
# Internal headers too: unit tests of the C++ implementation's helpers, apart from the
# conformance vectors (core/contract/tests).
target_include_directories(core_test PRIVATE ${PROJECT_SOURCE_DIR}/core/cpp/src)
gesture_target_options(core_test)

add_test(NAME core_test COMMAND core_test)
