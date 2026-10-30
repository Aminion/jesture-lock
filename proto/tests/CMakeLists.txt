add_executable(proto_test
    proto_test.cpp
)
target_link_libraries(proto_test PRIVATE gesture_proto gesture_test)
gesture_target_options(proto_test)

add_test(NAME proto_test COMMAND proto_test)
