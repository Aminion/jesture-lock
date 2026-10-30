cmake_minimum_required(VERSION 3.25)

project(gesture_imu LANGUAGES C CXX)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
include(project_options)

add_subdirectory(core)
add_subdirectory(proto)

if(CMAKE_CROSSCOMPILING)
    enable_language(ASM)
endif()
add_subdirectory(firmware/board) # host: headers only

if(CMAKE_CROSSCOMPILING)
    add_subdirectory(firmware/apps)
else()
    enable_testing()
    add_subdirectory(test_support)
    add_subdirectory(core/tests)
    add_subdirectory(proto/tests)
    add_subdirectory(firmware/board/tests)
endif()
