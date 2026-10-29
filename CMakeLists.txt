cmake_minimum_required(VERSION 3.25)

project(gesture_imu LANGUAGES C CXX)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
include(project_options)

add_subdirectory(core)
add_subdirectory(fw/proto)

if(CMAKE_CROSSCOMPILING)
    enable_language(ASM)
endif()
add_subdirectory(fw/hal) # host: headers only

if(CMAKE_CROSSCOMPILING)
    add_subdirectory(fw/apps)
else()
    enable_testing()
    add_subdirectory(test)
    add_subdirectory(core/tests)
    add_subdirectory(fw/proto/tests)
    add_subdirectory(fw/hal/tests)
endif()
