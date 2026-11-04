add_executable(core_test
    core_test.cpp
)
target_link_libraries(core_test PRIVATE gesture_core gesture_test)
gesture_target_options(core_test)

add_test(NAME core_test COMMAND core_test)

# core.h compiled as C11, the oldest C the contract supports. Build-only: a C++-only
# construct in the header fails the build.
enable_language(C)
add_library(core_c_check OBJECT core_c_check.c)
target_include_directories(core_c_check PRIVATE ${PROJECT_SOURCE_DIR}/core/include)
set_target_properties(core_c_check PROPERTIES C_STANDARD 11 C_STANDARD_REQUIRED ON C_EXTENSIONS OFF)
target_compile_options(core_c_check PRIVATE -Wall -Wextra -Wpedantic -Werror)
