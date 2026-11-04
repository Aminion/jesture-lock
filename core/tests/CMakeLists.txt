add_executable(core_test
    core_test.cpp
)
target_link_libraries(core_test PRIVATE gesture_core gesture_test)
# Internal headers too: unit tests of the helpers, apart from the conformance vectors.
target_include_directories(core_test PRIVATE ${PROJECT_SOURCE_DIR}/core/src)
gesture_target_options(core_test)

add_test(NAME core_test COMMAND core_test)

# The conformance vectors, through core.h only (no core/src on the include path).
add_executable(core_vectors
    core_vectors.cpp
)
target_link_libraries(core_vectors PRIVATE gesture_core)
gesture_target_options(core_vectors)
# Data, not sources: every committed vector (gesture-cli vectors writes them).
file(GLOB GESTURE_VECTORS CONFIGURE_DEPENDS
    ${PROJECT_SOURCE_DIR}/data/vectors/*.gcv
    ${PROJECT_SOURCE_DIR}/data/vectors/*/*.gcv)
add_test(NAME core_vectors COMMAND core_vectors ${GESTURE_VECTORS})

# core.h compiled as C11, the oldest C the contract supports. Build-only: a C++-only
# construct in the header fails the build.
enable_language(C)
add_library(core_c_check OBJECT core_c_check.c)
target_include_directories(core_c_check PRIVATE ${PROJECT_SOURCE_DIR}/core/include)
set_target_properties(core_c_check PROPERTIES C_STANDARD 11 C_STANDARD_REQUIRED ON C_EXTENSIONS OFF)
target_compile_options(core_c_check PRIVATE -Wall -Wextra -Wpedantic -Werror)
