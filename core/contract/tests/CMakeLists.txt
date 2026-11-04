# The conformance vectors, through core.h only (no implementation headers on the include
# path), against whichever implementation provides `gesture_core`.
add_executable(core_vectors
    core_vectors.cpp
)
target_link_libraries(core_vectors PRIVATE gesture_core)
gesture_target_options(core_vectors)
# Data, not sources: every committed vector (gesture-cli vectors writes them).
file(GLOB GESTURE_VECTORS CONFIGURE_DEPENDS
    ${PROJECT_SOURCE_DIR}/core/contract/vectors/v1/*.gcv
    ${PROJECT_SOURCE_DIR}/core/contract/vectors/v1/*/*.gcv)
add_test(NAME core_vectors COMMAND core_vectors ${GESTURE_VECTORS})

# core.h compiled as C11, the oldest C the contract supports. Build-only: a C++-only
# construct in the header fails the build.
enable_language(C)
add_library(core_c_check OBJECT core_c_check.c)
target_link_libraries(core_c_check PRIVATE gesture_core_abi)
set_target_properties(core_c_check PROPERTIES C_STANDARD 11 C_STANDARD_REQUIRED ON C_EXTENSIONS OFF)
target_compile_options(core_c_check PRIVATE -Wall -Wextra -Wpedantic -Werror)
