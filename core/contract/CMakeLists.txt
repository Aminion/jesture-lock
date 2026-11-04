# The contract of the core, apart from any implementation: core.h (docs/core.md), its C11
# check and the conformance vector runner (contract/tests). An implementation provides
# the `gesture_core` static library and links this target.
add_library(gesture_core_abi INTERFACE)
target_include_directories(gesture_core_abi INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
