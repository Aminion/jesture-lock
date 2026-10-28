# Shared compile options for first-party targets (not vendor code).
# core/ sets its own identical options so it can also build standalone (Rust build.rs).
function(gesture_target_options target)
    target_compile_features(${target} PUBLIC cxx_std_23)
    set_target_properties(${target} PROPERTIES CXX_EXTENSIONS OFF)
    target_compile_options(${target} PRIVATE
        $<$<COMPILE_LANGUAGE:CXX>:-fno-exceptions -fno-rtti>
        -Wall -Wextra -Wpedantic -Wconversion -Wsign-conversion -Wshadow
        -Wcast-align -Wnull-dereference -Wdouble-promotion -Werror
    )
endfunction()
