# Compile options for first-party targets (not vendor code). Its own file so core/ can
# include it when built standalone (host/crates/gesture-core/build.rs), without clang-tidy.
function(gesture_target_options target)
    target_compile_features(${target} PUBLIC cxx_std_26)
    set_target_properties(${target} PROPERTIES CXX_EXTENSIONS OFF)
    target_compile_options(${target} PRIVATE
        $<$<COMPILE_LANGUAGE:CXX>:-fno-exceptions -fno-rtti>
        -Wall -Wextra -Wpedantic -Wconversion -Wsign-conversion -Wshadow
        -Wcast-align -Wnull-dereference -Wdouble-promotion -Werror
    )
endfunction()
