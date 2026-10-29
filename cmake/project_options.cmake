# Shared compile options for first-party targets (not vendor code).
# core/ sets its own identical options so it can also build standalone (Rust build.rs).
function(gesture_target_options target)
    target_compile_features(${target} PUBLIC cxx_std_26)
    set_target_properties(${target} PROPERTIES CXX_EXTENSIONS OFF)
    target_compile_options(${target} PRIVATE
        $<$<COMPILE_LANGUAGE:CXX>:-fno-exceptions -fno-rtti>
        -Wall -Wextra -Wpedantic -Wconversion -Wsign-conversion -Wshadow
        -Wcast-align -Wnull-dereference -Wdouble-promotion -Werror
    )
endfunction()

# clang-tidy on every first-party C++ file as it compiles (vendor code is C/asm only).
# Config: .clang-tidy at the root, fw/.clang-tidy for firmware-specific exceptions.
option(GESTURE_CLANG_TIDY "Run clang-tidy while compiling" ON)
if(GESTURE_CLANG_TIDY)
    if(CMAKE_CROSSCOMPILING)
        # The Nix clang-tidy wrapper exports CPLUS_INCLUDE_PATH with the host glibc/libc++;
        # headers missing from our paths would silently resolve there. Use the bare binary.
        find_program(GESTURE_CLANG_TIDY_EXE NAMES clang-tidy-unwrapped clang-tidy REQUIRED)
    else()
        find_program(GESTURE_CLANG_TIDY_EXE clang-tidy REQUIRED)
    endif()
    set(tidy ${GESTURE_CLANG_TIDY_EXE} --quiet)
    if(CMAKE_CROSSCOMPILING)
        # clang would parse firmware with the host's libc headers. Give it the target and
        # GCC's libstdc++/newlib paths, but keep clang's own builtin headers: GCC's
        # (lib/gcc/arm-none-eabi/<ver>/include*, e.g. arm_acle.h) use GCC-only builtins.
        list(APPEND tidy --extra-arg-before=--target=arm-none-eabi --extra-arg=-nostdlibinc)
        foreach(dir IN LISTS CMAKE_CXX_IMPLICIT_INCLUDE_DIRECTORIES)
            if(NOT dir MATCHES "/lib/gcc/arm-none-eabi/[^/]+/include")
                list(APPEND tidy --extra-arg=-isystem${dir})
            endif()
        endforeach()
    endif()
    set(CMAKE_CXX_CLANG_TIDY ${tidy})
endif()
