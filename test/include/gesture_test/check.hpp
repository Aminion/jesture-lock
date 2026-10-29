#pragma once

// Minimal run-time checks for the host test executables (ctest): report the failing line,
// keep going, exit non-zero at the end. Most checks are static_asserts; this is for the rest.

#include <cstdio>
#include <print>
#include <source_location>
#include <string_view>

namespace gesture_test {

struct Tests {
    int failed = 0;
};

inline void check(Tests& t, bool ok,
                  std::source_location loc = std::source_location::current()) {
    if (!ok) {
        std::println(stderr, "{}:{}: check failed", loc.file_name(), loc.line());
        ++t.failed;
    }
}

// main's return value.
[[nodiscard]] inline int finish(const Tests& t, std::string_view name) {
    if (t.failed != 0) {
        std::println(stderr, "{}: {} checks failed", name, t.failed);
        return 1;
    }
    return 0;
}

} // namespace gesture_test
