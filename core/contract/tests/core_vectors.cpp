// The conformance vectors (core/contract/vectors/, docs/core.md): each file given on the command
// line is replayed from gc_init through gesture/core.h only, and its digest compared with
// the one stored in it. Any implementation of core.h linked here runs the same suite.
// Layout and digest: core/contract/vectors/v1/README.md (written by `gesture-cli vectors`).
#include "gesture/core.h"

#include <algorithm>
#include <array>
#include <bit>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <memory>
#include <print>
#include <span>
#include <string_view>
#include <vector>

namespace {

constexpr std::size_t header_size = 84;
constexpr std::size_t config_at = 16;
constexpr std::uint16_t points_cap = 64;
constexpr std::array<std::uint8_t, 4> magic{'G', 'C', 'V', '1'};

static_assert(std::endian::native == std::endian::little, "the vectors are little-endian");

template <typename T> T read_at(std::span<const std::uint8_t> b, std::size_t at) {
    std::array<std::uint8_t, sizeof(T)> x{};
    std::ranges::copy(b.subspan(at, sizeof(T)), x.begin());
    return std::bit_cast<T>(x);
}

// FNV-1a 32 over the C layouts.
struct Fnv {
    std::uint32_t h = 0x811c'9dc5;
};

template <typename T> void hash(Fnv& f, const T& v) {
    for (const std::uint8_t b : std::bit_cast<std::array<std::uint8_t, sizeof(T)>>(v)) {
        f.h = (f.h ^ b) * 0x0100'0193U;
    }
}

struct Result {
    std::uint32_t events = 0;
    std::uint32_t digest = 0;
};

std::vector<std::uint8_t> read_file(const char* path) {
    std::vector<std::uint8_t> b;
    const std::unique_ptr<std::FILE, int (*)(std::FILE*)> f{std::fopen(path, "rb"), std::fclose};
    if (!f) {
        return b;
    }
    std::array<std::uint8_t, 4096> chunk{};
    for (std::size_t n = 0; (n = std::fread(chunk.data(), 1, chunk.size(), f.get())) > 0;) {
        const std::span got = std::span{chunk}.first(n);
        b.insert(b.end(), got.begin(), got.end());
    }
    return b;
}

// Replays one vector. Returns false (with a message) if the file is malformed or gc_init
// rejects its config.
bool replay(std::span<const std::uint8_t> b, std::string_view name, Result& r) {
    if (b.size() < header_size || !std::ranges::equal(b.first(4), magic)) {
        std::println(stderr, "{}: not a GCV1 vector", name);
        return false;
    }
    const auto n = read_at<std::uint32_t>(b, 4);
    if (b.size() != header_size + (std::size_t{n} * sizeof(gc_sample))) {
        std::println(stderr, "{}: {} bytes for {} samples", name, b.size(), n);
        return false;
    }
    const auto cfg = read_at<gc_config>(b, config_at);
    const auto state = std::make_unique<gc_state>();
    if (const gc_status st = gc_init(state.get(), &cfg); st != GC_OK) {
        std::println(stderr, "{}: gc_init returned {}", name, st);
        return false;
    }
    std::array<gc_point, points_cap> points{};
    Fnv f;
    for (std::uint32_t k = 0; k < n; ++k) {
        const auto in = read_at<gc_sample>(b, header_size + (std::size_t{k} * sizeof(gc_sample)));
        const gc_event ev = gc_feed(state.get(), &in);
        if (ev.kind != GC_EV_NONE) {
            ++r.events;
            hash(f, k);
            hash(f, ev);
        }
        if (ev.kind == GC_EV_STROKE) {
            gc_stroke_info info{};
            const std::uint16_t got = gc_stroke(state.get(), &info, points.data(), points_cap);
            hash(f, got);
            hash(f, info);
            for (const gc_point& p : std::span{points}.first(got)) {
                hash(f, p);
            }
        }
        gc_point live{};
        hash(f, gc_live(state.get(), &live));
        hash(f, live);
    }
    r.digest = f.h;
    return true;
}

} // namespace

int main(int argc, char** argv) {
    const std::span args{argv, static_cast<std::size_t>(argc)};
    int failed = 0;
    for (const char* path : args.subspan(1)) {
        const std::vector<std::uint8_t> b = read_file(path);
        Result r;
        if (!replay(b, path, r)) {
            ++failed;
            continue;
        }
        const Result want{.events = read_at<std::uint32_t>(b, 8),
                          .digest = read_at<std::uint32_t>(b, 12)};
        if (r.events != want.events || r.digest != want.digest) {
            std::println(stderr, "{}: {} events, digest {:08x}; the vector says {}, {:08x}", path,
                         r.events, r.digest, want.events, want.digest);
            ++failed;
        }
    }
    std::println("core_vectors: {} vectors, {} failed", args.size() - 1, failed);
    return failed == 0 && args.size() > 1 ? 0 : 1;
}
