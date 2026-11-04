// Host tests for core/. Returns non-zero on failure (ctest). Internal helpers are
// constexpr and checked by static_assert; the C ABI is checked at run time. Conformance
// vectors (docs/core.md, through core.h only) are separate.
#include "arith.hpp"
#include "config.hpp"
#include "gesture/core.h"
#include "gesture/test/check.hpp"

#include <cstdint>
#include <limits>

namespace {

using gcore::rdiv;

// --- rdiv: to nearest, halves away from zero ------------------------------------

static_assert(rdiv(0, 5) == 0);
static_assert(rdiv(6, 3) == 2 && rdiv(-6, 3) == -2);
static_assert(rdiv(1, 3) == 0 && rdiv(-1, 3) == 0);
static_assert(rdiv(2, 3) == 1 && rdiv(-2, 3) == -1);
static_assert(rdiv(4, 3) == 1 && rdiv(5, 3) == 2 && rdiv(-5, 3) == -2);
static_assert(rdiv(1, 2) == 1 && rdiv(-1, 2) == -1); // halves
static_assert(rdiv(5, 2) == 3 && rdiv(-5, 2) == -3);
static_assert(rdiv(1, 4) == 0 && rdiv(2, 4) == 1 && rdiv(3, 4) == 1);
static_assert(rdiv(-2, 4) == -1 && rdiv(-3, 4) == -1 && rdiv(-6, 4) == -2);
static_assert(rdiv(7, 1) == 7 && rdiv(-7, 1) == -7);
static_assert(rdiv(rdiv(-5, 2), 1) == -rdiv(5, 2)); // odd: rdiv(-a, b) == -rdiv(a, b)
// Large operands, as in the Q30 products.
static_assert(rdiv((3LL << 61) + (1LL << 29), 1LL << 30) == (3LL << 31) + 1);
static_assert(rdiv((3LL << 61) + (1LL << 29) - 1, 1LL << 30) == 3LL << 31);
static_assert(rdiv(-(3LL << 61) - (1LL << 29), 1LL << 30) == -(3LL << 31) - 1);

// --- saturation ------------------------------------------------------------------

static_assert(gcore::sat16(32767) == 32767 && gcore::sat16(32768) == 32767);
static_assert(gcore::sat16(-32768) == -32768 && gcore::sat16(-32769) == -32768);
static_assert(gcore::sat16(std::numeric_limits<std::int64_t>::min()) == -32768);
static_assert(gcore::satu16(-1) == 0 && gcore::satu16(65535) == 65535);
static_assert(gcore::satu16(65536) == 65535);

// --- scales: every FS code, ODR bits ignored -------------------------------------

static_assert(gcore::accel_ug_per_lsb(0x40) == 61);     // 2 g
static_assert(gcore::accel_ug_per_lsb(0x44) == 488);    // 16 g
static_assert(gcore::accel_ug_per_lsb(0x48) == 122);    // 4 g
static_assert(gcore::accel_ug_per_lsb(0x4C) == 244);    // 8 g
static_assert(gcore::accel_ug_per_lsb(0xA8) == 122);    // other ODR, low bits
static_assert(gcore::gyro_udps_per_lsb(0x40) == 8750);  // 250 dps
static_assert(gcore::gyro_udps_per_lsb(0x44) == 17500); // 500 dps
static_assert(gcore::gyro_udps_per_lsb(0x48) == 35000); // 1000 dps
static_assert(gcore::gyro_udps_per_lsb(0x4C) == 70000); // 2000 dps
static_assert(gcore::gyro_udps_per_lsb(0x42) == 4375);  // 125 dps
static_assert(gcore::gyro_udps_per_lsb(0x4E) == 4375);  // FS_125 wins over FS_G

// --- gc_config validation ---------------------------------------------------------

// The defaults in docs/todo.md: hand, +-4 g / 500 dps, 104 Hz; unit calibration.
constexpr gc_config valid_config() {
    constexpr std::int32_t one_q30 = 1 << 30;
    return gc_config{
        .abi_version = GC_ABI_VERSION,
        .ctrl1_xl = 0x48,
        .ctrl2_g = 0x44,
        .reserved0 = 0,
        .accel_offset_lsb = {0, 0, 0},
        .reserved1 = 0,
        .accel_gain_q30 = {one_q30, one_q30, one_q30},
        .gyro_bias_q8 = {0, 0, 0},
        .still_gyro_std_lsb = 171,
        .still_accel_std_lsb = 123,
        .still_gyro_rate_lsb = 229,
        .hold_n = 20,
        .bias_gyro_std_lsb = 11,
        .bias_accel_std_lsb = 16,
        .clip_lsb = 32700,
        .max_dt_us = 14423,
        .min_stroke_us = 150000,
        .max_stroke_us = 4000000,
        .min_extent_mm = 20,
        .max_v_end_mm_s = 300,
    };
}

template <typename F> constexpr gc_status validate_with(F change) {
    gc_config cfg = valid_config();
    change(cfg);
    return gcore::validate(cfg);
}

static_assert(gcore::validate(valid_config()) == GC_OK);

static_assert(validate_with([](gc_config& c) { c.abi_version = 0; }) == GC_ERR_VERSION);
static_assert(validate_with([](gc_config& c) { c.abi_version = 2; }) == GC_ERR_VERSION);
static_assert(validate_with([](gc_config& c) { c.ctrl1_xl = 0; }) == GC_ERR_SCALE);
static_assert(validate_with([](gc_config& c) { c.ctrl2_g = 0; }) == GC_ERR_SCALE);

static_assert(validate_with([](gc_config& c) { c.reserved0 = 1; }) == GC_ERR_CONFIG);
static_assert(validate_with([](gc_config& c) { c.reserved1 = 1; }) == GC_ERR_CONFIG);
// Gain bounds are inclusive, on every axis.
static_assert(validate_with([](gc_config& c) { c.accel_gain_q30[0] = gcore::gain_min_q30; }) ==
              GC_OK);
static_assert(validate_with([](gc_config& c) { c.accel_gain_q30[1] = gcore::gain_max_q30; }) ==
              GC_OK);
static_assert(validate_with([](gc_config& c) { c.accel_gain_q30[1] = gcore::gain_min_q30 - 1; }) ==
              GC_ERR_CONFIG);
static_assert(validate_with([](gc_config& c) { c.accel_gain_q30[2] = gcore::gain_max_q30 + 1; }) ==
              GC_ERR_CONFIG);
static_assert(validate_with([](gc_config& c) { c.still_gyro_std_lsb = 0; }) == GC_ERR_CONFIG);
static_assert(validate_with([](gc_config& c) { c.still_accel_std_lsb = 0; }) == GC_ERR_CONFIG);
static_assert(validate_with([](gc_config& c) { c.still_gyro_rate_lsb = 0; }) == GC_ERR_CONFIG);
static_assert(validate_with([](gc_config& c) { c.bias_gyro_std_lsb = 0; }) == GC_ERR_CONFIG);
static_assert(validate_with([](gc_config& c) { c.bias_accel_std_lsb = 0; }) == GC_ERR_CONFIG);
static_assert(validate_with([](gc_config& c) { c.clip_lsb = 0; }) == GC_ERR_CONFIG);
static_assert(validate_with([](gc_config& c) { c.hold_n = gcore::guard_n + 1; }) == GC_OK);
static_assert(validate_with([](gc_config& c) { c.hold_n = gcore::guard_n; }) == GC_ERR_CONFIG);
static_assert(validate_with([](gc_config& c) { c.max_dt_us = 0; }) == GC_ERR_CONFIG);
static_assert(validate_with([](gc_config& c) { c.max_dt_us = 1; }) == GC_OK);
static_assert(validate_with([](gc_config& c) { c.max_dt_us = 20000; }) == GC_OK);
static_assert(validate_with([](gc_config& c) { c.max_dt_us = 20001; }) == GC_ERR_CONFIG);
static_assert(validate_with([](gc_config& c) { c.min_stroke_us = c.max_stroke_us; }) ==
              GC_ERR_CONFIG);
static_assert(validate_with([](gc_config& c) { c.max_stroke_us = 6553500; }) == GC_OK);
static_assert(validate_with([](gc_config& c) { c.max_stroke_us = 6553501; }) == GC_ERR_CONFIG);
// No range: any value passes.
static_assert(validate_with([](gc_config& c) {
                  c.min_extent_mm = 0;
                  c.max_v_end_mm_s = 0;
                  c.accel_offset_lsb[0] = -32768;
                  c.gyro_bias_q8[2] = std::numeric_limits<std::int32_t>::min();
              }) == GC_OK);
// Order: the first failing class in the gc_status table wins.
static_assert(validate_with([](gc_config& c) {
                  c.abi_version = 0;
                  c.ctrl1_xl = 0;
                  c.hold_n = 0;
              }) == GC_ERR_VERSION);
static_assert(validate_with([](gc_config& c) {
                  c.ctrl2_g = 0;
                  c.hold_n = 0;
              }) == GC_ERR_SCALE);

// --- gc_init through the C ABI ------------------------------------------------------

void test_init(gesture_test::Tests& t) {
    gc_state s{};
    gc_config cfg = valid_config();
    check(t, gc_init(&s, &cfg) == GC_OK);
    cfg.abi_version = 0;
    check(t, gc_init(&s, &cfg) == GC_ERR_VERSION);
    cfg = valid_config();
    cfg.max_dt_us = 0;
    check(t, gc_init(&s, &cfg) == GC_ERR_CONFIG);
    cfg = valid_config();
    check(t, gc_init(&s, &cfg) == GC_OK); // usable again after an error
}

} // namespace

int main() {
    gesture_test::Tests t;
    test_init(t);
    return gesture_test::finish(t, "core_test");
}
