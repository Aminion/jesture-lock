/*
 * Gesture recognition core: C ABI, contract in docs/core.md. ABI version 1 (frozen
 * 2026-10-30).
 *
 * Consumed by the recognize firmware (MCU) and by host/crates/gesture-core (Rust FFI).
 * Pure: no I/O, no clock, no global state; the caller owns all state buffers.
 * Inputs are raw LSM6DS3 LSB samples with MCU timestamps.
 */
#ifndef GESTURE_CORE_H
#define GESTURE_CORE_H

/* Valid C (C11 and later) and C++: the contract outlives the C++ implementation, and C
 * is what other languages and binding tools read. */
#include <stdint.h> // NOLINT(modernize-deprecated-headers): C has no <cstdint>
#ifndef __cplusplus
#include <assert.h>   /* static_assert before C23 */
#include <stdalign.h> /* alignas, alignof before C23 */
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* docs/core.md: "Enumerations are integer #define, never C enum" (another language may
 * implement this contract directly; a C++ enum has no such guarantee). */
// NOLINTBEGIN(cppcoreguidelines-macro-usage)
#define GC_ABI_VERSION 1u
#define GC_STATE_SIZE  2048u
#define GC_STATE_ALIGN 8u

#define GC_OK          0
#define GC_ERR_VERSION 1
#define GC_ERR_SCALE   2
#define GC_ERR_CONFIG  3

#define GC_EV_NONE     0
#define GC_EV_READY    1
#define GC_EV_MOVING   2
#define GC_EV_STROKE   3
#define GC_EV_REJECTED 4

#define GC_ST_WARMUP 0
#define GC_ST_STILL  1
#define GC_ST_MOVING 2

#define GC_REJ_SHORT    1
#define GC_REJ_LONG     2
#define GC_REJ_CLIPPED  3
#define GC_REJ_TIME_GAP 4
#define GC_REJ_DRIFT    5
// NOLINTEND(cppcoreguidelines-macro-usage)

/* typedef, not using: C has no alias declarations. */
// NOLINTBEGIN(modernize-use-using)
typedef int32_t gc_status;

typedef struct gc_state {
    alignas(GC_STATE_ALIGN) uint8_t opaque[GC_STATE_SIZE];
} gc_state;

/* The fields of the wire Sample (docs/protocol.md). */
typedef struct gc_sample {
    uint32_t t_us;
    int16_t  gyro[3];
    int16_t  accel[3];
} gc_sample;

/* Calibration and thresholds, in LSB of the scale selected by ctrl1_xl/ctrl2_g. */
typedef struct gc_config {
    uint32_t abi_version;
    uint8_t  ctrl1_xl;
    uint8_t  ctrl2_g;
    uint16_t reserved0;
    int16_t  accel_offset_lsb[3];
    uint16_t reserved1;
    int32_t  accel_gain_q30[3];
    int32_t  gyro_bias_q8[3];
    uint16_t still_gyro_std_lsb;
    uint16_t still_accel_std_lsb;
    uint16_t still_gyro_rate_lsb;
    uint16_t hold_n;
    uint16_t bias_gyro_std_lsb;
    uint16_t bias_accel_std_lsb;
    uint16_t clip_lsb;
    uint16_t max_dt_us;
    uint32_t min_stroke_us;
    uint32_t max_stroke_us;
    uint16_t min_extent_mm;
    uint16_t max_v_end_mm_s;
} gc_config;

/* state is the state after the sample. */
typedef struct gc_event {
    uint8_t  kind;
    uint8_t  state;
    uint8_t  reason;
    uint8_t  reserved;
    uint16_t points;
    uint16_t v_end_mm_s;
    uint32_t duration_us;
} gc_event;

/* Points are in the start frame: the board's axes at the stroke start. */
typedef struct gc_point {
    int16_t  x_mm;
    int16_t  y_mm;
    int16_t  z_mm;
    uint16_t t_100us;
} gc_point;

typedef struct gc_stroke_info {
    uint32_t duration_us;
    uint16_t points;
    uint16_t v_end_mm_s;
    int32_t  gravity_ug[3];
    uint16_t extent_mm;
    uint16_t step;
} gc_stroke_info;
// NOLINTEND(modernize-use-using)

/* Messages are required before C23. */
static_assert(sizeof(gc_sample) == 16, "gc_sample layout");
static_assert(sizeof(gc_config) == 68, "gc_config layout");
static_assert(sizeof(gc_event) == 12, "gc_event layout");
static_assert(sizeof(gc_point) == 8, "gc_point layout");
static_assert(sizeof(gc_stroke_info) == 24, "gc_stroke_info layout");
static_assert(sizeof(gc_state) == GC_STATE_SIZE, "gc_state size");
static_assert(alignof(gc_state) == GC_STATE_ALIGN, "gc_state alignment");

gc_status gc_init(gc_state *s, const gc_config *cfg);
gc_event  gc_feed(gc_state *s, const gc_sample *in);
uint16_t  gc_stroke(const gc_state *s, gc_stroke_info *info, gc_point *out, uint16_t cap);
uint16_t  gc_live(const gc_state *s, gc_point *out);

#ifdef __cplusplus
}
#endif

#endif /* GESTURE_CORE_H */
