/* core.h must compile as C (docs/core.md). Also pins every struct's alignment and field
 * offsets to the layout in docs/core.md; host/crates/gesture-core/src/ffi.rs checks the
 * same numbers on the Rust side. Sizes are asserted in core.h itself. */
#include "gesture/core.h"

#include <stddef.h>

#define OFF(type, field, off) static_assert(offsetof(type, field) == (off), #type "." #field)

static_assert(alignof(gc_sample) == 4, "gc_sample align");
OFF(gc_sample, t_us, 0);
OFF(gc_sample, gyro, 4);
OFF(gc_sample, accel, 10);

static_assert(alignof(gc_config) == 4, "gc_config align");
OFF(gc_config, abi_version, 0);
OFF(gc_config, ctrl1_xl, 4);
OFF(gc_config, ctrl2_g, 5);
OFF(gc_config, reserved0, 6);
OFF(gc_config, accel_offset_lsb, 8);
OFF(gc_config, reserved1, 14);
OFF(gc_config, accel_gain_q30, 16);
OFF(gc_config, gyro_bias_q8, 28);
OFF(gc_config, still_gyro_std_lsb, 40);
OFF(gc_config, still_accel_std_lsb, 42);
OFF(gc_config, still_gyro_rate_lsb, 44);
OFF(gc_config, hold_n, 46);
OFF(gc_config, bias_gyro_std_lsb, 48);
OFF(gc_config, bias_accel_std_lsb, 50);
OFF(gc_config, clip_lsb, 52);
OFF(gc_config, max_dt_us, 54);
OFF(gc_config, min_stroke_us, 56);
OFF(gc_config, max_stroke_us, 60);
OFF(gc_config, min_extent_mm, 64);
OFF(gc_config, max_v_end_mm_s, 66);

static_assert(alignof(gc_event) == 4, "gc_event align");
OFF(gc_event, kind, 0);
OFF(gc_event, state, 1);
OFF(gc_event, reason, 2);
OFF(gc_event, reserved, 3);
OFF(gc_event, points, 4);
OFF(gc_event, v_end_mm_s, 6);
OFF(gc_event, duration_us, 8);

static_assert(alignof(gc_point) == 2, "gc_point align");
OFF(gc_point, x_mm, 0);
OFF(gc_point, y_mm, 2);
OFF(gc_point, z_mm, 4);
OFF(gc_point, t_100us, 6);

static_assert(alignof(gc_stroke_info) == 4, "gc_stroke_info align");
OFF(gc_stroke_info, duration_us, 0);
OFF(gc_stroke_info, points, 4);
OFF(gc_stroke_info, v_end_mm_s, 6);
OFF(gc_stroke_info, gravity_ug, 8);
OFF(gc_stroke_info, extent_mm, 20);
OFF(gc_stroke_info, step, 22);
