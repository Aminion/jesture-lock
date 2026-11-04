// The C ABI (gesture/core.h) over the algorithm in docs/core.md.
#include "arith.hpp"
#include "config.hpp"
#include "gesture/core.h"
#include "ring.hpp"
#include "state.hpp"
#include "still.hpp"
#include "stroke.hpp"

#include <algorithm>
#include <cstdint>
#include <iterator>
#include <new>

extern "C" gc_status gc_init(gc_state* s, const gc_config* cfg) {
    // Reset first: after an error the state is zeroed, not half-configured.
    gcore::State& st = *new (&s->opaque) gcore::State{};
    const gc_status status = gcore::validate(*cfg);
    if (status != GC_OK) {
        return status;
    }
    st.cfg = *cfg;
    st.sc = gcore::scales(*cfg);
    std::ranges::copy(cfg->gyro_bias_q8, st.bias_q8.begin());
    st.mode = GC_ST_WARMUP;
    return GC_OK;
}

namespace {

gc_event rejected(std::uint8_t reason) {
    gc_event ev{};
    ev.kind = GC_EV_REJECTED;
    ev.reason = reason;
    return ev;
}

// MOVING, after integrating this sample: too long, or the end hold (snapshot, end).
gc_event moving(gcore::State& st) {
    gcore::Stroke& k = st.stroke;
    if (k.t - k.t_s0 > st.cfg.max_stroke_us) {
        st.mode = GC_ST_WARMUP;
        return rejected(GC_REJ_LONG);
    }
    gc_event ev{};
    if (st.streak == 0) {
        k.snapped = false;
    } else if (st.streak == 1) {
        k.snap = {.v = k.v, .p = k.p, .t = k.t, .clipped = k.clipped};
        k.snapped = true;
    } else if (st.streak == st.cfg.hold_n) { // > 1: the snapshot is there
        const std::uint8_t reason = gcore::finish(k, st.cfg);
        st.mode = GC_ST_STILL;
        if (reason != 0) {
            ev = rejected(reason);
        } else {
            ev.kind = GC_EV_STROKE;
            ev.points = k.n;
        }
        ev.v_end_mm_s = k.v_end_mm_s;
        ev.duration_us = k.duration_us;
    }
    return ev;
}

} // namespace

extern "C" gc_event gc_feed(gc_state* s, const gc_sample* in) {
    gcore::State& st = gcore::state(s);
    gc_event ev{};

    // 1. Time. Unsigned subtraction: the MCU clock wraps.
    const std::uint32_t dt = in->t_us - st.t_prev;
    const bool gap = !st.fed || dt == 0 || dt > st.cfg.max_dt_us;
    st.fed = true;
    st.t_prev = in->t_us;
    if (gap) {
        gcore::clear(st.ring);
        st.streak = 0;
        if (st.mode == GC_ST_MOVING) {
            ev = rejected(GC_REJ_TIME_GAP);
        }
        st.mode = GC_ST_WARMUP;
    }

    // 2. Ring. After a gap dt is not a step; nothing reads it (the replay needs a hold).
    gcore::Raw raw{};
    raw.dt = gap ? std::uint16_t{0} : static_cast<std::uint16_t>(dt);
    std::ranges::copy(in->gyro, raw.gyro.begin());
    std::ranges::copy(in->accel, raw.accel.begin());
    gcore::push(st.ring, raw);

    // 3. Still window, 4. bias update.
    bool quiet = false;
    if (st.ring.count >= gcore::win_n) {
        const gcore::WindowSums w = gcore::window_sums(st.ring);
        quiet = gcore::quiet(w, st.cfg, st.bias_q8);
        st.streak = quiet ? gcore::satu16(st.streak + 1) : std::uint16_t{0};
        if (quiet && st.mode == GC_ST_STILL) {
            gcore::update_bias(st.bias_q8, w, st.cfg);
        }
    }

    // 5. State. STILL implies a full window (a gap goes to WARMUP).
    switch (st.mode) {
    case GC_ST_WARMUP:
        if (st.streak >= st.cfg.hold_n) {
            st.mode = GC_ST_STILL;
            ev.kind = GC_EV_READY;
        }
        break;
    case GC_ST_STILL:
        if (!quiet) {
            gcore::start(st.stroke, st.ring, st.cfg, st.sc, st.bias_q8, in->t_us);
            st.mode = GC_ST_MOVING;
            ev.kind = GC_EV_MOVING;
        }
        break;
    default: // GC_ST_MOVING
        gcore::integrate(st.stroke, gcore::at(st.ring, 0), st.cfg, st.sc, in->t_us);
        ev = moving(st);
        break;
    }
    ev.state = st.mode;
    return ev;
}

extern "C" std::uint16_t gc_stroke(const gc_state* s, gc_stroke_info* info, gc_point* out,
                                   std::uint16_t cap) {
    const gcore::Stroke& k = gcore::state(s).stroke;
    *info = gc_stroke_info{};
    if (gcore::state(s).mode == GC_ST_MOVING || !k.done) {
        return 0;
    }
    info->duration_us = k.duration_us;
    info->points = k.n;
    info->v_end_mm_s = k.v_end_mm_s;
    std::ranges::copy(k.gravity_ug, std::begin(info->gravity_ug));
    info->extent_mm = k.extent_mm;
    info->step = gcore::satu16(k.step);
    const std::uint16_t n = std::min<std::uint16_t>(k.n, cap);
    std::copy_n(k.points.begin(), n, out);
    return n;
}

extern "C" std::uint16_t gc_live(const gc_state* s, gc_point* out) {
    const gcore::State& st = gcore::state(s);
    if (st.mode != GC_ST_MOVING) {
        *out = gc_point{};
        return 0;
    }
    *out = gcore::point(st.stroke.p, st.stroke.t - st.stroke.t_s0);
    return 1;
}
