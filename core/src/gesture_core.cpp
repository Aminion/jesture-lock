// The C ABI (gesture/core.h) over the algorithm in docs/core.md.
#include "arith.hpp"
#include "config.hpp"
#include "gesture/core.h"
#include "ring.hpp"
#include "state.hpp"
#include "still.hpp"

#include <algorithm>
#include <cstdint>
#include <new>

extern "C" gc_status gc_init(gc_state* s, const gc_config* cfg) {
    // Reset first: after an error the state is zeroed, not half-configured.
    gcore::State& st = *new (&s->opaque) gcore::State{};
    const gc_status status = gcore::validate(*cfg);
    if (status != GC_OK) {
        return status;
    }
    st.cfg = *cfg;
    st.sa = gcore::accel_ug_per_lsb(cfg->ctrl1_xl);
    st.sg = gcore::gyro_udps_per_lsb(cfg->ctrl2_g);
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
            // TODO(3c.1 step 3): stroke start, gravity window and replay.
            st.mode = GC_ST_MOVING;
            ev.kind = GC_EV_MOVING;
        }
        break;
    default: // GC_ST_MOVING
        // TODO(3c.1 steps 4, 5): integrate, LONG, snapshot, stroke end.
        break;
    }
    ev.state = st.mode;
    return ev;
}
