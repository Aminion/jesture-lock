// The C ABI (gesture/core.h) over the algorithm in docs/core.md.
#include "config.hpp"
#include "gesture/core.h"
#include "state.hpp"

#include <algorithm>
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
