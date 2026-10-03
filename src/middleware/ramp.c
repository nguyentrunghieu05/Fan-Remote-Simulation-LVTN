#include "ramp.h"

void ramp_init(ramp_t *r, float start, float rate_per_s)
{
    r->value      = start;
    r->rate_per_s = rate_per_s;
}

float ramp_step(ramp_t *r, float target, float dt_s)
{
    float max_step = r->rate_per_s * dt_s;
    float diff     = target - r->value;
    if (diff > max_step) {
        diff = max_step;
    } else if (diff < -max_step) {
        diff = -max_step;
    }
    r->value += diff;
    return r->value;
}
