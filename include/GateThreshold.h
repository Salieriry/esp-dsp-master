#pragma once

#include <math.h>

namespace gate_threshold {

static constexpr float MIN_LINEAR = 0.0001f;
static constexpr float MAX_LINEAR = 0.5f;
static constexpr float STEP_DB = 0.5f;

inline float apply_db_steps(float current, int delta) {
    if (!isfinite(current)) current = MIN_LINEAR;
    const float ratio = powf(10.0f, (float)delta * STEP_DB / 20.0f);
    const float updated = current * ratio;
    return fminf(fmaxf(updated, MIN_LINEAR), MAX_LINEAR);
}

} // namespace gate_threshold
