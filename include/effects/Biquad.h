#pragma once
#include <math.h>

class Biquad {
private:
    float b0 = 1.0f, b1 = 0.0f, b2 = 0.0f, a1 = 0.0f, a2 = 0.0f;
    float x1 = 0.0f, x2 = 0.0f, y1 = 0.0f, y2 = 0.0f;
    float last_gain = -999.0f;

public:
    inline void setPeaking(float f0, float gain_db, float Q, float fs) {
        if (gain_db == last_gain) return;
        last_gain = gain_db;

        float A = powf(10.0f, gain_db / 40.0f);
        float w0 = 2.0f * M_PI * f0 / fs;
        float alpha = sinf(w0) / (2.0f * Q);

        float a0 = 1.0f + alpha / A;
        b0 = (1.0f + alpha * A) / a0;
        b1 = (-2.0f * cosf(w0)) / a0;
        b2 = (1.0f - alpha * A) / a0;
        a1 = (-2.0f * cosf(w0)) / a0;
        a2 = (1.0f - alpha / A) / a0;
    }

    inline float process(float in) {
        float out = (b0 * in) + (b1 * x1) + (b2 * x2) - (a1 * y1) - (a2 * y2);
        x2 = x1; x1 = in;
        y2 = y1; y1 = out;

        if (fabsf(y1) < 1.0e-7f) y1 = 0.0f;
        return out;
    }
};