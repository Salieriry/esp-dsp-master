#pragma once
#include <math.h>
#include <stddef.h>
#include <stdint.h>

class NoiseGate {
private:
    float env = 0.0f;
    float gain = 1.0f;
    uint32_t transient_hold = 0;
    float cached_sample_rate = -1.0f;
    float attack = 0.0f;
    float release = 0.0f;

public:
    inline void process(float* buffer, size_t frames, float threshold,
                        float confidence = 1.0f, float sample_rate = 44100.0f) {
        threshold = fminf(fmaxf(threshold, 0.0001f), 0.5f);
        const float inverse_threshold = 1.0f / threshold;
        confidence = fminf(fmaxf(confidence, 0.0f), 1.0f);
        if (sample_rate != cached_sample_rate) {
            cached_sample_rate = sample_rate;
            attack = 1.0f - expf(-1.0f / (sample_rate * 0.005f));
            release = 1.0f - expf(-1.0f / (sample_rate * 0.200f));
        }
        for (size_t i = 0; i < frames; i++) {
            const float previous_env = env;
            const float sample = fabsf(buffer[i]);
            const float env_coeff = sample > env ? 0.01f : 0.001f;
            env += env_coeff * (sample - env);
            if (env < 1.0e-7f) env = 0.0f; // Flush-to-zero

            if (previous_env <= threshold && env > threshold)
                transient_hold = (uint32_t)(sample_rate * 0.030f);
            float knee = (env - threshold * 0.5f) * inverse_threshold;
            knee = fminf(fmaxf(knee, 0.0f), 1.0f);
            knee = knee * knee * (3.0f - 2.0f * knee);
            float effective_confidence = (transient_hold > 0) ? 1.0f : confidence;
            if (transient_hold > 0) --transient_hold;
            const float target = 0.02f + 0.98f * knee * effective_confidence;
            const float coefficient = target > gain ? attack : release;
            gain += coefficient * (target - gain);
            gain = fminf(fmaxf(gain, 0.02f), 1.0f);

            buffer[i] *= gain;
        }
    }
};
