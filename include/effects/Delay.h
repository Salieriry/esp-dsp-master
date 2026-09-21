#pragma once
#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include "esp_heap_caps.h"
#include <string.h>

#define MAX_DELAY_SAMPLES 53000

class Delay {
private:
    float* delay_line = nullptr;
    int delay_index = 0;

public:
    bool begin() {
        delay_line = (float*)heap_caps_calloc(MAX_DELAY_SAMPLES, sizeof(float), MALLOC_CAP_SPIRAM);
        return delay_line != nullptr;
    }
    bool ready() const { return delay_line != nullptr; }
    inline void process(float* buffer, size_t frames, uint32_t delay_len, float feedback, float mix) {
        if (!delay_line) return;
        float fb = fminf(fmaxf(feedback, 0.0f), 0.95f);
        float dry_mix = 1.0f - mix;

        if (delay_len < 1) delay_len = 1;
        if (delay_len > MAX_DELAY_SAMPLES) delay_len = MAX_DELAY_SAMPLES;

        for (size_t i = 0; i < frames; i++) {
            if (delay_index >= (int)delay_len) delay_index = 0;

            float delayed_sample = delay_line[delay_index];
            if (delayed_sample > -1.0e-7f && delayed_sample < 1.0e-7f) delayed_sample = 0.0f; // Flush-to-zero

            delay_line[delay_index] = buffer[i] + (delayed_sample * fb);
            buffer[i] = (buffer[i] * dry_mix) + (delayed_sample * mix);
            delay_index++;
        }
    }
};
