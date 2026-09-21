#pragma once
#include <math.h>
#include <stddef.h>

class Overdrive {
    float tone_state = 0.0f;
    float cached_tone = -1.0f;
    float cached_alpha = 0.0f;
public:
    inline void process(float* buffer, size_t frames, float drive, float tone, float level) {
        if (tone != cached_tone) {
            float cutoff = 700.0f + tone * 7300.0f;
            cached_alpha = 1.0f - expf(-2.0f * M_PI * cutoff / 44100.0f);
            cached_tone = tone;
        }
        for (size_t i = 0; i < frames; i++) {
            float s = buffer[i] * drive;
            float clipped = s / (1.0f + fabsf(s));
            tone_state += cached_alpha * (clipped - tone_state);
            buffer[i] = tone_state * level;
        }
    }
};
