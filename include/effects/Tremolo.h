#pragma once
#include <math.h>
#include <stddef.h>
#include <stdint.h>

class Tremolo {
    float phase_ = 0.0f;
    float rate_state_ = 4.0f;
    float depth_state_ = 0.5f;
    uint8_t shape_ = 0xff;
    float sine_table_[256] = {};
    float triangle_table_[256] = {};
    float wet_ = 0.0f;

    void build(uint8_t shape) {
        shape_ = shape > 1 ? 0 : shape;
    }

public:
    Tremolo() {
        for (int i = 0; i < 256; ++i) {
            const float p = (float)i / 256.0f;
            sine_table_[i] = 0.5f + 0.5f * sinf(2.0f * (float)M_PI * p);
            triangle_table_[i] = p < 0.5f ? p * 2.0f : 2.0f - p * 2.0f;
        }
        build(0);
    }
    void reset() { phase_ = 0.0f; rate_state_ = 4.0f; depth_state_ = 0.5f; wet_ = 0.0f; }
    void process(float* buffer, size_t frames, float rate, float depth, uint8_t shape,
                 float mix, float sample_rate, bool enabled = true) {
        rate = fminf(fmaxf(rate, 0.1f), 12.0f);
        depth = fminf(fmaxf(depth, 0.0f), 1.0f);
        mix = fminf(fmaxf(mix, 0.0f), 1.0f);
        if (shape != shape_) build(shape);
        const float* table = shape_ == 0 ? sine_table_ : triangle_table_;
        for (size_t i = 0; i < frames; ++i) {
            wet_ += 0.0015f * ((enabled ? mix : 0.0f) - wet_);
            rate_state_ += 0.002f * (rate - rate_state_);
            depth_state_ += 0.002f * (depth - depth_state_);
            const float pos = phase_ * 256.0f;
            const int index = (int)pos & 255;
            const float frac = pos - (float)(int)pos;
            const float lfo = table[index] + frac * (table[(index + 1) & 255] - table[index]);
            const float gain = 1.0f - depth_state_ * lfo;
            const float wet = buffer[i] * gain;
            buffer[i] += (wet - buffer[i]) * wet_;
            phase_ += rate_state_ / sample_rate;
            if (phase_ >= 1.0f) phase_ -= 1.0f;
        }
    }
};
