#pragma once
#include <math.h>
#include <stddef.h>
#include <stdint.h>

class Phaser {
    float state_[4] = {};
    float feedback_state_ = 0.0f;
    float phase_ = 0.0f;
    float rate_state_ = 0.6f;
    float depth_state_ = 0.7f;
    float feedback_state_param_ = 0.3f;
    float sine_[256] = {};
    float wet_ = 0.0f;

public:
    Phaser() {
        for (int i = 0; i < 256; ++i) sine_[i] = sinf(2.0f * (float)M_PI * (float)i / 256.0f);
    }
    void reset() { for (float& x : state_) x = 0.0f; feedback_state_ = 0.0f; phase_ = 0.0f; rate_state_ = 0.6f; depth_state_ = 0.7f; feedback_state_param_ = 0.3f; wet_ = 0.0f; }
    void process(float* buffer, size_t frames, float rate, float depth, float feedback,
                 float mix, float sample_rate, bool enabled = true) {
        rate = fminf(fmaxf(rate, 0.05f), 4.0f);
        depth = fminf(fmaxf(depth, 0.0f), 1.0f);
        feedback = fminf(fmaxf(feedback, 0.0f), 0.7f);
        mix = fminf(fmaxf(mix, 0.0f), 1.0f);
        if (!enabled && wet_ < 0.0005f) {
            wet_ = 0.0f;
            return;
        }
        for (size_t i = 0; i < frames; ++i) {
            wet_ += 0.0015f * ((enabled ? mix : 0.0f) - wet_);
            rate_state_ += 0.002f * (rate - rate_state_);
            depth_state_ += 0.002f * (depth - depth_state_);
            feedback_state_param_ += 0.002f * (feedback - feedback_state_param_);
            const float table_pos = phase_ * 256.0f;
            const int table_index = (int)table_pos & 255;
            const float table_frac = table_pos - (float)(int)table_pos;
            const float lfo_sine = sine_[table_index] + table_frac *
                (sine_[(table_index + 1) & 255] - sine_[table_index]);
            const float lfo = 0.5f + 0.5f * lfo_sine;
            const float coefficient = 0.08f + 0.86f * depth_state_ * lfo;
            float x = buffer[i] + feedback_state_ * feedback_state_param_;
            for (int stage = 0; stage < 4; ++stage) {
                const float y = -coefficient * x + state_[stage];
                state_[stage] = x + coefficient * y;
                x = y;
            }
            feedback_state_ = x;
            buffer[i] += (x - buffer[i]) * wet_;
            phase_ += rate_state_ / sample_rate;
            if (phase_ >= 1.0f) phase_ -= 1.0f;
        }
    }
};
