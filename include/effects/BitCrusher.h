#pragma once
#include <math.h>
#include <stddef.h>
#include <stdint.h>

class BitCrusher {
    float held_ = 0.0f;
    uint8_t counter_ = 0;
    uint8_t bits_ = 8;
    uint8_t hold_ = 4;
    float mix_ = 0.5f;
    float wet_ = 0.0f;

    void configure(uint8_t bits, uint8_t hold, float mix) {
        bits_ = bits < 4 ? 4 : (bits > 16 ? 16 : bits);
        hold_ = hold < 1 ? 1 : (hold > 16 ? 16 : hold);
        mix_ = fminf(fmaxf(mix, 0.0f), 1.0f);
        counter_ = 0;
    }

public:
    void reset() { held_ = 0.0f; counter_ = 0; wet_ = 0.0f; }

    void process(float* buffer, size_t frames, uint8_t bits, uint8_t hold, float mix, bool enabled = true) {
        if (bits != bits_ || hold != hold_ || mix != mix_) configure(bits, hold, mix);
        if (!enabled && wet_ < 0.0005f) {
            wet_ = 0.0f;
            return;
        }
        const float levels = (float)(1u << (bits_ - 1));
        for (size_t i = 0; i < frames; ++i) {
            const float target_wet = enabled ? mix_ : 0.0f;
            wet_ += 0.0015f * (target_wet - wet_);
            const float dry = 1.0f - wet_;
            // O ramo seco deve conservar o nível produzido pelos efeitos
            // anteriores. Somente a cópia que entra na quantização é limitada
            // ao intervalo representável pelo bitcrusher.
            const float dry_input = isfinite(buffer[i]) ? buffer[i] : 0.0f;
            const float input = fminf(fmaxf(dry_input, -1.0f), 1.0f);
            if (counter_ == 0) {
                held_ = roundf(input * levels) / levels;
                counter_ = (uint8_t)(hold_ - 1);
            } else {
                --counter_;
            }
            buffer[i] = dry_input * dry + held_ * wet_;
        }
    }
};
