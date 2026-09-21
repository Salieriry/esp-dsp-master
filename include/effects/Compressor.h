#pragma once
#include <math.h>
#include <stddef.h>

class Compressor {
private:
    float env = 0.0f;

public:
    inline void process(float* buffer, size_t frames, float threshold, float ratio, float makeup) {
        // Ratio e makeup permanecem constantes durante o bloco. Evite uma
        // divisão de ponto flutuante em cada amostra.
        const float inverse_ratio = 1.0f / fmaxf(ratio, 1.0f);
        const float makeup_gain = makeup;
        for (size_t i = 0; i < frames; i++) {
            float in_level = fabsf(buffer[i]);
            // Rastreamento de envelope com attack/release contínuo
            if (in_level > env) env = (0.9f * env) + (0.1f * in_level);
            else                env = (0.999f * env) + (0.001f * in_level);

            if (env < 1.0e-7f) env = 0.0f;

            float gain = 1.0f;
            if (env > threshold && env > 1.0e-6f) {
                float excess = env - threshold;
                gain = (threshold + (excess * inverse_ratio)) / env;
            }

            buffer[i] *= (gain * makeup_gain);
        }
    }
};
