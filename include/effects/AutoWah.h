#pragma once
#include <math.h>
#include <stddef.h>

class AutoWah {
private:
    float s1 = 0.0f, s2 = 0.0f;
    float env = 0.0f;

public:
    inline void process(float* buffer, size_t frames, float sensitivity, float base_freq, float resonance, float fs) {
        const float pi_over_fs = M_PI / fs;
        for (size_t i = 0; i < frames; i++) {
            float sample = buffer[i];
            env = (0.98f * env) + (0.02f * fabsf(sample));
            if (env < 1.0e-7f) env = 0.0f;

            // Modulação da frequência de corte de acordo com a intensidade da palhetada
            float f_cutoff = base_freq + (env * sensitivity * 3000.0f);
            if (f_cutoff > 5000.0f) f_cutoff = 5000.0f;

            // Filtro State-Variable (Chamberlin SVF)
            float angle = f_cutoff * pi_over_fs;
            float f = 2.0f * (angle - angle * angle * angle * (1.0f / 6.0f));
            float q = resonance;

            float hp = sample - s1 * q - s2;
            float bp = hp * f + s1;
            float lp = bp * f + s2;

            s1 = bp;
            s2 = lp;

            if (fabsf(s1) < 1.0e-7f) s1 = 0.0f;
            if (fabsf(s2) < 1.0e-7f) s2 = 0.0f;

            // Mistura: sinal passa-faixa acentuado somado ao dry
            buffer[i] = (sample * 0.3f) + (bp * 0.7f);
        }
    }
};
