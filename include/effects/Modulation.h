#pragma once
#include <math.h>
#include <stddef.h>

#define CHORUS_BUF_SIZE 2048

class Modulation {
private:
    float delay_buf[CHORUS_BUF_SIZE] = {0};
    int write_idx = 0;
    float lfo_phase = 0.0f;
    float sine_table[256];

public:
    Modulation() {
        for (int i = 0; i < 256; ++i)
            sine_table[i] = sinf(2.0f * M_PI * i / 256.0f);
    }
    inline void process(float* buffer, size_t frames, float rate_hz, float depth, float mix, float fs) {
        float lfo_inc = rate_hz / fs;
        float dry_mix = 1.0f - mix;

        for (size_t i = 0; i < frames; i++) {
            float in = buffer[i];
            delay_buf[write_idx] = in;

            // LFO triangular/senoidal modulando o tempo entre 2 ms e 15 ms
            lfo_phase += lfo_inc;
            if (lfo_phase >= 1.0f) lfo_phase -= 1.0f;

            float table_pos = lfo_phase * 256.0f;
            int phase_idx = (int)table_pos;
            float phase_frac = table_pos - phase_idx;
            float sine = sine_table[phase_idx] + phase_frac *
                (sine_table[(phase_idx + 1) & 255] - sine_table[phase_idx]);
            float mod_samples = (depth * 250.0f) * (0.5f + 0.5f * sine) + 88.0f;
            float read_pos = (float)write_idx - mod_samples;
            if (read_pos < 0.0f) read_pos += CHORUS_BUF_SIZE;

            // Interpolação Linear entre amostras fracionárias
            int idx0 = (int)read_pos;
            int idx1 = (idx0 + 1) % CHORUS_BUF_SIZE;
            float frac = read_pos - (float)idx0;

            float wet = delay_buf[idx0] + frac * (delay_buf[idx1] - delay_buf[idx0]);
            buffer[i] = (in * dry_mix) + (wet * mix);

            write_idx = (write_idx + 1) % CHORUS_BUF_SIZE;
        }
    }
};
