#pragma once
#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include "esp_heap_caps.h"

// Plate mono inspirado em Dattorro: quatro difusores de entrada e tanque de
// duas linhas cruzadas. A saída soma os ramos; taps modulados e saída estéreo
// do artigo original não são implementados nesta versão mono.
template<int N> class ReverbDelay {
    float* data = nullptr;
    int position = 0;
public:
    bool begin(bool internal = false) {
        uint32_t caps = (internal ? MALLOC_CAP_INTERNAL : MALLOC_CAP_SPIRAM) | MALLOC_CAP_8BIT;
        data = (float*)heap_caps_calloc(N, sizeof(float), caps);
        return data != nullptr;
    }
    void release() { if (data) heap_caps_free(data); data = nullptr; }
    void push(float value) {
        // Comparações diretas evitam uma chamada a fabsf em cada amostra.
        data[position] = (value > -1.0e-8f && value < 1.0e-8f) ? 0.0f : value;
        if (++position == N) position = 0;
    }
    float process(float value) {
        float delayed = data[position];
        push(value);
        return delayed;
    }
    float allpass(float value, float coefficient) {
        float delayed = data[position];
        float output = delayed - coefficient * value;
        push(value + coefficient * output);
        return output;
    }
};

class ReverbDattorro {
    ReverbDelay<210> in1;
    ReverbDelay<159> in2;
    ReverbDelay<562> in3;
    ReverbDelay<411> in4;
    ReverbDelay<996> tank_a1;
    ReverbDelay<1346> tank_b1;
    ReverbDelay<6599> tank_a_delay1;
    ReverbDelay<5500> tank_b_delay1;
    ReverbDelay<2667> tank_a2;
    ReverbDelay<3937> tank_b2;
    ReverbDelay<6250> tank_a_delay2;
    ReverbDelay<4688> tank_b_delay2;
    float feedback_a = 0.0f, feedback_b = 0.0f;
    float lowpass_a = 0.0f, lowpass_b = 0.0f;
    float input_lowpass = 0.0f;

public:
    bool begin() {
        // As linhas curtas são acessadas a cada amostra. Mantê-las na RAM
        // interna reduz a latência do PSRAM; as linhas longas continuam no
        // PSRAM para não consumir o pequeno heap interno.
        ready_ = allocate_buffers(true);
        if (!ready_) {
            release_buffers();
            // Fallback para placas com pouca RAM interna disponível.
            ready_ = allocate_buffers(false);
        }
        if (!ready_) release_buffers();
        return ready_;
    }
    bool ready() const { return ready_; }
    void process(float* buffer, size_t frames, float decay, float damp, float mix) {
        if (!ready_) return;
        decay = fminf(fmaxf(decay, 0.1f), 0.95f);
        damp = fminf(fmaxf(damp, 0.0f), 0.8f);
        mix = fminf(fmaxf(mix, 0.0f), 1.0f);
        for (size_t i = 0; i < frames; ++i) {
            float dry = buffer[i];
            input_lowpass += 0.8f * (dry - input_lowpass);
            float input = in4.allpass(in3.allpass(in2.allpass(
                in1.allpass(input_lowpass, 0.75f), 0.75f), 0.625f), 0.625f);

            // A realimentação de cada lado usa a saída anterior do outro lado.
            float a = tank_a1.allpass(input + decay * feedback_b, -0.7f);
            float b = tank_b1.allpass(input + decay * feedback_a, -0.7f);
            a = tank_a_delay1.process(a);
            b = tank_b_delay1.process(b);
            lowpass_a += (1.0f - damp) * (a - lowpass_a);
            lowpass_b += (1.0f - damp) * (b - lowpass_b);
            a = tank_a2.allpass(decay * lowpass_a, 0.5f);
            b = tank_b2.allpass(decay * lowpass_b, 0.5f);
            feedback_a = tank_a_delay2.process(a);
            feedback_b = tank_b_delay2.process(b);
            if (feedback_a > -1.0e-8f && feedback_a < 1.0e-8f) feedback_a = 0.0f;
            if (feedback_b > -1.0e-8f && feedback_b < 1.0e-8f) feedback_b = 0.0f;
            float wet = 0.5f * (a + b + feedback_a + feedback_b);
            buffer[i] = dry * (1.0f - mix) + wet * mix;
        }
    }
private:

    bool allocate_buffers(bool short_lines_internal) {
        const bool ok = in1.begin(true) && in2.begin(true) && in3.begin(true) && in4.begin(true) &&
            tank_a1.begin(short_lines_internal) && tank_b1.begin(short_lines_internal) &&
            tank_a_delay1.begin() && tank_b_delay1.begin() &&
            tank_a2.begin(short_lines_internal) && tank_b2.begin(short_lines_internal) &&
            tank_a_delay2.begin() && tank_b_delay2.begin();
        if (!ok) release_buffers();
        return ok;
    }

    void release_buffers() {
        in1.release(); in2.release(); in3.release(); in4.release();
        tank_a1.release(); tank_b1.release(); tank_a_delay1.release(); tank_b_delay1.release();
        tank_a2.release(); tank_b2.release(); tank_a_delay2.release(); tank_b_delay2.release();
    }

    bool ready_ = false;
};
