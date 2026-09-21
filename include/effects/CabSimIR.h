#pragma once

#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#define IR_TAPS 512
#define CAB_BLOCK_SIZE 64
#define CAB_FFT_SIZE (CAB_BLOCK_SIZE * 2)
#define CAB_PARTITIONS (IR_TAPS / CAB_BLOCK_SIZE)
#define CAB_MAX_IR_PRESETS 8

static_assert(IR_TAPS % CAB_BLOCK_SIZE == 0, "IR_TAPS must be a multiple of CAB_BLOCK_SIZE");

class CabSimIR {
private:
    struct Complex {
        float re;
        float im;
    };

    Complex input_spectra[CAB_PARTITIONS][CAB_FFT_SIZE] = {};
    Complex ir_spectra[CAB_MAX_IR_PRESETS][CAB_PARTITIONS][CAB_FFT_SIZE] = {};
    Complex work_spectrum[CAB_FFT_SIZE] = {};
    Complex fft_twiddles[CAB_FFT_SIZE / 2] = {};
    uint8_t bit_reverse[CAB_FFT_SIZE] = {};

    float input_block[CAB_BLOCK_SIZE] = {};
    float overlap[CAB_BLOCK_SIZE] = {};
    float output_block[CAB_BLOCK_SIZE] = {};
    size_t ir_count = 0;
    size_t newest_spectrum = 0;
    size_t input_pos = 0;
    size_t output_pos = 0;
    bool output_ready = false;
    bool fft_initialized = false;
    bool prepared = false;

    static Complex add(Complex a, Complex b) {
        return {a.re + b.re, a.im + b.im};
    }

    static Complex multiply(Complex a, Complex b) {
        return {a.re * b.re - a.im * b.im, a.re * b.im + a.im * b.re};
    }

    void init_fft_tables() {
        if (fft_initialized) return;
        for (size_t i = 0; i < CAB_FFT_SIZE; ++i) {
            uint8_t reversed = 0;
            size_t value = i;
            for (size_t bit = 0; bit < 7; ++bit) {
                reversed = (uint8_t)((reversed << 1) | (value & 1u));
                value >>= 1;
            }
            bit_reverse[i] = reversed;
        }
        for (size_t i = 0; i < CAB_FFT_SIZE / 2; ++i) {
            const float angle = -2.0f * (float)M_PI * (float)i / (float)CAB_FFT_SIZE;
            fft_twiddles[i] = {cosf(angle), sinf(angle)};
        }
        fft_initialized = true;
    }

    void fft(Complex* data, bool inverse) {
        for (size_t i = 0; i < CAB_FFT_SIZE; ++i) {
            const size_t j = bit_reverse[i];
            if (i < j) {
                Complex temporary = data[i];
                data[i] = data[j];
                data[j] = temporary;
            }
        }

        for (size_t length = 2; length <= CAB_FFT_SIZE; length <<= 1) {
            const size_t half = length >> 1;
            const size_t twiddle_step = CAB_FFT_SIZE / length;
            for (size_t start = 0; start < CAB_FFT_SIZE; start += length) {
                for (size_t j = 0; j < half; ++j) {
                    Complex twiddle = fft_twiddles[j * twiddle_step];
                    if (inverse) twiddle.im = -twiddle.im;
                    const Complex even = data[start + j];
                    const Complex odd = multiply(data[start + j + half], twiddle);
                    data[start + j] = add(even, odd);
                    data[start + j + half] = {even.re - odd.re, even.im - odd.im};
                }
            }
        }

        if (inverse) {
            const float scale = 1.0f / (float)CAB_FFT_SIZE;
            for (size_t i = 0; i < CAB_FFT_SIZE; ++i) {
                data[i].re *= scale;
                data[i].im *= scale;
            }
        }
    }

    void process_block(size_t ir_index) {
        newest_spectrum = (newest_spectrum + CAB_PARTITIONS - 1) % CAB_PARTITIONS;
        // Transforme o bloco diretamente no anel. Isso elimina uma cópia de
        // 1 KiB e evita alocar um espectro temporário na pilha da tarefa DSP.
        Complex* current = input_spectra[newest_spectrum];
        memset(current, 0, sizeof(input_spectra[newest_spectrum]));
        for (size_t i = 0; i < CAB_BLOCK_SIZE; ++i) current[i].re = input_block[i];
        fft(current, false);

        memset(work_spectrum, 0, sizeof(work_spectrum));
        for (size_t partition = 0; partition < CAB_PARTITIONS; ++partition) {
            const size_t history_index = (newest_spectrum + partition) % CAB_PARTITIONS;
            // Para sequências reais, os bins acima de Nyquist são o conjugado
            // dos bins inferiores. Calculamos somente 0..64 e reconstruímos a
            // metade superior antes da IFFT, reduzindo quase pela metade as
            // multiplicações complexas do produto de convolução.
            for (size_t bin = 0; bin <= CAB_BLOCK_SIZE; ++bin) {
                const Complex product = multiply(input_spectra[history_index][bin],
                                                 ir_spectra[ir_index][partition][bin]);
                work_spectrum[bin].re += product.re;
                work_spectrum[bin].im += product.im;
            }
        }
        for (size_t bin = 1; bin < CAB_BLOCK_SIZE; ++bin) {
            work_spectrum[CAB_FFT_SIZE - bin].re = work_spectrum[bin].re;
            work_spectrum[CAB_FFT_SIZE - bin].im = -work_spectrum[bin].im;
        }
        fft(work_spectrum, true);

        for (size_t i = 0; i < CAB_BLOCK_SIZE; ++i) {
            output_block[i] = work_spectrum[i].re + overlap[i];
            overlap[i] = work_spectrum[i + CAB_BLOCK_SIZE].re;
        }
    }

public:
    // Generate all spectra during startup, outside the real-time task.
    bool prepare(const float* const* tables, size_t count) {
        if (!tables || count == 0 || count > CAB_MAX_IR_PRESETS) return false;
        prepared = false;
        ir_count = 0;
        init_fft_tables();
        memset(input_spectra, 0, sizeof(input_spectra));
        memset(ir_spectra, 0, sizeof(ir_spectra));
        for (size_t index = 0; index < count; ++index) {
            if (!tables[index]) return false;
            for (size_t partition = 0; partition < CAB_PARTITIONS; ++partition) {
                for (size_t i = 0; i < CAB_BLOCK_SIZE; ++i) {
                    const float coefficient = tables[index][partition * CAB_BLOCK_SIZE + i];
                    if (!isfinite(coefficient)) return false;
                    ir_spectra[index][partition][i].re = coefficient;
                }
                fft(ir_spectra[index][partition], false);
            }
        }
        ir_count = count;
        prepared = true;
        reset();
        return true;
    }

    void reset() {
        memset(input_spectra, 0, sizeof(input_spectra));
        memset(input_block, 0, sizeof(input_block));
        memset(overlap, 0, sizeof(overlap));
        memset(output_block, 0, sizeof(output_block));
        newest_spectrum = 0;
        input_pos = 0;
        output_pos = 0;
        output_ready = false;
    }

    bool ready() const { return prepared; }

    // Carrega o caminho de FFT e cada tabela no cache antes da tarefa de áudio
    // começar. A primeira chamada em uma IR fria pode sofrer uma falta de
    // cache longa e não deve ser confundida com custo sustentado do efeito.
    void warmup() {
        if (!prepared) return;
        float input[CAB_BLOCK_SIZE] = {};
        float output[CAB_BLOCK_SIZE] = {};
        for (size_t ir = 0; ir < ir_count; ++ir) {
            process(input, output, CAB_BLOCK_SIZE, ir);
            reset();
        }
    }

    // The partitioned convolution introduces one CAB_BLOCK_SIZE sample block
    // of latency, keeping the real-time cost bounded.
    void process(const float* in, float* out, size_t frames, size_t ir_index) {
        if (!in || !out || !prepared || ir_index >= ir_count) {
            if (in && out) memcpy(out, in, frames * sizeof(float));
            else if (out) memset(out, 0, frames * sizeof(float));
            return;
        }
        for (size_t i = 0; i < frames; ++i) {
            out[i] = output_ready ? output_block[output_pos++] : 0.0f;
            input_block[input_pos++] = in[i];

            if (output_ready && output_pos == CAB_BLOCK_SIZE) {
                output_pos = 0;
                output_ready = false;
            }
            if (input_pos == CAB_BLOCK_SIZE) {
                input_pos = 0;
                process_block(ir_index);
                output_pos = 0;
                output_ready = true;
            }
        }
    }
};
