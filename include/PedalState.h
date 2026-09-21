#pragma once
#include <stdint.h>

struct __attribute__((packed)) PedalState {
    // 1. Noise Gate
    uint8_t  gate_active;
    float    gate_threshold;

    // 2. Compressor VCA
    uint8_t  comp_active;
    float    comp_threshold;      // 0.05f a 0.90f
    float    comp_ratio;          // 1.5f a 10.0f
    float    comp_makeup_gain;    // 1.0f a 4.0f

    // 3. Auto-Wah / Filtro Dinâmico
    uint8_t  wah_active;
    float    wah_sensitivity;     // Ganho do envelope
    float    wah_base_freq;       // Frequência base (ex: 350 Hz)
    float    wah_resonance;       // Fator Q do filtro

    // 4. Overdrive / Saturação
    uint8_t  dist_active;
    float    dist_drive;          // Ganho de entrada
    float    dist_tone;           // 0.0 escuro, 1.0 brilhante
    float    dist_level;          // Ganho de saída

    // 5. EQ Paramétrico de 4 Bandas
    uint8_t  eq_active;
    float    eq_low_gain;         // 100 Hz  (-12 dB a +12 dB)
    float    eq_mid1_gain;        // 500 Hz  (-12 dB a +12 dB)
    float    eq_mid2_gain;        // 1500 Hz (-12 dB a +12 dB)
    float    eq_high_gain;        // 4500 Hz (-12 dB a +12 dB)

    // 6. Modulação (Chorus / Flanger)
    uint8_t  mod_active;
    float    mod_rate_hz;         // 0.2 Hz a 5.0 Hz
    float    mod_depth;           // 0.0f a 1.0f
    float    mod_mix;             // 0.0f a 1.0f

    // 7. Tape Delay
    uint8_t  delay_active;
    uint32_t delay_time_samples;  // Atraso em amostras
    float    delay_feedback;      // Realimentação (0.0f a 0.95f)
    float    delay_mix;           // 0.0f a 1.0f

    // 8. Reverb Dattorro Plate
    uint8_t  reverb_active;
    float    reverb_decay;        // 0.1f a 0.95f
    float    reverb_damping;      // 0.0f a 0.8f
    float    reverb_mix;          // 0.0f a 1.0f

    // 9. Cab Sim / Convolução de IR (campos legados quando ENABLE_CAB_SIM=0)
    uint8_t  cab_active;
    uint8_t  cab_index;           // Índice do gabinete no array IR_PRESETS

    // 10. Bitcrusher (compilado mesmo quando o CAB está desativado)
    uint8_t  bitcrusher_active;
    uint8_t  bitcrusher_bits;
    uint8_t  bitcrusher_hold;
    float    bitcrusher_mix;

    // 11. Tremolo
    uint8_t  tremolo_active;
    float    tremolo_rate_hz;
    float    tremolo_depth;
    uint8_t  tremolo_shape;        // 0 = seno, 1 = triangular
    float    tremolo_mix;

    // 12. Phaser de quatro estágios
    uint8_t  phaser_active;
    float    phaser_rate_hz;
    float    phaser_depth;
    float    phaser_feedback;
    float    phaser_mix;
};

static_assert(sizeof(PedalState) == 140, "PedalState wire layout changed; update both boards");
