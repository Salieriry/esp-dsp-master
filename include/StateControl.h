#pragma once
#include <math.h>
#include "PedalState.h"
#include "AudioConfig.h"

inline PedalState defaultPedalState() {
    PedalState s = {};
    s.gate_active = 1;
    s.gate_threshold = 0.005f;
    s.comp_threshold = 0.4f;
    s.comp_ratio = 3.0f;
    s.comp_makeup_gain = 1.0f;
    s.wah_sensitivity = 0.5f;
    s.wah_base_freq = 350.0f;
    s.wah_resonance = 0.6f;
    s.dist_drive = 2.0f;
    s.dist_tone = 0.5f;
    s.dist_level = 0.8f;
    s.mod_rate_hz = 1.5f;
    s.mod_depth = 0.6f;
    s.mod_mix = 0.5f;
    s.delay_time_samples = 350 * SAMPLE_RATE / 1000;
    s.delay_feedback = 0.4f;
    s.delay_mix = 0.3f;
    s.reverb_decay = 0.5f;
    s.reverb_damping = 0.3f;
    s.reverb_mix = 0.25f;
    s.bitcrusher_active = 0;
    s.bitcrusher_bits = 8;
    s.bitcrusher_hold = 4;
    s.bitcrusher_mix = 0.5f;
    s.tremolo_active = 0;
    s.tremolo_rate_hz = 4.0f;
    s.tremolo_depth = 0.5f;
    s.tremolo_shape = 0;
    s.tremolo_mix = 1.0f;
    s.phaser_active = 0;
    s.phaser_rate_hz = 0.6f;
    s.phaser_depth = 0.7f;
    s.phaser_feedback = 0.3f;
    s.phaser_mix = 0.5f;
    return s;
}

inline bool validPedalState(const PedalState& s, unsigned ir_count) {
    return s.gate_active <= 1 && isfinite(s.gate_threshold) && s.gate_threshold >= 0.0001f && s.gate_threshold <= 0.5f &&
        s.comp_active <= 1 && isfinite(s.comp_threshold) && s.comp_threshold >= 0.05f && s.comp_threshold <= 0.95f &&
        isfinite(s.comp_ratio) && s.comp_ratio >= 1.0f && s.comp_ratio <= 10.0f &&
        isfinite(s.comp_makeup_gain) && s.comp_makeup_gain >= 1.0f && s.comp_makeup_gain <= 4.0f &&
        s.wah_active <= 1 && isfinite(s.wah_sensitivity) && s.wah_sensitivity >= 0.0f && s.wah_sensitivity <= 1.0f &&
        isfinite(s.wah_base_freq) && s.wah_base_freq >= 100.0f && s.wah_base_freq <= 1000.0f &&
        isfinite(s.wah_resonance) && s.wah_resonance >= 0.1f && s.wah_resonance <= 0.95f &&
        s.dist_active <= 1 && isfinite(s.dist_drive) && s.dist_drive >= 1.0f && s.dist_drive <= 20.0f &&
        isfinite(s.dist_tone) && s.dist_tone >= 0.0f && s.dist_tone <= 1.0f &&
        isfinite(s.dist_level) && s.dist_level >= 0.0f && s.dist_level <= 1.0f &&
        s.eq_active <= 1 && isfinite(s.eq_low_gain) && fabsf(s.eq_low_gain) <= 12.0f &&
        isfinite(s.eq_mid1_gain) && fabsf(s.eq_mid1_gain) <= 12.0f &&
        isfinite(s.eq_mid2_gain) && fabsf(s.eq_mid2_gain) <= 12.0f &&
        isfinite(s.eq_high_gain) && fabsf(s.eq_high_gain) <= 12.0f &&
        s.mod_active <= 1 && isfinite(s.mod_rate_hz) && s.mod_rate_hz >= 0.2f && s.mod_rate_hz <= 5.0f &&
        isfinite(s.mod_depth) && s.mod_depth >= 0.0f && s.mod_depth <= 1.0f &&
        isfinite(s.mod_mix) && s.mod_mix >= 0.0f && s.mod_mix <= 1.0f &&
        s.delay_active <= 1 && s.delay_time_samples >= SAMPLE_RATE / 50 && s.delay_time_samples <= SAMPLE_RATE * 12 / 10 &&
        isfinite(s.delay_feedback) && s.delay_feedback >= 0.0f && s.delay_feedback <= 0.95f &&
        isfinite(s.delay_mix) && s.delay_mix >= 0.0f && s.delay_mix <= 1.0f &&
        s.reverb_active <= 1 && isfinite(s.reverb_decay) && s.reverb_decay >= 0.1f && s.reverb_decay <= 0.95f &&
        isfinite(s.reverb_damping) && s.reverb_damping >= 0.0f && s.reverb_damping <= 0.8f &&
        isfinite(s.reverb_mix) && s.reverb_mix >= 0.0f && s.reverb_mix <= 1.0f &&
        s.cab_active <= 1 && s.cab_index < ir_count &&
        s.bitcrusher_active <= 1 && s.bitcrusher_bits >= 4 && s.bitcrusher_bits <= 16 &&
        s.bitcrusher_hold >= 1 && s.bitcrusher_hold <= 16 && isfinite(s.bitcrusher_mix) &&
        s.bitcrusher_mix >= 0.0f && s.bitcrusher_mix <= 1.0f &&
        s.tremolo_active <= 1 && isfinite(s.tremolo_rate_hz) && s.tremolo_rate_hz >= 0.1f && s.tremolo_rate_hz <= 12.0f &&
        isfinite(s.tremolo_depth) && s.tremolo_depth >= 0.0f && s.tremolo_depth <= 1.0f &&
        s.tremolo_shape <= 1 && isfinite(s.tremolo_mix) && s.tremolo_mix >= 0.0f && s.tremolo_mix <= 1.0f &&
        s.phaser_active <= 1 && isfinite(s.phaser_rate_hz) && s.phaser_rate_hz >= 0.05f && s.phaser_rate_hz <= 4.0f &&
        isfinite(s.phaser_depth) && s.phaser_depth >= 0.0f && s.phaser_depth <= 1.0f &&
        isfinite(s.phaser_feedback) && s.phaser_feedback >= 0.0f && s.phaser_feedback <= 0.7f &&
        isfinite(s.phaser_mix) && s.phaser_mix >= 0.0f && s.phaser_mix <= 1.0f;
}
