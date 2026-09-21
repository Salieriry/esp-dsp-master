#pragma once
#include <stdio.h>
#include <stddef.h>
#include <string.h>
#include "nvs_flash.h"
#include "nvs.h"
#include "PedalState.h"
#include "esp_log.h"

class PresetManager {
private:
    static constexpr const char* TAG = "PRESET_MGR";
    static constexpr const char* NVS_NAMESPACE = "pedal_storage";
    static constexpr uint32_t MAGIC = 0x5045444c;
    static constexpr uint16_t RECORD_VERSION = 2;
    static constexpr size_t LEGACY_STATE_SIZE = offsetof(PedalState, bitcrusher_active);
    struct __attribute__((packed)) Record {
        uint32_t magic;
        uint16_t version;
        uint16_t length;
        PedalState state;
        uint32_t crc;
    };
    struct __attribute__((packed)) LegacyRecord {
        uint32_t magic;
        uint16_t version;
        uint16_t length;
        uint8_t state[LEGACY_STATE_SIZE];
        uint32_t crc;
    };
    static uint32_t crc32(const uint8_t* data, size_t length) {
        uint32_t crc = 0xffffffffu;
        for (size_t i = 0; i < length; ++i) {
            crc ^= data[i];
            for (int bit = 0; bit < 8; ++bit)
                crc = (crc >> 1) ^ ((crc & 1) ? 0xedb88320u : 0u);
        }
        return ~crc;
    }

public:
    static void init() {
        esp_err_t err = nvs_flash_init();
        if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
            ESP_ERROR_CHECK(nvs_flash_erase());
            err = nvs_flash_init();
        }
        ESP_ERROR_CHECK(err);
        ESP_LOGI(TAG, "Partição NVS inicializada.");
    }

    static bool savePreset(uint8_t slot, const PedalState& state) {
        if (slot >= 16) return false;
        nvs_handle_t handle;
        if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle) != ESP_OK) return false;

        char key[16];
        snprintf(key, sizeof(key), "prst_%u", slot);

        Record record = {MAGIC, RECORD_VERSION, sizeof(PedalState), state, 0};
        record.crc = crc32((const uint8_t*)&record, offsetof(Record, crc));
        esp_err_t err = nvs_set_blob(handle, key, &record, sizeof(record));
        if (err == ESP_OK) {
            err = nvs_commit(handle);
            if (err == ESP_OK) ESP_LOGI(TAG, "Preset salvo no slot %u com sucesso.", slot);
        }
        nvs_close(handle);
        return (err == ESP_OK);
    }

    static bool loadPreset(uint8_t slot, PedalState& state) {
        if (slot >= 16) return false;
        nvs_handle_t handle;
        if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &handle) != ESP_OK) return false;

        char key[16];
        snprintf(key, sizeof(key), "prst_%u", slot);

        size_t required_size = 0;
        esp_err_t err = nvs_get_blob(handle, key, NULL, &required_size);
        uint8_t data[sizeof(Record)] = {};
        if (err == ESP_OK && required_size <= sizeof(data))
            err = nvs_get_blob(handle, key, data, &required_size);
        nvs_close(handle);

        if (err == ESP_OK && required_size == sizeof(Record)) {
            Record record;
            memcpy(&record, data, sizeof(record));
            if (record.magic != MAGIC || record.version != RECORD_VERSION ||
                record.length != sizeof(PedalState) ||
                record.crc != crc32(data, offsetof(Record, crc))) return false;
            state = record.state;
        } else if (err == ESP_OK && required_size == sizeof(LegacyRecord)) {
            LegacyRecord record;
            memcpy(&record, data, sizeof(record));
            if (record.magic != MAGIC || record.version != 1 || record.length != LEGACY_STATE_SIZE ||
                record.crc != crc32(data, offsetof(LegacyRecord, crc))) return false;
            state = {};
            memcpy(&state, record.state, LEGACY_STATE_SIZE);
            state.bitcrusher_active = 0;
            state.bitcrusher_bits = 8;
            state.bitcrusher_hold = 4;
            state.bitcrusher_mix = 0.5f;
            state.tremolo_active = 0;
            state.tremolo_rate_hz = 4.0f;
            state.tremolo_depth = 0.5f;
            state.tremolo_shape = 0;
            state.tremolo_mix = 1.0f;
            state.phaser_active = 0;
            state.phaser_rate_hz = 0.6f;
            state.phaser_depth = 0.7f;
            state.phaser_feedback = 0.3f;
            state.phaser_mix = 0.5f;
        } else if (err == ESP_OK && required_size == LEGACY_STATE_SIZE) {
            state = {};
            memcpy(&state, data, LEGACY_STATE_SIZE); // Estado bruto da versão anterior.
            state.bitcrusher_active = 0;
            state.bitcrusher_bits = 8;
            state.bitcrusher_hold = 4;
            state.bitcrusher_mix = 0.5f;
            state.tremolo_active = 0;
            state.tremolo_rate_hz = 4.0f;
            state.tremolo_depth = 0.5f;
            state.tremolo_shape = 0;
            state.tremolo_mix = 1.0f;
            state.phaser_active = 0;
            state.phaser_rate_hz = 0.6f;
            state.phaser_depth = 0.7f;
            state.phaser_feedback = 0.3f;
            state.phaser_mix = 0.5f;
        } else if (err == ESP_OK && required_size == LEGACY_STATE_SIZE - sizeof(float)) {
            // Migra a versão sem o controle Tone do overdrive.
            constexpr size_t prefix = offsetof(PedalState, dist_tone);
            state = {};
            memcpy(&state, data, prefix);
            state.dist_tone = 0.5f;
            memcpy((uint8_t*)&state + prefix + sizeof(float), data + prefix,
                   LEGACY_STATE_SIZE - prefix - sizeof(float));
            state.bitcrusher_active = 0;
            state.bitcrusher_bits = 8;
            state.bitcrusher_hold = 4;
            state.bitcrusher_mix = 0.5f;
            state.tremolo_active = 0;
            state.tremolo_rate_hz = 4.0f;
            state.tremolo_depth = 0.5f;
            state.tremolo_shape = 0;
            state.tremolo_mix = 1.0f;
            state.phaser_active = 0;
            state.phaser_rate_hz = 0.6f;
            state.phaser_depth = 0.7f;
            state.phaser_feedback = 0.3f;
            state.phaser_mix = 0.5f;
            if (state.gate_threshold == 0.0f) state.gate_threshold = 0.005f;
            if (state.comp_threshold == 0.0f) state.comp_threshold = 0.4f;
            if (state.comp_ratio == 0.0f) state.comp_ratio = 3.0f;
            if (state.comp_makeup_gain == 0.0f) state.comp_makeup_gain = 1.0f;
            if (state.wah_base_freq == 0.0f) state.wah_base_freq = 350.0f;
            if (state.wah_resonance == 0.0f) state.wah_resonance = 0.6f;
            if (state.dist_drive == 0.0f) state.dist_drive = 2.0f;
            if (state.mod_rate_hz == 0.0f) state.mod_rate_hz = 1.5f;
            if (state.delay_time_samples == 0) state.delay_time_samples = 350 * 44100 / 1000;
            if (state.reverb_decay == 0.0f) state.reverb_decay = 0.5f;
        } else {
            ESP_LOGW(TAG, "Slot %u vazio ou corrompido.", slot);
            return false;
        }
        ESP_LOGI(TAG, "Preset do slot %u carregado.", slot);
        return true;
    }
};
