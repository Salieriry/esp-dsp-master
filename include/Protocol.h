#pragma once
#include <stdint.h>
#include <string.h>
#include "PedalState.h"

// Identificador do tipo de mensagem dentro do quadro COBS
enum PacketType : uint8_t {
    PKT_PEDAL_STATE   = 0x01, // Placa A -> Placa B / PC (Espelho do Estado Oficial)
    PKT_AI_GATE       = 0x02, // Placa B -> Placa A (Confiança da IA: float 0.0 a 1.0)
    PKT_UI_PARAM_INC  = 0x03, // Placa B -> Placa A (Encoder girou: effect_idx, param_idx, delta)
    PKT_PRESET_ACTION = 0x04, // Placa B / PC -> Placa A (Salvar / Carregar preset)
    PKT_PRESET_RESULT = 0x05,
    PKT_AUDIO_ROUTE   = 0x06, // Placa A -> Placa B (rota DAC/USB/AMBOS)
    PKT_TUNER_STATE   = 0x07  // Placa B -> Placa A (silencia as saídas)
};

// Protocolo USB CDC da placa A. Os quadros também usam COBS + CRC8 e são
// terminados por zero. O cabeçalho permite que o software repita comandos sem
// aplicar a mesma alteração duas vezes.
static constexpr uint8_t CONTROL_PROTOCOL_VERSION = 2;
static constexpr uint8_t PEDAL_STATE_VERSION = 2;
// O bit 8 identifica a variante que inclui o convolutor de gabinete. Os
// efeitos 9--11 já ocupam os bits seguintes; manter esse mapa explícito
// permite que o capturador habilite o teste de IR somente quando a imagem o
// implementa.
static constexpr uint16_t EFFECT_CAP_CAB_SIM = 1u << 8;
static constexpr uint16_t EFFECT_CAP_BITCRUSHER = 1u << 9;
static constexpr uint16_t EFFECT_CAP_TREMOLO = 1u << 10;
static constexpr uint16_t EFFECT_CAP_PHASER = 1u << 11;

enum HostCommand : uint8_t {
    HOST_CMD_HELLO       = 0x10,
    HOST_CMD_GET_CAPS    = 0x11,
    HOST_CMD_GET_STATE   = 0x12,
    HOST_CMD_SET_PARAM   = 0x13,
    HOST_CMD_SET_ROUTE   = 0x14,
    HOST_CMD_PRESET      = 0x15,
    HOST_CMD_GET_METRICS = 0x16,
    HOST_CMD_PING        = 0x17,
    HOST_CMD_PROFILE     = 0x18,
};

enum HostResponse : uint8_t {
    HOST_RSP_ACK     = 0x80,
    HOST_RSP_CAPS    = 0x81,
    HOST_RSP_STATE   = 0x82,
    HOST_RSP_METRICS = 0x83,
    HOST_RSP_PROFILE = 0x84,
    HOST_RSP_ERROR   = 0xFF,
};

enum HostStatus : uint8_t {
    HOST_OK             = 0,
    HOST_ERR_BAD_VERSION = 1,
    HOST_ERR_BAD_LENGTH  = 2,
    HOST_ERR_BAD_VALUE   = 3,
    HOST_ERR_UNKNOWN_CMD = 4,
    HOST_ERR_BUSY        = 5,
};

enum AudioRoute : uint8_t {
    AUDIO_ROUTE_DAC  = 1,
    AUDIO_ROUTE_USB  = 2,
    AUDIO_ROUTE_BOTH = AUDIO_ROUTE_DAC | AUDIO_ROUTE_USB,
};

struct __attribute__((packed)) HostFrameHeader {
    uint8_t version;
    uint8_t type;
    uint16_t sequence;
};

struct __attribute__((packed)) HostSetParamPacket {
    uint8_t effect_idx;
    uint8_t param_idx;
    float value;
};

struct __attribute__((packed)) HostCapsPacket {
    uint32_t sample_rate;
    uint8_t adc_bits;
    uint8_t usb_bits;
    uint8_t usb_channels;
    uint8_t route;
    uint8_t shared_i2s_clocks;
    uint8_t protocol_version;
    uint8_t state_version;
    uint16_t effect_mask;
    uint16_t state_size;
};

struct __attribute__((packed)) HostMetricsPacket {
    uint32_t audio_io_errors;
    uint32_t i2s_drift_events;
    uint32_t slow_audio_blocks;
    uint32_t usb_overruns;
    uint32_t usb_underruns;
    uint32_t usb_control_overruns;
    uint32_t usb_control_tx_overruns;
    uint32_t usb_frames;
    uint32_t usb_stream_starts;
    uint32_t usb_stream_stops;
    uint32_t usb_cadence_errors;
    uint32_t max_audio_block_us;
    uint32_t max_cab_block_us;
    uint32_t cab_saturation_samples;
    uint8_t usb_mounted;
    uint8_t route;
    uint32_t max_effect_block_us;
    uint32_t dsp_quality_level;
    uint32_t usb_ring_frames;
    uint32_t usb_ring_high_water;
    uint32_t usb_resyncs;
    uint32_t usb_dropped_frames;
};

enum PresetAction : uint8_t {
    PRESET_LOAD = 0,
    PRESET_SAVE = 1
};

struct __attribute__((packed)) UiParamIncPacket {
    uint8_t effect_idx;
    uint8_t param_idx;
    int8_t  delta;
};

struct __attribute__((packed)) PresetPacket {
    uint8_t action; // 0 = Load, 1 = Save
    uint8_t slot;   // Slot de 0 a 15
};
struct __attribute__((packed)) PresetResultPacket {
    uint8_t action;
    uint8_t slot;
    uint8_t success;
};
struct __attribute__((packed)) AudioRoutePacket {
    uint8_t route;
};
struct __attribute__((packed)) TunerStatePacket {
    uint8_t active;
};

// Funções utilitárias COBS bidirecionais
inline size_t cobs_encode(const uint8_t *ptr, size_t length, uint8_t *dst) {
    size_t read_index = 0, write_index = 1, code_index = 0;
    uint8_t code = 1;
    while (read_index < length) {
        if (ptr[read_index] == 0) {
            dst[code_index] = code;
            code = 1;
            code_index = write_index++;
            read_index++;
        } else {
            dst[write_index++] = ptr[read_index++];
            code++;
            if (code == 0xFF) {
                dst[code_index] = code;
                code = 1;
                code_index = write_index++;
            }
        }
    }
    dst[code_index] = code;
    return write_index;
}

inline size_t cobs_decode(const uint8_t *ptr, size_t length, uint8_t *dst) {
    size_t read_index = 0, write_index = 0;
    uint8_t code, i;
    while (read_index < length) {
        code = ptr[read_index];
        if (code == 0 || read_index + code > length) return 0;
        read_index++;
        for (i = 1; i < code; i++) dst[write_index++] = ptr[read_index++];
        if (code != 0xFF && read_index != length) dst[write_index++] = '\0';
    }
    return write_index;
}

// Variant used for untrusted USB input. The legacy UART decoder has callers
// whose destination is known to be at least as large as the frame; USB frames
// are bounded explicitly before writing into their 255-byte scratch buffer.
inline size_t cobs_decode_bounded(const uint8_t *ptr, size_t length,
                                  uint8_t *dst, size_t capacity) {
    size_t read_index = 0, write_index = 0;
    while (read_index < length) {
        uint8_t code = ptr[read_index];
        if (code == 0 || read_index + code > length + 1) return 0;
        ++read_index;
        for (uint8_t i = 1; i < code; ++i) {
            if (read_index >= length || write_index >= capacity) return 0;
            dst[write_index++] = ptr[read_index++];
        }
        if (code != 0xFF && read_index != length) {
            if (write_index >= capacity) return 0;
            dst[write_index++] = 0;
        }
    }
    return write_index;
}

inline uint8_t packet_crc8(const uint8_t* data, size_t length) {
    uint8_t crc = 0;
    for (size_t i = 0; i < length; ++i) {
        crc ^= data[i];
        for (int bit = 0; bit < 8; ++bit)
            crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x07) : (uint8_t)(crc << 1);
    }
    return crc;
}

inline size_t encode_packet(const uint8_t* data, size_t length, uint8_t* dst) {
    if (length > 254) return 0;
    uint8_t payload[255];
    memcpy(payload, data, length);
    payload[length] = packet_crc8(data, length);
    return cobs_encode(payload, length + 1, dst);
}

inline size_t decode_packet(const uint8_t* src, size_t length, uint8_t* dst) {
    size_t decoded = cobs_decode(src, length, dst);
    if (decoded < 2 || packet_crc8(dst, decoded - 1) != dst[decoded - 1]) return 0;
    return decoded - 1;
}

inline size_t encode_host_frame(uint8_t type, uint16_t sequence,
                                const uint8_t* data, size_t length,
                                uint8_t* dst) {
    if (length > 245) return 0;
    uint8_t payload[255] = {};
    HostFrameHeader header{CONTROL_PROTOCOL_VERSION, type, sequence};
    memcpy(payload, &header, sizeof(header));
    if (length) memcpy(payload + sizeof(header), data, length);
    size_t encoded = encode_packet(payload, sizeof(header) + length, dst);
    if (encoded) dst[encoded++] = 0;
    return encoded;
}

inline bool decode_host_frame(const uint8_t* src, size_t length,
                              HostFrameHeader* header, uint8_t* data,
                              size_t data_capacity, size_t* data_length) {
    uint8_t decoded[255] = {};
    size_t decoded_length = cobs_decode_bounded(src, length, decoded, sizeof(decoded));
    if (decoded_length < 2 || packet_crc8(decoded, decoded_length - 1) != decoded[decoded_length - 1]) return false;
    size_t length_without_crc = decoded_length - 1;
    if (length_without_crc < sizeof(HostFrameHeader)) return false;
    memcpy(header, decoded, sizeof(*header));
    size_t payload_length = length_without_crc - sizeof(HostFrameHeader);
    if (payload_length > data_capacity) return false;
    if (payload_length) memcpy(data, decoded + sizeof(HostFrameHeader), payload_length);
    *data_length = payload_length;
    return true;
}
