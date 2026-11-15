#include <stdio.h>
#include <string.h>
#include <math.h>
#include <atomic>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "driver/i2s_std.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_task_wdt.h"
#include "esp_timer.h"
#include "esp_err.h"
#include "esp_memory_utils.h"
#include "../../common/ProfileMetrics.h"
#include "esp_cpu.h"
#include "esp_heap_caps.h"
#include "esp_idf_version.h"
#if ESP_IDF_VERSION_MAJOR >= 5
#include "esp_private/esp_clk.h"
#else
#include "esp_clk.h"
#endif
#include "../../common/ProfileProtocol.h"
#include "../../common/ProfileCycleCounter.h"
#include "DmaDriverMetrics.h"
#include "ProfileStages.h"
#include "AudioConfig.h"

#if ENABLE_USB_AUDIO
#include "tusb_config.h"
#include "usb_device_uac.h"
#include "tusb.h"
#include "class/cdc/cdc_device.h"
#endif

// Configurações, Protocolo e Catálogo de IRs
#include "PedalState.h"
#include "Protocol.h"
#include "PresetManager.h"
#include "StateControl.h"
#include "CdcMutationCache.h"
#include "GateThreshold.h"
#include "irs/ir_index.h"

#ifndef METRICS_FIRMWARE_ID
#define METRICS_FIRMWARE_ID "not-a-metrics-build"
#endif

// Motores Modulares de Efeito
#include "effects/NoiseGate.h"
#include "effects/Compressor.h"
#include "effects/AutoWah.h"
#include "effects/Overdrive.h"
#include "effects/Biquad.h"
#include "effects/Modulation.h"
#include "effects/Delay.h"
#include "effects/ReverbDattorro.h"
#include "effects/BitCrusher.h"
#include "effects/Tremolo.h"
#include "effects/Phaser.h"
#if ENABLE_CAB_SIM
#include "effects/CabSimIR.h"
#endif

#ifndef ENABLE_SERIAL_DEBUG
#define ENABLE_SERIAL_DEBUG 1
#endif
#ifndef ENABLE_WDT_TEST_HANG
#define ENABLE_WDT_TEST_HANG 0
#endif
#ifndef ENABLE_WDT_TEST_BUILD
#define ENABLE_WDT_TEST_BUILD 0
#endif
#if ENABLE_WDT_TEST_HANG && !ENABLE_WDT_TEST_BUILD
#error "ENABLE_WDT_TEST_HANG exige ENABLE_WDT_TEST_BUILD=1"
#endif

static const char *TAG = "DSP_MASTER";

static esp_err_t configure_task_watchdog() {
    esp_task_wdt_config_t config{};
    config.timeout_ms = 10000;
    config.idle_core_mask = (1u << portNUM_PROCESSORS) - 1u;
    config.trigger_panic = true;
    esp_err_t err = esp_task_wdt_init(&config);
    if (err == ESP_ERR_INVALID_STATE)
        err = esp_task_wdt_reconfigure(&config);
    return err;
}

// ESP-IDF 5.5 retains FreeRTOS' word-based high-water-mark unit. The wire
// protocol reports bytes, so convert the returned StackType_t count here.
#if ENABLE_RESOURCE_METRICS
static uint32_t task_stack_high_water_bytes(TaskHandle_t task) {
    if (!task) return 0;
    const uint64_t bytes = static_cast<uint64_t>(uxTaskGetStackHighWaterMark(task)) *
                           sizeof(StackType_t);
    return bytes > UINT32_MAX ? UINT32_MAX : static_cast<uint32_t>(bytes);
}
#endif

// Handles de I2S e Buffer de Streaming USB
static i2s_chan_handle_t rx_chan = NULL;
static i2s_chan_handle_t tx_chan = NULL;
#if ENABLE_RESOURCE_METRICS
struct DmaCallbackMetrics {
    uint32_t rx_interrupts = 0;
    uint32_t tx_interrupts = 0;
    uint32_t rx_bytes = 0;
    uint32_t tx_bytes = 0;
    uint32_t rx_errors = 0;
    uint32_t tx_errors = 0;
    uint64_t rx_cycles = 0;
    uint64_t tx_cycles = 0;
    uint32_t rx_cycles_max = 0;
    uint32_t tx_cycles_max = 0;
};
static DmaCallbackMetrics dma_callback_metrics;
static portMUX_TYPE dma_callback_lock = portMUX_INITIALIZER_UNLOCKED;

static bool IRAM_ATTR record_dma_callback(i2s_event_data_t *event, bool tx, bool error) {
    const uint32_t cycle_start = PROFILE_CYCLE_COUNT();
    portENTER_CRITICAL_ISR(&dma_callback_lock);
    if (tx) {
        ++dma_callback_metrics.tx_interrupts;
        if (error) ++dma_callback_metrics.tx_errors;
        else if (event) dma_callback_metrics.tx_bytes += (uint32_t)event->size;
    } else {
        ++dma_callback_metrics.rx_interrupts;
        if (error) ++dma_callback_metrics.rx_errors;
        else if (event) dma_callback_metrics.rx_bytes += (uint32_t)event->size;
    }
    portEXIT_CRITICAL_ISR(&dma_callback_lock);
    const uint32_t cycles = PROFILE_CYCLE_COUNT() - cycle_start;
    portENTER_CRITICAL_ISR(&dma_callback_lock);
    if (tx) {
        dma_callback_metrics.tx_cycles += cycles;
        if (cycles > dma_callback_metrics.tx_cycles_max) dma_callback_metrics.tx_cycles_max = cycles;
    } else {
        dma_callback_metrics.rx_cycles += cycles;
        if (cycles > dma_callback_metrics.rx_cycles_max) dma_callback_metrics.rx_cycles_max = cycles;
    }
    portEXIT_CRITICAL_ISR(&dma_callback_lock);
    return false;
}

static bool IRAM_ATTR on_i2s_rx(i2s_chan_handle_t, i2s_event_data_t *event, void*) {
    return record_dma_callback(event, false, false);
}
static bool IRAM_ATTR on_i2s_rx_overflow(i2s_chan_handle_t, i2s_event_data_t *event, void*) {
    return record_dma_callback(event, false, true);
}
static bool IRAM_ATTR on_i2s_tx(i2s_chan_handle_t, i2s_event_data_t *event, void*) {
    return record_dma_callback(event, true, false);
}
static bool IRAM_ATTR on_i2s_tx_overflow(i2s_chan_handle_t, i2s_event_data_t *event, void*) {
    return record_dma_callback(event, true, true);
}
#endif
static uint64_t i2s_started_us = 0;
static std::atomic<uint32_t> audio_io_errors{0};
static std::atomic<uint32_t> i2s_drift_events{0};
static std::atomic<uint32_t> slow_audio_blocks{0};
static std::atomic<uint32_t> usb_overruns{0};
static std::atomic<uint32_t> usb_underruns{0};
static std::atomic<uint32_t> usb_control_overruns{0};
static std::atomic<uint32_t> usb_control_tx_overruns{0};
static std::atomic<uint32_t> usb_frames{0};
static std::atomic<uint32_t> usb_stream_starts{0};
static std::atomic<uint32_t> usb_stream_stops{0};
static std::atomic<uint32_t> usb_cadence_errors{0};
static std::atomic<uint32_t> usb_ring_frames{0};
static std::atomic<uint32_t> usb_ring_high_water{0};
static std::atomic<uint32_t> usb_resyncs{0};
static std::atomic<uint32_t> usb_dropped_frames{0};
static std::atomic<uint32_t> max_audio_block_us{0};
static std::atomic<uint32_t> max_cab_block_us{0};
static std::atomic<uint32_t> last_audio_block_us{0};
static std::atomic<uint32_t> last_cab_block_us{0};
static std::atomic<uint32_t> cab_slow_blocks{0};
static std::atomic<uint32_t> cab_saturation_samples{0};
static std::atomic<uint32_t> max_new_effect_block_us{0};
// Nível de redução temporária de qualidade usado apenas quando a combinação
// de efeitos não cabe no prazo do DMA. Evita que o DSP monopolize o CPU1 e
// deixa o áudio passar de forma reconhecível até a carga se recuperar.
static std::atomic<uint8_t> dsp_quality_level{0};
static std::atomic<uint32_t> dsp_shed_blocks{0};
static std::atomic<bool> cab_failsafe_bypass{false};
static std::atomic<uint8_t> audio_route{AUDIO_ROUTE_BOTH};
// A placa B mantém o afinador local no sinal cru. Enquanto ele está ativo,
// silenciamos as duas saídas da placa A, mas continuamos processando os
// blocos para preservar clocks e o estado dos efeitos.
static std::atomic<bool> tuner_mute{false};
static std::atomic<TickType_t> last_tuner_state_tick{0};
static std::atomic<bool> usb_mounted{false};
static std::atomic<uint32_t> usb_control_session_epoch{0};
static std::atomic<bool> usb_stream_primed{false};
static std::atomic<TickType_t> usb_last_callback_tick{0};
// USB CDC and the Placa B UART are independent producers. This mutex makes
// each state/preset transaction atomic across both control paths.
static SemaphoreHandle_t control_mutex = nullptr;
static constexpr uint8_t PRESET_SLOT_UNKNOWN = 0xFF;
static std::atomic<uint8_t> active_preset_slot{PRESET_SLOT_UNKNOWN};
static std::atomic<bool> active_preset_edited{false};
static std::atomic<bool> recovery_tracking_enabled{false};
static std::atomic<bool> recovery_save_pending{false};
static std::atomic<bool> preset_status_pending{false};
static std::atomic<uint32_t> recovery_generation{0};
static std::atomic<TickType_t> recovery_last_change_tick{0};

static void request_recovery_save() {
    if (!recovery_tracking_enabled.load(std::memory_order_acquire)) return;
    recovery_generation.fetch_add(1, std::memory_order_acq_rel);
    recovery_last_change_tick.store(xTaskGetTickCount(), std::memory_order_release);
    recovery_save_pending.store(true, std::memory_order_release);
}

static void mark_active_preset_edited() {
    if (!recovery_tracking_enabled.load(std::memory_order_acquire)) return;
    active_preset_edited.store(true, std::memory_order_release);
    preset_status_pending.store(true, std::memory_order_release);
    request_recovery_save();
}

static void mark_active_preset_origin(uint8_t slot) {
    active_preset_slot.store(slot, std::memory_order_release);
    active_preset_edited.store(false, std::memory_order_release);
    preset_status_pending.store(true, std::memory_order_release);
    request_recovery_save();
}

static profile_metrics::Recorder<AP_STAGE_COUNT> audio_profiler;
#if ENABLE_RESOURCE_METRICS
static std::atomic<uint32_t> audio_task_buffer_locations{0};

static __attribute__((noinline)) uint32_t set_buffer_location(
        uint32_t flags, uint8_t slot, const void *pointer) {
    if (!pointer) return flags;
    uint8_t location = profile_protocol::BUFFER_LOCATION_OTHER;
    if (esp_ptr_internal(pointer)) location = profile_protocol::BUFFER_LOCATION_INTERNAL_SRAM;
    else if (esp_ptr_external_ram(pointer)) location = profile_protocol::BUFFER_LOCATION_PSRAM;
    return flags | (static_cast<uint32_t>(location) << (slot * 2));
}
#endif
#if ENABLE_RESOURCE_METRICS
static uint32_t cycle_marker_min_cycles = 0;
static uint32_t cycle_marker_mean_cycles = 0;
static uint32_t cycle_marker_max_cycles = 0;
static bool cycle_marker_cost_ready = false;

static void measure_cycle_marker_cost() {
    if (cycle_marker_cost_ready) return;
    uint64_t total = 0;
    uint32_t minimum = UINT32_MAX;
    uint32_t maximum = 0;
    uint32_t valid = 0;
    for (unsigned i = 0; i < 256; ++i) {
        const int start_core = PROFILE_CORE_ID();
        const uint32_t start = PROFILE_CYCLE_COUNT();
        const int64_t start_us = esp_timer_get_time();
        const uint32_t start_hz = (uint32_t)esp_clk_cpu_freq();
        const int64_t end_us = esp_timer_get_time();
        const uint32_t end = PROFILE_CYCLE_COUNT();
        const uint32_t end_hz = (uint32_t)esp_clk_cpu_freq();
        const int end_core = PROFILE_CORE_ID();
        const uint64_t elapsed_us = (uint64_t)(end_us - start_us);
        if (start_core != end_core || start_hz == 0 || start_hz != end_hz ||
            elapsed_us >= ((uint64_t)UINT32_MAX * 1000000ULL) / start_hz) continue;
        const uint32_t cycles = end - start;
        if (cycles < minimum) minimum = cycles;
        if (cycles > maximum) maximum = cycles;
        total += cycles;
        ++valid;
    }
    if (valid) {
        cycle_marker_min_cycles = minimum;
        cycle_marker_mean_cycles = (uint32_t)(total / valid);
        cycle_marker_max_cycles = maximum;
    }
    cycle_marker_cost_ready = true;
}
#endif

static CdcMutationCache usb_mutation_cache;
#if ENABLE_PROFILING
static std::atomic<uint32_t> profile_session_request{0};
static std::atomic<uint32_t> profile_snapshot_request{0};
static std::atomic<uint32_t> profile_timer_pair_us{0};
#endif

#if ENABLE_PROFILING
#define PROFILE_START(name) \
    const int64_t name##_t0 = esp_timer_get_time(); \
    const uint32_t name##_c0 = PROFILE_CYCLE_COUNT(); \
    const int name##_core = PROFILE_CORE_ID(); \
    const uint32_t name##_cpu_hz = (uint32_t)esp_clk_cpu_freq()
#define PROFILE_FINISH(rec, stage, mode, samples, name) do { \
    const int64_t _profile_t1 = esp_timer_get_time(); \
    const uint32_t _profile_c1 = PROFILE_CYCLE_COUNT(); \
    const uint32_t _profile_cpu_hz_end = (uint32_t)esp_clk_cpu_freq(); \
    const uint64_t _profile_elapsed_us = (uint64_t)(_profile_t1 - name##_t0); \
    const uint64_t _profile_wrap_us = name##_cpu_hz ? \
        ((uint64_t)UINT32_MAX * 1000000ULL) / name##_cpu_hz : 0; \
    const int _profile_core_end = PROFILE_CORE_ID(); \
    const uint8_t _profile_cycle_reason = _profile_core_end != name##_core ? \
        profile_metrics::CYCLE_CORE_CHANGED : \
        (name##_cpu_hz != _profile_cpu_hz_end || !name##_cpu_hz ? \
         profile_metrics::CYCLE_FREQUENCY_CHANGED : \
         (_profile_elapsed_us >= _profile_wrap_us ? profile_metrics::CYCLE_WRAP_AMBIGUOUS : 0)); \
    const bool _profile_cycle_valid = _profile_cycle_reason == 0; \
    (rec).record((stage), _profile_elapsed_us, \
                 (samples), (mode), (uint32_t)(_profile_c1 - name##_c0), \
                 _profile_cycle_valid, _profile_cycle_reason, \
                 name##_core >= 0 ? (uint8_t)name##_core : UINT8_MAX, \
                 name##_cpu_hz); \
} while (0)
#define PROFILE_EXEC(rec, stage, mode, samples, ...) do { \
    PROFILE_START(_profile_exec); \
    __VA_ARGS__; \
    PROFILE_FINISH(rec, stage, mode, samples, _profile_exec); \
} while (0)
#define PROFILE_COUNT(rec, stage, mode) (rec).count((stage), (mode))
#else
#define PROFILE_START(name) do { } while (0)
#define PROFILE_FINISH(rec, stage, mode, samples, name) do { } while (0)
#define PROFILE_EXEC(rec, stage, mode, samples, ...) do { __VA_ARGS__; } while (0)
#define PROFILE_COUNT(rec, stage, mode) do { } while (0)
#endif

static void update_max(std::atomic<uint32_t>& target, uint32_t value) {
    uint32_t previous = target.load(std::memory_order_relaxed);
    while (previous < value &&
           !target.compare_exchange_weak(previous, value,
                                         std::memory_order_relaxed,
                                         std::memory_order_relaxed)) {
    }
}

class ControlGuard {
public:
    ControlGuard() : locked_(control_mutex &&
                             xSemaphoreTake(control_mutex, portMAX_DELAY) == pdTRUE) {}
    ~ControlGuard() { if (locked_) xSemaphoreGive(control_mutex); }
    ControlGuard(const ControlGuard &) = delete;
    ControlGuard &operator=(const ControlGuard &) = delete;
private:
    bool locked_;
};

class PcmRing {
public:
    size_t push(const int32_t *samples, size_t count) {
        uint32_t w = write_.load(std::memory_order_relaxed);
        uint32_t r = read_.load(std::memory_order_acquire);
        const size_t available = static_cast<size_t>(w - r);
        const size_t room = available < USB_AUDIO_RING_FRAMES
            ? USB_AUDIO_RING_FRAMES - available : 0;
        const size_t accepted = count < room ? count : room;
        const size_t first = accepted < USB_AUDIO_RING_FRAMES - (w % USB_AUDIO_RING_FRAMES)
            ? accepted : USB_AUDIO_RING_FRAMES - (w % USB_AUDIO_RING_FRAMES);
        if (first) memcpy(data_ + (w % USB_AUDIO_RING_FRAMES), samples, first * sizeof(int32_t));
        if (accepted > first) memcpy(data_, samples + first, (accepted - first) * sizeof(int32_t));
        write_.store(w + static_cast<uint32_t>(accepted), std::memory_order_release);
        if (accepted < count) {
            usb_overruns.fetch_add(static_cast<uint32_t>(count - accepted), std::memory_order_relaxed);
            usb_dropped_frames.fetch_add(static_cast<uint32_t>(count - accepted), std::memory_order_relaxed);
        }
        update_occupancy(available + accepted);
        return accepted;
    }

    size_t pop(int32_t *out, size_t count) {
        // Apenas o consumidor avança read_. Produtores e callbacks de
        // montagem/desmontagem solicitam a limpeza por esta flag; o reset
        // efetivo acontece neste mesmo contexto SPSC antes do consumo.
        if (clear_requested_.exchange(false, std::memory_order_acq_rel)) {
            const uint32_t w = write_.load(std::memory_order_acquire);
            read_.store(w, std::memory_order_release);
            usb_ring_frames.store(0, std::memory_order_relaxed);
            reset_consumed_.store(true, std::memory_order_release);
            return 0;
        }
        uint32_t r = read_.load(std::memory_order_relaxed);
        uint32_t w = write_.load(std::memory_order_acquire);
        size_t available = (size_t)(w - r);
        if (available > count) available = count;
        const size_t first = available < USB_AUDIO_RING_FRAMES - (r % USB_AUDIO_RING_FRAMES)
            ? available : USB_AUDIO_RING_FRAMES - (r % USB_AUDIO_RING_FRAMES);
        if (first) memcpy(out, data_ + (r % USB_AUDIO_RING_FRAMES), first * sizeof(int32_t));
        if (available > first) memcpy(out + first, data_, (available - first) * sizeof(int32_t));
        read_.store(r + (uint32_t)available, std::memory_order_release);
        update_occupancy((size_t)(w - (r + (uint32_t)available)));
        return available;
    }

    size_t available() const {
        const uint32_t r = read_.load(std::memory_order_acquire);
        const uint32_t w = write_.load(std::memory_order_acquire);
        return static_cast<size_t>(w - r);
    }

    size_t discard_oldest(size_t count) {
        const uint32_t r = read_.load(std::memory_order_relaxed);
        const uint32_t w = write_.load(std::memory_order_acquire);
        const size_t available = static_cast<size_t>(w - r);
        const size_t discarded = count < available ? count : available;
        read_.store(r + static_cast<uint32_t>(discarded), std::memory_order_release);
        update_occupancy(available - discarded);
        return discarded;
    }

    void clear() { request_clear(); }

    void request_clear() {
        clear_requested_.store(true, std::memory_order_release);
    }

    bool take_reset_consumed() {
        return reset_consumed_.exchange(false, std::memory_order_acq_rel);
    }

private:
    static void update_occupancy(size_t value) {
        usb_ring_frames.store(static_cast<uint32_t>(value), std::memory_order_relaxed);
        update_max(usb_ring_high_water, static_cast<uint32_t>(value));
    }

    int32_t data_[USB_AUDIO_RING_FRAMES] = {};
    std::atomic<uint32_t> read_{0};
    std::atomic<uint32_t> write_{0};
    std::atomic<bool> clear_requested_{false};
    std::atomic<bool> reset_consumed_{false};
};

class ByteRing {
public:
    bool push(uint8_t byte) {
        uint32_t w = write_.load(std::memory_order_relaxed);
        uint32_t r = read_.load(std::memory_order_acquire);
        if (w - r >= USB_CONTROL_RX_BYTES) {
            usb_control_overruns.fetch_add(1, std::memory_order_relaxed);
            return false;
        }
        data_[w % USB_CONTROL_RX_BYTES] = byte;
        write_.store(w + 1, std::memory_order_release);
        return true;
    }

    bool pop(uint8_t *byte) {
        uint32_t r = read_.load(std::memory_order_relaxed);
        uint32_t w = write_.load(std::memory_order_acquire);
        if (r == w) return false;
        *byte = data_[r % USB_CONTROL_RX_BYTES];
        read_.store(r + 1, std::memory_order_release);
        return true;
    }

    // Only the consumer advances read_, so connection changes can discard
    // bytes from the previous CDC session without racing the USB producer.
    void clear() {
        read_.store(write_.load(std::memory_order_acquire), std::memory_order_release);
    }

private:
    uint8_t data_[USB_CONTROL_RX_BYTES] = {};
    std::atomic<uint32_t> read_{0};
    std::atomic<uint32_t> write_{0};
};

static PcmRing usb_pcm_ring;
static ByteRing usb_control_ring;
#if ENABLE_USB_AUDIO
static TaskHandle_t usb_control_task_handle = nullptr;
#endif

// Estado Oficial (Duplo Buffer Lock-Free)
static PedalState dsp_state;
static portMUX_TYPE state_lock = portMUX_INITIALIZER_UNLOCKED;

static void normalize_state_for_build(PedalState& state) {
#if !ENABLE_CAB_SIM
    // Presets e comandos de versões antigas ainda contêm estes campos. O
    // firmware sem CAB nunca publica nem executa um CAB ativo.
    state.cab_active = 0;
    state.cab_index = 0;
#endif
}

static PedalState state_snapshot() {
    PedalState result;
    portENTER_CRITICAL(&state_lock);
    memcpy(&result, &dsp_state, sizeof(result));
    portEXIT_CRITICAL(&state_lock);
    return result;
}

static void publish_state(const PedalState& value) {
    PedalState sanitized = value;
    normalize_state_for_build(sanitized);
    portENTER_CRITICAL(&state_lock);
    memcpy(&dsp_state, &sanitized, sizeof(sanitized));
    portEXIT_CRITICAL(&state_lock);
    mark_active_preset_edited();
}

// Fator dinâmico de atenuação alimentado pela IA da Placa B
std::atomic<float> external_ai_gate{ 1.0f };
std::atomic<TickType_t> last_ai_tick{ 0 };

// Instâncias dos Efeitos
static NoiseGate       noise_gate;
static Compressor      comp;
static AutoWah         autowah;
static Overdrive       overdrive;
static Biquad          eq_low, eq_mid1, eq_mid2, eq_high;
static Modulation      chorus;
static Delay           tape_delay;
static ReverbDattorro  reverb;
static BitCrusher      bitcrusher;
static Tremolo         tremolo;
static Phaser          phaser;
#if ENABLE_CAB_SIM
static CabSimIR        cab_sim;
#endif

static void deinit_i2s_system(void) {
    i2s_chan_handle_t rx = rx_chan;
    i2s_chan_handle_t tx = tx_chan;
    rx_chan = NULL;
    tx_chan = NULL;
    if (rx) (void)i2s_channel_disable(rx);
    if (tx && tx != rx) (void)i2s_channel_disable(tx);
    if (rx) (void)i2s_del_channel(rx);
    if (tx && tx != rx) (void)i2s_del_channel(tx);
}

// Inicialização dos barramentos I2S. Erros voltam ao dono da tarefa de áudio
// para que ele possa limpar os canais e tentar novamente sem reiniciar o MCU.
static esp_err_t init_i2s_system(void) {
    deinit_i2s_system();
    esp_err_t err = ESP_OK;
#define I2S_INIT_OR_FAIL(operation) do { \
        err = (operation); \
        if (err != ESP_OK) { deinit_i2s_system(); return err; } \
    } while (0)
#if I2S_SHARED_CLOCKS
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    chan_cfg.dma_desc_num = DMA_DESC_NUM;
    chan_cfg.dma_frame_num = DMA_FRAME_NUM;
    I2S_INIT_OR_FAIL(i2s_new_channel(&chan_cfg, &tx_chan, &rx_chan));

    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(SAMPLE_RATE),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = ADC_MCLK_IO, .bclk = SHARED_BCK_IO, .ws = SHARED_WS_IO,
            .dout = DAC_DOUT_IO, .din = ADC_DIN_IO,
            .invert_flags = { .mclk_inv = false, .bclk_inv = false, .ws_inv = false },
        },
    };
    std_cfg.clk_cfg.mclk_multiple = I2S_MCLK_MULTIPLE_256;
    I2S_INIT_OR_FAIL(i2s_channel_init_std_mode(rx_chan, &std_cfg));
    I2S_INIT_OR_FAIL(i2s_channel_init_std_mode(tx_chan, &std_cfg));
#if ENABLE_RESOURCE_METRICS
    const i2s_event_callbacks_t rx_callbacks = {
        .on_recv = on_i2s_rx,
        .on_recv_q_ovf = on_i2s_rx_overflow,
        .on_sent = NULL,
        .on_send_q_ovf = NULL,
    };
    const i2s_event_callbacks_t tx_callbacks = {
        .on_recv = NULL,
        .on_recv_q_ovf = NULL,
        .on_sent = on_i2s_tx,
        .on_send_q_ovf = on_i2s_tx_overflow,
    };
    I2S_INIT_OR_FAIL(i2s_channel_register_event_callback(rx_chan, &rx_callbacks, nullptr));
    I2S_INIT_OR_FAIL(i2s_channel_register_event_callback(tx_chan, &tx_callbacks, nullptr));
#endif
#else
    i2s_chan_config_t rx_chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    rx_chan_cfg.dma_desc_num = DMA_DESC_NUM;
    rx_chan_cfg.dma_frame_num = DMA_FRAME_NUM;
    I2S_INIT_OR_FAIL(i2s_new_channel(&rx_chan_cfg, NULL, &rx_chan));

    i2s_std_config_t rx_std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(SAMPLE_RATE),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = ADC_MCLK_IO, .bclk = ADC_BCK_IO, .ws = ADC_WS_IO,
            .dout = I2S_GPIO_UNUSED, .din = ADC_DIN_IO,
            .invert_flags = { .mclk_inv = false, .bclk_inv = false, .ws_inv = false },
        },
    };
    rx_std_cfg.clk_cfg.mclk_multiple = I2S_MCLK_MULTIPLE_256;
    I2S_INIT_OR_FAIL(i2s_channel_init_std_mode(rx_chan, &rx_std_cfg));
#if ENABLE_RESOURCE_METRICS
    const i2s_event_callbacks_t rx_callbacks = {
        .on_recv = on_i2s_rx,
        .on_recv_q_ovf = on_i2s_rx_overflow,
        .on_sent = NULL,
        .on_send_q_ovf = NULL,
    };
    I2S_INIT_OR_FAIL(i2s_channel_register_event_callback(rx_chan, &rx_callbacks, nullptr));
#endif

    i2s_chan_config_t tx_chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_1, I2S_ROLE_MASTER);
    tx_chan_cfg.dma_desc_num = DMA_DESC_NUM;
    tx_chan_cfg.dma_frame_num = DMA_FRAME_NUM;
    I2S_INIT_OR_FAIL(i2s_new_channel(&tx_chan_cfg, &tx_chan, NULL));

    i2s_std_config_t tx_std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(SAMPLE_RATE),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED, .bclk = DAC_BCK_IO, .ws = DAC_WS_IO,
            .dout = DAC_DOUT_IO, .din = I2S_GPIO_UNUSED,
            .invert_flags = { .mclk_inv = false, .bclk_inv = false, .ws_inv = false },
        },
    };
    I2S_INIT_OR_FAIL(i2s_channel_init_std_mode(tx_chan, &tx_std_cfg));
#if ENABLE_RESOURCE_METRICS
    const i2s_event_callbacks_t tx_callbacks = {
        .on_recv = NULL,
        .on_recv_q_ovf = NULL,
        .on_sent = on_i2s_tx,
        .on_send_q_ovf = on_i2s_tx_overflow,
    };
    I2S_INIT_OR_FAIL(i2s_channel_register_event_callback(tx_chan, &tx_callbacks, nullptr));
#endif
#endif
    I2S_INIT_OR_FAIL(i2s_channel_enable(rx_chan));
    I2S_INIT_OR_FAIL(i2s_channel_enable(tx_chan));
#undef I2S_INIT_OR_FAIL
    return ESP_OK;
}

static bool ensure_i2s_system(void) {
    for (unsigned attempt = 1; attempt <= 3; ++attempt) {
        const esp_err_t err = init_i2s_system();
        if (err == ESP_OK) {
            i2s_started_us = (uint64_t)esp_timer_get_time();
            ESP_LOGI(TAG, "I2S pronto na tentativa %u", attempt);
            return true;
        }
        ESP_LOGE(TAG, "Inicializacao I2S %u/3 falhou: %s", attempt, esp_err_to_name(err));
        if (attempt < 3) vTaskDelay(pdMS_TO_TICKS(500));
    }
    deinit_i2s_system();
    return false;
}

// Transmissão do estado oficial consolidado para a Placa B
void broadcast_pedal_state(const PedalState* state) {
    PedalState sanitized = *state;
    normalize_state_for_build(sanitized);
    uint8_t payload[1 + sizeof(PedalState)];
    payload[0] = PKT_PEDAL_STATE;
    memcpy(&payload[1], &sanitized, sizeof(PedalState));

    uint8_t tx_buffer[sizeof(payload) + 4];
    size_t enc_len = encode_packet(payload, sizeof(payload), tx_buffer);
    tx_buffer[enc_len] = 0x00;

    uart_write_bytes(UART_PORT_NUM, tx_buffer, enc_len + 1);
}

static void send_preset_result(uint8_t action, uint8_t slot, bool success) {
    uint8_t payload[] = {PKT_PRESET_RESULT, action, slot, (uint8_t)success};
    uint8_t frame[sizeof(payload) + 4];
    size_t count = encode_packet(payload, sizeof(payload), frame);
    frame[count] = 0;
    uart_write_bytes(UART_PORT_NUM, frame, count + 1);
}

static void send_preset_status() {
    ControlGuard guard;
    PresetStatusPacket status{
        active_preset_slot.load(std::memory_order_acquire),
        static_cast<uint8_t>(active_preset_edited.load(std::memory_order_acquire))
    };
    uint8_t payload[1 + sizeof(status)] = {PKT_PRESET_STATUS};
    memcpy(&payload[1], &status, sizeof(status));
    uint8_t frame[sizeof(payload) + 4];
    const size_t count = encode_packet(payload, sizeof(payload), frame);
    frame[count] = 0;
    uart_write_bytes(UART_PORT_NUM, frame, count + 1);
}

static void broadcast_audio_route(uint8_t route) {
    uint8_t payload[] = {PKT_AUDIO_ROUTE, route};
    uint8_t frame[sizeof(payload) + 4];
    size_t count = encode_packet(payload, sizeof(payload), frame);
    frame[count] = 0;
    uart_write_bytes(UART_PORT_NUM, frame, count + 1);
}

// Aplicação de incrementos de parâmetros no nó Master
bool apply_param_increment(uint8_t effect_idx, uint8_t param_idx, int8_t delta) {
    if (effect_idx >= 12 || param_idx >= 5 || delta == 0) return false;
    static const uint8_t parameter_count[12] = {2, 4, 4, 4, 5, 4, 4, 4, 2, 4, 5, 5};
    if (param_idx >= parameter_count[effect_idx]) return false;
    ControlGuard guard;
    PedalState updated = state_snapshot();
    PedalState* next = &updated;

    switch (effect_idx) {
        case 0: // Noise Gate
            if (param_idx == 0) next->gate_active = !next->gate_active;
            if (param_idx == 1)
                next->gate_threshold = gate_threshold::apply_db_steps(next->gate_threshold, delta);
            break;
        case 1: // Compressor
            if (param_idx == 0) next->comp_active = !next->comp_active;
            if (param_idx == 1) next->comp_threshold = fminf(fmaxf(next->comp_threshold + delta * 0.05f, 0.05f), 0.95f);
            if (param_idx == 2) next->comp_ratio = fminf(fmaxf(next->comp_ratio + delta * 0.5f, 1.0f), 10.0f);
            if (param_idx == 3) next->comp_makeup_gain = fminf(fmaxf(next->comp_makeup_gain + delta * 0.1f, 1.0f), 4.0f);
            break;
        case 2: // Auto-Wah
            if (param_idx == 0) next->wah_active = !next->wah_active;
            if (param_idx == 1) next->wah_sensitivity = fminf(fmaxf(next->wah_sensitivity + delta * 0.05f, 0.0f), 1.0f);
            if (param_idx == 2) next->wah_base_freq = fminf(fmaxf(next->wah_base_freq + delta * 25.0f, 100.0f), 1000.0f);
            if (param_idx == 3) next->wah_resonance = fminf(fmaxf(next->wah_resonance + delta * 0.05f, 0.1f), 0.95f);
            break;
        case 3: // Overdrive
            if (param_idx == 0) next->dist_active = !next->dist_active;
            if (param_idx == 1) next->dist_drive = fminf(fmaxf(next->dist_drive + delta * 0.5f, 1.0f), 20.0f);
            if (param_idx == 2) next->dist_tone = fminf(fmaxf(next->dist_tone + delta * 0.05f, 0.0f), 1.0f);
            if (param_idx == 3) next->dist_level = fminf(fmaxf(next->dist_level + delta * 0.05f, 0.0f), 1.0f);
            break;
        case 4: // Equalizador Paramétrico
            if (param_idx == 0) next->eq_active = !next->eq_active;
            if (param_idx == 1) next->eq_low_gain = fminf(fmaxf(next->eq_low_gain + delta * 1.0f, -12.0f), 12.0f);
            if (param_idx == 2) next->eq_mid1_gain = fminf(fmaxf(next->eq_mid1_gain + delta * 1.0f, -12.0f), 12.0f);
            if (param_idx == 3) next->eq_mid2_gain = fminf(fmaxf(next->eq_mid2_gain + delta * 1.0f, -12.0f), 12.0f);
            if (param_idx == 4) next->eq_high_gain = fminf(fmaxf(next->eq_high_gain + delta * 1.0f, -12.0f), 12.0f);
            break;
        case 5: // Chorus / Modulação
            if (param_idx == 0) next->mod_active = !next->mod_active;
            if (param_idx == 1) next->mod_rate_hz = fminf(fmaxf(next->mod_rate_hz + delta * 0.1f, 0.2f), 5.0f);
            if (param_idx == 2) next->mod_depth = fminf(fmaxf(next->mod_depth + delta * 0.05f, 0.0f), 1.0f);
            if (param_idx == 3) next->mod_mix = fminf(fmaxf(next->mod_mix + delta * 0.05f, 0.0f), 1.0f);
            break;
        case 6: // Tape Delay
            if (param_idx == 0) next->delay_active = !next->delay_active;
            if (param_idx == 1) {
                int32_t samples = (int32_t)next->delay_time_samples + (delta * (SAMPLE_RATE / 100)); // passos de 10ms
                if (samples < (SAMPLE_RATE / 50)) samples = (SAMPLE_RATE / 50); // min 20ms
                if (samples > (SAMPLE_RATE * 12 / 10)) samples = (SAMPLE_RATE * 12 / 10); // max 1.2s
                next->delay_time_samples = (uint32_t)samples;
            }
            if (param_idx == 2) next->delay_feedback = fminf(fmaxf(next->delay_feedback + delta * 0.05f, 0.0f), 0.95f);
            if (param_idx == 3) next->delay_mix = fminf(fmaxf(next->delay_mix + delta * 0.05f, 0.0f), 1.0f);
            break;
        case 7: // Reverb Dattorro Plate
            if (param_idx == 0) next->reverb_active = !next->reverb_active;
            if (param_idx == 1) next->reverb_decay = fminf(fmaxf(next->reverb_decay + delta * 0.05f, 0.1f), 0.95f);
            if (param_idx == 2) next->reverb_damping = fminf(fmaxf(next->reverb_damping + delta * 0.05f, 0.0f), 0.8f);
            if (param_idx == 3) next->reverb_mix = fminf(fmaxf(next->reverb_mix + delta * 0.05f, 0.0f), 1.0f);
            break;
        case 8: // Cab Sim / IR
#if ENABLE_CAB_SIM
            if (param_idx == 0) next->cab_active = !next->cab_active;
            if (param_idx == 1) {
                int idx = ((int)next->cab_index + delta) % TOTAL_IR_PRESETS;
                next->cab_index = (uint8_t)(idx < 0 ? idx + TOTAL_IR_PRESETS : idx);
            }
#else
            // Mantém compatibilidade com o índice legado, mas não permite
            // reativar um efeito removido desta imagem.
            return false;
#endif
            break;
        case 9: // Bitcrusher
            if (param_idx == 0) next->bitcrusher_active = !next->bitcrusher_active;
            if (param_idx == 1) next->bitcrusher_bits = (uint8_t)fminf(fmaxf(next->bitcrusher_bits + delta, 4), 16);
            if (param_idx == 2) next->bitcrusher_hold = (uint8_t)fminf(fmaxf(next->bitcrusher_hold + delta, 1), 16);
            if (param_idx == 3) next->bitcrusher_mix = fminf(fmaxf(next->bitcrusher_mix + delta * 0.05f, 0.0f), 1.0f);
            break;
        case 10: // Tremolo
            if (param_idx == 0) next->tremolo_active = !next->tremolo_active;
            if (param_idx == 1) next->tremolo_rate_hz = fminf(fmaxf(next->tremolo_rate_hz + delta * 0.1f, 0.1f), 12.0f);
            if (param_idx == 2) next->tremolo_depth = fminf(fmaxf(next->tremolo_depth + delta * 0.05f, 0.0f), 1.0f);
            if (param_idx == 3) next->tremolo_shape = (uint8_t)(((int)next->tremolo_shape + delta) & 1);
            if (param_idx == 4) next->tremolo_mix = fminf(fmaxf(next->tremolo_mix + delta * 0.05f, 0.0f), 1.0f);
            break;
        case 11: // Phaser
            if (param_idx == 0) next->phaser_active = !next->phaser_active;
            if (param_idx == 1) next->phaser_rate_hz = fminf(fmaxf(next->phaser_rate_hz + delta * 0.05f, 0.05f), 4.0f);
            if (param_idx == 2) next->phaser_depth = fminf(fmaxf(next->phaser_depth + delta * 0.05f, 0.0f), 1.0f);
            if (param_idx == 3) next->phaser_feedback = fminf(fmaxf(next->phaser_feedback + delta * 0.05f, 0.0f), 0.7f);
            if (param_idx == 4) next->phaser_mix = fminf(fmaxf(next->phaser_mix + delta * 0.05f, 0.0f), 1.0f);
            break;
        default:
            break;
    }

    if (!validPedalState(updated, TOTAL_IR_PRESETS)) return false;
    publish_state(updated);
    broadcast_pedal_state(next);
    return true;
}

static bool apply_toggle(uint8_t *field, float value) {
    if (value != 0.0f && value != 1.0f) return false;
    *field = (uint8_t)value;
    return true;
}

static bool apply_absolute_parameter(uint8_t effect_idx, uint8_t param_idx, float value) {
    if (!isfinite(value) || effect_idx >= 12 || param_idx >= 5) return false;
    static const uint8_t parameter_count[12] = {2, 4, 4, 4, 5, 4, 4, 4, 2, 4, 5, 5};
    if (param_idx >= parameter_count[effect_idx]) return false;
    ControlGuard guard;
    PedalState updated = state_snapshot();

    switch (effect_idx) {
        case 0:
            if (param_idx == 0) { if (!apply_toggle(&updated.gate_active, value)) return false; }
            else if (param_idx == 1) updated.gate_threshold = value;
            else return false;
            break;
        case 1:
            if (param_idx == 0) { if (!apply_toggle(&updated.comp_active, value)) return false; }
            else if (param_idx == 1) updated.comp_threshold = value;
            else if (param_idx == 2) updated.comp_ratio = value;
            else if (param_idx == 3) updated.comp_makeup_gain = value;
            else return false;
            break;
        case 2:
            if (param_idx == 0) { if (!apply_toggle(&updated.wah_active, value)) return false; }
            else if (param_idx == 1) updated.wah_sensitivity = value;
            else if (param_idx == 2) updated.wah_base_freq = value;
            else if (param_idx == 3) updated.wah_resonance = value;
            else return false;
            break;
        case 3:
            if (param_idx == 0) { if (!apply_toggle(&updated.dist_active, value)) return false; }
            else if (param_idx == 1) updated.dist_drive = value;
            else if (param_idx == 2) updated.dist_tone = value;
            else if (param_idx == 3) updated.dist_level = value;
            else return false;
            break;
        case 4:
            if (param_idx == 0) { if (!apply_toggle(&updated.eq_active, value)) return false; }
            else if (param_idx == 1) updated.eq_low_gain = value;
            else if (param_idx == 2) updated.eq_mid1_gain = value;
            else if (param_idx == 3) updated.eq_mid2_gain = value;
            else if (param_idx == 4) updated.eq_high_gain = value;
            else return false;
            break;
        case 5:
            if (param_idx == 0) { if (!apply_toggle(&updated.mod_active, value)) return false; }
            else if (param_idx == 1) updated.mod_rate_hz = value;
            else if (param_idx == 2) updated.mod_depth = value;
            else if (param_idx == 3) updated.mod_mix = value;
            else return false;
            break;
        case 6:
            if (param_idx == 0) { if (!apply_toggle(&updated.delay_active, value)) return false; }
            else if (param_idx == 1 && value >= (float)(SAMPLE_RATE / 50) &&
                     value <= (float)(SAMPLE_RATE * 12 / 10) &&
                     floorf(value) == value) updated.delay_time_samples = (uint32_t)value;
            else if (param_idx == 2) updated.delay_feedback = value;
            else if (param_idx == 3) updated.delay_mix = value;
            else return false;
            break;
        case 7:
            if (param_idx == 0) { if (!apply_toggle(&updated.reverb_active, value)) return false; }
            else if (param_idx == 1) updated.reverb_decay = value;
            else if (param_idx == 2) updated.reverb_damping = value;
            else if (param_idx == 3) updated.reverb_mix = value;
            else return false;
            break;
        case 8:
#if ENABLE_CAB_SIM
            if (param_idx == 0) { if (!apply_toggle(&updated.cab_active, value)) return false; }
            else if (param_idx == 1 && value >= 0.0f && value < (float)TOTAL_IR_PRESETS &&
                     floorf(value) == value) updated.cab_index = (uint8_t)value;
            else return false;
#else
            return false;
#endif
            break;
        case 9:
            if (param_idx == 0) { if (!apply_toggle(&updated.bitcrusher_active, value)) return false; }
            else if (param_idx == 1) {
                uint8_t converted;
                if (!convert_integral_parameter(value, 4.0f, 16.0f, converted)) return false;
                updated.bitcrusher_bits = converted;
            } else if (param_idx == 2) {
                uint8_t converted;
                if (!convert_integral_parameter(value, 1.0f, 16.0f, converted)) return false;
                updated.bitcrusher_hold = converted;
            }
            else if (param_idx == 3) updated.bitcrusher_mix = value;
            else return false;
            break;
        case 10:
            if (param_idx == 0) { if (!apply_toggle(&updated.tremolo_active, value)) return false; }
            else if (param_idx == 1) updated.tremolo_rate_hz = value;
            else if (param_idx == 2) updated.tremolo_depth = value;
            else if (param_idx == 3) {
                uint8_t converted;
                if (!convert_integral_parameter(value, 0.0f, 1.0f, converted)) return false;
                updated.tremolo_shape = converted;
            }
            else if (param_idx == 4) updated.tremolo_mix = value;
            else return false;
            break;
        case 11:
            if (param_idx == 0) { if (!apply_toggle(&updated.phaser_active, value)) return false; }
            else if (param_idx == 1) updated.phaser_rate_hz = value;
            else if (param_idx == 2) updated.phaser_depth = value;
            else if (param_idx == 3) updated.phaser_feedback = value;
            else if (param_idx == 4) updated.phaser_mix = value;
            else return false;
            break;
        default:
            return false;
    }

    if (!validPedalState(updated, TOTAL_IR_PRESETS)) return false;
    publish_state(updated);
    broadcast_pedal_state(&updated);
    return true;
}

static bool set_audio_route(uint8_t route) {
    if ((route & ~AUDIO_ROUTE_BOTH) != 0 || route == 0) return false;
    ControlGuard guard;
    audio_route.store(route, std::memory_order_release);
    if (!(route & AUDIO_ROUTE_USB)) {
        usb_stream_primed.store(false, std::memory_order_release);
        usb_pcm_ring.clear();
    }
    broadcast_audio_route(route);
    return true;
}

#if ENABLE_USB_AUDIO
static void usb_send_response(uint8_t type, uint16_t sequence,
                              const void *payload, size_t payload_len) {
    if (!usb_mounted.load(std::memory_order_acquire)) return;
    uint8_t frame[300];
    size_t encoded = encode_host_frame(type, sequence,
                                       static_cast<const uint8_t *>(payload), payload_len, frame);
    if (!encoded) return;
    size_t written_total = 0;
    for (int attempt = 0; written_total < encoded && attempt < 8; ++attempt) {
        uint32_t written = tud_cdc_write(frame + written_total,
                                         (uint32_t)(encoded - written_total));
        written_total += written;
        tud_cdc_write_flush();
        if (written == 0) vTaskDelay(pdMS_TO_TICKS(1));
    }
    if (written_total != encoded)
        usb_control_tx_overruns.fetch_add(1, std::memory_order_relaxed);
}

static void usb_send_status(uint8_t type, uint16_t sequence, HostStatus status) {
    uint8_t value = (uint8_t)status;
    usb_send_response(type, sequence, &value, sizeof(value));
}

static void usb_send_profile_response(uint16_t sequence, const void *payload, size_t length) {
    usb_send_response(HOST_RSP_PROFILE, sequence, payload, length);
}

static void usb_process_profile_request(uint16_t sequence, const uint8_t *payload, size_t length) {
    using namespace profile_protocol;
    if (length != sizeof(Request)) {
        usb_send_status(HOST_RSP_ERROR, sequence, HOST_ERR_BAD_LENGTH);
        return;
    }
    Request request{};
    memcpy(&request, payload, sizeof(request));
    if (request.version != VERSION) {
        usb_send_status(HOST_RSP_ERROR, sequence, HOST_ERR_BAD_VERSION);
        return;
    }

    ResponseHeader header{};
    header.version = VERSION;
    header.operation = request.operation;
    header.status = STATUS_OK;
    header.board = BOARD_A;
    header.request_id = request.request_id;
    header.owner = OWNER_AUDIO;
    header.session_id = request.session_id;
    header.snapshot_id = request.snapshot_id;
    if (request.operation == REQUEST_CAPS) {
        Capabilities caps{};
        caps.header = header;
        caps.sample_rate = SAMPLE_RATE;
        caps.audio_block_frames = DMA_FRAME_NUM;
        caps.ai_window_frames = 0;
        caps.tuner_window_frames = 0;
        caps.stage_count = AP_STAGE_COUNT;
        caps.histogram_bins = profile_metrics::HISTOGRAM_BINS;
        caps.cab_enabled = ENABLE_CAB_SIM;
        caps.model_trained = 0;
        caps.owner_count = 1;
        caps.profiling_enabled = ENABLE_PROFILING;
#if ENABLE_PROFILING
        caps.timer_pair_us = profile_timer_pair_us.load(std::memory_order_relaxed);
#else
        caps.timer_pair_us = 0;
#endif
        caps.audio_route = audio_route.load(std::memory_order_acquire);
        caps.shared_i2s_clocks = I2S_SHARED_CLOCKS;
        usb_send_profile_response(sequence, &caps, sizeof(caps));
        return;
    }

    if (request.operation == REQUEST_EXTENDED_CAPS) {
#if ENABLE_RESOURCE_METRICS
        ExtendedCapabilities ext{};
        ext.header = header;
        ext.feature_flags = FEATURE_CYCLE_PAGES | FEATURE_RESOURCE_SNAPSHOT |
                            FEATURE_DMA_EVENT_CALLBACKS | FEATURE_DMA_DRIVER_HOOKS;
        strncpy(ext.firmware_id, METRICS_FIRMWARE_ID, sizeof(ext.firmware_id) - 1);
        usb_send_profile_response(sequence, &ext, sizeof(ext));
#else
        header.status = STATUS_UNAVAILABLE;
        usb_send_profile_response(sequence, &header, sizeof(header));
#endif
        return;
    }

#if ENABLE_RESOURCE_METRICS
    if (request.operation == REQUEST_RESOURCE_SNAPSHOT) {
        ResourceSnapshot metrics{};
        metrics.header = header;
        metrics.cpu_frequency_hz = esp_clk_cpu_freq();
        measure_cycle_marker_cost();
        metrics.internal_free_bytes = heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
        metrics.internal_min_free_bytes = heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
        metrics.internal_largest_block_bytes = heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
        metrics.psram_free_bytes = heap_caps_get_free_size(MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        metrics.psram_min_free_bytes = heap_caps_get_minimum_free_size(MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        metrics.psram_largest_block_bytes = heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        TaskHandle_t audio_task = xTaskGetHandle("dsp_task");
        metrics.task_stack_high_water_bytes = task_stack_high_water_bytes(audio_task);
        TaskHandle_t comm_task = xTaskGetHandle("comm_task");
        TaskHandle_t usb_task = xTaskGetHandle("usb_ctrl");
        metrics.ui_stack_high_water_bytes = task_stack_high_water_bytes(comm_task);
        metrics.capture_stack_high_water_bytes = task_stack_high_water_bytes(usb_task);
        metrics.recorder_bytes = sizeof(audio_profiler);
        metrics.raw_i2s_buffer_bytes = sizeof(int32_t) * BUFFER_SAMPLES;
        metrics.audio_frame_buffer_bytes = sizeof(int32_t) * DMA_FRAME_NUM +
                                           sizeof(float) * DMA_FRAME_NUM;
#if ENABLE_CAB_SIM && !LATENCY_TEST_MODE
        metrics.audio_frame_buffer_bytes += sizeof(float) * DMA_FRAME_NUM;
#endif
        metrics.usb_ring_bytes = sizeof(usb_pcm_ring) + sizeof(usb_control_ring);
        metrics.buffer_location_flags = audio_task_buffer_locations.load(std::memory_order_acquire);
        metrics.buffer_location_flags = set_buffer_location(metrics.buffer_location_flags,
            profile_protocol::BUFFER_SLOT_PROFILER, &audio_profiler);
        metrics.buffer_location_flags = set_buffer_location(metrics.buffer_location_flags,
            profile_protocol::BUFFER_SLOT_USB_RING, &usb_pcm_ring);
        metrics.buffer_location_flags = set_buffer_location(metrics.buffer_location_flags,
            profile_protocol::BUFFER_SLOT_APP_INSTRUMENTATION, &dma_callback_metrics);
        metrics.app_instrumentation_bytes = sizeof(dma_callback_metrics) + sizeof(dma_callback_lock) +
            sizeof(audio_task_buffer_locations) + sizeof(cycle_marker_min_cycles) +
            sizeof(cycle_marker_mean_cycles) + sizeof(cycle_marker_max_cycles) +
            sizeof(cycle_marker_cost_ready);
        metrics.cycle_marker_min_cycles = cycle_marker_min_cycles;
        metrics.cycle_marker_mean_cycles = cycle_marker_mean_cycles;
        metrics.cycle_marker_max_cycles = cycle_marker_max_cycles;
        metrics.memory_valid = 1;
        portENTER_CRITICAL(&dma_callback_lock);
        metrics.dma_rx_interrupts = dma_callback_metrics.rx_interrupts;
        metrics.dma_tx_interrupts = dma_callback_metrics.tx_interrupts;
        metrics.dma_rx_bytes = dma_callback_metrics.rx_bytes;
        metrics.dma_tx_bytes = dma_callback_metrics.tx_bytes;
        metrics.dma_rx_errors = dma_callback_metrics.rx_errors;
        metrics.dma_tx_errors = dma_callback_metrics.tx_errors;
        metrics.dma_rx_cycles_total = dma_callback_metrics.rx_cycles;
        metrics.dma_tx_cycles_total = dma_callback_metrics.tx_cycles;
        metrics.dma_rx_cycles_max = dma_callback_metrics.rx_cycles_max;
        metrics.dma_tx_cycles_max = dma_callback_metrics.tx_cycles_max;
        portEXIT_CRITICAL(&dma_callback_lock);
        metrics.dma_events_valid = 1;
        i2s_chan_info_t rx_info{};
        i2s_chan_info_t tx_info{};
        const bool rx_info_valid = rx_chan && i2s_channel_get_info(rx_chan, &rx_info) == ESP_OK;
        const bool tx_info_valid = tx_chan && i2s_channel_get_info(tx_chan, &tx_info) == ESP_OK;
        if (rx_info_valid) metrics.dma_buffer_bytes += rx_info.total_dma_buf_size;
        if (tx_info_valid) metrics.dma_buffer_bytes += tx_info.total_dma_buf_size;
        DmaDriverMetrics i2s_dma{};
        DmaDriverMetrics gdma_dma{};
        i2s_driver_metrics_snapshot(&i2s_dma);
        gdma_driver_metrics_snapshot(&gdma_dma);
        metrics.dma_buffer_bytes = i2s_dma.rx_buffer_bytes + i2s_dma.tx_buffer_bytes;
        metrics.dma_descriptor_bytes = i2s_dma.rx_descriptor_bytes + i2s_dma.tx_descriptor_bytes;
        metrics.dma_memory_valid = rx_info_valid && tx_info_valid && i2s_dma.memory_valid;
        metrics.dma_valid = metrics.dma_events_valid && metrics.dma_memory_valid &&
                            i2s_dma.timing_valid && gdma_dma.events_valid && gdma_dma.timing_valid;
        usb_send_profile_response(sequence, &metrics, sizeof(metrics));
        return;
    }

    if (request.operation == REQUEST_DMA_DRIVER_SNAPSHOT) {
        DmaDriverMetrics i2s_dma{};
        DmaDriverMetrics gdma_dma{};
        i2s_driver_metrics_snapshot(&i2s_dma);
        // Pair cumulative I2S byte counts with the MCU high-resolution timer.
        const uint64_t counters_at_us = (uint64_t)esp_timer_get_time();
        gdma_driver_metrics_snapshot(&gdma_dma);
        DmaDriverSnapshot dma{};
        dma.header = header;
        dma.header.boundary_us = counters_at_us;
        dma.rx_interrupts = gdma_dma.rx_interrupts;
        dma.tx_interrupts = gdma_dma.tx_interrupts;
        dma.rx_bytes = i2s_dma.rx_bytes;
        dma.tx_bytes = i2s_dma.tx_bytes;
        dma.rx_errors = i2s_dma.rx_errors + gdma_dma.rx_errors;
        dma.tx_errors = i2s_dma.tx_errors + gdma_dma.tx_errors;
        dma.rx_buffer_bytes = i2s_dma.rx_buffer_bytes;
        dma.tx_buffer_bytes = i2s_dma.tx_buffer_bytes;
        dma.rx_descriptor_bytes = i2s_dma.rx_descriptor_bytes;
        dma.tx_descriptor_bytes = i2s_dma.tx_descriptor_bytes;
        dma.instrumentation_bytes = i2s_driver_metrics_storage_bytes() +
                                    gdma_driver_metrics_storage_bytes();
        dma.rx_isr_cycles_total = gdma_dma.rx_isr_cycles_total;
        dma.tx_isr_cycles_total = gdma_dma.tx_isr_cycles_total;
        dma.rx_isr_cycles_max = gdma_dma.rx_isr_cycles_max;
        dma.tx_isr_cycles_max = gdma_dma.tx_isr_cycles_max;
        dma.rx_callback_cycles_total = i2s_dma.rx_callback_cycles_total;
        dma.tx_callback_cycles_total = i2s_dma.tx_callback_cycles_total;
        dma.rx_callback_cycles_max = i2s_dma.rx_callback_cycles_max;
        dma.tx_callback_cycles_max = i2s_dma.tx_callback_cycles_max;
        dma.rx_copy_cycles_total = i2s_dma.rx_copy_cycles_total;
        dma.tx_copy_cycles_total = i2s_dma.tx_copy_cycles_total;
        dma.rx_copy_cycles_max = i2s_dma.rx_copy_cycles_max;
        dma.tx_copy_cycles_max = i2s_dma.tx_copy_cycles_max;
        dma.rx_wait_us = i2s_dma.rx_wait_us;
        dma.tx_wait_us = i2s_dma.tx_wait_us;
        dma.events_valid = i2s_dma.events_valid && gdma_dma.events_valid;
        dma.memory_valid = i2s_dma.memory_valid;
        dma.timing_valid = i2s_dma.timing_valid && gdma_dma.timing_valid;
        usb_send_profile_response(sequence, &dma, sizeof(dma));
        return;
    }
#endif

#if !ENABLE_PROFILING
    header.status = STATUS_DISABLED;
    usb_send_profile_response(sequence, &header, sizeof(header));
    return;
#else

    if (request.operation == REQUEST_RESET) {
        if (request.session_id == 0) header.status = STATUS_BAD_REQUEST;
        else {
            profile_snapshot_request.store(0, std::memory_order_release);
            const uint32_t previous_session = profile_session_request.exchange(
                request.session_id, std::memory_order_acq_rel);
#if ENABLE_RESOURCE_METRICS
            if (previous_session != request.session_id) {
                portENTER_CRITICAL(&dma_callback_lock);
                dma_callback_metrics = {};
                portEXIT_CRITICAL(&dma_callback_lock);
                i2s_driver_metrics_reset();
                gdma_driver_metrics_reset();
            }
#endif
            header.status = audio_profiler.acknowledged_session() == request.session_id
                ? STATUS_OK : STATUS_PENDING;
        }
        usb_send_profile_response(sequence, &header, sizeof(header));
        return;
    }

    if (request.operation == REQUEST_SNAPSHOT) {
        if (request.snapshot_id == 0 ||
            audio_profiler.acknowledged_session() != request.session_id) {
            header.status = STATUS_BAD_REQUEST;
        } else {
            profile_snapshot_request.store(request.snapshot_id, std::memory_order_release);
            SnapshotStatus status{};
            status.header = header;
            status.owner_count = 1;
            status.acknowledged_session[0] = audio_profiler.acknowledged_session();
            status.acknowledged_snapshot[0] = audio_profiler.acknowledged_snapshot();
            if (status.acknowledged_snapshot[0] == request.snapshot_id)
                status.ready_mask = 1;
            else {
                status.incomplete_mask = 1;
                status.header.status = STATUS_PENDING;
            }
            usb_send_profile_response(sequence, &status, sizeof(status));
        }
        return;
    }

    if (request.operation == REQUEST_RELEASE) {
        if (profile_snapshot_request.load(std::memory_order_acquire) == request.snapshot_id)
            profile_snapshot_request.store(0, std::memory_order_release);
        audio_profiler.release_snapshot(request.snapshot_id);
        header.status = STATUS_OK;
        usb_send_profile_response(sequence, &header, sizeof(header));
        return;
    }

    if (request.operation == REQUEST_PAGE) {
        const profile_metrics::Bank<AP_STAGE_COUNT> *bank =
            audio_profiler.published_bank(request.snapshot_id);
        if (!bank) {
            header.status = STATUS_STALE_SNAPSHOT;
            usb_send_profile_response(sequence, &header, sizeof(header));
            return;
        }
        if (request.stage_id >= AP_STAGE_COUNT) {
            header.status = STATUS_BAD_REQUEST;
            usb_send_profile_response(sequence, &header, sizeof(header));
            return;
        }
#if !ENABLE_CAB_SIM
        if (request.stage_id == AP_CAB_RESET || request.stage_id == AP_CAB_PROCESS) {
            header.status = STATUS_UNAVAILABLE;
            usb_send_profile_response(sequence, &header, sizeof(header));
            return;
        }
#endif
        const profile_metrics::Stats &stats = bank->stages[request.stage_id];
        if (request.histogram_offset >= profile_metrics::HISTOGRAM_BINS) {
            header.status = STATUS_BAD_REQUEST;
            usb_send_profile_response(sequence, &header, sizeof(header));
            return;
        }
        StagePage page{};
        page.header = header;
        page.header.boundary_us = bank->boundary_us;
        page.stage_id = request.stage_id;
        page.stage_count = AP_STAGE_COUNT;
        page.histogram_offset = request.histogram_offset;
        const uint16_t remaining = profile_metrics::HISTOGRAM_BINS - request.histogram_offset;
        page.histogram_count = remaining < PAGE_HISTOGRAM_BINS ? remaining : PAGE_HISTOGRAM_BINS;
        page.calls = stats.calls;
        page.elapsed_us = stats.elapsed_us;
        page.samples = stats.samples;
        page.min_us = stats.calls ? stats.min_us : 0;
        page.max_us = stats.max_us;
        memcpy(page.mode_calls, stats.mode_calls, sizeof(page.mode_calls));
        page.histogram_overflow = stats.histogram_overflow;
        memcpy(page.histogram, stats.histogram + request.histogram_offset,
               page.histogram_count * sizeof(uint16_t));
        usb_send_profile_response(sequence, &page, sizeof(page));
        return;
    }

    if (request.operation == REQUEST_CYCLE_PAGE) {
        const profile_metrics::Bank<AP_STAGE_COUNT> *bank =
            audio_profiler.published_bank(request.snapshot_id);
        if (!bank || request.stage_id >= AP_STAGE_COUNT) {
            header.status = bank ? STATUS_BAD_REQUEST : STATUS_STALE_SNAPSHOT;
            usb_send_profile_response(sequence, &header, sizeof(header));
            return;
        }
        const auto &stats = bank->stages[request.stage_id];
        CyclePage page{};
        page.header = header;
        page.header.boundary_us = bank->boundary_us;
        page.stage_id = request.stage_id;
        page.stage_count = AP_STAGE_COUNT;
        page.calls = stats.calls;
        page.valid_calls = stats.cycle_calls;
        page.cycles_total = stats.cycles_total;
        page.cycles_min = stats.cycle_calls ? stats.cycles_min : 0;
        page.cycles_max = stats.cycles_max;
        page.invalid_calls = stats.cycle_invalid;
        page.not_instrumented_calls = stats.cycle_not_instrumented;
        page.core_changed_calls = stats.cycle_core_changed;
        page.frequency_changed_calls = stats.cycle_frequency_changed;
        page.wrap_ambiguous_calls = stats.cycle_wrap_ambiguous;
        page.cycle_core_id = stats.cycle_core_id;
        page.cycle_metadata_flags = stats.cycle_metadata_flags;
        page.cycle_cpu_frequency_hz = stats.cycle_cpu_frequency_hz;
        usb_send_profile_response(sequence, &page, sizeof(page));
        return;
    }

    header.status = STATUS_BAD_REQUEST;
    usb_send_profile_response(sequence, &header, sizeof(header));
#endif
}

static void usb_process_frame(const uint8_t *raw, size_t raw_len) {
    HostFrameHeader header{};
    uint8_t payload[255]{};
    size_t payload_len = 0;
    if (!decode_host_frame(raw, raw_len, &header, payload, sizeof(payload), &payload_len)) return;
    if (header.version != CONTROL_PROTOCOL_VERSION) {
        usb_send_status(HOST_RSP_ERROR, header.sequence, HOST_ERR_BAD_VERSION);
        return;
    }

    // A host retries a frame when an ACK is lost. Keep the complete decoded
    // mutation so CRC collisions cannot suppress a different command.
    const bool mutation = header.type == HOST_CMD_SET_PARAM ||
                          header.type == HOST_CMD_SET_ROUTE ||
                          header.type == HOST_CMD_PRESET;
    if (mutation && usb_mutation_cache.matches(
            header.sequence, header.type, payload, payload_len)) {
        usb_send_status(HOST_RSP_ACK, header.sequence,
                        (HostStatus)usb_mutation_cache.status());
        return;
    }
    auto mutation_reply = [&](HostStatus status) {
        if (mutation) {
            usb_mutation_cache.remember(header.sequence, header.type, payload,
                                        payload_len, (uint8_t)status);
        }
        usb_send_status(HOST_RSP_ACK, header.sequence, status);
    };

    switch ((HostCommand)header.type) {
        case HOST_CMD_PROFILE:
            usb_process_profile_request(header.sequence, payload, payload_len);
            return;

        case HOST_CMD_HELLO:
        case HOST_CMD_PING:
            if (payload_len != 0) {
                usb_send_status(HOST_RSP_ERROR, header.sequence, HOST_ERR_BAD_LENGTH);
                return;
            }
            usb_send_status(HOST_RSP_ACK, header.sequence, HOST_OK);
            if (header.type == HOST_CMD_HELLO) {
                usb_mutation_cache.reset();
                uint16_t effect_mask = (uint16_t)(EFFECT_CAP_BITCRUSHER | EFFECT_CAP_TREMOLO | EFFECT_CAP_PHASER);
#if ENABLE_CAB_SIM
                effect_mask |= EFFECT_CAP_CAB_SIM;
#endif
                HostCapsPacket caps{
                    SAMPLE_RATE, 24, 24, 1, audio_route.load(),
                    (uint8_t)I2S_SHARED_CLOCKS, CONTROL_PROTOCOL_VERSION,
                    PEDAL_STATE_VERSION,
                    effect_mask,
                    (uint16_t)sizeof(PedalState)
                };
                usb_send_response(HOST_RSP_CAPS, header.sequence, &caps, sizeof(caps));
            }
            return;

        case HOST_CMD_GET_CAPS: {
            if (payload_len != 0) {
                usb_send_status(HOST_RSP_ERROR, header.sequence, HOST_ERR_BAD_LENGTH);
                return;
            }
            uint16_t effect_mask = (uint16_t)(EFFECT_CAP_BITCRUSHER | EFFECT_CAP_TREMOLO | EFFECT_CAP_PHASER);
#if ENABLE_CAB_SIM
            effect_mask |= EFFECT_CAP_CAB_SIM;
#endif
            HostCapsPacket caps{
                SAMPLE_RATE, 24, 24, 1, audio_route.load(),
                (uint8_t)I2S_SHARED_CLOCKS, CONTROL_PROTOCOL_VERSION,
                PEDAL_STATE_VERSION,
                effect_mask,
                (uint16_t)sizeof(PedalState)
            };
            usb_send_response(HOST_RSP_CAPS, header.sequence, &caps, sizeof(caps));
            return;
        }

        case HOST_CMD_GET_STATE: {
            if (payload_len != 0) {
                usb_send_status(HOST_RSP_ERROR, header.sequence, HOST_ERR_BAD_LENGTH);
                return;
            }
            PedalState state = state_snapshot();
            usb_send_response(HOST_RSP_STATE, header.sequence, &state, sizeof(state));
            return;
        }

        case HOST_CMD_SET_PARAM: {
            if (payload_len != sizeof(HostSetParamPacket)) {
                usb_send_status(HOST_RSP_ERROR, header.sequence, HOST_ERR_BAD_LENGTH);
                return;
            }
            HostSetParamPacket command{};
            memcpy(&command, payload, sizeof(command));
            mutation_reply(apply_absolute_parameter(command.effect_idx, command.param_idx, command.value)
                               ? HOST_OK : HOST_ERR_BAD_VALUE);
            return;
        }

        case HOST_CMD_SET_ROUTE:
            if (payload_len != 1) {
                usb_send_status(HOST_RSP_ERROR, header.sequence, HOST_ERR_BAD_LENGTH);
                return;
            }
            mutation_reply(set_audio_route(payload[0]) ? HOST_OK : HOST_ERR_BAD_VALUE);
            return;

        case HOST_CMD_PRESET: {
            if (payload_len != sizeof(PresetPacket)) {
                usb_send_status(HOST_RSP_ERROR, header.sequence, HOST_ERR_BAD_LENGTH);
                return;
            }
            PresetPacket command{};
            memcpy(&command, payload, sizeof(command));
            bool ok = false;
            {
                ControlGuard guard;
                if (command.action == PRESET_SAVE) {
                    ok = PresetManager::savePreset(command.slot, state_snapshot());
                    if (ok) mark_active_preset_origin(command.slot);
                } else if (command.action == PRESET_LOAD) {
                    PedalState loaded{};
                    const bool found = PresetManager::loadPreset(command.slot, loaded);
                    normalize_state_for_build(loaded);
                    if (found && validPedalState(loaded, TOTAL_IR_PRESETS)) {
                        publish_state(loaded);
                        mark_active_preset_origin(command.slot);
                        broadcast_pedal_state(&loaded);
                        ok = true;
                    }
                }
                // A resposta e o espelho de estado usam a mesma UART da
                // Placa B. Mantê-los sob o mesmo lock evita intercalar o
                // quadro de resultado com uma atualização concorrente.
                send_preset_result(command.action, command.slot, ok);
            }
            mutation_reply(ok ? HOST_OK : HOST_ERR_BAD_VALUE);
            return;
        }

        case HOST_CMD_GET_METRICS: {
            if (payload_len != 0) {
                usb_send_status(HOST_RSP_ERROR, header.sequence, HOST_ERR_BAD_LENGTH);
                return;
            }
            HostMetricsPacket metrics{
                audio_io_errors.load(), i2s_drift_events.load(), slow_audio_blocks.load(),
                usb_overruns.load(), usb_underruns.load(),
                usb_control_overruns.load(), usb_control_tx_overruns.load(), usb_frames.load(),
                usb_stream_starts.load(), usb_stream_stops.load(), usb_cadence_errors.load(),
                max_audio_block_us.load(), max_cab_block_us.load(), cab_saturation_samples.load(),
                (uint8_t)usb_mounted.load(), audio_route.load(),
                max_new_effect_block_us.load(), dsp_quality_level.load(),
                usb_ring_frames.load(), usb_ring_high_water.load(),
                usb_resyncs.load(), usb_dropped_frames.load()
            };
            usb_send_response(HOST_RSP_METRICS, header.sequence, &metrics, sizeof(metrics));
            return;
        }

        default:
            usb_send_status(HOST_RSP_ERROR, header.sequence, HOST_ERR_UNKNOWN_CMD);
            return;
    }
}

static esp_err_t usb_audio_input_cb(uint8_t *buffer, size_t length,
                                    size_t *bytes_read, void *context) {
    (void)context;

    // Keep the 44.1 kHz cadence even when USB is selected as a silent route.
    // At the fixed 2 ms interval this is 88 or 89 samples per callback. For
    // intervals that do not contain an integer number of samples, the phase
    // accumulator alternates adjacent packet sizes.
    static uint32_t sample_phase = 0;
    sample_phase += SAMPLE_RATE * MIC_INTERVAL_MS;
    size_t samples = sample_phase / 1000;
    sample_phase %= 1000;
    const size_t frame_bytes = sizeof(int32_t);
    size_t output_bytes = samples * frame_bytes;

    // usb_device_uac 1.3.1 passes bytes_require (44 samples/ms rounded down)
    // as len, but allocates a double buffer large enough for one extra frame.
    // Use that extra frame for the 441st sample only when the compiled
    // component advertises the required capacity; otherwise stay within len.
    size_t callback_capacity = length;
#if defined(CFG_TUD_AUDIO_FUNC_1_EP_IN_SW_BUF_SZ)
    if ((size_t)CFG_TUD_AUDIO_FUNC_1_EP_IN_SW_BUF_SZ >= length + frame_bytes)
        callback_capacity = length + frame_bytes;
#endif
    if (output_bytes > callback_capacity) {
        usb_cadence_errors.fetch_add(1, std::memory_order_relaxed);
        samples = callback_capacity / frame_bytes;
        output_bytes = samples * sizeof(int32_t);
    }

    const TickType_t now = xTaskGetTickCount();
    const TickType_t previous = usb_last_callback_tick.exchange(now, std::memory_order_acq_rel);
    if (previous == 0) {
        usb_stream_starts.fetch_add(1, std::memory_order_relaxed);
    } else {
        const TickType_t elapsed = now - previous;
        const TickType_t expected = pdMS_TO_TICKS(MIC_INTERVAL_MS);
        const TickType_t tolerance = pdMS_TO_TICKS(2);
        if (elapsed > expected + tolerance ||
            (expected > tolerance && elapsed + tolerance < expected)) {
            usb_cadence_errors.fetch_add(1, std::memory_order_relaxed);
        }
        if (elapsed > pdMS_TO_TICKS(100)) {
            usb_stream_stops.fetch_add(1, std::memory_order_relaxed);
            usb_stream_starts.fetch_add(1, std::memory_order_relaxed);
            usb_pcm_ring.clear();
        }
    }

    if (!usb_mounted.load(std::memory_order_acquire) ||
        !(audio_route.load(std::memory_order_acquire) & AUDIO_ROUTE_USB)) {
        usb_stream_primed.store(false, std::memory_order_release);
        *bytes_read = output_bytes;
        memset(buffer, 0, output_bytes);
        usb_frames.fetch_add((uint32_t)samples, std::memory_order_relaxed);
        return ESP_OK;
    }

    size_t available = usb_pcm_ring.available();
    if (available > USB_AUDIO_RESYNC_WATER) {
        const size_t discarded = usb_pcm_ring.discard_oldest(available - USB_AUDIO_PRIME_FRAMES);
        if (discarded) {
            usb_resyncs.fetch_add(1, std::memory_order_relaxed);
            usb_dropped_frames.fetch_add(static_cast<uint32_t>(discarded), std::memory_order_relaxed);
        }
        available = usb_pcm_ring.available();
    }

    // Prime with only a few milliseconds of audio. This avoids a startup
    // underrun without allowing the former multi-hundred-millisecond queue.
    if (!usb_stream_primed.load(std::memory_order_acquire)) {
        if (available < USB_AUDIO_PRIME_FRAMES) {
            *bytes_read = output_bytes;
            memset(buffer, 0, output_bytes);
            // Silêncio durante a pré-carga é intencional, não uma falha de
            // regime: o contador só deve refletir uma fila que já estava
            // sincronizada e depois não conseguiu atender o callback.
            usb_frames.fetch_add((uint32_t)samples, std::memory_order_relaxed);
            return ESP_OK;
        }
        usb_stream_primed.store(true, std::memory_order_release);
    }

    // A small occupancy correction compensates for the independent I2S and
    // USB clocks while preserving the 44.1 kHz average cadence.
    const size_t min_samples = (SAMPLE_RATE * MIC_INTERVAL_MS) / 1000;
    const size_t max_samples = callback_capacity / frame_bytes;
    if (available > USB_AUDIO_HIGH_WATER && samples < max_samples)
        ++samples;
    else if (available < USB_AUDIO_LOW_WATER && samples > min_samples)
        --samples;
    output_bytes = samples * frame_bytes;

    size_t popped = usb_pcm_ring.pop(reinterpret_cast<int32_t *>(buffer), samples);
    if (usb_pcm_ring.take_reset_consumed()) {
        usb_stream_primed.store(false, std::memory_order_release);
        memset(buffer, 0, output_bytes);
        *bytes_read = output_bytes;
        usb_frames.fetch_add((uint32_t)samples, std::memory_order_relaxed);
        return ESP_OK;
    }
    if (popped < samples) {
        memset(buffer + popped * sizeof(int32_t), 0,
               (samples - popped) * sizeof(int32_t));
        usb_underruns.fetch_add(1, std::memory_order_relaxed);
    }
    *bytes_read = output_bytes;
    usb_frames.fetch_add((uint32_t)samples, std::memory_order_relaxed);
    return ESP_OK;
}

extern "C" void tud_cdc_rx_cb(uint8_t itf) {
    if (itf != 0) return;
    uint8_t buffer[64];
    while (tud_cdc_available()) {
        uint32_t count = tud_cdc_read(buffer, sizeof(buffer));
        for (uint32_t i = 0; i < count; ++i) usb_control_ring.push(buffer[i]);
    }
    if (usb_control_task_handle) xTaskNotifyGive(usb_control_task_handle);
}

extern "C" void tud_cdc_line_state_cb(uint8_t itf, bool dtr, bool rts) {
    (void)dtr;
    (void)rts;
    if (itf == 0)
        usb_control_session_epoch.fetch_add(1, std::memory_order_acq_rel);
}

extern "C" void tud_mount_cb(void) {
    usb_control_session_epoch.fetch_add(1, std::memory_order_acq_rel);
    usb_mounted.store(true, std::memory_order_release);
    usb_last_callback_tick.store(0, std::memory_order_release);
    usb_stream_primed.store(false, std::memory_order_release);
    usb_pcm_ring.clear();
}

extern "C" void tud_umount_cb(void) {
    usb_control_session_epoch.fetch_add(1, std::memory_order_acq_rel);
    usb_mounted.store(false, std::memory_order_release);
    usb_stream_primed.store(false, std::memory_order_release);
    if (usb_last_callback_tick.exchange(0, std::memory_order_acq_rel) != 0)
        usb_stream_stops.fetch_add(1, std::memory_order_relaxed);
    usb_pcm_ring.clear();
}

extern "C" void tud_suspend_cb(bool remote_wakeup_en) {
    (void)remote_wakeup_en;
    usb_control_session_epoch.fetch_add(1, std::memory_order_acq_rel);
    usb_mounted.store(false, std::memory_order_release);
    usb_stream_primed.store(false, std::memory_order_release);
    if (usb_last_callback_tick.exchange(0, std::memory_order_acq_rel) != 0)
        usb_stream_stops.fetch_add(1, std::memory_order_relaxed);
    usb_pcm_ring.clear();
}

extern "C" void tud_resume_cb(void) {
    usb_control_session_epoch.fetch_add(1, std::memory_order_acq_rel);
    usb_mounted.store(true, std::memory_order_release);
    usb_last_callback_tick.store(0, std::memory_order_release);
    usb_stream_primed.store(false, std::memory_order_release);
}

static void usb_control_task(void *args) {
    (void)args;
    ESP_ERROR_CHECK(esp_task_wdt_add(nullptr));
#if ENABLE_WDT_TEST_HANG
    while (true) taskYIELD();
#endif
    uint8_t raw[512];
    size_t raw_len = 0;
    uint32_t observed_session_epoch = usb_control_session_epoch.load(std::memory_order_acquire);
    while (true) {
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(20));
        const uint32_t session_epoch = usb_control_session_epoch.load(std::memory_order_acquire);
        if (session_epoch != observed_session_epoch) {
            usb_mutation_cache.reset();
            observed_session_epoch = session_epoch;
            raw_len = 0;
            usb_control_ring.clear();
        }
        uint8_t byte;
        while (usb_control_ring.pop(&byte)) {
            if (byte == 0) {
                if (raw_len) usb_process_frame(raw, raw_len);
                raw_len = 0;
            } else if (raw_len < sizeof(raw)) {
                raw[raw_len++] = byte;
            } else {
                raw_len = 0;
                usb_control_overruns.fetch_add(1, std::memory_order_relaxed);
            }
        }
        (void)esp_task_wdt_reset();
    }
}

static esp_err_t init_usb_audio(void) {
    if (xTaskCreatePinnedToCore(usb_control_task, "usb_ctrl", 6144, NULL, 8,
                                &usb_control_task_handle, 0) != pdPASS)
        return ESP_ERR_NO_MEM;
    uac_device_config_t config = {
        .skip_tinyusb_init = false,
        .output_cb = NULL,
        .input_cb = usb_audio_input_cb,
        .set_mute_cb = NULL,
        .set_volume_cb = NULL,
        .cb_ctx = NULL,
        .spk_itf_num = 0,
        .mic_itf_num = 1,
    };
    return uac_device_init(&config);
}
#else
static esp_err_t init_usb_audio(void) { return ESP_OK; }
#endif

// -------------------------------------------------------------
// CORE 0: PROTOCOLO FULL-DUPLEX UART (460.800 BPS) & NVS
// -------------------------------------------------------------
static void full_duplex_comm_task(void *args) {
    uint8_t rx_raw[512];
    uint8_t decoded[512];
    int rx_pos = 0;
    TickType_t last_state_sync = xTaskGetTickCount();

    ESP_ERROR_CHECK(esp_task_wdt_add(nullptr));
#if ENABLE_WDT_TEST_HANG
    while (true) taskYIELD();
#endif
    vTaskDelay(pdMS_TO_TICKS(500));
    PedalState initial = state_snapshot();
    broadcast_pedal_state(&initial);
    broadcast_audio_route(audio_route.load(std::memory_order_acquire));
    send_preset_status();
    last_state_sync = xTaskGetTickCount();

    while (1) {
        uint8_t byte;
        if (uart_read_bytes(UART_PORT_NUM, &byte, 1, pdMS_TO_TICKS(50)) > 0) {
            if (byte == 0x00) {
                if (rx_pos > 0) {
                    size_t dec_len = decode_packet(rx_raw, rx_pos, decoded);
                    if (dec_len > 0) {
                        uint8_t type = decoded[0];
                        const uint8_t* data = &decoded[1];

                        switch (type) {
                            case PKT_AI_GATE: {
                                if (dec_len == 1 + sizeof(float)) {
                                    float ai_val;
                                    memcpy(&ai_val, data, sizeof(float));
                                    if (isfinite(ai_val)) {
                                        external_ai_gate.store(fminf(fmaxf(ai_val, 0.0f), 1.0f));
                                        last_ai_tick.store(xTaskGetTickCount());
                                    }
                                }
                                break;
                            }
                            case PKT_TUNER_STATE: {
                                if (dec_len == 1 + sizeof(TunerStatePacket) &&
                                    (data[0] == 0 || data[0] == 1)) {
                                    tuner_mute.store(data[0] != 0, std::memory_order_release);
                                    last_tuner_state_tick.store(xTaskGetTickCount(), std::memory_order_release);
                                }
                                break;
                            }
                            case PKT_UI_PARAM_INC: {
                                if (dec_len == 1 + sizeof(UiParamIncPacket)) {
                                    UiParamIncPacket pkt;
                                    memcpy(&pkt, data, sizeof(UiParamIncPacket));
                                    (void)apply_param_increment(pkt.effect_idx, pkt.param_idx, pkt.delta);
                                }
                                break;
                            }
                            case PKT_PRESET_ACTION: {
                                if (dec_len == 1 + sizeof(PresetPacket)) {
                                    PresetPacket pkt;
                                    memcpy(&pkt, data, sizeof(PresetPacket));
                                    bool ok = false;
                                    {
                                        ControlGuard guard;
                                        if (pkt.action == PRESET_SAVE) {
                                            PedalState saved = state_snapshot();
                                            ok = PresetManager::savePreset(pkt.slot, saved);
                                            if (ok) mark_active_preset_origin(pkt.slot);
                                        } else if (pkt.action == PRESET_LOAD) {
                                            PedalState loaded{};
                                            const bool found = PresetManager::loadPreset(pkt.slot, loaded);
                                            normalize_state_for_build(loaded);
                                            if (found && validPedalState(loaded, TOTAL_IR_PRESETS)) {
                                                publish_state(loaded);
                                                mark_active_preset_origin(pkt.slot);
                                                broadcast_pedal_state(&loaded);
                                                ok = true;
                                            }
                                        }
                                        send_preset_result(pkt.action, pkt.slot, ok);
                                    }
                                }
                                break;
                            }
                        }
                    }
                }
                rx_pos = 0;
            } else {
                if (rx_pos < sizeof(rx_raw)) rx_raw[rx_pos++] = byte;
            }
        }
        const TickType_t now = xTaskGetTickCount();
        if (recovery_save_pending.load(std::memory_order_acquire) &&
            now - recovery_last_change_tick.load(std::memory_order_acquire) >=
                pdMS_TO_TICKS(2000)) {
            const uint32_t generation = recovery_generation.load(std::memory_order_acquire);
            bool saved = false;
            {
                ControlGuard guard;
                saved = PresetManager::saveRecovery(
                    state_snapshot(), active_preset_slot.load(std::memory_order_acquire),
                    active_preset_edited.load(std::memory_order_acquire));
            }
            if (saved && generation == recovery_generation.load(std::memory_order_acquire))
                recovery_save_pending.store(false, std::memory_order_release);
        }
        if (preset_status_pending.exchange(false, std::memory_order_acq_rel)) {
            send_preset_status();
        }

        // Reenvia o espelho em intervalos curtos para cobrir a inicialização
        // tardia ou uma perda isolada na UART. O quadro é pequeno frente a
        // 460.800 bps e não altera o fluxo de áudio.
        if (now - last_state_sync >= pdMS_TO_TICKS(500)) {
            PedalState current = state_snapshot();
            broadcast_pedal_state(&current);
            broadcast_audio_route(audio_route.load(std::memory_order_acquire));
            send_preset_status();
            last_state_sync = now;
        }
        (void)esp_task_wdt_reset();
    }
}

#if ENABLE_SERIAL_DEBUG
static void debug_print_task(void *args) {
    while (1) {
        PedalState snapshot = state_snapshot();
        PedalState* state = &snapshot;
#if ENABLE_CAB_SIM
        uint8_t cur_ir = (state->cab_index < TOTAL_IR_PRESETS) ? state->cab_index : 0;
#endif

        printf("\n=== DSP MASTER CORE 1 | IA GATE: %3.0f%% ===\n", external_ai_gate.load() * 100.0f);
        printf("GATE:[%s] | COMP:[%s] | WAH:[%s] | DIST:[%s]\n",
               state->gate_active ? "ON" : "OFF", state->comp_active ? "ON" : "OFF",
               state->wah_active ? "ON" : "OFF", state->dist_active ? "ON" : "OFF");
        printf("EQ:[%s] | MOD:[%s] | DLY:[%s] | REV:[%s]\n",
               state->eq_active ? "ON" : "OFF", state->mod_active ? "ON" : "OFF",
               state->delay_active ? "ON" : "OFF", state->reverb_active ? "ON" : "OFF");
        printf("BIT:[%s] | TREM:[%s] | PHAS:[%s]\n",
               state->bitcrusher_active ? "ON" : "OFF",
               state->tremolo_active ? "ON" : "OFF",
               state->phaser_active ? "ON" : "OFF");
        const uint8_t quality = dsp_quality_level.load(std::memory_order_relaxed);
#if ENABLE_CAB_SIM
        printf("CAB:[%s] -> [%s] (Slot %d/%d) | processamento:[%s]\n",
               state->cab_active ? "ON" : "OFF", IR_PRESETS[cur_ir].name, cur_ir + 1, TOTAL_IR_PRESETS,
               (state->cab_active && !cab_failsafe_bypass.load(std::memory_order_relaxed) && quality < 4) ? "ON" : "BYPASS");
#else
        printf("CAB:[DESATIVADO POR MACRO] | processamento:[BYPASS]\n");
#endif
        printf("Erros I2S: %lu\n", (unsigned long)audio_io_errors.load());
        printf("Deriva/blocos I2S fora do esperado: %lu\n", (unsigned long)i2s_drift_events.load());
        printf("Blocos DSP acima do prazo: %lu\n", (unsigned long)slow_audio_blocks.load());
        printf("USB: %s | rota %u | overrun %lu | underrun acumulado %lu\n",
               usb_mounted.load() ? "conectado" : "desconectado",
               (unsigned)audio_route.load(),
               (unsigned long)usb_overruns.load(),
               (unsigned long)usb_underruns.load());
        printf("USB CDC: RX overrun %lu | TX overrun %lu\n",
               (unsigned long)usb_control_overruns.load(),
               (unsigned long)usb_control_tx_overruns.load());
        printf("USB fluxo: inicio %lu | fim %lu | cadencia %lu\n",
               (unsigned long)usb_stream_starts.load(),
               (unsigned long)usb_stream_stops.load(),
               (unsigned long)usb_cadence_errors.load());
        printf("USB anel: atual %lu | max %lu | resincronizacoes %lu | descartadas %lu\n",
               (unsigned long)usb_ring_frames.load(std::memory_order_relaxed),
               (unsigned long)usb_ring_high_water.load(std::memory_order_relaxed),
               (unsigned long)usb_resyncs.load(std::memory_order_relaxed),
               (unsigned long)usb_dropped_frames.load(std::memory_order_relaxed));
        printf("DSP: ultimo %lu us / max %lu us | novos efeitos max: %lu us | Cab: ultimo %lu us / max %lu us | Cab lentos: %lu | clips: %lu\n",
               (unsigned long)last_audio_block_us.load(std::memory_order_relaxed),
               (unsigned long)max_audio_block_us.load(),
               (unsigned long)max_new_effect_block_us.load(),
               (unsigned long)last_cab_block_us.load(std::memory_order_relaxed),
               (unsigned long)max_cab_block_us.load(),
               (unsigned long)cab_slow_blocks.load(std::memory_order_relaxed),
               (unsigned long)cab_saturation_samples.load());
        printf("Qualidade DSP: nivel %u | blocos reduzidos: %lu\n",
               (unsigned)quality,
               (unsigned long)dsp_shed_blocks.load(std::memory_order_relaxed));
        printf("===========================================\n");

        vTaskDelay(pdMS_TO_TICKS(3000));
    }
}
#endif

// -------------------------------------------------------------
// CORE 1: KERNEL DE ÁUDIO EM TEMPO REAL (DSP COMPLETO)
// -------------------------------------------------------------
static void audio_processing_task(void *args) {
#if ENABLE_RESOURCE_METRICS
    alignas(16) int32_t audio_buffer[BUFFER_SAMPLES] = {};
#else
    alignas(16) int32_t audio_buffer[BUFFER_SAMPLES];
#endif
    alignas(16) int32_t usb_block[DMA_FRAME_NUM];
    alignas(16) float mono_samples[DMA_FRAME_NUM];
#if ENABLE_CAB_SIM && !LATENCY_TEST_MODE
    alignas(16) float ir_output[DMA_FRAME_NUM];
#endif
#if ENABLE_RESOURCE_METRICS
    uint32_t task_buffer_locations = 0;
    task_buffer_locations = set_buffer_location(task_buffer_locations,
        profile_protocol::BUFFER_SLOT_RAW_I2S, audio_buffer);
    task_buffer_locations = set_buffer_location(task_buffer_locations,
        profile_protocol::BUFFER_SLOT_AUDIO_FRAME, mono_samples);
    audio_task_buffer_locations.store(task_buffer_locations, std::memory_order_release);
#endif

    size_t bytes_read = 0, bytes_written = 0;
    uint8_t consecutive_io_errors = 0;
#if !LATENCY_TEST_MODE
        bool eq_configured = false;
        float eq_gains[4] = {NAN, NAN, NAN, NAN};
#endif
#if ENABLE_CAB_SIM && !LATENCY_TEST_MODE
        bool cab_was_active = false;
        uint8_t cab_last_ir = 0;
        uint8_t cab_requested_ir = 0;
        bool cab_requested_prev = false;
        bool cab_shed_latched = false;
#endif
        uint8_t shed_level = 0;
        uint8_t slow_streak = 0;
        uint16_t recovery_streak = 0;
#if !LATENCY_TEST_MODE
        float tuner_output_gain = 1.0f;
#endif

    ESP_ERROR_CHECK(esp_task_wdt_add(nullptr));
#if ENABLE_WDT_TEST_HANG
    while (true) taskYIELD();
#endif
    // Dá tempo para os reguladores e codecs externos estabilizarem antes de
    // habilitar os clocks I2S. O DSP permanece nesta tarefa, dono exclusivo
    // dos canais, inclusive durante a recuperação.
    vTaskDelay(pdMS_TO_TICKS(500));
    while (1) {
        (void)esp_task_wdt_reset();
        if (!rx_chan || !tx_chan) {
            if (!ensure_i2s_system()) {
                ESP_LOGE(TAG, "I2S indisponivel; nova tentativa em 5 s");
                vTaskDelay(pdMS_TO_TICKS(5000));
                continue;
            }
            consecutive_io_errors = 0;
        }
#if ENABLE_PROFILING
        PROFILE_START(rx_wait_profile);
#endif
        bytes_read = 0;
        const esp_err_t read_status = i2s_channel_read(rx_chan, audio_buffer, sizeof(audio_buffer), &bytes_read, 100);
#if ENABLE_PROFILING
        PROFILE_FINISH(audio_profiler, AP_RX_WAIT, profile_metrics::MODE_ACTIVE,
                       bytes_read / 8, rx_wait_profile);
#endif
        if (read_status != ESP_OK || bytes_read == 0 || bytes_read % 8 != 0) {
            audio_io_errors.fetch_add(1);
            i2s_drift_events.fetch_add(1);
            memset(audio_buffer, 0, sizeof(audio_buffer));
            size_t silence_written = 0;
            const esp_err_t silence_status = i2s_channel_write(
                tx_chan, audio_buffer, sizeof(audio_buffer), &silence_written, 100);
            if (silence_status != ESP_OK || silence_written != sizeof(audio_buffer))
                audio_io_errors.fetch_add(1);
            if (++consecutive_io_errors >= 3) {
                ESP_LOGE(TAG, "Tres falhas consecutivas na leitura I2S; reinicializando canais");
                deinit_i2s_system();
                consecutive_io_errors = 0;
            }
            continue;
        }
        int64_t processing_started = esp_timer_get_time();
#if ENABLE_PROFILING
        PROFILE_START(block_total_profile);
        PROFILE_START(housekeeping_profile);
        audio_profiler.reset_request(profile_session_request.load(std::memory_order_acquire));
        audio_profiler.snapshot_request(profile_snapshot_request.load(std::memory_order_acquire));
        audio_profiler.service((uint64_t)esp_timer_get_time());
        PROFILE_FINISH(audio_profiler, AP_HOUSEKEEPING, profile_metrics::MODE_ACTIVE,
                       0, housekeeping_profile);
#endif
        const int frames = bytes_read / 8;
        if (frames != DMA_FRAME_NUM) i2s_drift_events.fetch_add(1, std::memory_order_relaxed);
        PedalState snapshot{};
#if ENABLE_PROFILING
        PROFILE_EXEC(audio_profiler, AP_STATE_SNAPSHOT, profile_metrics::MODE_ACTIVE, 0,
                     snapshot = state_snapshot());
#else
        snapshot = state_snapshot();
#endif
        PedalState* state = &snapshot;
#if ENABLE_CAB_SIM && !LATENCY_TEST_MODE
        const uint8_t requested_ir = (state->cab_index < TOTAL_IR_PRESETS) ? state->cab_index : 0;
        // Uma sobrecarga persistente não deve ficar alternando o CAB a cada
        // poucos blocos, pois isso produziria uma modulação audível. Depois de
        // um nível 4, o bypass fica travado até o usuário desligar o CAB ou
        // selecionar outra IR.
        if (!state->cab_active || !cab_requested_prev || cab_requested_ir != requested_ir) {
            cab_shed_latched = false;
            cab_failsafe_bypass.store(false, std::memory_order_relaxed);
        }
        cab_requested_prev = state->cab_active;
        cab_requested_ir = requested_ir;
#endif
        // O nível é aplicado ao bloco inteiro para evitar que um efeito seja
        // interrompido no meio do processamento. O CAB permanece prioritário
        // até o nível 4; EQ e efeitos de memória longa são reduzidos antes.
        const uint8_t quality_level = shed_level;
        if (quality_level > 0) dsp_shed_blocks.fetch_add(1, std::memory_order_relaxed);

        // 1. Desentrelaçamento & Conversão int32 -> float
#if ENABLE_PROFILING
        PROFILE_EXEC(audio_profiler, AP_INPUT_CONVERT, profile_metrics::MODE_ACTIVE, frames,
        {
#endif
        #pragma GCC ivdep
        for (int i = 0; i < frames; i++) {
            mono_samples[i] = (float)audio_buffer[i * 2] * INT32_TO_FLOAT;
        }
#if ENABLE_PROFILING
        });
#endif

#if LATENCY_TEST_MODE
        // Ignora o estado dos efeitos e o comando de gate da placa B. Os
        // valores originais continuam armazenados e sincronizados normalmente.
        PROFILE_COUNT(audio_profiler, AP_GATE, profile_metrics::MODE_SKIPPED);
        PROFILE_COUNT(audio_profiler, AP_COMPRESSOR, profile_metrics::MODE_SKIPPED);
        PROFILE_COUNT(audio_profiler, AP_AUTOWAH, profile_metrics::MODE_SKIPPED);
        PROFILE_COUNT(audio_profiler, AP_OVERDRIVE, profile_metrics::MODE_SKIPPED);
        PROFILE_COUNT(audio_profiler, AP_BITCRUSHER, profile_metrics::MODE_SKIPPED);
        PROFILE_COUNT(audio_profiler, AP_EQ_CONFIG, profile_metrics::MODE_SKIPPED);
        PROFILE_COUNT(audio_profiler, AP_EQ_PROCESS, profile_metrics::MODE_SKIPPED);
        PROFILE_COUNT(audio_profiler, AP_PHASER, profile_metrics::MODE_SKIPPED);
        PROFILE_COUNT(audio_profiler, AP_CHORUS, profile_metrics::MODE_SKIPPED);
        PROFILE_COUNT(audio_profiler, AP_TREMOLO, profile_metrics::MODE_SKIPPED);
        PROFILE_COUNT(audio_profiler, AP_DELAY, profile_metrics::MODE_SKIPPED);
        PROFILE_COUNT(audio_profiler, AP_REVERB, profile_metrics::MODE_SKIPPED);
        PROFILE_COUNT(audio_profiler, AP_CAB_RESET, profile_metrics::MODE_SKIPPED);
        PROFILE_COUNT(audio_profiler, AP_CAB_PROCESS, profile_metrics::MODE_SKIPPED);
#else
        // 2. Noise Gate Híbrido (Envelope interno modulado pela IA externa)
        if (state->gate_active) {
            TickType_t age = xTaskGetTickCount() - last_ai_tick.load();
            float ai_gate_factor = age < pdMS_TO_TICKS(500)
                ? external_ai_gate.load() : 1.0f;
#if ENABLE_PROFILING
            PROFILE_EXEC(audio_profiler, AP_GATE, profile_metrics::MODE_ACTIVE, frames,
                         noise_gate.process(mono_samples, frames, state->gate_threshold,
                                            ai_gate_factor, SAMPLE_RATE));
#else
            noise_gate.process(mono_samples, frames, state->gate_threshold,
                               ai_gate_factor, SAMPLE_RATE);
#endif
        } else {
            PROFILE_COUNT(audio_profiler, AP_GATE, profile_metrics::MODE_SKIPPED);
        }

        // 3. Compressor VCA
        if (state->comp_active) {
#if ENABLE_PROFILING
            PROFILE_EXEC(audio_profiler, AP_COMPRESSOR, profile_metrics::MODE_ACTIVE, frames,
                         comp.process(mono_samples, frames, state->comp_threshold,
                                      state->comp_ratio, state->comp_makeup_gain));
#else
            comp.process(mono_samples, frames, state->comp_threshold, state->comp_ratio, state->comp_makeup_gain);
#endif
        } else {
            PROFILE_COUNT(audio_profiler, AP_COMPRESSOR, profile_metrics::MODE_SKIPPED);
        }

        // 4. Auto-Wah
        if (state->wah_active) {
#if ENABLE_PROFILING
            PROFILE_EXEC(audio_profiler, AP_AUTOWAH, profile_metrics::MODE_ACTIVE, frames,
                         autowah.process(mono_samples, frames, state->wah_sensitivity,
                                         state->wah_base_freq, state->wah_resonance, SAMPLE_RATE));
#else
            autowah.process(mono_samples, frames, state->wah_sensitivity, state->wah_base_freq, state->wah_resonance, SAMPLE_RATE);
#endif
        } else {
            PROFILE_COUNT(audio_profiler, AP_AUTOWAH, profile_metrics::MODE_SKIPPED);
        }

        // 5. Overdrive / Saturação (SIMD)
        if (state->dist_active) {
#if ENABLE_PROFILING
            PROFILE_EXEC(audio_profiler, AP_OVERDRIVE, profile_metrics::MODE_ACTIVE, frames,
                         overdrive.process(mono_samples, frames, state->dist_drive,
                                           state->dist_tone, state->dist_level));
#else
            overdrive.process(mono_samples, frames, state->dist_drive, state->dist_tone, state->dist_level);
#endif
        } else {
            PROFILE_COUNT(audio_profiler, AP_OVERDRIVE, profile_metrics::MODE_SKIPPED);
        }

        const int64_t new_effects_started = esp_timer_get_time();
        // 6. Bitcrusher: quantização e retenção de amostras, sem memória longa.
#if ENABLE_PROFILING
        PROFILE_EXEC(audio_profiler, AP_BITCRUSHER,
                     state->bitcrusher_active ? profile_metrics::MODE_ACTIVE : profile_metrics::MODE_BYPASS,
                     frames,
                     bitcrusher.process(mono_samples, frames, state->bitcrusher_bits,
                                        state->bitcrusher_hold, state->bitcrusher_mix,
                                        state->bitcrusher_active));
#else
        bitcrusher.process(mono_samples, frames, state->bitcrusher_bits,
                           state->bitcrusher_hold, state->bitcrusher_mix,
                           state->bitcrusher_active);
#endif

        // 7. Equalizador Paramétrico de 4 Bandas
        if (state->eq_active && quality_level < 3) {
            const float gains[4] = {
                state->eq_low_gain, state->eq_mid1_gain,
                state->eq_mid2_gain, state->eq_high_gain
            };
            if (!eq_configured || memcmp(gains, eq_gains, sizeof(gains)) != 0) {
#if ENABLE_PROFILING
                PROFILE_EXEC(audio_profiler, AP_EQ_CONFIG, profile_metrics::MODE_ACTIVE, 0,
                {
#endif
                eq_low.setPeaking(100.0f, gains[0], 0.707f, SAMPLE_RATE);
                eq_mid1.setPeaking(500.0f, gains[1], 1.0f, SAMPLE_RATE);
                eq_mid2.setPeaking(1500.0f, gains[2], 1.0f, SAMPLE_RATE);
                eq_high.setPeaking(4500.0f, gains[3], 0.707f, SAMPLE_RATE);
#if ENABLE_PROFILING
                });
#endif
                memcpy(eq_gains, gains, sizeof(gains));
                eq_configured = true;
            }

#if ENABLE_PROFILING
            PROFILE_EXEC(audio_profiler, AP_EQ_PROCESS, profile_metrics::MODE_ACTIVE, frames,
            {
#endif
            for (int i = 0; i < frames; i++) {
                mono_samples[i] = eq_high.process(eq_mid2.process(eq_mid1.process(eq_low.process(mono_samples[i]))));
            }
#if ENABLE_PROFILING
            });
#endif
        } else {
            eq_configured = false;
            PROFILE_COUNT(audio_profiler, AP_EQ_PROCESS,
                quality_level >= 3 ? profile_metrics::MODE_REDUCED : profile_metrics::MODE_SKIPPED);
        }

        // 8. Phaser de quatro all-pass. É o primeiro efeito reduzido quando
        // a combinação completa ultrapassa o orçamento de um bloco.
#if ENABLE_PROFILING
        const profile_metrics::CallMode phaser_mode = quality_level >= 3
            ? profile_metrics::MODE_REDUCED
            : (state->phaser_active ? profile_metrics::MODE_ACTIVE : profile_metrics::MODE_BYPASS);
        PROFILE_EXEC(audio_profiler, AP_PHASER, phaser_mode, frames,
                     phaser.process(mono_samples, frames, state->phaser_rate_hz,
                                    state->phaser_depth, state->phaser_feedback,
                                    state->phaser_mix, SAMPLE_RATE,
                                    state->phaser_active && quality_level < 3));
#else
        phaser.process(mono_samples, frames, state->phaser_rate_hz,
                       state->phaser_depth, state->phaser_feedback,
                       state->phaser_mix, SAMPLE_RATE,
                       state->phaser_active && quality_level < 3);
#endif

        // 9. Modulação (Chorus / Flanger)
        if (state->mod_active) {
#if ENABLE_PROFILING
            PROFILE_EXEC(audio_profiler, AP_CHORUS, profile_metrics::MODE_ACTIVE, frames,
                         chorus.process(mono_samples, frames, state->mod_rate_hz,
                                        state->mod_depth, state->mod_mix, SAMPLE_RATE));
#else
            chorus.process(mono_samples, frames, state->mod_rate_hz, state->mod_depth, state->mod_mix, SAMPLE_RATE);
#endif
        } else {
            PROFILE_COUNT(audio_profiler, AP_CHORUS, profile_metrics::MODE_SKIPPED);
        }

        // Tremolo vem antes dos efeitos de ambiência para que as repetições
        // mantenham a dinâmica modulada.
#if ENABLE_PROFILING
        PROFILE_EXEC(audio_profiler, AP_TREMOLO,
                     state->tremolo_active ? profile_metrics::MODE_ACTIVE : profile_metrics::MODE_BYPASS,
                     frames,
                     tremolo.process(mono_samples, frames, state->tremolo_rate_hz,
                                     state->tremolo_depth, state->tremolo_shape,
                                     state->tremolo_mix, SAMPLE_RATE,
                                     state->tremolo_active));
#else
        tremolo.process(mono_samples, frames, state->tremolo_rate_hz,
                        state->tremolo_depth, state->tremolo_shape,
                        state->tremolo_mix, SAMPLE_RATE,
                        state->tremolo_active);
#endif
        update_max(max_new_effect_block_us,
                   (uint32_t)(esp_timer_get_time() - new_effects_started));

        // Quando o bloco anterior perdeu o prazo, suspenda primeiro os efeitos
        // com memória longa e o EQ. O CAB só é suspenso no nível 4.

        // 10. Tape Delay
        if (state->delay_active && quality_level < 2) {
#if ENABLE_PROFILING
            PROFILE_EXEC(audio_profiler, AP_DELAY, profile_metrics::MODE_ACTIVE, frames,
                         tape_delay.process(mono_samples, frames, state->delay_time_samples,
                                            state->delay_feedback, state->delay_mix));
#else
            tape_delay.process(mono_samples, frames, state->delay_time_samples, state->delay_feedback, state->delay_mix);
#endif
        } else {
            PROFILE_COUNT(audio_profiler, AP_DELAY,
                quality_level >= 2 ? profile_metrics::MODE_REDUCED : profile_metrics::MODE_SKIPPED);
        }

        // 11. Reverb Dattorro Plate
        if (state->reverb_active && quality_level < 1) {
#if ENABLE_PROFILING
            PROFILE_EXEC(audio_profiler, AP_REVERB, profile_metrics::MODE_ACTIVE, frames,
                         reverb.process(mono_samples, frames, state->reverb_decay,
                                        state->reverb_damping, state->reverb_mix));
#else
            reverb.process(mono_samples, frames, state->reverb_decay, state->reverb_damping, state->reverb_mix);
#endif
        } else {
            PROFILE_COUNT(audio_profiler, AP_REVERB,
                quality_level >= 1 ? profile_metrics::MODE_REDUCED : profile_metrics::MODE_SKIPPED);
        }

        // 10. Cab Sim com Seleção Dinâmica de IR
#if ENABLE_CAB_SIM
        if (state->cab_active && quality_level < 4 && !cab_shed_latched) {
            uint8_t ir_idx = requested_ir;
#if ENABLE_PROFILING
            if (!cab_was_active || cab_last_ir != ir_idx)
                PROFILE_EXEC(audio_profiler, AP_CAB_RESET, profile_metrics::MODE_ACTIVE, 0,
                             cab_sim.reset());
#else
            if (!cab_was_active || cab_last_ir != ir_idx) cab_sim.reset();
#endif
#if ENABLE_PROFILING
            PROFILE_START(cab_process_profile);
#endif
            int64_t cab_started = esp_timer_get_time();
            cab_sim.process(mono_samples, ir_output, frames, ir_idx);
            const uint32_t cab_processing_us = (uint32_t)(esp_timer_get_time() - cab_started);
#if ENABLE_PROFILING
            PROFILE_FINISH(audio_profiler, AP_CAB_PROCESS, profile_metrics::MODE_ACTIVE,
                           frames, cab_process_profile);
#endif
            last_cab_block_us.store(cab_processing_us, std::memory_order_relaxed);
            update_max(max_cab_block_us, cab_processing_us);
            const uint32_t cab_budget_us = (uint32_t)((int64_t)frames * 1000000 / SAMPLE_RATE);
            if (cab_processing_us > cab_budget_us)
                cab_slow_blocks.fetch_add(1, std::memory_order_relaxed);
            memcpy(mono_samples, ir_output, frames * sizeof(float));
            cab_was_active = true;
            cab_last_ir = ir_idx;
        } else if (cab_was_active) {
#if ENABLE_PROFILING
            PROFILE_EXEC(audio_profiler, AP_CAB_RESET, profile_metrics::MODE_BYPASS, 0,
                         cab_sim.reset());
            PROFILE_COUNT(audio_profiler, AP_CAB_PROCESS,
                quality_level >= 4 ? profile_metrics::MODE_REDUCED : profile_metrics::MODE_BYPASS);
#else
            cab_sim.reset();
#endif
            cab_was_active = false;
        }
#if ENABLE_PROFILING
        else {
            PROFILE_COUNT(audio_profiler, AP_CAB_PROCESS,
                quality_level >= 4 ? profile_metrics::MODE_REDUCED : profile_metrics::MODE_SKIPPED);
        }
#endif
#else
        // O caminho é compilado fora da imagem padrão. Os campos legados do
        // estado já foram normalizados para cab_active=0.
#endif
#endif // LATENCY_TEST_MODE

        // 11. Clamping, cópia PCM de 24 bits para USB e saída estéreo do DAC.
        const uint8_t route = audio_route.load(std::memory_order_acquire);
#if !LATENCY_TEST_MODE
        const TickType_t tuner_state_age =
            xTaskGetTickCount() - last_tuner_state_tick.load(std::memory_order_acquire);
        const bool tuner_silent = tuner_mute.load(std::memory_order_acquire) &&
            tuner_state_age < pdMS_TO_TICKS(1000);
        const float tuner_target_gain = tuner_silent ? 0.0f : 1.0f;
        // Fade de 5 ms evita um clique quando a placa B entra ou sai do
        // afinador. O mute continua valendo para DAC e USB, mantendo apenas
        // o fluxo dos clocks e dos buffers.
        constexpr float TUNER_FADE_STEP = 1.0f / (0.005f * SAMPLE_RATE);
#endif
        const TickType_t last_usb = usb_last_callback_tick.load(std::memory_order_acquire);
        const bool usb_streaming = last_usb != 0 &&
            (xTaskGetTickCount() - last_usb < pdMS_TO_TICKS(100));
        const bool usb_ready = (route & AUDIO_ROUTE_USB) &&
            usb_mounted.load(std::memory_order_acquire) && usb_streaming;
        const uint64_t output_block_us = (uint64_t)esp_timer_get_time();
        const bool startup_ramp_needed = output_block_us < i2s_started_us + 110000u;
        uint32_t block_saturation_samples = 0;
#if ENABLE_PROFILING
        PROFILE_EXEC(audio_profiler, AP_OUTPUT_CONVERT, profile_metrics::MODE_ACTIVE, frames,
        {
#endif
        #pragma GCC ivdep
        for (int i = 0; i < frames; i++) {
            float raw_sample = mono_samples[i];
            if (!isfinite(raw_sample)) {
                // Um efeito defeituoso nunca pode propagar NaN/Inf para o
                // periférico I2S ou para o pacote USB.
                ++block_saturation_samples;
                raw_sample = 0.0f;
            } else if (raw_sample > 1.0f || raw_sample < -1.0f) {
                ++block_saturation_samples;
            }
            float s = fminf(fmaxf(raw_sample, -1.0f), 1.0f);
#if !LATENCY_TEST_MODE
            if (tuner_output_gain < tuner_target_gain)
                tuner_output_gain = fminf(tuner_target_gain, tuner_output_gain + TUNER_FADE_STEP);
            else if (tuner_output_gain > tuner_target_gain)
                tuner_output_gain = fmaxf(tuner_target_gain, tuner_output_gain - TUNER_FADE_STEP);
            float startup_gain = 1.0f;
            if (startup_ramp_needed) {
                const uint64_t sample_us = output_block_us +
                    (uint64_t)i * 1000000u / SAMPLE_RATE;
                const uint64_t elapsed_since_i2s = sample_us >= i2s_started_us
                    ? sample_us - i2s_started_us : 0;
                startup_gain = elapsed_since_i2s <= 100000u ? 0.0f :
                    fminf(1.0f, (float)(elapsed_since_i2s - 100000u) / 10000.0f);
            }
            s *= tuner_output_gain * startup_gain;
#else
            float startup_gain = 1.0f;
            if (startup_ramp_needed) {
                const uint64_t sample_us = output_block_us +
                    (uint64_t)i * 1000000u / SAMPLE_RATE;
                const uint64_t elapsed_since_i2s = sample_us >= i2s_started_us
                    ? sample_us - i2s_started_us : 0;
                startup_gain = elapsed_since_i2s <= 100000u ? 0.0f :
                    fminf(1.0f, (float)(elapsed_since_i2s - 100000u) / 10000.0f);
            }
            s *= startup_gain;
#endif
            // UAC 24-in-32: os 24 bits válidos ficam alinhados à esquerda,
            // como no barramento I2S dos codecs. O fator de 24 bits evita o
            // caso +1.0 * 2147483647, que em float arredonda para 2^31.
            const int32_t pcm24 = (int32_t)lrintf(s * 8388607.0f);
            const int32_t out_sample = pcm24 * 256;
            const int32_t usb_sample = out_sample;

            usb_block[i] = usb_sample;
#if LATENCY_TEST_MODE
            // Depois da rampa inicial, preserva cada bit recebido no canal L.
            // Durante a rampa, mantém a proteção contra estalo já existente.
            int32_t dac_left = 0;
            if (route & AUDIO_ROUTE_DAC) {
                if (startup_gain >= 1.0f) {
                    dac_left = audio_buffer[i * 2];
                } else {
                    const float ramped = (float)audio_buffer[i * 2] *
                        INT32_TO_FLOAT * startup_gain;
                    dac_left = (int32_t)lrintf(ramped * 8388607.0f) * 256;
                }
            }
            audio_buffer[i * 2] = dac_left;
            audio_buffer[i * 2 + 1] = 0;
#else
            audio_buffer[i * 2]     = (route & AUDIO_ROUTE_DAC) ? out_sample : 0;
            audio_buffer[i * 2 + 1] = (route & AUDIO_ROUTE_DAC) ? out_sample : 0;
#endif

        }
#if ENABLE_PROFILING
        });
#endif

        if (state->cab_active && block_saturation_samples)
            cab_saturation_samples.fetch_add(block_saturation_samples, std::memory_order_relaxed);
        if (usb_ready) {
#if ENABLE_PROFILING
            PROFILE_EXEC(audio_profiler, AP_USB_RING_PUSH, profile_metrics::MODE_ACTIVE, frames,
                         usb_pcm_ring.push(usb_block, static_cast<size_t>(frames)));
#else
            usb_pcm_ring.push(usb_block, static_cast<size_t>(frames));
#endif
        } else {
            PROFILE_COUNT(audio_profiler, AP_USB_RING_PUSH, profile_metrics::MODE_SKIPPED);
        }
        const uint32_t processing_us = (uint32_t)(esp_timer_get_time() - processing_started);
#if ENABLE_PROFILING
        PROFILE_FINISH(audio_profiler, AP_BLOCK_TOTAL, profile_metrics::MODE_ACTIVE,
                       frames, block_total_profile);
#endif
        last_audio_block_us.store(processing_us, std::memory_order_relaxed);
        update_max(max_audio_block_us, processing_us);
        const uint32_t block_budget_us = (uint32_t)((int64_t)frames * 1000000 / SAMPLE_RATE);
        if (processing_us > block_budget_us) {
            slow_audio_blocks.fetch_add(1);
            recovery_streak = 0;
            if (slow_streak < 255) ++slow_streak;
            // Dois blocos consecutivos fora do prazo ativam um nível de
            // redução. O nível 4 é o último recurso e pausa o CAB.
            if (slow_streak >= 2 && shed_level < 4) {
                ++shed_level;
                slow_streak = 0;
            }
            if (shed_level >= 4) {
#if ENABLE_CAB_SIM && !LATENCY_TEST_MODE
                cab_shed_latched = true;
                cab_failsafe_bypass.store(true, std::memory_order_relaxed);
#endif
            }
        } else {
            slow_streak = 0;
            // Uma margem de 10% evita oscilações, mas permite sair do nível
            // máximo quando apenas os efeitos de baixa latência estão ativos.
            if (shed_level > 0 && processing_us < (block_budget_us * 9u) / 10u) {
                if (recovery_streak < UINT16_MAX) ++recovery_streak;
                // Histerese para evitar liga/desliga a cada bloco.
                if (recovery_streak >= 128) {
                    --shed_level;
                    recovery_streak = 0;
                }
            } else {
                recovery_streak = 0;
            }
        }
        dsp_quality_level.store(shed_level, std::memory_order_relaxed);
#if ENABLE_PROFILING
        PROFILE_START(tx_wait_profile);
#endif
        const esp_err_t write_status = i2s_channel_write(
            tx_chan, audio_buffer, bytes_read, &bytes_written, 100);
        if (write_status != ESP_OK || bytes_written != bytes_read) {
            audio_io_errors.fetch_add(1);
            if (++consecutive_io_errors >= 3) {
                ESP_LOGE(TAG, "Tres falhas consecutivas na escrita I2S; reinicializando canais");
                deinit_i2s_system();
                consecutive_io_errors = 0;
            }
        } else {
            consecutive_io_errors = 0;
        }
#if ENABLE_PROFILING
        PROFILE_FINISH(audio_profiler, AP_TX_WAIT, profile_metrics::MODE_ACTIVE,
                       bytes_written / 8, tx_wait_profile);
#endif
    }
}

extern "C" void app_main(void) {
    ESP_LOGI(TAG, "Iniciando firmware; motivo do reset=%d", (int)esp_reset_reason());
#if LATENCY_TEST_MODE
    ESP_LOGW(TAG, "LATENCY_TEST_MODE ativo: efeitos/gate ignorados; DAC L=ADC L apos rampa, DAC R=0");
#endif
    ESP_ERROR_CHECK(configure_task_watchdog());
#if ENABLE_PROFILING
    uint64_t timer_calibration_start = (uint64_t)esp_timer_get_time();
    constexpr uint32_t TIMER_CALIBRATION_PAIRS = 4096;
    for (uint32_t i = 0; i < TIMER_CALIBRATION_PAIRS; ++i) {
        (void)esp_timer_get_time();
        (void)esp_timer_get_time();
    }
    profile_timer_pair_us.store((uint32_t)(((uint64_t)esp_timer_get_time() -
        timer_calibration_start) / TIMER_CALIBRATION_PAIRS), std::memory_order_relaxed);
#endif
    PresetManager::init();
    control_mutex = xSemaphoreCreateMutex();
    configASSERT(control_mutex != nullptr);

    // Restaura o som em uma chave separada dos slots de preset. Se ainda não
    // houver retomada, mantém o comportamento antigo de iniciar pelo slot 0.
    PedalState boot_state{};
    uint8_t boot_slot = PRESET_SLOT_UNKNOWN;
    bool boot_edited = false;
    bool boot_loaded = PresetManager::loadRecovery(boot_state, boot_slot, boot_edited) &&
                       validPedalState(boot_state, TOTAL_IR_PRESETS);
    if (!boot_loaded) {
        boot_loaded = PresetManager::loadPreset(0, boot_state) &&
                      validPedalState(boot_state, TOTAL_IR_PRESETS);
        if (boot_loaded) boot_slot = 0;
        boot_edited = false;
    }
    normalize_state_for_build(boot_state);
    if (!boot_loaded) {
        boot_state = defaultPedalState();
        boot_slot = PRESET_SLOT_UNKNOWN;
        boot_edited = false;
    }
    active_preset_slot.store(boot_slot, std::memory_order_release);
    active_preset_edited.store(boot_edited, std::memory_order_release);
    publish_state(boot_state);
    recovery_tracking_enabled.store(true, std::memory_order_release);
    request_recovery_save();

    if (!tape_delay.begin()) ESP_LOGE(TAG, "PSRAM indisponivel: delay em bypass");
    if (!reverb.begin()) ESP_LOGE(TAG, "Memoria indisponivel: reverb em bypass");

#if ENABLE_CAB_SIM
    const float* cab_tables[TOTAL_IR_PRESETS];
    for (size_t i = 0; i < TOTAL_IR_PRESETS; ++i) cab_tables[i] = IR_PRESETS[i].data;
    if (!cab_sim.prepare(cab_tables, TOTAL_IR_PRESETS))
        ESP_LOGE(TAG, "IRs do cab sim indisponiveis: efeito permanecera em bypass");
    else
        cab_sim.warmup();
#else
    ESP_LOGI(TAG, "CAB/IR desativado por ENABLE_CAB_SIM=0; usando bypass fixo");
#endif

    // Inicialização da UART Full-Duplex sem warnings de compilador
    uart_config_t uart_config = {};
    uart_config.baud_rate = 460800;
    uart_config.data_bits = UART_DATA_8_BITS;
    uart_config.parity    = UART_PARITY_DISABLE;
    uart_config.stop_bits = UART_STOP_BITS_1;
    uart_config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    uart_config.source_clk = UART_SCLK_APB;

    // O aplicativo pode alterar dezenas de parâmetros em sequência. Cada
    // alteração gera um espelho de 140 bytes para a Placa B; buffers maiores
    // evitam que o lote fique limitado ao tamanho do anel padrão.
    ESP_ERROR_CHECK(uart_driver_install(UART_PORT_NUM, 4096, 8192, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_PORT_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT_NUM, UART_TX_IO, UART_RX_IO, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    // Core 0: UART e diagnostico. A captura bruta para treino sai pela placa B.
    configASSERT(xTaskCreatePinnedToCore(full_duplex_comm_task, "comm_task", 4096, NULL, 10, NULL, 0) == pdPASS);

#if ENABLE_SERIAL_DEBUG
    configASSERT(xTaskCreatePinnedToCore(debug_print_task, "debug_task", 4096, NULL, 4, NULL, 0) == pdPASS);
#endif

    // Core 1: Kernel DSP exclusivo em tempo real (prioridade 22)
    configASSERT(xTaskCreatePinnedToCore(audio_processing_task, "dsp_task", 8192, NULL, 22, NULL, 1) == pdPASS);

    // O áudio já está executando e fará sua própria inicialização I2S antes
    // de tentar abrir o USB. Falha na pilha UAC não derruba o DAC.
    const esp_err_t usb_status = init_usb_audio();
    if (usb_status != ESP_OK)
        ESP_LOGE(TAG, "USB audio indisponivel (%s); DAC continua independente",
                 esp_err_to_name(usb_status));
}
