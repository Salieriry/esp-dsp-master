#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint32_t rx_interrupts;
    uint32_t tx_interrupts;
    uint32_t rx_bytes;
    uint32_t tx_bytes;
    uint32_t rx_errors;
    uint32_t tx_errors;
    uint32_t rx_buffer_bytes;
    uint32_t tx_buffer_bytes;
    uint32_t rx_descriptor_bytes;
    uint32_t tx_descriptor_bytes;
    uint64_t rx_isr_cycles_total;
    uint64_t tx_isr_cycles_total;
    uint32_t rx_isr_cycles_max;
    uint32_t tx_isr_cycles_max;
    uint64_t rx_callback_cycles_total;
    uint64_t tx_callback_cycles_total;
    uint32_t rx_callback_cycles_max;
    uint32_t tx_callback_cycles_max;
    uint64_t rx_copy_cycles_total;
    uint64_t tx_copy_cycles_total;
    uint32_t rx_copy_cycles_max;
    uint32_t tx_copy_cycles_max;
    uint64_t rx_wait_us;
    uint64_t tx_wait_us;
    uint8_t events_valid;
    uint8_t memory_valid;
    uint8_t timing_valid;
    uint8_t reserved;
} DmaDriverMetrics;

void i2s_driver_metrics_snapshot(DmaDriverMetrics *out);
void i2s_driver_metrics_reset(void);
uint32_t i2s_driver_metrics_storage_bytes(void);
void gdma_driver_metrics_snapshot(DmaDriverMetrics *out);
void gdma_driver_metrics_reset(void);
uint32_t gdma_driver_metrics_storage_bytes(void);

#ifdef __cplusplus
}
#endif
