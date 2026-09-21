#pragma once

// TinyUSB configuration shared by the USB Audio Class component and the CDC
// control channel. The UAC component supplies the format-specific macros.
#include "sdkconfig.h"

#define CFG_TUSB_MCU               OPT_MCU_ESP32S3
#define CFG_TUSB_OS                OPT_OS_FREERTOS
#define CFG_TUSB_RHPORT0_MODE      (OPT_MODE_DEVICE | OPT_MODE_FULL_SPEED)
#define CFG_TUD_ENABLED            1
#define CFG_TUD_ENDPOINT0_SIZE     64

#define CFG_TUD_CDC                1
#define CFG_TUD_CDC_RX_BUFSIZE     1024
#define CFG_TUD_CDC_TX_BUFSIZE     2048

// This header defines CFG_TUD_AUDIO and all UAC buffer/format sizes from
// CONFIG_UAC_* (44.1 kHz, mono, 24-bit in a 32-bit slot in sdkconfig.defaults).
#include "tusb_config_uac.h"

// Keep the component's endpoint feature macros at their default values. The
// microphone-only descriptors below do not announce OUT/feedback endpoints,
// but usb_device_uac.c still compiles the generic speaker helpers and needs
// their type declarations present.

#define CFG_TUSB_DEBUG              0
#define CFG_TUSB_MEM_SECTION
#define CFG_TUSB_MEM_ALIGN          TU_ATTR_ALIGNED(4)
