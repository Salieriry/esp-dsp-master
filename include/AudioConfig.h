#pragma once
#include "driver/gpio.h"

#define SAMPLE_RATE         44100
#define DMA_FRAME_NUM       64
#define DMA_DESC_NUM        4
#define BUFFER_SAMPLES      (DMA_FRAME_NUM * 2)

// Mantém a fiação original por padrão. Para a revisão de hardware, compile com
// -DI2S_SHARED_CLOCKS=1 e ligue BCK/LRCK do DAC às linhas do ADC.
#ifndef I2S_SHARED_CLOCKS
#define I2S_SHARED_CLOCKS    0
#endif

#ifndef ENABLE_USB_AUDIO
#define ENABLE_USB_AUDIO     1
#endif

// PlatformIO compila o perfil CAB por padrão; a macro permanece desativada
// apenas em compilações externas que não selecionem explicitamente um perfil.
#ifndef ENABLE_CAB_SIM
#define ENABLE_CAB_SIM       0
#endif

#define USB_AUDIO_RING_FRAMES 1024
#define USB_AUDIO_PRIME_FRAMES 192
#define USB_AUDIO_LOW_WATER 128
#define USB_AUDIO_HIGH_WATER 256
#define USB_AUDIO_RESYNC_WATER 512
#define USB_CONTROL_RX_BYTES  2048
#define USB_CONTROL_TX_BYTES  2048

// Mapeamento dos pinos do I2S (Codec ADC/DAC)
#define ADC_BCK_IO          GPIO_NUM_7
#define ADC_WS_IO           GPIO_NUM_16
#define ADC_DIN_IO          GPIO_NUM_15
#define ADC_MCLK_IO         GPIO_NUM_17

#define DAC_BCK_IO          GPIO_NUM_9
#define DAC_WS_IO           GPIO_NUM_11
#define DAC_DOUT_IO         GPIO_NUM_10

#define SHARED_BCK_IO       ADC_BCK_IO
#define SHARED_WS_IO        ADC_WS_IO

// UART de controle (comunicação com a Placa B)
#define UART_PORT_NUM       UART_NUM_1
#define UART_TX_IO          GPIO_NUM_4
#define UART_RX_IO          GPIO_NUM_5

// Inversos multiplicativos pré-computados (elimina divisões de ponto flutuante na FPU)
static constexpr float INT32_TO_FLOAT = 1.0f / 2147483648.0f;
static constexpr float FLOAT_TO_INT32 = 2147483647.0f;
