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

// O CAB/IR exige mais tempo de CPU do que a combinação completa de efeitos
// consegue entregar no prazo de 64 amostras nesta revisão da placa. Ele fica
// desativado por padrão e pode ser reativado em uma variante de firmware com
// -D ENABLE_CAB_SIM=1 depois de uma medição de carga.
#ifndef ENABLE_CAB_SIM
#define ENABLE_CAB_SIM       0
#endif

#define USB_AUDIO_RING_FRAMES 8192
#define USB_CONTROL_RX_BYTES  2048
#define USB_CONTROL_TX_BYTES  2048

// Mapeamento dos pinos do I2S (Codec ADC/DAC)
#define ADC_BCK_IO          GPIO_NUM_39
#define ADC_WS_IO           GPIO_NUM_40
#define ADC_DIN_IO          GPIO_NUM_41
#define ADC_MCLK_IO         GPIO_NUM_42

#define DAC_BCK_IO          GPIO_NUM_47
#define DAC_WS_IO           GPIO_NUM_48
#define DAC_DOUT_IO         GPIO_NUM_21

#define SHARED_BCK_IO       ADC_BCK_IO
#define SHARED_WS_IO        ADC_WS_IO

// UART de controle (comunicação com a Placa B)
#define UART_PORT_NUM       UART_NUM_1
#define UART_TX_IO          GPIO_NUM_1
#define UART_RX_IO          GPIO_NUM_2

// Inversos multiplicativos pré-computados (elimina divisões de ponto flutuante na FPU)
static constexpr float INT32_TO_FLOAT = 1.0f / 2147483648.0f;
static constexpr float FLOAT_TO_INT32 = 2147483647.0f;
