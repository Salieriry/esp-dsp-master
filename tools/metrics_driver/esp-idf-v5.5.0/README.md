# Instrumentação do driver I²S/GDMA · ESP-IDF 5.5.0

Este overlay fixa as fontes oficiais usadas nos builds -metrics da Placa A.
Os arquivos em components/ são cópias upstream sem modificações; os hooks
estão nas cópias patched/, e metrics.patch registra as alterações.

| Fonte upstream | SHA-256 |
|---|---|
| components/esp_hw_support/dma/gdma.c | 12f5c760b6e9efbbfd1d12278ba61fd878e12fec6f11052758a605f13e81de2b |
| components/esp_hw_support/dma/gdma_priv.h | 1c140abeed7b14fd3dd35a57e41ba6531413dcff03653c2d061b2140344543a0 |
| components/esp_driver_i2s/i2s_common.c | 9a3f1df3cfdf385c6ea9f05027dfcd8cb9f33f3f8a7af193bdc34ac0d0a7785c |
| components/esp_driver_i2s/i2s_private.h | de56c1f41d10ea0e61d19c976c5b8a384d8ecadedaa9257dcff31bfab2163fde |

O CMake do projeto verifica a versão e os hashes antes de substituir os dois
arquivos de componente apenas no build de diagnóstico. A instalação global do
PlatformIO não é modificada. Versão ou hash incompatível interrompe o build.
A licença Apache-2.0 da Espressif acompanha as fontes.

## Escopo e interpretação

A instrumentação acompanha I²S0. Registra interrupções e ciclos do corpo do
ISR GDMA, callbacks internos I²S, bytes processados, erros indicados, ciclos de
cópia, espera bloqueante por descritores e bytes vivos de buffers/descritores.
O tempo de callback também aparece separado e está contido no tempo inclusivo
do ISR; esses valores não devem ser somados. Erros de fila e flags de erro de
descritor são indicações distintas e não representam necessariamente eventos
físicos únicos.

A espera da fila é tempo bloqueado observado pelo driver; ciclos de ISR,
callback e cópia são intervalos de CPU que podem sofrer interferência de
interrupções de prioridade superior. A transferência autônoma do periférico
DMA, o tempo de conversão analógica e qualquer latência física ADC–DAC não são
atribuídos a esses contadores. Os hooks usam contadores prealocados, sem saída
serial nem alocação dinâmica.

Fontes oficiais: GDMA v5.5.0 (https://github.com/espressif/esp-idf/blob/v5.5.0/components/esp_hw_support/dma/gdma.c),
GDMA private header v5.5.0 (https://github.com/espressif/esp-idf/blob/v5.5.0/components/esp_hw_support/dma/gdma_priv.h),
I²S common v5.5.0 (https://github.com/espressif/esp-idf/blob/v5.5.0/components/esp_driver_i2s/i2s_common.c)
e I²S private header v5.5.0 (https://github.com/espressif/esp-idf/blob/v5.5.0/components/esp_driver_i2s/i2s_private.h).
