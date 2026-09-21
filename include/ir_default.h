#pragma once
#include "effects/CabSimIR.h"

// Impulso padrão (passa-direto inicial; substitua pelos 512 valores do seu arquivo .wav de IR)
alignas(16) static const float ir_default_coefficients[IR_TAPS] = {
    1.0f, 0.0f /* ... demais coeficientes ... */
};