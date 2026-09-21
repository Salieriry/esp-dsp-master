#pragma once
#include "AudioConfig.h"

#if ENABLE_CAB_SIM
#include "effects/CabSimIR.h"

#include "deluxe_reverb_oxford_big_48_24.h"
#include "deluxe_reverb_oxford_lean_48_24.h"
#include "ev_mix_b.h"
#include "ev_mix_d.h"
#include "mesa_os_4x12_57_m160.h"

struct IrPreset {
    const char* name;
    const float* data;
};

#define TOTAL_IR_PRESETS 5

static const IrPreset IR_PRESETS[TOTAL_IR_PRESETS] = {
    {"DELUXE REVERB OXFORD", ir_deluxe_reverb_oxford_big_48_24_coefficients},
    {"DELUXE REVERB OXFORD", ir_deluxe_reverb_oxford_lean_48_24_coefficients},
    {"EV MIX B", ir_ev_mix_b_coefficients},
    {"EV MIX D", ir_ev_mix_d_coefficients},
    {"Mesa_OS_4x12_57_m160", ir_mesa_os_4x12_57_m160_coefficients},
};
#else
// A interface de estado continua carregando os campos cab_active/cab_index
// para manter compatibilidade com presets e com a Placa B antigas, mas não
// traz coeficientes nem o motor de convolução para a imagem final.
#define TOTAL_IR_PRESETS 1
#endif
