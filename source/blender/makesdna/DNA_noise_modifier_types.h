#ifndef __DNA_NOISE_MODIFIER_TYPES_H__
#define __DNA_NOISE_MODIFIER_TYPES_H__

#include "DNA_modifier_types.h"

typedef struct NoiseModifierData {
    ModifierData modifier;
    
    float strength;  /* Simple single strength value */
    float scale;     /* Noise scale */
    int seed;        /* Random seed */
    int pad;
} NoiseModifierData;

#endif