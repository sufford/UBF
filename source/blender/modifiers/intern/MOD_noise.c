/*
 * MOD_noise.c - 3ds Max style Noise modifier
 */

#include "MEM_guardedalloc.h"

#include "BLI_math.h"
#include "BLI_noise.h"
#include "BLI_utildefines.h"

#include "DNA_meshdata_types.h"
#include "DNA_modifier_types.h"
#include "DNA_object_types.h"

#include "BKE_cdderivedmesh.h"
#include "BKE_modifier.h"

#include "MOD_modifiertypes.h"

/* Generate fractal noise value */
static float fractal_noise(float x, float y, float z, NoiseModifierData *nmd)
{
    float total = 0.0f;
    float amplitude = 1.0f;
    float frequency = 1.0f;
    float total_amplitude = 0.0f;
    
    for (int i = 0; i < nmd->iterations; i++) {
        float noise_val = BLI_gNoise(
            1.0f,
            x * frequency + nmd->seed,
            y * frequency + nmd->seed,
            z * frequency + nmd->seed,
            0, 0
        );
        
        total += noise_val * amplitude;
        total_amplitude += amplitude;
        
        amplitude *= nmd->roughness;
        frequency *= 2.0f;
    }
    
    /* Normalize */
    if (total_amplitude > 0.0f) {
        total /= total_amplitude;
    }
    
    /* Center around zero */
    return (total - 0.5f) * 2.0f;
}

/* Generate single noise value */
static float single_noise(float x, float y, float z, NoiseModifierData *nmd)
{
    float noise_val = BLI_gNoise(
        1.0f,
        x + nmd->seed,
        y + nmd->seed,
        z + nmd->seed,
        0, 0
    );
    
    return (noise_val - 0.5f) * 2.0f;
}

static void deformVerts(
    ModifierData *md, 
    Object *ob,
    DerivedMesh *derivedData,
    float (*vertexCos)[3],
    int numVerts,
    ModifierApplyFlag UNUSED(flag))
{
    NoiseModifierData *nmd = (NoiseModifierData *)md;
    
    for (int i = 0; i < numVerts; i++) {
        float *co = vertexCos[i];
        
        /* Generate noise for each axis */
        float noise_x, noise_y, noise_z;
        
        if (nmd->fractal) {
            noise_x = fractal_noise(
                co[0] * nmd->scale, 
                co[1] * nmd->scale, 
                co[2] * nmd->scale, 
                nmd
            );
            
            noise_y = fractal_noise(
                co[0] * nmd->scale + 100.0f, 
                co[1] * nmd->scale + 100.0f, 
                co[2] * nmd->scale + 100.0f, 
                nmd
            );
            
            noise_z = fractal_noise(
                co[0] * nmd->scale + 200.0f, 
                co[1] * nmd->scale + 200.0f, 
                co[2] * nmd->scale + 200.0f, 
                nmd
            );
        }
        else {
            noise_x = single_noise(
                co[0] * nmd->scale, 
                co[1] * nmd->scale, 
                co[2] * nmd->scale, 
                nmd
            );
            
            noise_y = single_noise(
                co[0] * nmd->scale + 100.0f, 
                co[1] * nmd->scale + 100.0f, 
                co[2] * nmd->scale + 100.0f, 
                nmd
            );
            
            noise_z = single_noise(
                co[0] * nmd->scale + 200.0f, 
                co[1] * nmd->scale + 200.0f, 
                co[2] * nmd->scale + 200.0f, 
                nmd
            );
        }
        
        /* Apply noise to axes */
        co[0] += noise_x * nmd->strength[0];
        co[1] += noise_y * nmd->strength[1];
        co[2] += noise_z * nmd->strength[2];
    }
}

/* Initialize with default values */
static void initData(ModifierData *md) {
    NoiseModifierData *nmd = (NoiseModifierData *)md;
    
    nmd->strength[0] = 0.0f;
    nmd->strength[1] = 0.0f;
    nmd->strength[2] = 0.0f;
    nmd->scale = 1.0f;
    nmd->seed = 0;
    nmd->use_normals = 0;
    nmd->fractal = 1;          /* Enable fractal by default */
    nmd->roughness = 0.5f;     /* Default roughness */
    nmd->iterations = 6;       /* Default octaves */
}

/* Register the modifier type */
ModifierTypeInfo modifierType_Noise = {
    /* name */              "Noise",
    /* structName */        "NoiseModifierData",
    /* structSize */        sizeof(NoiseModifierData),
    /* type */              eModifierTypeType_OnlyDeform,
    /* flags */             eModifierTypeFlag_AcceptsMesh |
                            eModifierTypeFlag_SupportsEditmode,
    
    /* copyData */          NULL,
    
    /* deformVerts */       deformVerts,
    /* deformMatrices */    NULL,
    /* deformVertsEM */     NULL,
    /* deformMatricesEM */  NULL,
    /* applyModifier */     NULL,
    /* applyModifierEM */   NULL,
    
    /* initData */          initData,
    /* requiredDataMask */  NULL,
    /* freeData */          NULL,
    /* isDisabled */        NULL,
    /* updateDepgraph */    NULL,
    /* updateDepsgraph */   NULL,
    /* dependsOnTime */     NULL,
    /* dependsOnNormals */  NULL,
    /* foreachObjectLink */ NULL,
    /* foreachIDLink */     NULL,
    /* foreachTexLink */    NULL,
};
