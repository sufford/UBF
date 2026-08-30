/*
 * MOD_wavemax.c - 3ds Max Style Wave/Ripple Hybrid modifier
 */

#include "MEM_guardedalloc.h"

#include "BLI_math.h"
#include "BLI_utildefines.h"

#include "DNA_mesh_types.h"
#include "DNA_meshdata_types.h"
#include "DNA_modifier_types.h"
#include "DNA_object_types.h"

#include "BKE_cdderivedmesh.h"
#include "BKE_modifier.h"

#include "MOD_modifiertypes.h"

static void deformVerts(
    ModifierData *md, 
    Object *ob,
    DerivedMesh *derivedData,
    float (*vertexCos)[3],
    int numVerts,
    ModifierApplyFlag UNUSED(flag))
{
    WaveMaxModifierData *wmd = (WaveMaxModifierData *)md;
    
    // Stop early if execution variables are zeroed out
    if (wmd->wave_length <= 0.0001f) return;
    if (wmd->amplitude[0] == 0.0f && wmd->amplitude[1] == 0.0f) return;
    
    /* 1. Build Local Gizmo Alignment System 
     * To mirror Max faithfully, the wave operates along a local spatial coordinate grid.
     * We map a transform matrix based on the user's targeted Travel Axis. */
    float gizmoMat[4][4], invGizmoMat[4][4];
    unit_m4(gizmoMat);
    
    // Max convention baseline: Wave travels down Y, displaces up Z vertical path
    if (wmd->axis == 0) { // X-Axis configuration
        gizmoMat[0][0] = 0.0f; gizmoMat[0][1] = 1.0f;
        gizmoMat[1][0] = -1.0f; gizmoMat[1][1] = 0.0f;
    } else if (wmd->axis == 2) { // Z-Axis configuration
        gizmoMat[1][1] = 0.0f; gizmoMat[1][2] = 1.0f;
        gizmoMat[2][1] = -1.0f; gizmoMat[2][2] = 0.0f;
    }
    invert_m4_m4(invGizmoMat, gizmoMat);

    // Angular wave frequency multiplier (k = 2*PI / wavelength)
    float k = (2.0f * (float)M_PI) / wmd->wave_length;

    for (int i = 0; i < numVerts; i++) {
        float co[3];
        copy_v3_v3(co, vertexCos[i]);
        
        /* 2. Map target coordinates directly into Local Gizmo space */
        float local_co[3];
        mul_v3_m4v3(local_co, gizmoMat, co);
        
        float y_travel = local_co[1]; // Position along the active direction of travel
        float x_side   = local_co[0]; // Perpendicular horizontal baseline
        
        // Calculate standard concentric radial distance from center
        float radial_dist = sqrtf(x_side * x_side + y_travel * y_travel);
        
        float travel_coordinate = 0.0f;
        float decay_distance = 0.0f;
        
        /* 3. Run selected Mode evaluation switch */
        switch (wmd->mode) {
            case 0: // MOD_WAVEMAX_MODE_WAVE
                travel_coordinate = y_travel;
                decay_distance = fabsf(y_travel); // Linearly damp down the plane
                break;
                
            case 1: // MOD_WAVEMAX_MODE_RIPPLE
                travel_coordinate = radial_dist;
                decay_distance = radial_dist;    // Concentrically damp outward
                break;
                
            case 2: // MOD_WAVEMAX_MODE_MIXED
                // Custom blended travel paths morphing linearly to radial rings
                travel_coordinate = (y_travel * (1.0f - wmd->blend)) + (radial_dist * wmd->blend);
                decay_distance = (fabsf(y_travel) * (1.0f - wmd->blend)) + (radial_dist * wmd->blend);
                break;
                
            default:
                travel_coordinate = y_travel;
                decay_distance = fabsf(y_travel);
                break;
        }
        
        /* 4. Exact Max Dual-Wave Profile Blending (Cosine and Sine interference pattern)
         * Max blends Amplitude 1 (Cosine) and Amplitude 2 (Sine) to allow rich profile offsets. */
        float wave1 = wmd->amplitude[0] * cosf(travel_coordinate * k - wmd->phase);
        float wave2 = wmd->amplitude[1] * sinf(travel_coordinate * k - wmd->phase);
        float total_displacement = wave1 + wave2;
        
        /* 5. Authentic Exponential Decay Calculation */
        if (wmd->decay > 0.0f) {
            total_displacement *= expf(-decay_distance * wmd->decay);
        }
        
        // Displace the vertex along the local vertical coordinate path (Z-local)
        local_co[2] += total_displacement;
        
        /* 6. Invert the transform tracking map cleanly back into Blender Object space */
        mul_v3_m4v3(vertexCos[i], invGizmoMat, local_co);
    }
}

static void initData(ModifierData *md) {
    WaveMaxModifierData *wmd = (WaveMaxModifierData *)md;
    wmd->amplitude[0] = 0.5f;   // Amplitude 1 defaults to 0.5 units
    wmd->amplitude[1] = 0.0f;   // Amplitude 2 standardly initializes flat in Max
    wmd->wave_length = 1.0f;
    wmd->phase = 0.0f;
    wmd->decay = 0.0f;
    wmd->axis = 1;              // Standard Y-axis traveling default
    wmd->mode = 0;              // Defaults to Pure Wave mode selection
    wmd->blend = 0.5f;          // Blending slider splits the center evenly
}

ModifierTypeInfo modifierType_WaveMax = {
    /* name */              "Wave Max",
    /* structName */        "WaveMaxModifierData",
    /* structSize */        sizeof(WaveMaxModifierData),
    /* type */              eModifierTypeType_OnlyDeform,
    /* flags */             eModifierTypeFlag_AcceptsMesh |
                            eModifierTypeFlag_SupportsEditmode,
    
    /* copyData */          NULL,
    /* deformVerts */       deformVerts,
    /* deformMatrices */    NULL,
    /* deformVertsEM */     deformVerts,
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
