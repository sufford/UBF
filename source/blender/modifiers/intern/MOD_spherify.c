/*
 * MOD_spherify.c - Spherify modifier
 */

#include "MEM_guardedalloc.h"

#include "BLI_math.h"
#include "BLI_utildefines.h"

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
    SpherifyModifierData *smd = (SpherifyModifierData *)md;
    
    if (smd->percent <= 0.0f) {
        return;  /* No effect */
    }
    
    /* Calculate center of all vertices */
    float center[3] = {0.0f, 0.0f, 0.0f};
    for (int i = 0; i < numVerts; i++) {
        center[0] += vertexCos[i][0];
        center[1] += vertexCos[i][1];
        center[2] += vertexCos[i][2];
    }
    
    if (numVerts > 0) {
        center[0] /= numVerts;
        center[1] /= numVerts;
        center[2] /= numVerts;
    }
    
    /* Calculate average radius */
    float avg_radius = 0.0f;
    for (int i = 0; i < numVerts; i++) {
        float diff[3];
        sub_v3_v3v3(diff, vertexCos[i], center);
        avg_radius += len_v3(diff);
    }
    
    if (numVerts > 0) {
        avg_radius /= numVerts;
    }
    
    /* Use custom radius if specified */
    float target_radius = (smd->radius > 0.0f) ? smd->radius : avg_radius;
    
    if (target_radius <= 0.0f) {
        return;  /* Can't calculate radius */
    }
    
    /* Spherify each vertex */
    for (int i = 0; i < numVerts; i++) {
        float diff[3];
        sub_v3_v3v3(diff, vertexCos[i], center);
        
        float current_dist = len_v3(diff);
        
        if (current_dist > 0.0f) {
            /* Normalize direction */
            float dir[3];
            mul_v3_v3fl(dir, diff, 1.0f / current_dist);
            
            /* Target position on sphere */
            float target_pos[3];
            mul_v3_v3fl(target_pos, dir, target_radius);
            add_v3_v3(target_pos, center);
            
            /* Blend between original and spherical based on percent */
            float blend = smd->percent;
            interp_v3_v3v3(vertexCos[i], vertexCos[i], target_pos, blend);
        }
    }
}

static void initData(ModifierData *md) {
    SpherifyModifierData *smd = (SpherifyModifierData *)md;
    
    smd->percent = 0.0f;
    smd->radius = 0.0f;  /* 0 = auto */
}

ModifierTypeInfo modifierType_Spherify = {
    /* name */              "Spherify",
    /* structName */        "SpherifyModifierData",
    /* structSize */        sizeof(SpherifyModifierData),
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