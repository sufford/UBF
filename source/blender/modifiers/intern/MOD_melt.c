/*
 * MOD_melt.c - Melt modifier (3ds Max Gizmo Style)
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
    MeltModifierData *mmd = (MeltModifierData *)md;
    
    if (mmd->amount <= 0.0f) return;
    
    int axis = mmd->axis;
    int axis_a, axis_b;
    
    switch (axis) {
        case 0: axis_a = 1; axis_b = 2; break;
        case 1: axis_a = 0; axis_b = 2; break;
        case 2: axis_a = 0; axis_b = 1; break;
        default: axis_a = 0; axis_b = 1; break;
    }
    
    /* Build gizmo transformation matrix */
    float gizmo_mat[4][4];
    
    if (mmd->gizmo) {
        /* Transform vertices into gizmo's local space */
        float imat[4][4];
        invert_m4_m4(imat, ob->obmat);
        mul_m4_m4m4(gizmo_mat, mmd->gizmo->obmat, imat);
        invert_m4_m4(gizmo_mat, gizmo_mat);
    } else {
        /* Fallback: use object's own local space */
        unit_m4(gizmo_mat);
    }
    
    /* First pass: find min/max height in gizmo space */
    float min_val = FLT_MAX;
    float max_val = -FLT_MAX;
    
    for (int i = 0; i < numVerts; i++) {
        float local_v[3];
        mul_v3_m4v3(local_v, gizmo_mat, vertexCos[i]);
        
        if (local_v[axis] < min_val) min_val = local_v[axis];
        if (local_v[axis] > max_val) max_val = local_v[axis];
    }
    
    float total_height = max_val - min_val;
    if (total_height <= 0.0001f) return;
    
    /* Second pass: apply melt deformation */
    for (int i = 0; i < numVerts; i++) {
        /* Transform to gizmo local space */
        float local_v[3];
        mul_v3_m4v3(local_v, gizmo_mat, vertexCos[i]);
        
        /* Height above gizmo floor (0 = floor, 1 = top) */
        float height_factor = (local_v[axis] - min_val) / total_height;
        
        /* Melt amount: higher verts drop more */
        float drop_amount = height_factor * mmd->amount * total_height;
        
        /* Apply drop in local space */
        local_v[axis] -= drop_amount;
        
        /* Spread outward at the base (wax pooling) */
        if (mmd->spread > 0.0f) {
            float spread_factor = height_factor * mmd->spread;
            float spread_scale = 1.0f + (spread_factor * (1.0f - height_factor));
            
            local_v[axis_a] *= spread_scale;
            local_v[axis_b] *= spread_scale;
        }
        
        /* Transform back to world space */
        float inv_gizmo_mat[4][4];
        invert_m4_m4(inv_gizmo_mat, gizmo_mat);
        mul_v3_m4v3(vertexCos[i], inv_gizmo_mat, local_v);
    }
}

static void initData(ModifierData *md) {
    MeltModifierData *mmd = (MeltModifierData *)md;
    mmd->gizmo = NULL;
    mmd->amount = 0.0f;
    mmd->spread = 0.0f;
    mmd->solidity_val = 1.0f;
    mmd->solidity_type = 1;
    mmd->axis = 2;
    mmd->pad = 0;
}

ModifierTypeInfo modifierType_Melt = {
    /* name */              "Melt",
    /* structName */        "MeltModifierData",
    /* structSize */        sizeof(MeltModifierData),
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