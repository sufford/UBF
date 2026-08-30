/*
 * MOD_twist.c - Twist modifier (Clean 3ds Max Style)
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
    float (*vertexCos)[3], /* Explicit standard array dimensioning */
    int numVerts,
    ModifierApplyFlag UNUSED(flag))
{
    TwistModifierData *tmd = (TwistModifierData *)md;
    if (tmd->angle == 0.0f) return;

    /* Standalone Apply wrapper architecture */
    DerivedMesh *dm = derivedData;
    bool free_dm = false;
    if (!dm) {
        Mesh *mesh = (Mesh *)ob->data;
        if (!mesh) return;
        dm = CDDM_from_mesh(mesh, ob);
        if (!dm) return;
        free_dm = true;
    }

    /* Blender's PROP_ANGLE property already delivers this variable in pure radians */
    float angle_rad = tmd->angle; 
    int axis = tmd->axis;

    /* 1. Calculate the maximum absolute spread distance from the center (0.0) along the twist axis */
    float max_dist = 0.0f;
    for (int i = 0; i < numVerts; i++) {
        float dist = fabsf(vertexCos[i][axis]);
        if (dist > max_dist) {
            max_dist = dist;
        }
    }

    if (max_dist <= 0.0001f) {
        if (free_dm) dm->release(dm);
        return;
    }

    /* 2. Twist calculations spanning both directions outward from 0.0 */
    for (int i = 0; i < numVerts; i++) {
        /* 
         * FIX: Factor is relative to 0.0, preserving the sign (+ or -).
         * Positive heights twist one way, negative heights automatically twist the other way.
         */
        float factor = vertexCos[i][axis] / max_dist;
        float twist_angle = angle_rad * factor;
        
        float cos_a = cosf(twist_angle);
        float sin_a = sinf(twist_angle);

        /* Read original coordinates safely into localized variables using sub-brackets */
        const float orig_x = vertexCos[i][0];
        const float orig_y = vertexCos[i][1];
        const float orig_z = vertexCos[i][2];

        /* Run standard, un-skewed 2D cross-section rotations */
        switch (axis) {
            case 0: /* Twist along X: Rotate Y and Z coordinates */
                vertexCos[i][0] = orig_x; 
                vertexCos[i][1] = orig_y * cos_a - orig_z * sin_a;
                vertexCos[i][2] = orig_y * sin_a + orig_z * cos_a;
                break;
                
            case 1: /* Twist along Y: Rotate X and Z coordinates */
                vertexCos[i][0] = orig_x * cos_a + orig_z * sin_a;
                vertexCos[i][1] = orig_y; 
                vertexCos[i][2] = -orig_x * sin_a + orig_z * cos_a;
                break;
                
            case 2: /* Twist along Z: Rotate X and Y coordinates */
            default:
                vertexCos[i][0] = orig_x * cos_a - orig_y * sin_a;
                vertexCos[i][1] = orig_x * sin_a + orig_y * cos_a;
                vertexCos[i][2] = orig_z; 
                break;
        }
    }

    if (free_dm) {
        dm->release(dm);
    }
}






static void initData(ModifierData *md) {
    TwistModifierData *tmd = (TwistModifierData *)md;
    tmd->angle = 0.0f;
    tmd->axis = 2;  /* Default to Z axis */
}

static CustomDataMask requiredDataMask(Object *UNUSED(ob), ModifierData *UNUSED(md)) {
    CustomDataMask mask = 0;
    mask |= CD_MASK_MVERT;
    return mask;
}

ModifierTypeInfo modifierType_Twist = {
    /* name */              "Twist",
    /* structName */        "TwistModifierData",
    /* structSize */        sizeof(TwistModifierData),
    /* type */              eModifierTypeType_OnlyDeform,
    /* flags */             eModifierTypeFlag_AcceptsMesh |
                            eModifierTypeFlag_SupportsEditmode |
                            eModifierTypeFlag_SupportsMapping,
    
    /* copyData */          NULL,
    
    /* deformVerts */       deformVerts,
    /* deformMatrices */    NULL,
    /* deformVertsEM */     deformVerts,
    /* deformMatricesEM */  NULL,
    /* applyModifier */     NULL,
    /* applyModifierEM */   NULL,
    
    /* initData */          initData,
    /* requiredDataMask */  requiredDataMask, 
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
