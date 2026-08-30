/*
 * MOD_taper.c - Taper modifier (3ds Max Style)
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
    float (*vertexCos)[3], /* Restored explicit array syntax */
    int numVerts,
    ModifierApplyFlag UNUSED(flag))
{
    TaperModifierData *tmd = (TaperModifierData *)md;
    if (tmd->amount == 0.0f && tmd->curve == 0.0f) return;

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

    int axis = tmd->axis;
    int effect = tmd->effect_axis;

    /* Calculate explicit boundaries strictly along the primary taper axis */
    float min_val = FLT_MAX;
    float max_val = -FLT_MAX;
    for (int i = 0; i < numVerts; i++) {
        float pos = vertexCos[i][axis];
        if (pos < min_val) min_val = pos;
        if (pos > max_val) max_val = pos;
    }

    float total_height = max_val - min_val;
    if (total_height <= 0.0001f) {
        if (free_dm) dm->release(dm);
        return;
    }

    /* Run Taper calculations on orthogonal cross-sections */
    for (int i = 0; i < numVerts; i++) {
        float factor = 0.0f;

        if (tmd->flag & MOD_TAPER_SYMMETRY) {
            /* 3DS MAX SYMMETRY MODE: Absolute mirror outward from local center 0.0 */
            float distance = fabsf(vertexCos[i][axis]);
            float max_extent = fmaxf(fabsf(min_val), fabsf(max_val));
            if (max_extent > 0.0001f) {
                factor = distance / max_extent;
            }
        }
        else {
            /* STANDARD MODE: Taper smoothly from absolute base boundary up to the top */
            factor = (vertexCos[i][axis] - min_val) / total_height;
        }

        /* Calculate Linear Taper Scale factor */
        float taper_scale = 1.0f + (tmd->amount * factor);

        /* Calculate Max Parabolic Curvature Component (Bowing effect) */
        if (tmd->curve != 0.0f) {
            float curve_factor = 1.0f - (factor * factor);
            taper_scale += tmd->curve * curve_factor;
        }

        if (taper_scale < 0.0f) taper_scale = 0.0f;

        /* Isolate source variables using immutable constants */
        const float orig_x = vertexCos[i][0];
        const float orig_y = vertexCos[i][1];
        const float orig_z = vertexCos[i][2];

        /* Default cross-section scalars to unchanged (1.0) */
        float scale_u = 1.0f;
        float scale_v = 1.0f;

        /* Assign scale factors strictly based on requested effect selection dropdown */
        if (effect == 0) {       /* Effect Axis: X Only */
            scale_u = taper_scale;
        }
        else if (effect == 1) {  /* Effect Axis: Y Only */
            scale_v = taper_scale;
        }
        else {                   /* Effect Axis: XY / Both (Default Max behavior) */
            scale_u = taper_scale;
            scale_v = taper_scale;
        }

        /* Remap cross-sections safely based on chosen Primary Axis orientation */
        switch (axis) {
            case 0: /* Primary Axis X: Deform Y (u) and Z (v) */
                vertexCos[i][0] = orig_x; 
                vertexCos[i][1] = orig_y * scale_u;
                vertexCos[i][2] = orig_z * scale_v;
                break;
                
            case 1: /* Primary Axis Y: Deform X (u) and Z (v) */
                vertexCos[i][0] = orig_x * scale_u;
                vertexCos[i][1] = orig_y; 
                vertexCos[i][2] = orig_z * scale_v;
                break;
                
            case 2: /* Primary Axis Z: Deform X (u) and Y (v) */
            default:
                vertexCos[i][0] = orig_x * scale_u;
                vertexCos[i][1] = orig_y * scale_v;
                vertexCos[i][2] = orig_z; 
                break;
        }
    }

    if (free_dm) {
        dm->release(dm);
    }
}


static void initData(ModifierData *md) {
    TaperModifierData *tmd = (TaperModifierData *)md;
    tmd->amount = 1.0f;
    tmd->curve = 0.0f;
    tmd->axis = 2;  /* Default to Z axis */
    tmd->flag = 0;  /* Standard linear layout */
}

static CustomDataMask requiredDataMask(Object *UNUSED(ob), ModifierData *UNUSED(md)) {
    CustomDataMask mask = 0;
    mask |= CD_MASK_MVERT;
    return mask;
}

ModifierTypeInfo modifierType_Taper = {
    /* name */              "Taper",
    /* structName */        "TaperModifierData",
    /* structSize */        sizeof(TaperModifierData),
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
