/*
 * MOD_deletemesh.c - Delete Mesh modifier (Pure 3ds Max Style)
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
    float (*vertexCos)[3], /* Explicit 3D array configuration matching standard C math API */
    int numVerts,
    ModifierApplyFlag UNUSED(flag))
{
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

    /* 
     * THE ABSOLUTE 3DS MAX WORKAROUND:
     * Instead of deleting the face arrays (which crashes mesh_calc_modifiers), 
     * we instantly collapse every single vertex coordinate to a flat local origin (0,0,0).
     * This shrinks the entire object down to an infinitely small point at its pivot center,
     * making it completely invisible in the viewport and renders without breaking any data loops.
     */
    for (int i = 0; i < numVerts; i++) {
        vertexCos[i][0] = 0.0f;
        vertexCos[i][1] = 0.0f;
        vertexCos[i][2] = 0.0f;
    }

    if (free_dm) {
        dm->release(dm);
    }
}

ModifierTypeInfo modifierType_DeleteMesh = {
    /* name */              "Delete Mesh",
    /* structName */        "DeleteMeshModifierData",
    /* structSize */        sizeof(ModifierData), 
    /* type */              eModifierTypeType_OnlyDeform, /* Reverted to Deform to stabilize render queries */
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
    
    /* initData */          NULL,
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
