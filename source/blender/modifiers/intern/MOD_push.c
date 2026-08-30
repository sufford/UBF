/*
 * MOD_push.c - Push modifier
 */

#include "MEM_guardedalloc.h"

#include "BLI_math.h"
#include "BLI_utildefines.h"

 /* FIX: Added missing Mesh structure definition header */
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
	float(*vertexCos)[3], /* Restored dimensioning for array math */
	int numVerts,
	ModifierApplyFlag UNUSED(flag))
{
	PushModifierData *pmd = (PushModifierData *)md;
	if (pmd->distance == 0.0f) return;

	DerivedMesh *dm = derivedData;
	bool free_dm = false;

	/* Fallback: Create a temporary DerivedMesh if Blender sends a NULL pointer on Apply */
	if (!dm) {
		Mesh *mesh = (Mesh *)ob->data;
		if (!mesh) return;

		dm = CDDM_from_mesh(mesh, ob);
		if (!dm) return;

		free_dm = true;
	}

	/* Get vertex data normals */
	MVert *mvert = dm->getVertArray(dm);
	if (!mvert) {
		if (free_dm) dm->release(dm);
		return;
	}

	/* Push each vertex along its normal */
	for (int i = 0; i < numVerts; i++) {
		float normal[3];
		normal[0] = mvert[i].no[0] / 32767.0f;
		normal[1] = mvert[i].no[1] / 32767.0f;
		normal[2] = mvert[i].no[2] / 32767.0f;

		/* Update the vertex coordinates mapping cleanly to final baked output */
		vertexCos[i][0] += normal[0] * pmd->distance;
		vertexCos[i][1] += normal[1] * pmd->distance;
		vertexCos[i][2] += normal[2] * pmd->distance;
	}

	if (free_dm) {
		dm->release(dm);
	}
}

static void initData(ModifierData *md) {
	PushModifierData *pmd = (PushModifierData *)md;
	pmd->distance = 0.0f;
}

static CustomDataMask requiredDataMask(Object *UNUSED(ob), ModifierData *UNUSED(md)) {
	CustomDataMask mask = 0;
	mask |= CD_MASK_MVERT;
	return mask;
}

static bool dependsOnNormals(ModifierData *UNUSED(md)) {
	return true;
}

ModifierTypeInfo modifierType_Push = {
	/* name */              "Push",
	/* structName */        "PushModifierData",
	/* structSize */        sizeof(PushModifierData),
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
	/* dependsOnNormals */  dependsOnNormals,
	/* foreachObjectLink */ NULL,
	/* foreachIDLink */     NULL,
	/* foreachTexLink */    NULL,
};
