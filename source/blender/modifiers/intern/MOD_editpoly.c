/*
 * Edit Poly modifier - non-destructive Edit Poly (3ds Max style).
 */

#include <string.h>

#include "MEM_guardedalloc.h"

#include "DNA_mesh_types.h"
#include "DNA_modifier_types.h"
#include "DNA_object_types.h"
#include "DNA_scene_types.h"

#include "BLI_utildefines.h"
#include "BLI_math.h"

#include "BKE_cdderivedmesh.h"
#include "BKE_DerivedMesh.h"
#include "BKE_global.h"           /* G.main */
#include "BKE_library.h"
#include "BKE_library_query.h"    /* IDWALK_NOP */
#include "BKE_mesh.h"
#include "BKE_modifier.h"

#include "MOD_modifiertypes.h"

static void initData(ModifierData *md)
{
	EditPolyModifierData *epmd = (EditPolyModifierData *)md;
	epmd->edit_mesh = NULL;
	epmd->flag = 0;
}

static void copyData(ModifierData *md, ModifierData *target)
{
	EditPolyModifierData *epmd = (EditPolyModifierData *)md;
	EditPolyModifierData *tepmd = (EditPolyModifierData *)target;

	if (epmd->edit_mesh) {
		tepmd->edit_mesh = BKE_mesh_copy(G.main, epmd->edit_mesh);
	}
	else {
		tepmd->edit_mesh = NULL;
	}
	tepmd->flag = epmd->flag;
}

static void freeData(ModifierData *md)
{
	EditPolyModifierData *epmd = (EditPolyModifierData *)md;

	if (epmd->edit_mesh) {
		BKE_mesh_free(epmd->edit_mesh);
		MEM_freeN(epmd->edit_mesh);
		epmd->edit_mesh = NULL;
	}
}

static void foreachIDLink(ModifierData *md, Object *ob, IDWalkFunc walk, void *userData)
{
	EditPolyModifierData *epmd = (EditPolyModifierData *)md;

	if (epmd->edit_mesh) {
		walk(userData, ob, (ID **)&epmd->edit_mesh, IDWALK_NOP);
	}
}

static DerivedMesh *applyModifier(ModifierData *md, Object *ob,
	DerivedMesh *derivedData,
	ModifierApplyFlag UNUSED(flag))
{
	EditPolyModifierData *epmd = (EditPolyModifierData *)md;

	(void)ob;

	if (epmd->edit_mesh == NULL) {
		return derivedData;
	}

	/* Позже: построить DerivedMesh из epmd->edit_mesh. */
	return derivedData;
}

ModifierTypeInfo modifierType_EditPoly = {
	/* name */              "EditPoly",
	/* structName */        "EditPolyModifierData",
	/* structSize */        sizeof(EditPolyModifierData),
	/* type */              eModifierTypeType_Constructive,
	/* flags */             eModifierTypeFlag_AcceptsMesh |
							eModifierTypeFlag_SupportsEditmode,

	/* copyData */          copyData,
	/* deformVerts */       NULL,
	/* deformMatrices */    NULL,
	/* deformVertsEM */     NULL,
	/* deformMatricesEM */  NULL,
	/* applyModifier */     applyModifier,
	/* applyModifierEM */   NULL,
	/* initData */          initData,
	/* requiredDataMask */  NULL,
	/* freeData */          freeData,
	/* isDisabled */        NULL,
	/* updateDepgraph */    NULL,
	/* updateDepsgraph */   NULL,
	/* dependsOnTime */     NULL,
	/* dependsOnNormals */  NULL,
	/* foreachObjectLink */ NULL,
	/* foreachIDLink */     foreachIDLink,
	/* foreachTexLink */    NULL,
};
