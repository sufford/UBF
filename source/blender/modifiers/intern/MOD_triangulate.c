/*
 * MOD_triangulate.c - Triangulate modifier (backported improvements from Blender 5.2)
 */

#include "DNA_object_types.h"
#include "DNA_meshdata_types.h"

#include "BLI_utildefines.h"

#include "BKE_cdderivedmesh.h"
#include "BKE_modifier.h"

#include "bmesh.h"
#include "bmesh_tools.h"

#include "MOD_modifiertypes.h"

static DerivedMesh *triangulate_dm(
        DerivedMesh *dm,
        const int quad_method,
        const int ngon_method,
        const int min_vertices,
        const int flag)
{
	DerivedMesh *result;
	BMesh *bm;
	int total_edges, i;
	MEdge *me;

	(void)min_vertices;  /* Not fully supported in 2.79 BM_mesh_triangulate */
	(void)flag;

	bm = DM_to_bmesh(dm, true);

	BM_mesh_triangulate(bm, quad_method, ngon_method, false, NULL, NULL, NULL);

	result = CDDM_from_bmesh(bm, false);
	BM_mesh_free(bm);

	total_edges = result->getNumEdges(result);
	me = CDDM_get_edges(result);

	for (i = 0; i < total_edges; i++, me++)
		me->flag |= ME_EDGEDRAW | ME_EDGERENDER;

	result->dirty |= DM_DIRTY_NORMALS;

	return result;
}

static void initData(ModifierData *md)
{
	TriangulateModifierData *tmd = (TriangulateModifierData *)md;

	md->mode |= eModifierMode_Editmode;
	tmd->quad_method = MOD_TRIANGULATE_QUAD_SHORTEDGE;
	tmd->ngon_method = MOD_TRIANGULATE_NGON_BEAUTY;
	tmd->min_vertices = 4;
	tmd->flag = 0;
}

static DerivedMesh *applyModifier(
        ModifierData *md,
        Object *UNUSED(ob),
        DerivedMesh *dm,
        ModifierApplyFlag UNUSED(flag))
{
	TriangulateModifierData *tmd = (TriangulateModifierData *)md;
	DerivedMesh *result;

	result = triangulate_dm(dm, tmd->quad_method, tmd->ngon_method, tmd->min_vertices, tmd->flag);

	if (!result) {
		return dm;
	}

	return result;
}

ModifierTypeInfo modifierType_Triangulate = {
	/* name */              "Triangulate",
	/* structName */        "TriangulateModifierData",
	/* structSize */        sizeof(TriangulateModifierData),
	/* type */              eModifierTypeType_Constructive,
	/* flags */             eModifierTypeFlag_AcceptsMesh |
	                        eModifierTypeFlag_SupportsEditmode |
	                        eModifierTypeFlag_SupportsMapping |
	                        eModifierTypeFlag_EnableInEditmode |
	                        eModifierTypeFlag_AcceptsCVs,

	/* copyData */          modifier_copyData_generic,
	/* deformVerts */       NULL,
	/* deformMatrices */    NULL,
	/* deformVertsEM */     NULL,
	/* deformMatricesEM */  NULL,
	/* applyModifier */     applyModifier,
	/* applyModifierEM */   NULL,
	/* initData */          initData,
	/* requiredDataMask */  NULL,
	/* freeData */          NULL,
	/* isDisabled */        NULL,
	/* updateDepgraph */    NULL,
	/* updateDepsgraph */   NULL,
	/* dependsOnTime */     NULL,
	/* dependsOnNormals */	NULL,
	/* foreachObjectLink */ NULL,
	/* foreachIDLink */     NULL,
	/* foreachTexLink */    NULL,
};