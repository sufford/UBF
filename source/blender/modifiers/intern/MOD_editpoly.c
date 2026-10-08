/*
 * Edit Poly modifier - non-destructive Edit Poly (3ds Max style).
 *
 * The modifier stores a history of operations, never the resulting geometry.
 * On evaluation the history is replayed on top of the modifier input.
 */

#include <string.h>

#include "MEM_guardedalloc.h"

#include "BLI_listbase.h"
#include "BLI_utildefines.h"
#include "BLI_math.h"

#include "DNA_mesh_types.h"
#include "DNA_modifier_types.h"
#include "DNA_object_types.h"
#include "DNA_scene_types.h"

#include "BKE_cdderivedmesh.h"
#include "BKE_DerivedMesh.h"
#include "BKE_editmesh.h"
#include "BKE_editpoly.h"
#include "BKE_mesh.h"
#include "BKE_modifier.h"

#include "MOD_modifiertypes.h"

/* -------------------------------------------------------------------- */
/** \name Operation list
 * \{ */

static void editpoly_op_free(EditPolyOp *op)
{
	MEM_SAFE_FREE(op->vert_ids);
	MEM_SAFE_FREE(op->vert_deltas);
	MEM_SAFE_FREE(op->kill_vert_ids);
	MEM_SAFE_FREE(op->kill_edge_ids);
	MEM_SAFE_FREE(op->kill_face_ids);
	MEM_SAFE_FREE(op->new_vert_ids);
	MEM_SAFE_FREE(op->new_vert_co);
	MEM_SAFE_FREE(op->new_vert_srcs);
	MEM_SAFE_FREE(op->new_edge_ids);
	MEM_SAFE_FREE(op->new_edge_verts);
	MEM_SAFE_FREE(op->new_edge_srcs);
	MEM_SAFE_FREE(op->new_face_ids);
	MEM_SAFE_FREE(op->new_face_offsets);
	MEM_SAFE_FREE(op->new_loop_verts);
	MEM_SAFE_FREE(op->new_face_srcs);
	MEM_SAFE_FREE(op->new_face_mats);
	MEM_SAFE_FREE(op->mat_face_ids);
	MEM_SAFE_FREE(op->mat_values);
	MEM_freeN(op);
}

void editpoly_ops_free(EditPolyModifierData *epmd)
{
	EditPolyOp *op, *op_next;

	for (op = epmd->ops.first; op; op = op_next) {
		op_next = op->next;
		editpoly_op_free(op);
	}

	BLI_listbase_clear(&epmd->ops);
}

#define EDITPOLY_OP_COPY_ARRAY(member) \
	if (op->member) { \
		op_new->member = MEM_dupallocN(op->member); \
	} \
	else { \
		op_new->member = NULL; \
	} \
	((void)0)

static EditPolyOp *editpoly_op_copy(const EditPolyOp *op)
{
	EditPolyOp *op_new = MEM_dupallocN(op);

	op_new->next = op_new->prev = NULL;

	EDITPOLY_OP_COPY_ARRAY(vert_ids);
	EDITPOLY_OP_COPY_ARRAY(vert_deltas);
	EDITPOLY_OP_COPY_ARRAY(kill_vert_ids);
	EDITPOLY_OP_COPY_ARRAY(kill_edge_ids);
	EDITPOLY_OP_COPY_ARRAY(kill_face_ids);
	EDITPOLY_OP_COPY_ARRAY(new_vert_ids);
	EDITPOLY_OP_COPY_ARRAY(new_vert_co);
	EDITPOLY_OP_COPY_ARRAY(new_vert_srcs);
	EDITPOLY_OP_COPY_ARRAY(new_edge_ids);
	EDITPOLY_OP_COPY_ARRAY(new_edge_verts);
	EDITPOLY_OP_COPY_ARRAY(new_edge_srcs);
	EDITPOLY_OP_COPY_ARRAY(new_face_ids);
	EDITPOLY_OP_COPY_ARRAY(new_face_offsets);
	EDITPOLY_OP_COPY_ARRAY(new_loop_verts);
	EDITPOLY_OP_COPY_ARRAY(new_face_srcs);
	EDITPOLY_OP_COPY_ARRAY(new_face_mats);
	EDITPOLY_OP_COPY_ARRAY(mat_face_ids);
	EDITPOLY_OP_COPY_ARRAY(mat_values);

	return op_new;
}

void editpoly_ops_copy(EditPolyModifierData *epmd, EditPolyModifierData *target)
{
	EditPolyOp *op;

	BLI_listbase_clear(&target->ops);

	for (op = epmd->ops.first; op; op = op->next) {
		BLI_addtail(&target->ops, editpoly_op_copy(op));
	}
}

int editpoly_ops_count(EditPolyModifierData *epmd)
{
	EditPolyOp *op;
	int count = 0;

	if (epmd == NULL) {
		return 0;
	}

	for (op = epmd->ops.first; op; op = op->next) {
		/* operations that were undone are not part of the history any more */
		if ((op->flag & EDITPOLY_OP_FLAG_SKIPPED) == 0) {
			count++;
		}
	}

	return count;
}

bool editpoly_op_skip_last(EditPolyModifierData *epmd)
{
	EditPolyOp *op;

	if (epmd == NULL) {
		return false;
	}

	for (op = epmd->ops.last; op; op = op->prev) {
		if ((op->flag & EDITPOLY_OP_FLAG_SKIPPED) == 0) {
			op->flag |= EDITPOLY_OP_FLAG_SKIPPED;
			epmd->flag |= EDITPOLY_DIRTY;
			return true;
		}
	}

	return false;
}

bool editpoly_op_unskip_first(EditPolyModifierData *epmd)
{
	EditPolyOp *op;

	if (epmd == NULL) {
		return false;
	}

	for (op = epmd->ops.first; op; op = op->next) {
		if (op->flag & EDITPOLY_OP_FLAG_SKIPPED) {
			op->flag &= ~EDITPOLY_OP_FLAG_SKIPPED;
			epmd->flag |= EDITPOLY_HAS_OPS | EDITPOLY_DIRTY;
			return true;
		}
	}

	return false;
}

void editpoly_op_remove_last(EditPolyModifierData *epmd)
{
	EditPolyOp *op;

	if (epmd == NULL || epmd->ops.last == NULL) {
		return;
	}

	op = epmd->ops.last;
	BLI_remlink(&epmd->ops, op);
	editpoly_op_free(op);

	if (epmd->ops.first == NULL) {
		epmd->flag &= ~EDITPOLY_HAS_OPS;
		/* the history is gone, the current input is the new reference */
		epmd->flag &= ~EDITPOLY_INPUT_MISMATCH;
		epmd->input_vert_count = 0;
		epmd->input_edge_count = 0;
		epmd->input_face_count = 0;
	}

	epmd->flag |= EDITPOLY_DIRTY;
}

void editpoly_ops_drop_skipped(EditPolyModifierData *epmd)
{
	if (epmd == NULL) {
		return;
	}

	/* the undone operations are always the tail of the list */
	while (epmd->ops.last != NULL &&
	       (((EditPolyOp *)epmd->ops.last)->flag & EDITPOLY_OP_FLAG_SKIPPED))
	{
		editpoly_op_remove_last(epmd);
	}
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Modifier callbacks
 * \{ */

static void initData(ModifierData *md)
{
	EditPolyModifierData *epmd = (EditPolyModifierData *)md;

	BLI_listbase_clear(&epmd->ops);
	epmd->flag = 0;
}

static void copyData(ModifierData *md, ModifierData *target)
{
	EditPolyModifierData *epmd = (EditPolyModifierData *)md;
	EditPolyModifierData *tepmd = (EditPolyModifierData *)target;

	editpoly_ops_copy(epmd, tepmd);
	tepmd->flag = epmd->flag;
	tepmd->input_vert_count = epmd->input_vert_count;
	tepmd->input_edge_count = epmd->input_edge_count;
	tepmd->input_face_count = epmd->input_face_count;
}

static void freeData(ModifierData *md)
{
	EditPolyModifierData *epmd = (EditPolyModifierData *)md;

	editpoly_ops_free(epmd);
}

static DerivedMesh *applyModifier(ModifierData *md, Object *ob,
                                  DerivedMesh *derivedData,
                                  ModifierApplyFlag UNUSED(flag))
{
	EditPolyModifierData *epmd = (EditPolyModifierData *)md;
	DerivedMesh *ref_dm, *result;
	Mesh *me;

	if ((epmd->flag & EDITPOLY_IN_EDITMODE) == 0 && BLI_listbase_is_empty(&epmd->ops)) {
		return derivedData;
	}

	/* While this modifier is the one being edited the edit mesh already contains
	 * the whole history, applying it again would double it. The geometry of the
	 * edit mode is what the stack has to work with: the modifiers are not
	 * blocked, so the ones that come after Edit Poly keep working (both the
	 * other Edit Poly modifiers and every other modifier type). */
	if (epmd->flag & EDITPOLY_IN_EDITMODE) {
		BMEditMesh *em = BKE_editmesh_from_object(ob);

		if (em && em->bm) {
			return CDDM_from_editbmesh(em, false, false);
		}

		return derivedData;
	}

	/* the history is replayed on the input geometry of the modifier */
	me = editpoly_mesh_from_derived(ob, derivedData);

	/* detect that the modifiers above Edit Poly changed the input after the
	 * history was recorded: the replay would reference other elements */
	editpoly_input_check(me, epmd);

	editpoly_mesh_apply_history(me, epmd);
	editpoly_mesh_ids_free(me);

	/* CDDM_from_mesh() only references the mesh data (the layers get
	 * CD_FLAG_NOFREE), so the mesh must stay alive as long as that derived mesh
	 * lives. Copy it into a self contained one before freeing the mesh. */
	ref_dm = CDDM_from_mesh(me);
	result = CDDM_copy(ref_dm);
	ref_dm->release(ref_dm);

	BKE_mesh_free(me);
	MEM_freeN(me);

	return result;
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
	/* foreachIDLink */     NULL,
	/* foreachTexLink */    NULL,
};

/** \} */
