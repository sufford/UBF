/*
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software Foundation,
 * Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 */

/** \file blender/modifiers/intern/MOD_editpoly_ops.c
 *  \ingroup modifiers
 *
 *  Evaluation of the Edit Poly operation history.
 *
 *  The history is replayed on top of the *input* geometry of the modifier,
 *  which is the base mesh with all modifiers that come before Edit Poly applied.
 *
 *  Elements are addressed by an id stored in the "editpoly_id" custom data
 *  layer, because the order of bmesh elements is not stable across operators.
 */

#include <string.h>

#include "MEM_guardedalloc.h"

#include "BLI_utildefines.h"
#include "BLI_ghash.h"
#include "BLI_listbase.h"
#include "BLI_math.h"

#include "DNA_ID.h"
#include "DNA_mesh_types.h"
#include "DNA_modifier_types.h"
#include "DNA_object_types.h"
#include "DNA_scene_types.h"

#include "BKE_customdata.h"
#include "BKE_DerivedMesh.h"
#include "BKE_editpoly.h"
#include "BKE_library.h"
#include "BKE_main.h"
#include "BKE_mesh.h"
#include "BKE_modifier.h"

#include "bmesh.h"

/* -------------------------------------------------------------------- */
/** \name Element ids
 *
 * Element ids are the only stable way to name a vertex / edge / face between
 * two operations, they are stored in an int layer of the mesh (and of the
 * bmesh while editing).
 * \{ */

#define EDITPOLY_ID_LAYER "editpoly_id"

static int editpoly_cd_offset(const struct CustomData *data, const int layer_index)
{
	return (layer_index != -1) ? data->layers[layer_index].offset : -1;
}

bool editpoly_ids_offsets(BMesh *bm, int *r_vert, int *r_edge, int *r_face)
{
	const int vi = CustomData_get_named_layer_index(&bm->vdata, CD_PROP_INT, EDITPOLY_ID_LAYER);
	const int ei = CustomData_get_named_layer_index(&bm->edata, CD_PROP_INT, EDITPOLY_ID_LAYER);
	const int fi = CustomData_get_named_layer_index(&bm->pdata, CD_PROP_INT, EDITPOLY_ID_LAYER);

	*r_vert = editpoly_cd_offset(&bm->vdata, vi);
	*r_edge = editpoly_cd_offset(&bm->edata, ei);
	*r_face = editpoly_cd_offset(&bm->pdata, fi);

	return (vi != -1 && ei != -1 && fi != -1);
}

int editpoly_ids_max(BMesh *bm)
{
	BMIter iter;
	BMVert *v;
	BMEdge *e;
	BMFace *f;
	int voff, eoff, foff;
	int max = 0;

	if (!editpoly_ids_offsets(bm, &voff, &eoff, &foff)) {
		return 0;
	}

	BM_ITER_MESH (v, &iter, bm, BM_VERTS_OF_MESH) {
		max = max_ii(max, BM_ELEM_CD_GET_INT(v, voff));
	}
	BM_ITER_MESH (e, &iter, bm, BM_EDGES_OF_MESH) {
		max = max_ii(max, BM_ELEM_CD_GET_INT(e, eoff));
	}
	BM_ITER_MESH (f, &iter, bm, BM_FACES_OF_MESH) {
		max = max_ii(max, BM_ELEM_CD_GET_INT(f, foff));
	}

	return max;
}

/**
 * Create the "editpoly_id" layers of \a me and number the elements 1..N.
 * Does nothing when the layers are already there.
 */
static void editpoly_mesh_ids_ensure(Mesh *me)
{
	int *ids;
	int i;

	if (CustomData_get_named_layer_index(&me->vdata, CD_PROP_INT, EDITPOLY_ID_LAYER) != -1 &&
	    CustomData_get_named_layer_index(&me->edata, CD_PROP_INT, EDITPOLY_ID_LAYER) != -1 &&
	    CustomData_get_named_layer_index(&me->pdata, CD_PROP_INT, EDITPOLY_ID_LAYER) != -1)
	{
		return;
	}

	ids = CustomData_add_layer_named(&me->vdata, CD_PROP_INT, CD_CALLOC, NULL, me->totvert,
	                                 EDITPOLY_ID_LAYER);
	for (i = 0; i < me->totvert; i++) {
		ids[i] = i + 1;
	}

	ids = CustomData_add_layer_named(&me->edata, CD_PROP_INT, CD_CALLOC, NULL, me->totedge,
	                                 EDITPOLY_ID_LAYER);
	for (i = 0; i < me->totedge; i++) {
		ids[i] = i + 1;
	}

	ids = CustomData_add_layer_named(&me->pdata, CD_PROP_INT, CD_CALLOC, NULL, me->totpoly,
	                                 EDITPOLY_ID_LAYER);
	for (i = 0; i < me->totpoly; i++) {
		ids[i] = i + 1;
	}
}

static void editpoly_mesh_ids_remove(Mesh *me)
{
	int index;

	index = CustomData_get_named_layer_index(&me->vdata, CD_PROP_INT, EDITPOLY_ID_LAYER);
	if (index != -1) {
		CustomData_free_layer(&me->vdata, CD_PROP_INT, me->totvert, index);
	}
	index = CustomData_get_named_layer_index(&me->edata, CD_PROP_INT, EDITPOLY_ID_LAYER);
	if (index != -1) {
		CustomData_free_layer(&me->edata, CD_PROP_INT, me->totedge, index);
	}
	index = CustomData_get_named_layer_index(&me->pdata, CD_PROP_INT, EDITPOLY_ID_LAYER);
	if (index != -1) {
		CustomData_free_layer(&me->pdata, CD_PROP_INT, me->totpoly, index);
	}
}

/** id -> element lookup table of the current state of \a bm. */
static GHash *editpoly_id_hash_new(BMesh *bm, const char htype, const int offset)
{
	GHash *gh = BLI_ghash_ptr_new("EditPoly id map");
	BMIter iter;

	if (offset == -1) {
		return gh;
	}

	if (htype == BM_VERT) {
		BMVert *v;
		BM_ITER_MESH (v, &iter, bm, BM_VERTS_OF_MESH) {
			const int id = BM_ELEM_CD_GET_INT(v, offset);
			if (id != 0) {
				BLI_ghash_insert(gh, POINTER_FROM_INT(id), v);
			}
		}
	}
	else if (htype == BM_EDGE) {
		BMEdge *e;
		BM_ITER_MESH (e, &iter, bm, BM_EDGES_OF_MESH) {
			const int id = BM_ELEM_CD_GET_INT(e, offset);
			if (id != 0) {
				BLI_ghash_insert(gh, POINTER_FROM_INT(id), e);
			}
		}
	}
	else {
		BMFace *f;
		BM_ITER_MESH (f, &iter, bm, BM_FACES_OF_MESH) {
			const int id = BM_ELEM_CD_GET_INT(f, offset);
			if (id != 0) {
				BLI_ghash_insert(gh, POINTER_FROM_INT(id), f);
			}
		}
	}

	return gh;
}

/** Set of element ids, used to find out what has to be removed. */
static GHash *editpoly_id_set_new(const int *ids, const int ids_count)
{
	GHash *gh;
	int i;

	if (ids == NULL || ids_count <= 0) {
		return NULL;
	}

	gh = BLI_ghash_ptr_new("EditPoly id set");
	for (i = 0; i < ids_count; i++) {
		if (ids[i] != 0) {
			BLI_ghash_insert(gh, POINTER_FROM_INT(ids[i]), POINTER_FROM_INT(1));
		}
	}

	return gh;
}

#define EDITPOLY_ID_LOOKUP(gh, id) ((gh) ? BLI_ghash_lookup((gh), POINTER_FROM_INT(id)) : NULL)

/** \} */

/* -------------------------------------------------------------------- */
/** \name History playback
 * \{ */

/**
 * Copy the loop layers (UV, vertex colors, ...) of the loops of \a f from
 * \a fsrc, matching the loops by the vertex they belong to.
 */
static void editpoly_copy_loop_attrs(BMesh *bm, BMFace *f, BMFace *fsrc,
                                     const int voff, GHash *vert_srcs)
{
	BMIter liter;
	BMLoop *l;

	if (fsrc == NULL) {
		return;
	}

	BM_ITER_ELEM (l, &liter, f, BM_LOOPS_OF_FACE) {
		const BMVert *v_key = EDITPOLY_ID_LOOKUP(vert_srcs, BM_ELEM_CD_GET_INT(l->v, voff));
		const int key = BM_ELEM_CD_GET_INT(v_key ? v_key : l->v, voff);
		BMIter liter_src;
		BMLoop *l_src, *l_match = NULL;

		BM_ITER_ELEM (l_src, &liter_src, fsrc, BM_LOOPS_OF_FACE) {
			if (BM_ELEM_CD_GET_INT(l_src->v, voff) == key) {
				l_match = l_src;
				break;
			}
		}
		if (l_match == NULL) {
			l_match = fsrc->l_first;
		}

		BM_elem_attrs_copy(bm, bm, l_match, l);
	}
}

/**
 * Remove the elements listed in the kill lists of \a op.
 *
 * The elements are searched in the mesh itself (by their id) and not in a
 * lookup table, so that elements that were already removed as a side effect
 * of removing another element are never touched twice.
 */
static void editpoly_kill_removed(BMesh *bm, const EditPolyOp *op,
                                  const int voff, const int eoff, const int foff)
{
	GHash *kv = editpoly_id_set_new(op->kill_vert_ids, op->kill_vert_count);
	GHash *ke = editpoly_id_set_new(op->kill_edge_ids, op->kill_edge_count);
	GHash *kf = editpoly_id_set_new(op->kill_face_ids, op->kill_face_count);
	BMIter iter;

	if (kf) {
		BMFace *f, *f_next;
		BM_ITER_MESH_MUTABLE (f, f_next, &iter, bm, BM_FACES_OF_MESH) {
			if (BLI_ghash_lookup(kf, POINTER_FROM_INT(BM_ELEM_CD_GET_INT(f, foff)))) {
				BM_face_kill(bm, f);
			}
		}
		BLI_ghash_free(kf, NULL, NULL);
	}

	if (ke) {
		BMEdge *e, *e_next;
		BM_ITER_MESH_MUTABLE (e, e_next, &iter, bm, BM_EDGES_OF_MESH) {
			if (BLI_ghash_lookup(ke, POINTER_FROM_INT(BM_ELEM_CD_GET_INT(e, eoff))) && e->l == NULL) {
				BM_edge_kill(bm, e);
			}
		}
		BLI_ghash_free(ke, NULL, NULL);
	}

	if (kv) {
		BMVert *v, *v_next;
		BM_ITER_MESH_MUTABLE (v, v_next, &iter, bm, BM_VERTS_OF_MESH) {
			if (BLI_ghash_lookup(kv, POINTER_FROM_INT(BM_ELEM_CD_GET_INT(v, voff)))) {
				BM_vert_kill(bm, v);
			}
		}
		BLI_ghash_free(kv, NULL, NULL);
	}
}

/** Add the geometry created by \a op. */
static void editpoly_apply_patch(BMesh *bm, const EditPolyOp *op,
                                 const int voff, const int eoff, const int foff,
                                 GHash *vhash, GHash *ehash, GHash *fhash)
{
	GHash *vert_srcs = BLI_ghash_ptr_new("EditPoly vert sources");
	int i;

	/* new vertices */
	for (i = 0; i < op->new_vert_count; i++) {
		const int id = op->new_vert_ids[i];
		BMVert *v_src = EDITPOLY_ID_LOOKUP(vhash, op->new_vert_srcs ? op->new_vert_srcs[i] : 0);
		BMVert *v = BM_vert_create(bm, op->new_vert_co[i], v_src, BM_CREATE_NOP);

		BM_ELEM_CD_SET_INT(v, voff, id);
		BLI_ghash_insert(vhash, POINTER_FROM_INT(id), v);
		if (v_src) {
			BLI_ghash_insert(vert_srcs, POINTER_FROM_INT(id), v_src);
		}
	}

	/* new edges */
	for (i = 0; i < op->new_edge_count; i++) {
		BMVert *v1 = EDITPOLY_ID_LOOKUP(vhash, op->new_edge_verts[i][0]);
		BMVert *v2 = EDITPOLY_ID_LOOKUP(vhash, op->new_edge_verts[i][1]);
		BMEdge *e_src, *e;
		int id = op->new_edge_ids[i];

		if (v1 == NULL || v2 == NULL || v1 == v2) {
			continue;
		}

		e_src = EDITPOLY_ID_LOOKUP(ehash, op->new_edge_srcs ? op->new_edge_srcs[i] : 0);
		e = BM_edge_create(bm, v1, v2, e_src, BM_CREATE_NO_DOUBLE);
		if (e == NULL) {
			continue;
		}
		if (BM_ELEM_CD_GET_INT(e, eoff) == 0) {
			BM_ELEM_CD_SET_INT(e, eoff, id);
			BLI_ghash_insert(ehash, POINTER_FROM_INT(id), e);
		}
	}

	/* new faces */
	for (i = 0; i < op->new_face_count; i++) {
		const int start = op->new_face_offsets[i];
		const int len = op->new_face_offsets[i + 1] - start;
		BMFace *f_src = EDITPOLY_ID_LOOKUP(fhash, op->new_face_srcs ? op->new_face_srcs[i] : 0);
		BMFace *f;
		BMVert **verts;
		int j;

		if (len < 3) {
			continue;
		}

		verts = MEM_mallocN(sizeof(BMVert *) * len, "EditPoly face verts");
		for (j = 0; j < len; j++) {
			verts[j] = EDITPOLY_ID_LOOKUP(vhash, op->new_loop_verts[start + j]);
			if (verts[j] == NULL) {
				break;
			}
		}

		if (j == len) {
			f = BM_face_create_verts(bm, verts, len, f_src, BM_CREATE_NOP, true);
			if (f != NULL) {
				const int id = op->new_face_ids[i];
				BM_ELEM_CD_SET_INT(f, foff, id);
				if (op->new_face_mats) {
					f->mat_nr = op->new_face_mats[i];
				}
				BLI_ghash_insert(fhash, POINTER_FROM_INT(id), f);
				editpoly_copy_loop_attrs(bm, f, f_src, voff, vert_srcs);
			}
		}

		MEM_freeN(verts);
	}

	BLI_ghash_free(vert_srcs, NULL, NULL);

	/* removed geometry, done last so that the elements above could still be
	 * copied from the geometry they replace */
	editpoly_kill_removed(bm, op, voff, eoff, foff);
}

void editpoly_apply_op(BMesh *bm, const EditPolyOp *op)
{
	const bool has_patch = (op->flag & EDITPOLY_OP_FLAG_TOPOLOGY) != 0;
	int voff, eoff, foff;
	GHash *vhash = NULL, *ehash = NULL, *fhash = NULL;
	int i;

	if (bm == NULL || op == NULL) {
		return;
	}

	if (!editpoly_ids_offsets(bm, &voff, &eoff, &foff)) {
		/* the ids are gone: the history does not belong to this geometry */
		return;
	}

	if (has_patch || op->mat_face_count) {
		vhash = editpoly_id_hash_new(bm, BM_VERT, voff);
		ehash = editpoly_id_hash_new(bm, BM_EDGE, eoff);
		fhash = editpoly_id_hash_new(bm, BM_FACE, foff);
	}
	else if (op->vert_count) {
		vhash = editpoly_id_hash_new(bm, BM_VERT, voff);
	}

	/* transformation of the vertices */
	for (i = 0; i < op->vert_count; i++) {
		BMVert *v = EDITPOLY_ID_LOOKUP(vhash, op->vert_ids[i]);

		if (v == NULL) {
			continue;
		}

		if (op->vert_deltas) {
			add_v3_v3(v->co, op->vert_deltas[i]);
		}
		else if (ELEM(op->type, EDITPOLY_OP_ROTATE, EDITPOLY_OP_SCALE)) {
			mul_m4_v3((const float (*)[4])op->matrix, v->co);
		}
		else {
			add_v3_v3(v->co, op->offset);
		}
	}

	/* material of existing polygons */
	for (i = 0; i < op->mat_face_count; i++) {
		BMFace *f = EDITPOLY_ID_LOOKUP(fhash, op->mat_face_ids[i]);
		if (f != NULL) {
			f->mat_nr = op->mat_values[i];
		}
	}

	if (has_patch) {
		editpoly_apply_patch(bm, op, voff, eoff, foff, vhash, ehash, fhash);
	}

	if (vhash) BLI_ghash_free(vhash, NULL, NULL);
	if (ehash) BLI_ghash_free(ehash, NULL, NULL);
	if (fhash) BLI_ghash_free(fhash, NULL, NULL);
}

void editpoly_apply_history(BMesh *bm, EditPolyModifierData *epmd)
{
	EditPolyOp *op;

	if (bm == NULL || epmd == NULL) {
		return;
	}

	for (op = epmd->ops.first; op; op = op->next) {
		/* operations that were undone are kept for redo, they are not applied */
		if (op->flag & EDITPOLY_OP_FLAG_SKIPPED) {
			continue;
		}

		editpoly_apply_op(bm, op);
	}
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Input geometry
 * \{ */

/**
 * Compare the input topology with the one the history was recorded on, and keep
 * the EDITPOLY_INPUT_MISMATCH flag up to date. When the history is empty the
 * current input becomes the new reference.
 *
 * Returns whether the history no longer matches the input.
 */
bool editpoly_input_check(Mesh *me, EditPolyModifierData *epmd)
{
	bool mismatch = false;

	if (BLI_listbase_is_empty(&epmd->ops)) {
		/* the history starts from this input */
		epmd->input_vert_count = me->totvert;
		epmd->input_edge_count = me->totedge;
		epmd->input_face_count = me->totpoly;
		epmd->flag &= ~EDITPOLY_INPUT_MISMATCH;
	}
	else if (epmd->input_vert_count > 0) {
		mismatch = (me->totvert != epmd->input_vert_count ||
		            me->totedge != epmd->input_edge_count ||
		            me->totpoly != epmd->input_face_count);
		epmd->flag &= ~EDITPOLY_INPUT_MISMATCH;
		if (mismatch) {
			epmd->flag |= EDITPOLY_INPUT_MISMATCH;
		}
	}

	return mismatch;
}

/**
 * Replay the whole history on a #Mesh, assigning the element ids first.
 */
void editpoly_mesh_apply_history(Mesh *me, EditPolyModifierData *epmd)
{
	BMesh *bm;
	const BMAllocTemplate allocsize = BMALLOC_TEMPLATE_FROM_ME(me);

	editpoly_mesh_ids_ensure(me);

	bm = BM_mesh_create(&allocsize, &(struct BMeshCreateParams){.use_toolflags = true});
	BM_mesh_bm_from_me(bm, me, &(struct BMeshFromMeshParams){.calc_face_normal = true});

	if (epmd != NULL && !BLI_listbase_is_empty(&epmd->ops)) {
		editpoly_apply_history(bm, epmd);
	}

	BM_mesh_bm_to_me(NULL, bm, me, &(struct BMeshToMeshParams){0});
	BM_mesh_free(bm);
}

/**
 * Build a free standing #Mesh out of the geometry of a derived mesh, keeping
 * the material slots of \a ob.
 */
Mesh *editpoly_mesh_from_derived(Object *ob, DerivedMesh *dm)
{
	Mesh *src_me = (ob && ob->type == OB_MESH) ? (Mesh *)ob->data : NULL;
	Mesh *me;

	me = BKE_libblock_alloc(NULL, ID_ME, "EditPoly",
	                        LIB_ID_CREATE_NO_MAIN | LIB_ID_CREATE_NO_USER_REFCOUNT);
	BKE_mesh_init(me);

	/* The temporary mesh becomes ob->data, so the object holds one reference to it
	 * even though it is not refcounted (LIB_ID_CREATE_NO_USER_REFCOUNT). Keeping
	 * the count at 1 leaves the regular id_us_min() of a deleted object in range;
	 * the mesh itself is freed by editpoly (see editpoly_runtime_object_free). */
	me->id.us = 1;

	DM_to_mesh(dm, me, ob, CD_MASK_MESH, false);

	if (src_me && src_me->totcol) {
		me->totcol = src_me->totcol;
		me->mat = MEM_dupallocN(src_me->mat);
	}

	return me;
}

/**
 * Build the geometry the modifier is applied to: the object mesh with every
 * modifier that comes before Edit Poly evaluated, plus the recorded history.
 *
 * The returned mesh is a free standing datablock (not part of \a bmain), it has
 * to be freed with #BKE_mesh_free() + MEM_freeN() by the caller.
 */
Mesh *editpoly_build_mesh(Main *bmain, Scene *scene, Object *ob, EditPolyModifierData *epmd)
{
	DerivedMesh *dm = NULL;
	Mesh *me = NULL;
	ModifierData *md;
	bool found = false;
	bool ok = false;

	(void)bmain;

	if (ob == NULL || epmd == NULL || ob->type != OB_MESH || ob->data == NULL) {
		return NULL;
	}

	/* Temporarily disable Edit Poly and everything below it, so the evaluated
	 * mesh is exactly the input of the modifier. */
	for (md = ob->modifiers.first; md; md = md->next) {
		if (md == (ModifierData *)epmd) {
			found = true;
		}
		if (found) {
			md->mode |= eModifierMode_DisableTemporary;
		}
	}

	if (found) {
		dm = mesh_create_derived_view(scene, ob, CD_MASK_MESH);
		ok = true;
	}

	/* restore the modifier modes */
	found = false;
	for (md = ob->modifiers.first; md; md = md->next) {
		if (md == (ModifierData *)epmd) {
			found = true;
		}
		if (found) {
			md->mode &= ~eModifierMode_DisableTemporary;
		}
	}

	if (!ok || dm == NULL) {
		return NULL;
	}

	me = editpoly_mesh_from_derived(ob, dm);
	dm->release(dm);

	/* the history is replayed on this input: detect that the modifiers above
	 * Edit Poly changed it (and the history references other elements) */
	editpoly_input_check(me, epmd);

	/* also assigns the element ids when the history is still empty */
	editpoly_mesh_apply_history(me, epmd);

	return me;
}

/**
 * Remove the "editpoly_id" layers, they are an implementation detail and must
 * not end up in the evaluated geometry of the modifier.
 */
void editpoly_mesh_ids_free(Mesh *me)
{
	editpoly_mesh_ids_remove(me);
}

/** \} */
