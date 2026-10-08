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

/** \file blender/editors/object/editpoly_record.c
 *  \ingroup editors
 *
 *  Recording of the Edit Poly operation history.
 *
 *  The history is not built by intercepting the mesh operators (there are far
 *  too many of them), but by comparing the geometry with the state it had when
 *  the previous operation was recorded:
 *
 *  - the previous state is kept as a #EditPolySnapshot (positions, topology and
 *    the "editpoly_id" of every element),
 *  - elements are recognised by their #BMVert / #BMEdge / #BMFace pointer, which
 *    is stable as long as an element lives, and are named inside the operation
 *    by their id,
 *  - a vertex that only moved becomes a delta in the operation,
 *  - everything else (extrude, bevel, delete, ...) becomes a topology patch:
 *    the removed elements, the new elements and, for every new element, the
 *    element its attributes have to be copied from.
 */

#include <string.h>

#include "MEM_guardedalloc.h"

#include "BLI_utildefines.h"
#include "BLI_ghash.h"
#include "BLI_listbase.h"
#include "BLI_math.h"
#include "BLI_string.h"

#include "DNA_mesh_types.h"
#include "DNA_modifier_types.h"
#include "DNA_object_types.h"

#include "BKE_customdata.h"
#include "BKE_depsgraph.h"
#include "BKE_editmesh.h"
#include "BKE_editpoly.h"

#include "bmesh.h"

#include "ED_editpoly.h"

/* -------------------------------------------------------------------- */
/** \name Growable arrays
 * \{ */

typedef struct EditPolyVec {
	void *data;
	int count;
	int cap;
	int elem_size;
} EditPolyVec;

static void epvec_init(EditPolyVec *v, const int elem_size)
{
	memset(v, 0, sizeof(*v));
	v->elem_size = elem_size;
}

static void epvec_push(EditPolyVec *v, const void *elem)
{
	if (v->count == v->cap) {
		v->cap = v->cap ? v->cap * 2 : 16;
		if (v->data) {
			v->data = MEM_reallocN(v->data, (size_t)v->cap * v->elem_size);
		}
		else {
			v->data = MEM_mallocN((size_t)v->cap * v->elem_size, "EditPoly op data");
		}
	}

	memcpy((char *)v->data + (size_t)v->count * v->elem_size, elem, v->elem_size);
	v->count++;
}

static void *epvec_detach(EditPolyVec *v, int *r_count)
{
	void *data = v->data;

	if (r_count) {
		*r_count = v->count;
	}
	v->data = NULL;
	v->count = 0;
	v->cap = 0;

	return data;
}

static void epvec_free(EditPolyVec *v)
{
	MEM_SAFE_FREE(v->data);
	v->count = v->cap = 0;
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Snapshot
 * \{ */

/** Geometry of the mesh as it was right after the last recorded operation. */
typedef struct EditPolySnapshot {
	int totvert, totedge, totface, totloop;

	const void **vptr;
	int *vid;
	float (*vco)[3];

	const void **eptr;
	int *eid;
	int (*everts)[2];

	const void **fptr;
	int *fid;
	int *foffs;         /* totface + 1 offsets into floops */
	int *floops;        /* totloop vertex ids */
	short *fmat;
} EditPolySnapshot;

static void editpoly_snapshot_free(EditPolySnapshot *s)
{
	if (s == NULL) {
		return;
	}
	MEM_SAFE_FREE(s->vptr);
	MEM_SAFE_FREE(s->vid);
	MEM_SAFE_FREE(s->vco);
	MEM_SAFE_FREE(s->eptr);
	MEM_SAFE_FREE(s->eid);
	MEM_SAFE_FREE(s->everts);
	MEM_SAFE_FREE(s->fptr);
	MEM_SAFE_FREE(s->fid);
	MEM_SAFE_FREE(s->foffs);
	MEM_SAFE_FREE(s->floops);
	MEM_SAFE_FREE(s->fmat);
	MEM_freeN(s);
}

/** The runtime keeps the snapshot as an opaque pointer and frees it with this
 *  callback, which lets BKE drop a session of an object that is being deleted. */
static void editpoly_snapshot_free_cb(void *s)
{
	editpoly_snapshot_free(s);
}

static EditPolySnapshot *editpoly_snapshot_new(BMesh *bm,
                                               const int voff, const int eoff, const int foff)
{
	EditPolySnapshot *s = MEM_callocN(sizeof(EditPolySnapshot), "EditPolySnapshot");
	BMIter iter, liter;
	BMVert *v;
	BMEdge *e;
	BMFace *f;
	BMLoop *l;
	int i, loop_off;

	s->totvert = bm->totvert;
	s->totedge = bm->totedge;
	s->totface = bm->totface;
	s->totloop = bm->totloop;

	if (s->totvert) {
		s->vptr = MEM_mallocN(sizeof(void *) * s->totvert, "EditPoly snapshot verts");
		s->vid = MEM_mallocN(sizeof(int) * s->totvert, "EditPoly snapshot vert ids");
		s->vco = MEM_mallocN(sizeof(float[3]) * s->totvert, "EditPoly snapshot vert cos");
		i = 0;
		BM_ITER_MESH (v, &iter, bm, BM_VERTS_OF_MESH) {
			s->vptr[i] = v;
			s->vid[i] = BM_ELEM_CD_GET_INT(v, voff);
			copy_v3_v3(s->vco[i], v->co);
			i++;
		}
	}

	if (s->totedge) {
		s->eptr = MEM_mallocN(sizeof(void *) * s->totedge, "EditPoly snapshot edges");
		s->eid = MEM_mallocN(sizeof(int) * s->totedge, "EditPoly snapshot edge ids");
		s->everts = MEM_mallocN(sizeof(int[2]) * s->totedge, "EditPoly snapshot edge verts");
		i = 0;
		BM_ITER_MESH (e, &iter, bm, BM_EDGES_OF_MESH) {
			s->eptr[i] = e;
			s->eid[i] = BM_ELEM_CD_GET_INT(e, eoff);
			s->everts[i][0] = BM_ELEM_CD_GET_INT(e->v1, voff);
			s->everts[i][1] = BM_ELEM_CD_GET_INT(e->v2, voff);
			i++;
		}
	}

	if (s->totface) {
		s->fptr = MEM_mallocN(sizeof(void *) * s->totface, "EditPoly snapshot faces");
		s->fid = MEM_mallocN(sizeof(int) * s->totface, "EditPoly snapshot face ids");
		s->foffs = MEM_mallocN(sizeof(int) * (s->totface + 1), "EditPoly snapshot face offsets");
		s->floops = s->totloop ? MEM_mallocN(sizeof(int) * s->totloop, "EditPoly snapshot loops") : NULL;
		s->fmat = MEM_mallocN(sizeof(short) * s->totface, "EditPoly snapshot face materials");
		i = 0;
		loop_off = 0;
		BM_ITER_MESH (f, &iter, bm, BM_FACES_OF_MESH) {
			s->fptr[i] = f;
			s->fid[i] = BM_ELEM_CD_GET_INT(f, foff);
			s->fmat[i] = f->mat_nr;
			s->foffs[i] = loop_off;
			BM_ITER_ELEM (l, &liter, f, BM_LOOPS_OF_FACE) {
				s->floops[loop_off++] = BM_ELEM_CD_GET_INT(l->v, voff);
			}
			i++;
		}
		s->foffs[s->totface] = loop_off;
	}

	return s;
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Lookup helpers
 * \{ */

/** pointer -> index + 1 of the snapshot arrays, 0 when not found. */
static GHash *ep_ptr_map(const void **ptrs, const int count)
{
	GHash *gh = BLI_ghash_ptr_new("EditPoly snapshot map");
	int i;

	for (i = 0; i < count; i++) {
		if (ptrs[i]) {
			BLI_ghash_insert(gh, (void *)ptrs[i], POINTER_FROM_INT(i + 1));
		}
	}

	return gh;
}

static int ep_ptr_index(GHash *gh, const void *ptr)
{
	return (int)POINTER_AS_INT(BLI_ghash_lookup(gh, ptr));
}

static GHash *ep_id_set(const int *ids, const int count)
{
	GHash *gh = BLI_ghash_ptr_new("EditPoly id set");
	int i;

	for (i = 0; i < count; i++) {
		if (ids[i] != 0) {
			BLI_ghash_insert(gh, POINTER_FROM_INT(ids[i]), POINTER_FROM_INT(1));
		}
	}

	return gh;
}

static bool ep_id_in_set(GHash *gh, const int id)
{
	return (id != 0) && (BLI_ghash_lookup(gh, POINTER_FROM_INT(id)) != NULL);
}

static bool ep_same_pair(const int a1, const int a2, const int b1, const int b2)
{
	return ((a1 == b1 && a2 == b2) || (a1 == b2 && a2 == b1));
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Operation builder
 * \{ */

typedef struct EditPolyOpBuild {
	EditPolyVec move_ids;       /* int */
	EditPolyVec move_deltas;    /* float[3] */
	EditPolyVec kill_v;         /* int */
	EditPolyVec kill_e;         /* int */
	EditPolyVec kill_f;         /* int */
	EditPolyVec nv_ids;         /* int */
	EditPolyVec nv_co;          /* float[3] */
	EditPolyVec nv_srcs;        /* int */
	EditPolyVec ne_ids;         /* int */
	EditPolyVec ne_verts;       /* int[2] */
	EditPolyVec ne_srcs;        /* int */
	EditPolyVec nf_ids;         /* int */
	EditPolyVec nf_offs;        /* int */
	EditPolyVec nf_srcs;        /* int */
	EditPolyVec nf_mats;        /* short */
	EditPolyVec nl_verts;       /* int */
	EditPolyVec mat_ids;        /* int */
	EditPolyVec mat_vals;       /* short */
	bool topology;
	bool moved;
} EditPolyOpBuild;

static void epbuild_init(EditPolyOpBuild *b)
{
	memset(b, 0, sizeof(*b));
	epvec_init(&b->move_ids, sizeof(int));
	epvec_init(&b->move_deltas, sizeof(float[3]));
	epvec_init(&b->kill_v, sizeof(int));
	epvec_init(&b->kill_e, sizeof(int));
	epvec_init(&b->kill_f, sizeof(int));
	epvec_init(&b->nv_ids, sizeof(int));
	epvec_init(&b->nv_co, sizeof(float[3]));
	epvec_init(&b->nv_srcs, sizeof(int));
	epvec_init(&b->ne_ids, sizeof(int));
	epvec_init(&b->ne_verts, sizeof(int[2]));
	epvec_init(&b->ne_srcs, sizeof(int));
	epvec_init(&b->nf_ids, sizeof(int));
	epvec_init(&b->nf_offs, sizeof(int));
	epvec_init(&b->nf_srcs, sizeof(int));
	epvec_init(&b->nf_mats, sizeof(short));
	epvec_init(&b->nl_verts, sizeof(int));
	epvec_init(&b->mat_ids, sizeof(int));
	epvec_init(&b->mat_vals, sizeof(short));
}

static void epbuild_free(EditPolyOpBuild *b)
{
	epvec_free(&b->move_ids);
	epvec_free(&b->move_deltas);
	epvec_free(&b->kill_v);
	epvec_free(&b->kill_e);
	epvec_free(&b->kill_f);
	epvec_free(&b->nv_ids);
	epvec_free(&b->nv_co);
	epvec_free(&b->nv_srcs);
	epvec_free(&b->ne_ids);
	epvec_free(&b->ne_verts);
	epvec_free(&b->ne_srcs);
	epvec_free(&b->nf_ids);
	epvec_free(&b->nf_offs);
	epvec_free(&b->nf_srcs);
	epvec_free(&b->nf_mats);
	epvec_free(&b->nl_verts);
	epvec_free(&b->mat_ids);
	epvec_free(&b->mat_vals);
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Diff
 * \{ */

static int ep_neighbor_vert_id(BMVert *v, const int voff, GHash *vmap, const EditPolySnapshot *prev)
{
	BMIter iter;
	BMEdge *e;

	BM_ITER_ELEM (e, &iter, v, BM_EDGES_OF_VERT) {
		BMVert *v_other = BM_edge_other_vert(e, v);
		const int pi = ep_ptr_index(vmap, v_other);
		if (pi > 0) {
			return prev->vid[pi - 1];
		}
	}

	return 0;
}

static bool ep_face_same(BMFace *f, const int voff, const EditPolySnapshot *prev, const int idx)
{
	const int start = prev->foffs[idx];
	const int count = prev->foffs[idx + 1] - start;
	BMIter liter;
	BMLoop *l;
	int i = 0;

	if (f->len != count) {
		return false;
	}

	BM_ITER_ELEM (l, &liter, f, BM_LOOPS_OF_FACE) {
		if (BM_ELEM_CD_GET_INT(l->v, voff) != prev->floops[start + i]) {
			return false;
		}
		i++;
	}

	return true;
}

/** Id a vertex stands for when looking for the face its attributes come from. */
static int ep_vert_key(BMVert *v, const int voff, GHash *src_map)
{
	const int id = BM_ELEM_CD_GET_INT(v, voff);
	void *src = BLI_ghash_lookup(src_map, POINTER_FROM_INT(id));

	return src ? (int)POINTER_AS_INT(src) : id;
}

/**
 * Source polygon for the attributes of a new polygon: the polygon it shares the
 * most vertices with (for an extruded side face that is the face it grew from).
 */
static int ep_face_best_src(BMFace *f, const int voff, GHash *fmap, GHash *src_map,
                            const EditPolySnapshot *prev)
{
	BMIter liter, liter2;
	BMLoop *l, *l2;
	int best_id = 0, best_score = 0;

	BM_ITER_ELEM (l, &liter, f, BM_LOOPS_OF_FACE) {
		BMLoop *l_rad = l->radial_next;

		while (l_rad != l) {
			const int pi = ep_ptr_index(fmap, l_rad->f);

			if (pi > 0) {
				const int idx = pi - 1;
				const int start = prev->foffs[idx];
				const int count = prev->foffs[idx + 1] - start;
				int score = 0;

				BM_ITER_ELEM (l2, &liter2, f, BM_LOOPS_OF_FACE) {
					const int key = ep_vert_key(l2->v, voff, src_map);
					int k;

					for (k = 0; k < count; k++) {
						if (prev->floops[start + k] == key) {
							score++;
							break;
						}
					}
				}

				if (score > best_score) {
					best_score = score;
					best_id = prev->fid[idx];
				}
			}

			l_rad = l_rad->radial_next;
		}
	}

	return best_id;
}

static void ep_face_push(EditPolyOpBuild *b, BMFace *f, const int voff,
                         const int id, const int src)
{
	BMIter liter;
	BMLoop *l;
	short mat = (short)f->mat_nr;
	int off = b->nl_verts.count;

	epvec_push(&b->nf_ids, &id);
	epvec_push(&b->nf_srcs, &src);
	epvec_push(&b->nf_mats, &mat);
	epvec_push(&b->nf_offs, &off);

	BM_ITER_ELEM (l, &liter, f, BM_LOOPS_OF_FACE) {
		const int vid = BM_ELEM_CD_GET_INT(l->v, voff);
		epvec_push(&b->nl_verts, &vid);
	}
}

static int ep_op_type_from_name(const char *name, const bool topology)
{
	if (name) {
		if (strstr(name, "Extrude")) return EDITPOLY_OP_EXTRUDE;
		if (strstr(name, "Bevel")) return EDITPOLY_OP_BEVEL;
		if (strstr(name, "Delete") || strstr(name, "Dissolve")) return EDITPOLY_OP_DELETE;
		if (strstr(name, "Rotate")) return EDITPOLY_OP_ROTATE;
		if (strstr(name, "Scale") || strstr(name, "Resize")) return EDITPOLY_OP_SCALE;
		if (strstr(name, "Move") || strstr(name, "Transform") || strstr(name, "Translate")) {
			return EDITPOLY_OP_MOVE;
		}
	}

	return topology ? EDITPOLY_OP_PATCH : EDITPOLY_OP_MOVE;
}

/**
 * Compare the geometry of the bmesh with the previous snapshot and build the
 * operation describing the difference. Returns NULL when nothing changed.
 *
 * New elements get their id here, so the caller has to take a new snapshot
 * afterwards.
 */
static EditPolyOp *editpoly_diff(EditPolyRuntime *rt, EditPolySnapshot *prev, const char *opname)
{
	BMesh *bm = rt->bm;
	const int voff = rt->cd_id_vert;
	const int eoff = rt->cd_id_edge;
	const int foff = rt->cd_id_face;
	EditPolyOpBuild b;
	EditPolyOp *op = NULL;
	GHash *vmap, *emap, *fmap;
	GHash *vids, *eids, *fids;
	GHash *src_map;
	bool *vseen, *eseen, *fseen;
	BMIter iter;
	BMVert *v;
	BMEdge *e;
	BMFace *f;
	int i, off;

	epbuild_init(&b);

	vmap = ep_ptr_map(prev->vptr, prev->totvert);
	emap = ep_ptr_map(prev->eptr, prev->totedge);
	fmap = ep_ptr_map(prev->fptr, prev->totface);
	vids = ep_id_set(prev->vid, prev->totvert);
	eids = ep_id_set(prev->eid, prev->totedge);
	fids = ep_id_set(prev->fid, prev->totface);
	src_map = BLI_ghash_ptr_new("EditPoly vert sources");
	vseen = prev->totvert ? MEM_callocN(sizeof(bool) * prev->totvert, "EditPoly seen verts") : NULL;
	eseen = prev->totedge ? MEM_callocN(sizeof(bool) * prev->totedge, "EditPoly seen edges") : NULL;
	fseen = prev->totface ? MEM_callocN(sizeof(bool) * prev->totface, "EditPoly seen faces") : NULL;

	/* ---- vertices ---- */
	BM_ITER_MESH (v, &iter, bm, BM_VERTS_OF_MESH) {
		const int pi = ep_ptr_index(vmap, v);

		if (pi > 0) {
			const int idx = pi - 1;
			vseen[idx] = true;

			if (!compare_v3v3(v->co, prev->vco[idx], 1e-9f)) {
				float delta[3];
				sub_v3_v3v3(delta, v->co, prev->vco[idx]);
				epvec_push(&b.move_ids, &prev->vid[idx]);
				epvec_push(&b.move_deltas, delta);
				b.moved = true;
			}
		}
		else {
			const int old_id = BM_ELEM_CD_GET_INT(v, voff);
			const int id = rt->next_id++;
			float co[3];
			int src = ep_id_in_set(vids, old_id) ? old_id : 0;

			if (src == 0) {
				src = ep_neighbor_vert_id(v, voff, vmap, prev);
			}

			copy_v3_v3(co, v->co);
			BM_ELEM_CD_SET_INT(v, voff, id);

			epvec_push(&b.nv_ids, &id);
			epvec_push(&b.nv_co, co);
			epvec_push(&b.nv_srcs, &src);

			if (src != 0) {
				BLI_ghash_insert(src_map, POINTER_FROM_INT(id), POINTER_FROM_INT(src));
			}

			b.topology = true;
		}
	}

	/* ---- edges ---- */
	BM_ITER_MESH (e, &iter, bm, BM_EDGES_OF_MESH) {
		const int pi = ep_ptr_index(emap, e);
		const int a = BM_ELEM_CD_GET_INT(e->v1, voff);
		const int c = BM_ELEM_CD_GET_INT(e->v2, voff);

		if (pi > 0) {
			const int idx = pi - 1;
			eseen[idx] = true;

			if (!ep_same_pair(a, c, prev->everts[idx][0], prev->everts[idx][1])) {
				const int id = rt->next_id++;
				int verts[2];

				verts[0] = a;
				verts[1] = c;

				epvec_push(&b.kill_e, &prev->eid[idx]);
				epvec_push(&b.ne_ids, &id);
				epvec_push(&b.ne_verts, verts);
				epvec_push(&b.ne_srcs, &prev->eid[idx]);

				BM_ELEM_CD_SET_INT(e, eoff, id);
				b.topology = true;
			}
		}
		else {
			const int old_id = BM_ELEM_CD_GET_INT(e, eoff);
			const int id = rt->next_id++;
			int verts[2];
			int src = ep_id_in_set(eids, old_id) ? old_id : 0;

			verts[0] = a;
			verts[1] = c;

			epvec_push(&b.ne_ids, &id);
			epvec_push(&b.ne_verts, verts);
			epvec_push(&b.ne_srcs, &src);

			BM_ELEM_CD_SET_INT(e, eoff, id);
			b.topology = true;
		}
	}

	/* ---- faces ---- */
	BM_ITER_MESH (f, &iter, bm, BM_FACES_OF_MESH) {
		const int pi = ep_ptr_index(fmap, f);

		if (pi > 0) {
			const int idx = pi - 1;
			fseen[idx] = true;

			if (ep_face_same(f, voff, prev, idx)) {
				if (f->mat_nr != prev->fmat[idx]) {
					epvec_push(&b.mat_ids, &prev->fid[idx]);
					epvec_push(&b.mat_vals, &f->mat_nr);
				}
			}
			else {
				/* the polygon kept its id slot but changed its vertices */
				const int id = rt->next_id++;

				epvec_push(&b.kill_f, &prev->fid[idx]);
				ep_face_push(&b, f, voff, id, prev->fid[idx]);
				BM_ELEM_CD_SET_INT(f, foff, id);
				b.topology = true;
			}
		}
		else {
			const int old_id = BM_ELEM_CD_GET_INT(f, foff);
			const int id = rt->next_id++;
			int src = ep_id_in_set(fids, old_id) ? old_id : 0;

			if (src == 0) {
				src = ep_face_best_src(f, voff, fmap, src_map, prev);
			}

			ep_face_push(&b, f, voff, id, src);
			BM_ELEM_CD_SET_INT(f, foff, id);
			b.topology = true;
		}
	}

	/* ---- removed elements ---- */
	for (i = 0; i < prev->totvert; i++) {
		if (!vseen[i]) {
			epvec_push(&b.kill_v, &prev->vid[i]);
		}
	}
	for (i = 0; i < prev->totedge; i++) {
		if (!eseen[i]) {
			epvec_push(&b.kill_e, &prev->eid[i]);
		}
	}
	for (i = 0; i < prev->totface; i++) {
		if (!fseen[i]) {
			epvec_push(&b.kill_f, &prev->fid[i]);
		}
	}

	if (b.kill_v.count || b.kill_e.count || b.kill_f.count) {
		b.topology = true;
	}

	if (!b.topology && !b.moved && b.mat_ids.count == 0) {
		goto cleanup;
	}

	/* ---- assembly ---- */
	op = MEM_callocN(sizeof(EditPolyOp), "EditPolyOp");
	op->flag = b.topology ? EDITPOLY_OP_FLAG_TOPOLOGY : 0;
	op->type = ep_op_type_from_name(opname, b.topology);
	BLI_strncpy(op->name, opname ? opname : "Edit", sizeof(op->name));

	op->vert_ids = epvec_detach(&b.move_ids, &op->vert_count);
	op->vert_deltas = epvec_detach(&b.move_deltas, NULL);

	/* a plain translation is stored in offset instead of the delta array */
	if (op->vert_deltas && op->vert_count > 0 &&
	    !ELEM(op->type, EDITPOLY_OP_ROTATE, EDITPOLY_OP_SCALE))
	{
		bool uniform = true;
		for (i = 1; i < op->vert_count; i++) {
			if (!compare_v3v3(op->vert_deltas[0], op->vert_deltas[i], 1e-9f)) {
				uniform = false;
				break;
			}
		}
		if (uniform) {
			copy_v3_v3(op->offset, op->vert_deltas[0]);
			MEM_freeN(op->vert_deltas);
			op->vert_deltas = NULL;
		}
	}

	op->kill_vert_ids = epvec_detach(&b.kill_v, &op->kill_vert_count);
	op->kill_edge_ids = epvec_detach(&b.kill_e, &op->kill_edge_count);
	op->kill_face_ids = epvec_detach(&b.kill_f, &op->kill_face_count);

	op->new_vert_ids = epvec_detach(&b.nv_ids, &op->new_vert_count);
	op->new_vert_co = epvec_detach(&b.nv_co, NULL);
	op->new_vert_srcs = epvec_detach(&b.nv_srcs, NULL);

	op->new_edge_ids = epvec_detach(&b.ne_ids, &op->new_edge_count);
	op->new_edge_verts = epvec_detach(&b.ne_verts, NULL);
	op->new_edge_srcs = epvec_detach(&b.ne_srcs, NULL);

	op->new_face_ids = epvec_detach(&b.nf_ids, &op->new_face_count);
	off = b.nl_verts.count;
	epvec_push(&b.nf_offs, &off);
	op->new_face_offsets = epvec_detach(&b.nf_offs, NULL);
	op->new_loop_verts = epvec_detach(&b.nl_verts, &op->new_loop_count);
	op->new_face_srcs = epvec_detach(&b.nf_srcs, NULL);
	op->new_face_mats = epvec_detach(&b.nf_mats, NULL);

	op->mat_face_ids = epvec_detach(&b.mat_ids, &op->mat_face_count);
	op->mat_values = epvec_detach(&b.mat_vals, NULL);

cleanup:
	epbuild_free(&b);
	BLI_ghash_free(vmap, NULL, NULL);
	BLI_ghash_free(emap, NULL, NULL);
	BLI_ghash_free(fmap, NULL, NULL);
	BLI_ghash_free(vids, NULL, NULL);
	BLI_ghash_free(eids, NULL, NULL);
	BLI_ghash_free(fids, NULL, NULL);
	BLI_ghash_free(src_map, NULL, NULL);
	MEM_SAFE_FREE(vseen);
	MEM_SAFE_FREE(eseen);
	MEM_SAFE_FREE(fseen);

	return op;
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Public API
 * \{ */

/** The bmesh being edited, or NULL when it is not available (any more). */
static BMesh *editpoly_record_bmesh(Object *ob, EditPolyRuntime *rt)
{
	BMEditMesh *em;

	if (ob == NULL || ob->type != OB_MESH) {
		return NULL;
	}

	em = BKE_editmesh_from_object(ob);
	if (em == NULL || em->bm == NULL) {
		return NULL;
	}

	rt->bm = em->bm;

	if (!editpoly_ids_offsets(rt->bm, &rt->cd_id_vert, &rt->cd_id_edge, &rt->cd_id_face)) {
		/* without ids the elements cannot be named, recording is not possible */
		return NULL;
	}

	return rt->bm;
}

void editpoly_record_begin(Object *ob)
{
	EditPolyRuntime *rt = editpoly_runtime_get(ob);
	BMesh *bm;

	if (rt == NULL) {
		return;
	}

	bm = editpoly_record_bmesh(ob, rt);
	if (bm == NULL) {
		return;
	}

	rt->next_id = editpoly_ids_max(bm) + 1;

	editpoly_snapshot_free(rt->snapshot);
	rt->snapshot = editpoly_snapshot_new(bm, rt->cd_id_vert, rt->cd_id_edge, rt->cd_id_face);
	/* the snapshot outlives the object data it was made of: the runtime frees it
	 * through this callback (see editpoly_runtime_object_free) */
	rt->snapshot_free = editpoly_snapshot_free_cb;
}

void editpoly_record_sync(Object *ob, const char *opname)
{
	EditPolyRuntime *rt = editpoly_runtime_get(ob);
	EditPolySnapshot *prev;
	EditPolyOp *op;
	BMesh *bm;

	if (rt == NULL) {
		return;
	}

	/* The modifier data may have been re-created in the meantime (an undo step
	 * restores whole IDs), so never trust the pointer stored in the runtime.
	 * Only the active Edit Poly modifier is edited, the others must not get the
	 * operations of this session. */
	rt->epmd = editpoly_modifier_find_active(ob);

	if (rt->epmd == NULL) {
		return;
	}

	bm = editpoly_record_bmesh(ob, rt);
	if (bm == NULL || rt->snapshot == NULL) {
		return;
	}

	prev = rt->snapshot;
	op = editpoly_diff(rt, prev, opname);

	if (op != NULL) {
		/* a new operation makes the undone ones unreachable */
		editpoly_ops_drop_skipped(rt->epmd);

		BLI_addtail(&rt->epmd->ops, op);
		rt->epmd->flag |= EDITPOLY_HAS_OPS;

		/* the ids of the new elements are known now */
		editpoly_snapshot_free(prev);
		rt->snapshot = editpoly_snapshot_new(bm, rt->cd_id_vert, rt->cd_id_edge, rt->cd_id_face);
	}
}

void editpoly_record_end(Object *ob)
{
	EditPolyRuntime *rt = editpoly_runtime_get(ob);

	if (rt == NULL) {
		return;
	}

	editpoly_snapshot_free(rt->snapshot);
	rt->snapshot = NULL;
}

/** Drop the snapshot of the active session. Used on a forced exit (quitting
 *  Blender while the mode is active), where the regular cleanup never runs. */
void editpoly_record_free_all(void)
{
	EditPolyRuntime *rt = editpoly_runtime_first();

	if (rt != NULL) {
		editpoly_snapshot_free(rt->snapshot);
		rt->snapshot = NULL;
	}
}

bool editpoly_record_undo_hook(const char *opname)
{
	Object *ob = editpoly_session_object();
	EditPolyRuntime *rt;

	if (ob == NULL) {
		return false;
	}

	rt = editpoly_runtime_get(ob);

	if (rt == NULL) {
		return false;
	}

	/* While the mode is active the object data is the temporary edit mesh, which
	 * is not part of the main database: a native undo step (this fork pushes
	 * memfile steps) could not link it back and would leave the object without
	 * data. Edit Poly therefore keeps its own history and swallows the step. */
	editpoly_record_sync(ob, opname);

	/* the temporary edit mesh is not in the main database, so the mesh tag done
	 * by the edit operators does not reach the depsgraph: tag the object itself
	 * so the modifier stack is recomputed right after the operation. */
	DAG_id_tag_update(&ob->id, OB_RECALC_DATA);

	return true;
}

/** \} */
