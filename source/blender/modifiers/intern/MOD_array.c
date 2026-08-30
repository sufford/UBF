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
 * along with this program; if not, write to the Free Software  Foundation,
 * Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 *
 * The Original Code is Copyright (C) 2005 by the Blender Foundation.
 * All rights reserved.
 */

/** \file blender/modifiers/intern/MOD_array.c
 *  \ingroup modifiers
 *
 * Array modifier: duplicates the object multiple times along an axis.
 */

#include "MEM_guardedalloc.h"

#include "BLI_math.h"
#include "BLI_utildefines.h"

#include "DNA_curve_types.h"
#include "DNA_meshdata_types.h"
#include "DNA_object_types.h"
#include "DNA_scene_types.h"

#include "BKE_cdderivedmesh.h"
#include "BKE_displist.h"
#include "BKE_curve.h"
#include "BKE_library_query.h"
#include "BKE_modifier.h"
#include "BKE_mesh.h"
#include "BKE_object_deform.h"
 /* for fucks sake i bet they wont work*/
#include "BKE_anim.h"          /* FIXES: where_on_path */
#include "BKE_derivedmesh.h"    /* FIXES: bnd_ensure_derived_mesh framework */
#include "BLI_math.h"           /* FIXES: vec_to_mat4 */

#include "BLI_rand.h"


#include "MOD_util.h"

#include "depsgraph_private.h"

/* Due to cyclic dependencies it's possible that curve used for
 * deformation here is not evaluated at the time of evaluating
 * this modifier.
 */
#define CYCLIC_DEPENDENCY_WORKAROUND

static void initData(ModifierData *md)
{
	ArrayModifierData *amd = (ArrayModifierData *)md;

	/* default to 2 duplicates distributed along the x-axis by an
	 * offset of 1 object-width
	 */
	amd->start_cap = amd->end_cap = amd->curve_ob = amd->offset_ob = NULL;
	amd->count = 2;
	zero_v3(amd->offset);
	amd->scale[0] = 1;
	amd->scale[1] = amd->scale[2] = 0;
	amd->length = 0;
	amd->merge_dist = 0.01;
	amd->fit_type = MOD_ARR_FIXEDCOUNT;
	amd->offset_type = MOD_ARR_OFF_RELATIVE;
	amd->flags = 0;
	zero_v2(amd->uv_offset);

	/* Shape defaults - THESE WERE MISSING! */
	amd->shape_type = MOD_ARR_SHAPE_LINE;
	amd->circle_axis = 2;
	amd->circle_radius = 1.0f;
	amd->circle_angle = 360.0f;
	zero_v3(amd->transform_rot);
	amd->transform_scale[0] = 1.0f;
	amd->transform_scale[1] = 1.0f;
	amd->transform_scale[2] = 1.0f;
	amd->transform_ref = MOD_ARR_TRANSFORM_LINEAR;

	zero_v3(amd->random_offset);
	zero_v3(amd->random_rot);
	zero_v3(amd->random_scale);
	amd->random_scale_uniform = 0.0f;
	amd->random_flip = 0;
	amd->random_exclude_first = 0;
	amd->random_exclude_last = 0;
	amd->random_seed = 0;
}

static void foreachObjectLink(
        ModifierData *md, Object *ob,
        ObjectWalkFunc walk, void *userData)
{
	ArrayModifierData *amd = (ArrayModifierData *) md;

	walk(userData, ob, &amd->start_cap, IDWALK_CB_NOP);
	walk(userData, ob, &amd->end_cap, IDWALK_CB_NOP);
	walk(userData, ob, &amd->curve_ob, IDWALK_CB_NOP);
	walk(userData, ob, &amd->offset_ob, IDWALK_CB_NOP);
}

static void updateDepgraph(ModifierData *md, const ModifierUpdateDepsgraphContext *ctx)
{
	ArrayModifierData *amd = (ArrayModifierData *) md;

	if (amd->start_cap) {
		DagNode *curNode = dag_get_node(ctx->forest, amd->start_cap);

		dag_add_relation(ctx->forest, curNode, ctx->obNode,
		                 DAG_RL_DATA_DATA | DAG_RL_OB_DATA, "Array Modifier");
	}
	if (amd->end_cap) {
		DagNode *curNode = dag_get_node(ctx->forest, amd->end_cap);

		dag_add_relation(ctx->forest, curNode, ctx->obNode,
		                 DAG_RL_DATA_DATA | DAG_RL_OB_DATA, "Array Modifier");
	}
	if (amd->curve_ob) {
		DagNode *curNode = dag_get_node(ctx->forest, amd->curve_ob);
		curNode->eval_flags |= DAG_EVAL_NEED_CURVE_PATH;

		dag_add_relation(ctx->forest, curNode, ctx->obNode,
		                 DAG_RL_DATA_DATA | DAG_RL_OB_DATA, "Array Modifier");
	}
	if (amd->offset_ob) {
		DagNode *curNode = dag_get_node(ctx->forest, amd->offset_ob);

		dag_add_relation(ctx->forest, curNode, ctx->obNode,
		                 DAG_RL_DATA_DATA | DAG_RL_OB_DATA, "Array Modifier");
	}
}

static void updateDepsgraph(ModifierData *md, const ModifierUpdateDepsgraphContext *ctx)
{
	ArrayModifierData *amd = (ArrayModifierData *)md;
	if (amd->start_cap != NULL) {
		DEG_add_object_relation(ctx->node, amd->start_cap, DEG_OB_COMP_TRANSFORM, "Array Modifier Start Cap");
		DEG_add_object_relation(ctx->node, amd->start_cap, DEG_OB_COMP_GEOMETRY, "Array Modifier Start Cap");
	}
	if (amd->end_cap != NULL) {
		DEG_add_object_relation(ctx->node, amd->end_cap, DEG_OB_COMP_TRANSFORM, "Array Modifier End Cap");
		DEG_add_object_relation(ctx->node, amd->end_cap, DEG_OB_COMP_GEOMETRY, "Array Modifier End Cap");
	}
	if (amd->curve_ob) {
		struct Depsgraph *depsgraph = DEG_get_graph_from_handle(ctx->node);
		DEG_add_object_relation(ctx->node, amd->curve_ob, DEG_OB_COMP_GEOMETRY, "Array Modifier Curve");
		DEG_add_special_eval_flag(depsgraph, &amd->curve_ob->id, DAG_EVAL_NEED_CURVE_PATH);
	}
	if (amd->offset_ob != NULL) {
		DEG_add_object_relation(ctx->node, amd->offset_ob, DEG_OB_COMP_TRANSFORM, "Array Modifier Offset");
	}
}

BLI_INLINE float sum_v3(const float v[3])
{
	return v[0] + v[1] + v[2];
}

/* Structure used for sorting vertices, when processing doubles */
typedef struct SortVertsElem {
	int vertex_num;     /* The original index of the vertex, prior to sorting */
	float co[3];        /* Its coordinates */
	float sum_co;       /* sum_v3(co), just so we don't do the sum many times.  */
} SortVertsElem;


static int svert_sum_cmp(const void *e1, const void *e2)
{
	const SortVertsElem *sv1 = e1;
	const SortVertsElem *sv2 = e2;

	if      (sv1->sum_co > sv2->sum_co) return  1;
	else if (sv1->sum_co < sv2->sum_co) return -1;
	else                                return  0;
}

static void svert_from_mvert(SortVertsElem *sv, const MVert *mv, const int i_begin, const int i_end)
{
	int i;
	for (i = i_begin; i < i_end; i++, sv++, mv++) {
		sv->vertex_num = i;
		copy_v3_v3(sv->co, mv->co);
		sv->sum_co = sum_v3(mv->co);
	}
}

/**
 * Take as inputs two sets of verts, to be processed for detection of doubles and mapping.
 * Each set of verts is defined by its start within mverts array and its num_verts;
 * It builds a mapping for all vertices within source, to vertices within target, or -1 if no double found
 * The int doubles_map[num_verts_source] array must have been allocated by caller.
 */
static void dm_mvert_map_doubles(
        int *doubles_map,
        const MVert *mverts,
        const int target_start,
        const int target_num_verts,
        const int source_start,
        const int source_num_verts,
        const float dist)
{
	const float dist3 = ((float)M_SQRT3 + 0.00005f) * dist;   /* Just above sqrt(3) */
	int i_source, i_target, i_target_low_bound, target_end, source_end;
	SortVertsElem *sorted_verts_target, *sorted_verts_source;
	SortVertsElem *sve_source, *sve_target, *sve_target_low_bound;
	bool target_scan_completed;

	target_end = target_start + target_num_verts;
	source_end = source_start + source_num_verts;

	/* build array of MVerts to be tested for merging */
	sorted_verts_target = MEM_malloc_arrayN(target_num_verts, sizeof(SortVertsElem), __func__);
	sorted_verts_source = MEM_malloc_arrayN(source_num_verts, sizeof(SortVertsElem), __func__);

	/* Copy target vertices index and cos into SortVertsElem array */
	svert_from_mvert(sorted_verts_target, mverts + target_start, target_start, target_end);

	/* Copy source vertices index and cos into SortVertsElem array */
	svert_from_mvert(sorted_verts_source, mverts + source_start, source_start, source_end);

	/* sort arrays according to sum of vertex coordinates (sumco) */
	qsort(sorted_verts_target, target_num_verts, sizeof(SortVertsElem), svert_sum_cmp);
	qsort(sorted_verts_source, source_num_verts, sizeof(SortVertsElem), svert_sum_cmp);

	sve_target_low_bound = sorted_verts_target;
	i_target_low_bound = 0;
	target_scan_completed = false;

	/* Scan source vertices, in SortVertsElem sorted array, */
	/* all the while maintaining the lower bound of possible doubles in target vertices */
	for (i_source = 0, sve_source = sorted_verts_source;
	     i_source < source_num_verts;
	     i_source++, sve_source++)
	{
		int best_target_vertex = -1;
		float best_dist_sq = dist * dist;
		float sve_source_sumco;

		/* If source has already been assigned to a target (in an earlier call, with other chunks) */
		if (doubles_map[sve_source->vertex_num] != -1) {
			continue;
		}

		/* If target fully scanned already, then all remaining source vertices cannot have a double */
		if (target_scan_completed) {
			doubles_map[sve_source->vertex_num] = -1;
			continue;
		}

		sve_source_sumco = sum_v3(sve_source->co);

		/* Skip all target vertices that are more than dist3 lower in terms of sumco */
		/* and advance the overall lower bound, applicable to all remaining vertices as well. */
		while ((i_target_low_bound < target_num_verts) &&
		       (sve_target_low_bound->sum_co < sve_source_sumco - dist3))
		{
			i_target_low_bound++;
			sve_target_low_bound++;
		}
		/* If end of target list reached, then no more possible doubles */
		if (i_target_low_bound >= target_num_verts) {
			doubles_map[sve_source->vertex_num] = -1;
			target_scan_completed = true;
			continue;
		}
		/* Test target candidates starting at the low bound of possible doubles, ordered in terms of sumco */
		i_target = i_target_low_bound;
		sve_target = sve_target_low_bound;

		/* i_target will scan vertices in the [v_source_sumco - dist3;  v_source_sumco + dist3] range */

		while ((i_target < target_num_verts) &&
		       (sve_target->sum_co <= sve_source_sumco + dist3))
		{
			/* Testing distance for candidate double in target */
			/* v_target is within dist3 of v_source in terms of sumco;  check real distance */
			float dist_sq;
			if ((dist_sq = len_squared_v3v3(sve_source->co, sve_target->co)) <= best_dist_sq) {
				/* Potential double found */
				best_dist_sq = dist_sq;
				best_target_vertex = sve_target->vertex_num;

				/* If target is already mapped, we only follow that mapping if final target remains
				 * close enough from current vert (otherwise no mapping at all).
				 * Note that if we later find another target closer than this one, then we check it. But if other
				 * potential targets are farther, then there will be no mapping at all for this source. */
				while (best_target_vertex != -1 && !ELEM(doubles_map[best_target_vertex], -1, best_target_vertex)) {
					if (compare_len_v3v3(mverts[sve_source->vertex_num].co,
					                     mverts[doubles_map[best_target_vertex]].co,
					                     dist))
					{
						best_target_vertex = doubles_map[best_target_vertex];
					}
					else {
						best_target_vertex = -1;
					}
				}
			}
			i_target++;
			sve_target++;
		}
		/* End of candidate scan: if none found then no doubles */
		doubles_map[sve_source->vertex_num] = best_target_vertex;
	}

	MEM_freeN(sorted_verts_source);
	MEM_freeN(sorted_verts_target);
}


static void dm_merge_transform(
        DerivedMesh *result, DerivedMesh *cap_dm, float cap_offset[4][4],
        unsigned int cap_verts_index, unsigned int cap_edges_index, int cap_loops_index, int cap_polys_index,
        int cap_nverts, int cap_nedges, int cap_nloops, int cap_npolys, int *remap, int remap_len)
{
	int *index_orig;
	int i;
	MVert *mv;
	MEdge *me;
	MLoop *ml;
	MPoly *mp;
	MDeformVert *dvert;

	/* needed for subsurf so arrays are allocated */
	cap_dm->getVertArray(cap_dm);
	cap_dm->getEdgeArray(cap_dm);
	cap_dm->getLoopArray(cap_dm);
	cap_dm->getPolyArray(cap_dm);

	DM_copy_vert_data(cap_dm, result, 0, cap_verts_index, cap_nverts);
	DM_copy_edge_data(cap_dm, result, 0, cap_edges_index, cap_nedges);
	DM_copy_loop_data(cap_dm, result, 0, cap_loops_index, cap_nloops);
	DM_copy_poly_data(cap_dm, result, 0, cap_polys_index, cap_npolys);

	mv = CDDM_get_verts(result) + cap_verts_index;

	for (i = 0; i < cap_nverts; i++, mv++) {
		mul_m4_v3(cap_offset, mv->co);
		/* Reset MVert flags for caps */
		mv->flag = mv->bweight = 0;
	}

	/* remap the vertex groups if necessary */
	dvert = DM_get_vert_data(result, cap_verts_index, CD_MDEFORMVERT);
	if (dvert != NULL) {
		BKE_object_defgroup_index_map_apply(dvert, cap_nverts, remap, remap_len);
	}

	/* adjust cap edge vertex indices */
	me = CDDM_get_edges(result) + cap_edges_index;
	for (i = 0; i < cap_nedges; i++, me++) {
		me->v1 += cap_verts_index;
		me->v2 += cap_verts_index;
	}

	/* adjust cap poly loopstart indices */
	mp = CDDM_get_polys(result) + cap_polys_index;
	for (i = 0; i < cap_npolys; i++, mp++) {
		mp->loopstart += cap_loops_index;
	}

	/* adjust cap loop vertex and edge indices */
	ml = CDDM_get_loops(result) + cap_loops_index;
	for (i = 0; i < cap_nloops; i++, ml++) {
		ml->v += cap_verts_index;
		ml->e += cap_edges_index;
	}

	/* set origindex */
	index_orig = result->getVertDataArray(result, CD_ORIGINDEX);
	if (index_orig) {
		copy_vn_i(index_orig + cap_verts_index, cap_nverts, ORIGINDEX_NONE);
	}

	index_orig = result->getEdgeDataArray(result, CD_ORIGINDEX);
	if (index_orig) {
		copy_vn_i(index_orig + cap_edges_index, cap_nedges, ORIGINDEX_NONE);
	}

	index_orig = result->getPolyDataArray(result, CD_ORIGINDEX);
	if (index_orig) {
		copy_vn_i(index_orig + cap_polys_index, cap_npolys, ORIGINDEX_NONE);
	}

	index_orig = result->getLoopDataArray(result, CD_ORIGINDEX);
	if (index_orig) {
		copy_vn_i(index_orig + cap_loops_index, cap_nloops, ORIGINDEX_NONE);
	}
}

static DerivedMesh *arrayModifier_doArray(
        ArrayModifierData *amd,
        Scene *scene, Object *ob, DerivedMesh *dm,
        ModifierApplyFlag flag)
{
	const float eps = 1e-6f;
	const MVert *src_mvert;
	MVert *mv, *mv_prev, *result_dm_verts;

	MEdge *me;
	MLoop *ml;
	MPoly *mp;
	int i, j, c, count;
	float length = amd->length;
	/* offset matrix */
	float offset[4][4];
	float scale[3];
	bool offset_has_scale;
	float current_offset[4][4];
	float final_offset[4][4];
	int *full_doubles_map = NULL;
	int tot_doubles;

	const bool use_merge = (amd->flags & MOD_ARR_MERGE) != 0;
	const bool use_recalc_normals = (dm->dirty & DM_DIRTY_NORMALS) || use_merge;
	const bool use_offset_ob = ((amd->offset_type & MOD_ARR_OFF_OBJ) && amd->offset_ob);

	int start_cap_nverts = 0, start_cap_nedges = 0, start_cap_npolys = 0, start_cap_nloops = 0;
	int end_cap_nverts = 0, end_cap_nedges = 0, end_cap_npolys = 0, end_cap_nloops = 0;
	int result_nverts = 0, result_nedges = 0, result_npolys = 0, result_nloops = 0;
	int chunk_nverts, chunk_nedges, chunk_nloops, chunk_npolys;
	int first_chunk_start, first_chunk_nverts, last_chunk_start, last_chunk_nverts;

	DerivedMesh *result, *start_cap_dm = NULL, *end_cap_dm = NULL;

	int *vgroup_start_cap_remap = NULL;
	int vgroup_start_cap_remap_len = 0;
	int *vgroup_end_cap_remap = NULL;
	int vgroup_end_cap_remap_len = 0;

	chunk_nverts = dm->getNumVerts(dm);
	chunk_nedges = dm->getNumEdges(dm);
	chunk_nloops = dm->getNumLoops(dm);
	chunk_npolys = dm->getNumPolys(dm);

	count = amd->count;

	if (amd->start_cap && amd->start_cap != ob && amd->start_cap->type == OB_MESH) {
		vgroup_start_cap_remap = BKE_object_defgroup_index_map_create(amd->start_cap, ob, &vgroup_start_cap_remap_len);

		start_cap_dm = get_dm_for_modifier(amd->start_cap, flag);
		if (start_cap_dm) {
			start_cap_nverts = start_cap_dm->getNumVerts(start_cap_dm);
			start_cap_nedges = start_cap_dm->getNumEdges(start_cap_dm);
			start_cap_nloops = start_cap_dm->getNumLoops(start_cap_dm);
			start_cap_npolys = start_cap_dm->getNumPolys(start_cap_dm);
		}
	}
	if (amd->end_cap && amd->end_cap != ob && amd->end_cap->type == OB_MESH) {
		vgroup_end_cap_remap = BKE_object_defgroup_index_map_create(amd->end_cap, ob, &vgroup_end_cap_remap_len);

		end_cap_dm = get_dm_for_modifier(amd->end_cap, flag);
		if (end_cap_dm) {
			end_cap_nverts = end_cap_dm->getNumVerts(end_cap_dm);
			end_cap_nedges = end_cap_dm->getNumEdges(end_cap_dm);
			end_cap_nloops = end_cap_dm->getNumLoops(end_cap_dm);
			end_cap_npolys = end_cap_dm->getNumPolys(end_cap_dm);
		}
	}

	/* Build up offset array, cumulating all settings options */

	unit_m4(offset);
	src_mvert = dm->getVertArray(dm);

	if (amd->offset_type & MOD_ARR_OFF_CONST) {
		add_v3_v3(offset[3], amd->offset);
	}

	if (amd->offset_type & MOD_ARR_OFF_RELATIVE) {
		float min[3], max[3];
		const MVert *src_mv;

		INIT_MINMAX(min, max);
		for (src_mv = src_mvert, j = chunk_nverts; j--; src_mv++) {
			minmax_v3v3_v3(min, max, src_mv->co);
		}

		for (j = 3; j--; ) {
			offset[3][j] += amd->scale[j] * (max[j] - min[j]);
		}
	}

	if (use_offset_ob) {
		float obinv[4][4];
		float result_mat[4][4];

		if (ob)
			invert_m4_m4(obinv, ob->obmat);
		else
			unit_m4(obinv);

		mul_m4_series(result_mat, offset,
		              obinv, amd->offset_ob->obmat);
		copy_m4_m4(offset, result_mat);
	}

	/* Check if there is some scaling.  If scaling, then we will not translate mapping */
	mat4_to_size(scale, offset);
	offset_has_scale = !is_one_v3(scale);

	if (amd->fit_type == MOD_ARR_FITCURVE && amd->curve_ob) {
		Curve *cu = amd->curve_ob->data;
		if (cu) {
#ifdef CYCLIC_DEPENDENCY_WORKAROUND
			if (amd->curve_ob->curve_cache == NULL) {
				BKE_displist_make_curveTypes(scene, amd->curve_ob, false);
			}
#endif

			if (amd->curve_ob->curve_cache && amd->curve_ob->curve_cache->path) {
				float scale_fac = mat4_to_scale(amd->curve_ob->obmat);
				length = scale_fac * amd->curve_ob->curve_cache->path->totdist;
			}
		}
	}

	/* calculate the maximum number of copies which will fit within the
	 * prescribed length */
	if (amd->fit_type == MOD_ARR_FITLENGTH || amd->fit_type == MOD_ARR_FITCURVE) {
		float dist = len_v3(offset[3]);

		if (dist > eps) {
			/* this gives length = first copy start to last copy end
			 * add a tiny offset for floating point rounding errors */
			count = (length + eps) / dist + 1;
		}
		else {
			/* if the offset has no translation, just make one copy */
			count = 1;
		}
	}

	if (count < 1)
		count = 1;

	/* The number of verts, edges, loops, polys, before eventually merging doubles */
	result_nverts = chunk_nverts * count + start_cap_nverts + end_cap_nverts;
	result_nedges = chunk_nedges * count + start_cap_nedges + end_cap_nedges;
	result_nloops = chunk_nloops * count + start_cap_nloops + end_cap_nloops;
	result_npolys = chunk_npolys * count + start_cap_npolys + end_cap_npolys;

	/* Initialize a result dm */
	result = CDDM_from_template(dm, result_nverts, result_nedges, 0, result_nloops, result_npolys);
	result_dm_verts = CDDM_get_verts(result);

	if (use_merge) {
		/* Will need full_doubles_map for handling merge */
		full_doubles_map = MEM_malloc_arrayN(result_nverts, sizeof(int), "mod array doubles map");
		copy_vn_i(full_doubles_map, result_nverts, -1);
	}

	/* copy customdata to original geometry */
	DM_copy_vert_data(dm, result, 0, 0, chunk_nverts);
	DM_copy_edge_data(dm, result, 0, 0, chunk_nedges);
	DM_copy_loop_data(dm, result, 0, 0, chunk_nloops);
	DM_copy_poly_data(dm, result, 0, 0, chunk_npolys);

	/* Subsurf for eg won't have mesh data in the custom data arrays.
	 * now add mvert/medge/mpoly layers. */

	if (!CustomData_has_layer(&dm->vertData, CD_MVERT)) {
		dm->copyVertArray(dm, result_dm_verts);
	}
	if (!CustomData_has_layer(&dm->edgeData, CD_MEDGE)) {
		dm->copyEdgeArray(dm, CDDM_get_edges(result));
	}
	if (!CustomData_has_layer(&dm->polyData, CD_MPOLY)) {
		dm->copyLoopArray(dm, CDDM_get_loops(result));
		dm->copyPolyArray(dm, CDDM_get_polys(result));
	}

	/* Remember first chunk, in case of cap merge */
	first_chunk_start = 0;
	first_chunk_nverts = chunk_nverts;

	unit_m4(current_offset);
	for (c = 1; c < count; c++) {
		/* copy customdata to new geometry */
		DM_copy_vert_data(result, result, 0, c * chunk_nverts, chunk_nverts);
		DM_copy_edge_data(result, result, 0, c * chunk_nedges, chunk_nedges);
		DM_copy_loop_data(result, result, 0, c * chunk_nloops, chunk_nloops);
		DM_copy_poly_data(result, result, 0, c * chunk_npolys, chunk_npolys);

		mv_prev = result_dm_verts;
		mv = mv_prev + c * chunk_nverts;

		if (amd->shape_type == MOD_ARR_SHAPE_CIRCLE) {
			/* Circle distribution */
			float angle_step = 0.0f;
			if (count > 1) {
				if (amd->circle_fill_type == MOD_ARR_CIRCLE_FULL) {
					/* Full circle: 360 degrees */
					angle_step = (2.0f * (float)M_PI) / (float)count;
				}
				else {
					/* Arc mode: use sweep angle */
					angle_step = DEG2RADF(amd->circle_angle) / (float)(count - 1);
				}
			}

			/* Start loop at 0 so the first element is placed properly on the circle radius */
			for (c = 0; c < count; c++) {
				/* FIXED: Swapped 'derivedData' to Blender 2.79's native input name 'dm' */
				DM_copy_vert_data(dm, result, 0, c * chunk_nverts, chunk_nverts);
				DM_copy_edge_data(dm, result, 0, c * chunk_nedges, chunk_nedges);
				DM_copy_loop_data(dm, result, 0, c * chunk_nloops, chunk_nloops);
				DM_copy_poly_data(dm, result, 0, c * chunk_npolys, chunk_npolys);

				float angle = angle_step * (float)c;
				float circle_offset[4][4];
				float rot_mat[4][4];
				float temp_mat[4][4];
				float offset_vec[3] = { 0.0f, 0.0f, 0.0f };

				/* 1. Establish precise outward radial projection based on your UI axis selector */
				if (amd->circle_axis == 2)      offset_vec[0] = amd->circle_radius; /* Z Axis: push out X */
				else if (amd->circle_axis == 0) offset_vec[1] = amd->circle_radius; /* X Axis: push out Y */
				else                            offset_vec[2] = amd->circle_radius; /* Y Axis: push out Z */

				/* 2. FIXED: Define raw C arrays to satisfy 2.79's axis_angle_to_mat4 call constraints */
				float axis_vec[3] = { 0.0f, 0.0f, 0.0f };
				axis_vec[amd->circle_axis] = 1.0f; /* Lock rotation pivot path vector cleanly */

				/* FIXED: Passed 3 explicit variables instead of the modern missing AxisAngle struct wrapper */
				axis_angle_to_mat4(rot_mat, axis_vec, angle);

				/* 3. Build baseline row-major translation matrix */
				unit_m4(circle_offset);
				copy_v3_v3(circle_offset[3], offset_vec);

				/* 4. Combine transforms into a standalone target pointer to prevent overlapping memory state corruptions */
				mul_m4_m4m4(temp_mat, rot_mat, circle_offset);

				/* 5. Transform vertex coordinates safely from baseline geometry chunk data indices */
				mv = CDDM_get_verts(result) + c * chunk_nverts;
				for (i = 0; i < chunk_nverts; i++, mv++) {
					mul_m4_v3(temp_mat, mv->co);

					/* Correct vertex normals along your rotation vector matrices path */
					if (!use_recalc_normals) {
						float no[3];
						normal_short_to_float_v3(no, mv->no);
						mul_mat3_m4_v3(temp_mat, no);
						normalize_v3(no);
						normal_float_to_short_v3(mv->no, no);
					}
				}

				/* 6. Correct mesh topological layout element offset data pointers */
				me = CDDM_get_edges(result) + c * chunk_nedges;
				for (i = 0; i < chunk_nedges; i++, me++) {
					me->v1 += c * chunk_nverts;
					me->v2 += c * chunk_nverts;
				}

				mp = CDDM_get_polys(result) + c * chunk_npolys;
				for (i = 0; i < chunk_npolys; i++, mp++) {
					mp->loopstart += c * chunk_nloops;
				}

				ml = CDDM_get_loops(result) + c * chunk_nloops;
				for (i = 0; i < chunk_nloops; i++, ml++) {
					ml->v += c * chunk_nverts;
					ml->e += c * chunk_nedges;
				}
			}

			/* Skip the rest of the old legacy sequential accumulation loop structures */
			goto after_copies;
		}
		if (amd->shape_type == MOD_ARR_SHAPE_CURVE) {

			if (amd->curve_ob == NULL || amd->curve_ob->data == NULL || amd->curve_ob->type != OB_CURVE) {
				/* Curve was deleted or invalid - skip curve mode entirely */
				goto after_copies;
			}

			/* Ensure we have a valid curve object assigned and it has a processed path cache */
			if (amd->curve_ob && amd->curve_ob->type == OB_CURVE &&
				amd->curve_ob->curve_cache && amd->curve_ob->curve_cache->path)
			{
				Path *path = amd->curve_ob->curve_cache->path;
				PathPoint *path_data = path->data;
				int path_len = path->len;

				if (path_len > 0 && path_data) {
					/* Cache raw pristine source vertices to prevent multi-pass buffer mutation bugs */
					MVert *src_verts = CDDM_get_verts(dm);

					for (c = 0; c < count; c++) {
						/* Duplicate pristine base mesh data structural data into the active output slice */
						DM_copy_vert_data(dm, result, 0, c * chunk_nverts, chunk_nverts);
						DM_copy_edge_data(dm, result, 0, c * chunk_nedges, chunk_nedges);
						DM_copy_loop_data(dm, result, 0, c * chunk_nloops, chunk_nloops);
						DM_copy_poly_data(dm, result, 0, c * chunk_npolys, chunk_npolys);

						/* Calculate normalized position factor smoothly (0.0f to 1.0f) */
						float factor = 0.0f;
						if (count > 1) {
							factor = (float)c / (float)(count - 1);
						}

						/* Compute float path position index with linear interpolation decimal remainders */
						float findex = factor * (float)(path_len - 1);
						int idx = (int)findex;
						float remainder = findex - (float)idx;
						CLAMP(idx, 0, path_len - 1);

						float target_pos[3];
						if (idx < path_len - 1 && remainder > 0.0f) {
							/* Linearly interpolate smoothly between curve points so geometry doesn't jitter */
							interp_v3_v3v3(target_pos, path_data[idx].vec, path_data[idx + 1].vec, remainder);
						}
						else {
							copy_v3_v3(target_pos, path_data[idx].vec);
						}

						/* 1. Calculate space transformations: Curve Object Space -> Mesh Object Space */
						float ob_diff_mat[4][4];
						/* Note: Ensure the outer system wrapper passes 'ob' (the mesh object) to this modifier code */
						mul_m4_m4m4(ob_diff_mat, ob->imat, amd->curve_ob->obmat);

						/* 2. Apply the curve object's scale/rotation/translation adjustments to our target point */
						float world_target_pos[3];
						mul_m4_v3(ob_diff_mat, target_pos); // target_pos is now correctly aligned to the mesh space

						/* 3. Construct your final translation transformation matrix wrapper */
						float c_mat[4][4];
						unit_m4(c_mat);
						copy_v3_v3(c_mat[3], target_pos);

						/* Apply translation matrix using the PRISTINE source coordinates to avoid over-accumulation */
						mv = CDDM_get_verts(result) + c * chunk_nverts;
						for (i = 0; i < chunk_nverts; i++, mv++) {
							/* Read from pristine source vertex buffer array */
							copy_v3_v3(mv->co, src_verts[i].co);
							/* Safely transform position vector */
							mul_m4_v3(c_mat, mv->co);
						}

						/* Correct topological structural tracking offsets */
						me = CDDM_get_edges(result) + c * chunk_nedges;
						for (i = 0; i < chunk_nedges; i++, me++) {
							me->v1 += c * chunk_nverts;
							me->v2 += c * chunk_nverts;
						}

						mp = CDDM_get_polys(result) + c * chunk_npolys;
						for (i = 0; i < chunk_npolys; i++, mp++) {
							mp->loopstart += c * chunk_nloops;
						}

						ml = CDDM_get_loops(result) + c * chunk_nloops;
						for (i = 0; i < chunk_nloops; i++, ml++) {
							ml->v += c * chunk_nverts;
							ml->e += c * chunk_nedges;
						}
					}
					/* Safely jump to bypass the standard linear accumulation sequence loops below */
					goto after_copies;
				}
			}
			/* Fallthrough safety protection: If validation checks fail, do not hit goto! */
			/* Letting it fall through means the modifier gracefully drops back to standard offsets instead of crashing. */
		}

		if (amd->shape_type == MOD_ARR_SHAPE_TRANSFORM) {
			MVert *src_verts = CDDM_get_verts(dm);

			/* 1. PRE-CALCULATE THE SINGLE-STEP MATRIX */
			float step_mat[4][4];
			unit_m4(step_mat);

			bool use_object_offset = (amd->transform_ref == MOD_ARR_TRANSFORM_OBJECT && amd->offset_ob);

			if (use_object_offset) {
				/* OBJECT REFERENCE MODE: Calculate single-step delta matrix */
				float ob_inv[4][4];
				if (ob) {
					invert_m4_m4(ob_inv, ob->obmat);
				}
				else {
					unit_m4(ob_inv);
				}
				/* step_mat represents the transformation delta from 'ob' to 'offset_ob' */
				mul_m4_m4m4(step_mat, ob_inv, amd->offset_ob->obmat);
			}

			/* 2. SEQUENTIAL DUPLICATION LOOP */
			for (c = 0; c < count; c++) {
				/* Copy pristine base mesh data */
				DM_copy_vert_data(dm, result, 0, c * chunk_nverts, chunk_nverts);
				DM_copy_edge_data(dm, result, 0, c * chunk_nedges, chunk_nedges);
				DM_copy_loop_data(dm, result, 0, c * chunk_nloops, chunk_nloops);
				DM_copy_poly_data(dm, result, 0, c * chunk_npolys, chunk_npolys);

				float transform_mat[4][4];
				unit_m4(transform_mat);

				if (use_object_offset) {
					/* OBJECT MODE: Accumulate the step_mat by raising it to the power of 'c' */
					for (int power = 0; power < c; power++) {
						float temp_accum[4][4];
						copy_m4_m4(temp_accum, transform_mat);
						mul_m4_m4m4(transform_mat, temp_accum, step_mat);
					}
				}
				else {
					/* LINEAR MODE: Keep your custom sequential translation/rotation/scale math */
					float loc_mat[4][4], rot_mat[4][4], scale_mat[4][4], temp_mat[4][4];
					unit_m4(loc_mat);
					unit_m4(rot_mat);
					unit_m4(scale_mat);

					loc_mat[3][0] = amd->offset[0] * (float)c;
					loc_mat[3][1] = amd->offset[1] * (float)c;
					loc_mat[3][2] = amd->offset[2] * (float)c;

					float rot_angle[3];
					rot_angle[0] = DEG2RADF(amd->transform_rot[0] * (float)c);
					rot_angle[1] = DEG2RADF(amd->transform_rot[1] * (float)c);
					rot_angle[2] = DEG2RADF(amd->transform_rot[2] * (float)c);
					eul_to_mat4(rot_mat, rot_angle);

					scale_mat[0][0] = powf(amd->transform_scale[0], (float)c);
					scale_mat[1][1] = powf(amd->transform_scale[1], (float)c);
					scale_mat[2][2] = powf(amd->transform_scale[2], (float)c);

					/* Custom Linear Order: Translation * Rotation * Scale */
					mul_m4_m4m4(temp_mat, loc_mat, rot_mat);
					mul_m4_m4m4(transform_mat, temp_mat, scale_mat);
				}

				/* Apply transformation to vertices */
				mv = CDDM_get_verts(result) + c * chunk_nverts;
				for (i = 0; i < chunk_nverts; i++, mv++) {
					copy_v3_v3(mv->co, src_verts[i].co);
					mul_m4_v3(transform_mat, mv->co);

					/* Correct normals using simple matrix transform */
					if (!use_recalc_normals) {
						float no[3];
						normal_short_to_float_v3(no, src_verts[i].no);
						mul_mat3_m4_v3(transform_mat, no);
						normalize_v3(no);
						normal_float_to_short_v3(mv->no, no);
					}
				}

				/* Adjust topology offsets */
				me = CDDM_get_edges(result) + c * chunk_nedges;
				for (i = 0; i < chunk_nedges; i++, me++) {
					me->v1 += c * chunk_nverts;
					me->v2 += c * chunk_nverts;
				}

				mp = CDDM_get_polys(result) + c * chunk_npolys;
				for (i = 0; i < chunk_npolys; i++, mp++) {
					mp->loopstart += c * chunk_nloops;
				}

				ml = CDDM_get_loops(result) + c * chunk_nloops;
				for (i = 0; i < chunk_nloops; i++, ml++) {
					ml->v += c * chunk_nverts;
					ml->e += c * chunk_nedges;
				}
			}

			goto after_copies;
		}






		else {
			/* DEFAULT LEGACY LINEAR ACCUMULATION MODE */
			mul_m4_m4m4(current_offset, current_offset, offset);
			
			/* Build full transformation matrix with rotation and scale progression */
			float loc_mat[4][4], rot_mat[4][4], scale_mat[4][4], temp_mat[4][4], line_mat[4][4];
			unit_m4(loc_mat);
			unit_m4(rot_mat);
			unit_m4(scale_mat);
			
			/* Location (from current_offset accumulation) */
			copy_v3_v3(loc_mat[3], current_offset[3]);
			
			/* Rotation progression */
			float rot_angle[3];
			rot_angle[0] = DEG2RADF(amd->transform_rot[0] * (float)c);
			rot_angle[1] = DEG2RADF(amd->transform_rot[1] * (float)c);
			rot_angle[2] = DEG2RADF(amd->transform_rot[2] * (float)c);
			eul_to_mat4(rot_mat, rot_angle);
			
			/* Scale progression */
			scale_mat[0][0] = powf(amd->transform_scale[0], (float)c);
			scale_mat[1][1] = powf(amd->transform_scale[1], (float)c);
			scale_mat[2][2] = powf(amd->transform_scale[2], (float)c);
			
			/* Combine: Location * Rotation * Scale */
			mul_m4_m4m4(temp_mat, loc_mat, rot_mat);
			mul_m4_m4m4(line_mat, temp_mat, scale_mat);
			
			/* Apply to vertices */
			mv = result_dm_verts + c * chunk_nverts;
			for (i = 0; i < chunk_nverts; i++, mv++) {
				MVert *base_mv = result_dm_verts + i;
				copy_v3_v3(mv->co, base_mv->co);
				mul_m4_v3(line_mat, mv->co);
				
				if (!use_recalc_normals) {
					float no[3];
					normal_short_to_float_v3(no, base_mv->no);
					mul_mat3_m4_v3(line_mat, no);
					normalize_v3(no);
					normal_float_to_short_v3(mv->no, no);
				}
			}
			
			/* Adjust topology indices */
			me = CDDM_get_edges(result) + c * chunk_nedges;
			for (i = 0; i < chunk_nedges; i++, me++) {
				me->v1 += c * chunk_nverts;
				me->v2 += c * chunk_nverts;
			}
			
			mp = CDDM_get_polys(result) + c * chunk_npolys;
			for (i = 0; i < chunk_npolys; i++, mp++) {
				mp->loopstart += c * chunk_nloops;
			}
			
			ml = CDDM_get_loops(result) + c * chunk_nloops;
			for (i = 0; i < chunk_nloops; i++, ml++) {
				ml->v += c * chunk_nverts;
				ml->e += c * chunk_nedges;
			}
			
			/* Merge handling */
			if (use_merge && (c >= 1)) {
				if (!offset_has_scale && (c >= 2) && (amd->shape_type != 1)) {
					int k;
					int this_chunk_index = c * chunk_nverts;
					int prev_chunk_index = (c - 1) * chunk_nverts;
					for (k = 0; k < chunk_nverts; k++, this_chunk_index++, prev_chunk_index++) {
						int target = full_doubles_map[prev_chunk_index];
						if (target != -1) {
							target += chunk_nverts;
							while (target != -1 && !ELEM(full_doubles_map[target], -1, target)) {
								if (compare_len_v3v3(result_dm_verts[this_chunk_index].co,
								                     result_dm_verts[full_doubles_map[target]].co,
								                     amd->merge_dist))
								{
									target = full_doubles_map[target];
								}
								else {
									target = -1;
								}
							}
						}
						full_doubles_map[this_chunk_index] = target;
					}
				}
				else {
					dm_mvert_map_doubles(
					        full_doubles_map, result_dm_verts,
					        (c - 1) * chunk_nverts, chunk_nverts,
					        c * chunk_nverts, chunk_nverts,
					        amd->merge_dist);
				}
			}
		}
	}
	after_copies:

	/* === FIXED UNIVERSAL RANDOMIZER LAYER === */
	/* Initialize Blender's native random engine to get zero math correlation */
	RNG *rng_sys = BLI_rng_new(amd->random_seed);

	for (c = 0; c < count; c++) {
		bool apply_random = !((amd->random_exclude_first && c == 0) ||
			(amd->random_exclude_last && c == count - 1));

		if (apply_random) {
			float rnd_offset[3], rnd_rot[3], rnd_scale[3];

			/* BLI_rng_get_float gives a perfect, uniform 0.0 to 1.0 distribution */
			rnd_offset[0] = (BLI_rng_get_float(rng_sys) - 0.5f) * 2.0f * amd->random_offset[0];
			rnd_offset[1] = (BLI_rng_get_float(rng_sys) - 0.5f) * 2.0f * amd->random_offset[1];
			rnd_offset[2] = (BLI_rng_get_float(rng_sys) - 0.5f) * 2.0f * amd->random_offset[2];

			rnd_rot[0] = DEG2RADF((BLI_rng_get_float(rng_sys) - 0.5f) * 2.0f * amd->random_rot[0]);
			rnd_rot[1] = DEG2RADF((BLI_rng_get_float(rng_sys) - 0.5f) * 2.0f * amd->random_rot[1]);
			rnd_rot[2] = DEG2RADF((BLI_rng_get_float(rng_sys) - 0.5f) * 2.0f * amd->random_rot[2]);

			rnd_scale[0] = 1.0f + (BLI_rng_get_float(rng_sys) - 0.5f) * 2.0f * amd->random_scale[0];
			rnd_scale[1] = 1.0f + (BLI_rng_get_float(rng_sys) - 0.5f) * 2.0f * amd->random_scale[1];
			rnd_scale[2] = 1.0f + (BLI_rng_get_float(rng_sys) - 0.5f) * 2.0f * amd->random_scale[2];

			/* Build a local matrix for the random alterations */
			float r_loc[4][4], r_rot[4][4], r_scale[4][4], r_tmp[4][4], r_matrix[4][4];
			unit_m4(r_loc); unit_m4(r_rot); unit_m4(r_scale);

			r_loc[3][0] = rnd_offset[0];
			r_loc[3][1] = rnd_offset[1];
			r_loc[3][2] = rnd_offset[2];

			eul_to_mat4(r_rot, rnd_rot);

			r_scale[0][0] = rnd_scale[0];
			r_scale[1][1] = rnd_scale[1];
			r_scale[2][2] = rnd_scale[2];

			/* Combine local random changes: Loc * Rot * Scale */
			mul_m4_m4m4(r_tmp, r_loc, r_rot);
			mul_m4_m4m4(r_matrix, r_tmp, r_scale);

			/* Calculate the approximate pivot center of this specific chunk copy */
			float chunk_center[3] = { 0.0f, 0.0f, 0.0f };
			mv = CDDM_get_verts(result) + c * chunk_nverts;
			for (i = 0; i < chunk_nverts; i++) {
				add_v3_v3(chunk_center, (mv + i)->co);
			}
			if (chunk_nverts > 0) {
				mul_v3_fl(chunk_center, 1.0f / (float)chunk_nverts);
			}

			/* Apply transformation relative to the chunk center */
			mv = CDDM_get_verts(result) + c * chunk_nverts;
			for (i = 0; i < chunk_nverts; i++, mv++) {
				/* Move vertex to local center space */
				sub_v3_v3(mv->co, chunk_center);

				/* Run the random matrix math */
				mul_m4_v3(r_matrix, mv->co);

				/* Restore back to world space position */
				add_v3_v3(mv->co, chunk_center);

				/* Safely transform normals */
				if (!use_recalc_normals) {
					float no[3];
					normal_short_to_float_v3(no, mv->no);
					mul_mat3_m4_v3(r_matrix, no);
					normalize_v3(no);
					normal_float_to_short_v3(mv->no, no);
				}
			}
		}
		else {
			/* Advance RNG state for excluded copies to keep seeds consistent if count changes */
			BLI_rng_get_float(rng_sys); BLI_rng_get_float(rng_sys); BLI_rng_get_float(rng_sys);
			BLI_rng_get_float(rng_sys); BLI_rng_get_float(rng_sys); BLI_rng_get_float(rng_sys);
			BLI_rng_get_float(rng_sys); BLI_rng_get_float(rng_sys); BLI_rng_get_float(rng_sys);
		}
	}

	BLI_rng_free(rng_sys);
	/* === MERGE HANDLING (works for ALL modes) === */
	if (use_merge) {
		/* For non-linear modes, we need to build the doubles map now */
		if (amd->shape_type != MOD_ARR_SHAPE_LINE) {
			/* Build doubles map between adjacent copies */
			for (c = 1; c < count; c++) {
				dm_mvert_map_doubles(
					full_doubles_map,
					result_dm_verts,
					(c - 1) * chunk_nverts, chunk_nverts,
					c * chunk_nverts, chunk_nverts,
					amd->merge_dist);
			}
		}

		/* Merge first and last copies if enabled */
		if (amd->flags & MOD_ARR_MERGEFINAL && count > 1) {
			dm_mvert_map_doubles(
				full_doubles_map,
				result_dm_verts,
				(count - 1) * chunk_nverts, chunk_nverts,
				0, chunk_nverts,
				amd->merge_dist);
		}

		/* Perform the actual merge */
		tot_doubles = 0;
		for (i = 0; i < result_nverts; i++) {
			int new_i = full_doubles_map[i];
			if (new_i != -1) {
				while (!ELEM(full_doubles_map[new_i], -1, new_i)) {
					new_i = full_doubles_map[new_i];
				}
				if (i == new_i) {
					full_doubles_map[i] = -1;
				}
				else {
					full_doubles_map[i] = new_i;
					tot_doubles++;
				}
			}
		}
		if (tot_doubles > 0) {
			result = CDDM_merge_verts(result, full_doubles_map, tot_doubles, CDDM_MERGE_VERTS_DUMP_IF_EQUAL);
		}
		MEM_freeN(full_doubles_map);
	}

	if (use_recalc_normals) {
		result->dirty |= DM_DIRTY_NORMALS;
	}

	if (vgroup_start_cap_remap) {
		MEM_freeN(vgroup_start_cap_remap);
	}
	if (vgroup_end_cap_remap) {
		MEM_freeN(vgroup_end_cap_remap);
	}

	return result;

}


static DerivedMesh *applyModifier(
        ModifierData *md, Object *ob,
        DerivedMesh *dm,
        ModifierApplyFlag flag)
{
	ArrayModifierData *amd = (ArrayModifierData *) md;
	return arrayModifier_doArray(amd, md->scene, ob, dm, flag);
}


ModifierTypeInfo modifierType_Array = {
	/* name */              "Array",
	/* structName */        "ArrayModifierData",
	/* structSize */        sizeof(ArrayModifierData),
	/* type */              eModifierTypeType_Constructive,
	/* flags */             eModifierTypeFlag_AcceptsMesh |
	                        eModifierTypeFlag_SupportsMapping |
	                        eModifierTypeFlag_SupportsEditmode |
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
	/* updateDepgraph */    updateDepgraph,
	/* updateDepsgraph */   updateDepsgraph,
	/* dependsOnTime */     NULL,
	/* dependsOnNormals */	NULL,
	/* foreachObjectLink */ foreachObjectLink,
	/* foreachIDLink */     NULL,
	/* foreachTexLink */    NULL,
};
