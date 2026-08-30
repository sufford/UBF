/*
 * MOD_capholes.c - Cap Holes modifier for Blender 2.79 (Production Stable Fan Edge Mapping)
 */

#include "MEM_guardedalloc.h"

#include "BLI_utildefines.h"
#include "BLI_math.h"

#include "DNA_meshdata_types.h"
#include "DNA_modifier_types.h"
#include "DNA_object_types.h"

#include "BKE_cdderivedmesh.h"
#include "BKE_modifier.h"

#include "MOD_modifiertypes.h"

static DerivedMesh *applyModifier(
	ModifierData *md, Object *ob,
	DerivedMesh *dm,
	ModifierApplyFlag UNUSED(flag))
{
	CapHolesModifierData *chmd = (CapHolesModifierData *)md;
	DerivedMesh *result = NULL;

	const MVert *mvert_src = dm->getVertArray(dm);
	const MEdge *medge_src = dm->getEdgeArray(dm);
	const MPoly *mpoly_src = dm->getPolyArray(dm);
	const MLoop *mloop_src = dm->getLoopArray(dm);

	int maxVerts = dm->getNumVerts(dm);
	int maxEdges = dm->getNumEdges(dm);
	int maxPolys = dm->getNumPolys(dm);
	int maxLoops = dm->getNumLoops(dm);

	/* 1. Count face usage per edge */
	int *edge_count = MEM_callocN(sizeof(int) * maxEdges, "edge_count");
	for (int i = 0; i < maxPolys; i++) {
		const MPoly *mp = &mpoly_src[i];
		const MLoop *ml = &mloop_src[mp->loopstart];
		for (int j = 0; j < mp->totloop; j++, ml++) {
			if (ml->e >= 0 && ml->e < maxEdges) {
				edge_count[ml->e]++;
			}
		}
	}

	/* 2. Flag border edges and allocate visited state */
	char *is_border_edge = MEM_callocN(sizeof(char) * maxEdges, "is_border_edge");
	int num_border_edges = 0;
	for (int i = 0; i < maxEdges; i++) {
		if (edge_count[i] == 1) {
			is_border_edge[i] = 1;
			num_border_edges++;
		}
	}

	if (num_border_edges == 0) {
		MEM_freeN(edge_count);
		MEM_freeN(is_border_edge);
		return dm;
	}

	/* Temporary structures to store identified loops */
	int **loop_verts_list = MEM_callocN(sizeof(int *) * num_border_edges, "loop_verts");
	int **loop_edges_list = MEM_callocN(sizeof(int *) * num_border_edges, "loop_edges");
	int *loop_sizes = MEM_callocN(sizeof(int) * num_border_edges, "loop_sizes");
	int total_holes_found = 0;
	int total_new_loops = 0;
	int total_new_polys = 0;
	int total_new_edges = 0;

	/* 3. Topological Loop Walking Algorithm */
	for (int i = 0; i < maxEdges; i++) {
		if (!is_border_edge[i]) continue;

		int start_edge = i;
		int current_edge = start_edge;
		
		int *v_seq = MEM_mallocN(sizeof(int) * num_border_edges, "v_seq");
		int *e_seq = MEM_mallocN(sizeof(int) * num_border_edges, "e_seq");
		int count = 0;

		int current_vert = medge_src[current_edge].v1;
		int next_vert = medge_src[current_edge].v2;

		while (1) {
			v_seq[count] = current_vert;
			e_seq[count] = current_edge;
			is_border_edge[current_edge] = 0; /* Mark as processed */
			count++;

			current_vert = next_vert;

			int found_next = 0;
			for (int j = 0; j < maxEdges; j++) {
				if (is_border_edge[j] && (medge_src[j].v1 == current_vert || medge_src[j].v2 == current_vert)) {
					current_edge = j;
					next_vert = (medge_src[j].v1 == current_vert) ? medge_src[j].v2 : medge_src[j].v1;
					found_next = 1;
					break;
				}
			}

			if (!found_next) {
				break;
			}
		}

		if (count >= 3) {
			loop_verts_list[total_holes_found] = v_seq;
			loop_edges_list[total_holes_found] = e_seq;
			loop_sizes[total_holes_found] = count;
			
			if (chmd->triangulate) {
				total_new_polys += (count - 2);
				total_new_loops += (count - 2) * 3;
				total_new_edges += (count - 3); /* Triangulating an N-gon requires count-3 interior lines */
			} else {
				total_new_polys += 1;
				total_new_loops += count;
			}
			
			total_holes_found++;
		} else {
			MEM_freeN(v_seq);
			MEM_freeN(e_seq);
		}
	}

	/* 4. Allocate space for the new mesh structure */
	int numVerts = maxVerts;
	int numEdges = maxEdges + total_new_edges; /* FIXED: Growing layout allocations securely */
	int numPolys = maxPolys + total_new_polys;
	int numLoops = maxLoops + total_new_loops;

	result = CDDM_from_template(dm, numVerts, numEdges, 0, numLoops, numPolys);

	MPoly *mpoly_dst = CDDM_get_polys(result);
	MLoop *mloop_dst = CDDM_get_loops(result);
	MEdge *medge_dst = CDDM_get_edges(result);
	MVert *mvert_dst = CDDM_get_verts(result);

	/* 5. Copy over original data arrays */
	for (int i = 0; i < maxVerts; i++) { mvert_dst[i] = mvert_src[i]; DM_copy_vert_data(dm, result, i, i, 1); }
	for (int i = 0; i < maxEdges; i++) { medge_dst[i] = medge_src[i]; DM_copy_edge_data(dm, result, i, i, 1); }
	for (int i = 0; i < maxPolys; i++) { mpoly_dst[i] = mpoly_src[i]; DM_copy_poly_data(dm, result, i, i, 1); }
	for (int i = 0; i < maxLoops; i++) { mloop_dst[i] = mloop_src[i]; DM_copy_loop_data(dm, result, i, i, 1); }

	/* 6. Populate Cap Faces, Loops, and Custom Internal Edges properly */
	int current_loop_idx = maxLoops;
	int current_poly_idx = maxPolys;
	int current_edge_idx = maxEdges;

	for (int h = 0; h < total_holes_found; h++) {
		int first_edge_idx = loop_edges_list[h][0];
		int first_vert_idx = loop_verts_list[h][0];
		bool normal_needs_flip = false;

		for (int i = 0; i < maxPolys; i++) {
			const MPoly *mp = &mpoly_src[i];
			const MLoop *ml = &mloop_src[mp->loopstart];
			bool found_edge = false;
			for (int j = 0; j < mp->totloop; j++, ml++) {
				if (ml->e == first_edge_idx) {
					if (ml->v == first_vert_idx) normal_needs_flip = true;
					found_edge = true;
					break;
				}
			}
			if (found_edge) break;
		}

		int count = loop_sizes[h];
		int *v_final = MEM_mallocN(sizeof(int) * count, "v_final");
		int *e_final = MEM_mallocN(sizeof(int) * count, "e_final");

		for (int j = 0; j < count; j++) {
			if (normal_needs_flip) {
				int reverse_v_idx = (count - 1) - j;
				int reverse_e_idx = (count - 2) - j;
				if (reverse_e_idx < 0) reverse_e_idx = count - 1;

				v_final[j] = loop_verts_list[h][reverse_v_idx];
				e_final[j] = loop_edges_list[h][reverse_e_idx];
			} else {
				v_final[j] = loop_verts_list[h][j];
				e_final[j] = loop_edges_list[h][j];
			}
		}

		if (chmd->triangulate) {
			/* First, pre-generate all required internal MEdge structures for this fan layout */
			int internal_edge_start = current_edge_idx;
			for (int t = 0; t < count - 3; t++) {
				MEdge *me = &medge_dst[current_edge_idx++];
				memset(me, 0, sizeof(MEdge));
				me->v1 = v_final[0];
				me->v2 = v_final[t + 2];
				me->flag = SELECT; /* Standard visibility flag settings */
			}

			/* Process Triangle Fan Face Loop structures securely */
			for (int t = 0; t < count - 2; t++) {
				MPoly *tri_poly = &mpoly_dst[current_poly_idx++];
				memset(tri_poly, 0, sizeof(MPoly));
				tri_poly->loopstart = current_loop_idx;
				tri_poly->totloop = 3;
				if (chmd->smooth) {
					tri_poly->flag |= ME_SMOOTH;
				}

				/* Element 0: Path goes from Vertex 0 to Vertex t+1 */
				mloop_dst[current_loop_idx].v = v_final[0];
				if (t == 0) {
					mloop_dst[current_loop_idx].e = e_final[0]; /* Outer boundary edge */
				} else {
					mloop_dst[current_loop_idx].e = internal_edge_start + (t - 1); /* Internal edge */
				}
				current_loop_idx++;

				/* Element 1: Path goes from Vertex t+1 to Vertex t+2 */
				mloop_dst[current_loop_idx].v = v_final[t + 1];
				mloop_dst[current_loop_idx].e = e_final[t + 1]; /* Outer boundary edge */
				current_loop_idx++;

				/* Element 2: Path goes from Vertex t+2 back to Vertex 0 */
				mloop_dst[current_loop_idx].v = v_final[t + 2];
				if (t == count - 3) {
					mloop_dst[current_loop_idx].e = e_final[count - 1]; /* Outer boundary edge */
				} else {
					mloop_dst[current_loop_idx].e = internal_edge_start + t; /* Internal edge */
				}
				current_loop_idx++;
			}
		} else {
			MPoly *cap_poly = &mpoly_dst[current_poly_idx++];
			memset(cap_poly, 0, sizeof(MPoly));
			cap_poly->loopstart = current_loop_idx;
			cap_poly->totloop = count;
			if (chmd->smooth) {
				cap_poly->flag |= ME_SMOOTH;
			}

			for (int j = 0; j < count; j++) {
				mloop_dst[current_loop_idx].v = v_final[j];
				mloop_dst[current_loop_idx].e = e_final[j];
				current_loop_idx++;
			}
		}

		MEM_freeN(v_final);
		v_final = NULL;
		MEM_freeN(e_final);
		e_final = NULL;
		MEM_freeN(loop_verts_list[h]);
		MEM_freeN(loop_edges_list[h]);
	}

	/* 7. Structural Cleanup */
	result->dirty |= DM_DIRTY_NORMALS;

	MEM_freeN(edge_count);
	MEM_freeN(is_border_edge);
	MEM_freeN(loop_verts_list);
	MEM_freeN(loop_edges_list);
	MEM_freeN(loop_sizes);

	return result;
}

/* Edit Mode evaluator wrapper function */
static DerivedMesh *applyModifierEM(
    ModifierData *md, Object *ob,
    struct BMEditMesh *UNUSED(editData),
    DerivedMesh *dm,
    ModifierApplyFlag flag)
{
    return applyModifier(md, ob, dm, flag);
}

/* Initialize with default values */
static void initData(ModifierData *md) {
	CapHolesModifierData *chmd = (CapHolesModifierData *)md;
	chmd->smooth = 0;
	chmd->triangulate = 0;
}

/* Register the modifier type */
ModifierTypeInfo modifierType_CapHoles = {
	/* name */              "Cap Holes",
	/* structName */        "CapHolesModifierData",
	/* structSize */        sizeof(CapHolesModifierData),
	/* type */              eModifierTypeType_Nonconstructive,
	/* flags */             eModifierTypeFlag_AcceptsMesh |
							eModifierTypeFlag_SupportsMapping |
							eModifierTypeFlag_SupportsEditmode,

	/* copyData */          modifier_copyData_generic,
	/* deformVerts */       NULL,
	/* deformMatrices */    NULL,
	/* deformVertsEM */     NULL,
	/* deformMatricesEM */  NULL,
	/* applyModifier */     applyModifier,
	/* applyModifierEM */   applyModifierEM,
	/* initData */          initData,
	/* requiredDataMask */  NULL,
	/* freeData */          NULL,
	/* isDisabled */  		NULL,
	/* updateDepgraph */    NULL,
	/* updateDepsgraph */   NULL,
	/* dependsOnTime */     NULL,
	/* dependsOnNormals */  NULL,
	/* foreachObjectLink */ NULL,
	/* foreachIDLink */     NULL,
	/* foreachTexLink */    NULL,
};
