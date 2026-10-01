/*
 * RE_RasterScene.c — построение bucket-структуры по материалу.
 */

#include <stdlib.h>
#include <string.h>

#include "MEM_guardedalloc.h"
#include "BLI_math.h"
#include "BLI_blenlib.h"
#include "BLI_utildefines.h"

#include "DNA_meshdata_types.h"
#include "DNA_material_types.h"

#include "BKE_scene.h"

#include "render_types.h"
#include "renderdatabase.h"
#include "RE_Rasterizer.h"

/* ------------------------------------------------------------------------- */

static RE_RasterBucket *scene_find_or_add_bucket(RE_RasterScene *scene, Material *ma)
{
    int i;
    for (i = 0; i < scene->num_buckets; i++)
        if (scene->buckets[i]->material == ma) return scene->buckets[i];

    if (scene->num_buckets >= scene->cap_buckets) {
        scene->cap_buckets = scene->cap_buckets ? scene->cap_buckets * 2 : 16;
        scene->buckets = MEM_reallocN(scene->buckets,
                                      sizeof(RE_RasterBucket *) * scene->cap_buckets);
    }

    RE_RasterBucket *b = MEM_callocN(sizeof(*b), "RE_RasterBucket");
    b->material = ma;
    BLI_listbase_clear(&b->slots);
    scene->buckets[scene->num_buckets++] = b;
    return b;
}

static RE_RasterSlot *bucket_find_or_add_slot(RE_RasterBucket *bucket, ObjectInstanceRen *obi)
{
	for (RE_RasterSlot *slot = bucket->slots.first; slot; slot = slot->next)
		if (slot->obi == obi) return slot;

	RE_RasterSlot *slot = MEM_callocN(sizeof(*slot), "RE_RasterSlot");
	slot->obi = obi;
	slot->visible = true;
	slot->object_color = false;
	slot->rgba[0] = slot->rgba[1] = slot->rgba[2] = slot->rgba[3] = 1.0f;
	BLI_listbase_clear(&slot->display_arrays);
	BLI_addtail(&bucket->slots, slot);

	return slot;
}

static RE_RasterDisplayArray *slot_find_or_add_darray(RE_RasterSlot *slot, int prim_type)
{
    for (RE_RasterDisplayArray *da = slot->display_arrays.first; da; da = da->next)
        if (da->prim_type == prim_type) return da;

    RE_RasterDisplayArray *da = MEM_callocN(sizeof(*da), "RE_RasterDisplayArray");
    da->prim_type = prim_type;
    da->cap_verts = 1024;
    da->cap_indices = 1024 * prim_type;
    da->cap_prims = 1024;                                    /* ★ НОВОЕ */
    da->verts = MEM_mallocN(sizeof(RE_RasterVertex) * da->cap_verts, "RE_RasterVertices");
    da->indices = MEM_mallocN(sizeof(unsigned short) * da->cap_indices, "RE_RasterIndices");
    da->vlr_indices = MEM_mallocN(sizeof(int) * da->cap_prims, "RE_RasterVlrIndices");  /* ★ НОВОЕ */
    BLI_addtail(&slot->display_arrays, da);
    return da;
}

static void darray_ensure_vert_cap(RE_RasterDisplayArray *da, int need)
{
    if (da->num_verts + need <= da->cap_verts) return;
    while (da->num_verts + need > da->cap_verts) da->cap_verts *= 2;
    da->verts = MEM_reallocN(da->verts, sizeof(RE_RasterVertex) * da->cap_verts);
}

static void darray_ensure_index_cap(RE_RasterDisplayArray *da, int need)
{
    if (da->num_indices + need <= da->cap_indices) return;
    while (da->num_indices + need > da->cap_indices) da->cap_indices *= 2;
    da->indices = MEM_reallocN(da->indices, sizeof(unsigned short) * da->cap_indices);
}

/* ------------------------------------------------------------------------- */

static void emit_vlakren(RE_RasterDisplayArray *da, VlakRen *vlr,
                         ObjectRen *obr, ObjectInstanceRen *UNUSED(obi))
{
    const int num_verts = vlr->v4 ? 4 : 3;
    const int base = da->num_verts;

    MTFace *mtface = NULL;
    MCol   *mcol   = NULL;

    if (obr->actmtface >= 0)
        mtface = RE_vlakren_get_tface(obr, vlr, obr->actmtface, NULL, 0);
    if (obr->actmcol >= 0)
        mcol = RE_vlakren_get_mcol(obr, vlr, obr->actmcol, NULL, 0);

    darray_ensure_vert_cap(da, num_verts);
    darray_ensure_index_cap(da, num_verts);
	
	/* ★ РЕГИСТРИРУЕМ ПРИМИТИВ */
    if (da->num_prims >= da->cap_prims) {
        da->cap_prims = da->cap_prims ? da->cap_prims * 2 : 1024;
        da->vlr_indices = MEM_reallocN(da->vlr_indices,
                                       sizeof(int) * da->cap_prims);
    }
    da->vlr_indices[da->num_prims++] = vlr->index + 1;

    VertRen *vv[4] = { vlr->v1, vlr->v2, vlr->v3, vlr->v4 };

    for (int i = 0; i < num_verts; i++) {
        RE_RasterVertex *rv = &da->verts[da->num_verts++];
        VertRen *v = vv[i];

        copy_v3_v3(rv->co, v->co);

        if (vlr->flag & R_SMOOTH)
            copy_v3_v3(rv->n, v->n);
        else
            copy_v3_v3(rv->n, vlr->n);

        if (mtface) {
            rv->uv[0] = mtface->uv[i][0];
            rv->uv[1] = mtface->uv[i][1];
        } else {
            rv->uv[0] = rv->uv[1] = 0.0f;
        }

        if (mcol) {
            rv->rgba[0] = mcol[i].r;
            rv->rgba[1] = mcol[i].g;
            rv->rgba[2] = mcol[i].b;
            rv->rgba[3] = mcol[i].a;
        } else {
            rv->rgba[0] = rv->rgba[1] = rv->rgba[2] = rv->rgba[3] = 255;
        }

        rv->orig_index = v->index;
        rv->flags = (vlr->flag & R_SMOOTH) ? 1 : 0;
    }

    for (int i = 0; i < num_verts; i++)
        da->indices[da->num_indices++] = base + i;
}

/* ------------------------------------------------------------------------- */

static int bucket_compare(const void *a, const void *b)
{
    const RE_RasterBucket *ba = *(const RE_RasterBucket * const *)a;
    const RE_RasterBucket *bb = *(const RE_RasterBucket * const *)b;
    if (ba->material < bb->material) return -1;
    if (ba->material > bb->material) return  1;
    return 0;
}

RE_RasterScene *RE_raster_scene_build(RE_Rasterizer *rasty)
{
    RE_RasterScene *scene = MEM_callocN(sizeof(*scene), "RE_RasterScene");
    scene->re = rasty->re;

    /* ★ ОТЛАДКА */
    int obi_count = 0;
    int vlr_count = 0;
    int skipped_hidden = 0;

    printf("[RASTERIZER] scene_build start: instancetable.first=%p\n",
           (void*)rasty->re->instancetable.first);

    for (ObjectInstanceRen *obi = rasty->re->instancetable.first;
         obi; obi = obi->next)
    {
        obi_count++;

        /* ★ ОТЛАДКА */
        printf("[RASTERIZER]   obi[%d]: obr=%p, totvlak=%d\n",
               obi_count - 1, (void*)obi->obr,
               obi->obr ? obi->obr->totvlak : -1);

        ObjectRen *obr = obi->obr;
        RE_RasterBucket *cur_bucket = NULL;
        RE_RasterSlot *cur_slot = NULL;
        RE_RasterDisplayArray *cur_da = NULL;
        Material *cur_ma = NULL;
        int cur_stride = 0;

        for (int v = 0; v < obr->totvlak; v++) {
            VlakRen *vlr = RE_findOrAddVlak(obr, v);
            int stride;

            if (!vlr) {
                printf("[RASTERIZER]   vlr[%d] NULL, skip\n", v);
                continue;
            }

            if (vlr->flag & R_HIDDEN) {
                skipped_hidden++;
                continue;
            }

            vlr_count++;
            stride = vlr->v4 ? 4 : 3;

            if (vlr->mat != cur_ma || cur_da == NULL) {
                cur_ma = vlr->mat;
                cur_bucket = scene_find_or_add_bucket(scene, cur_ma);
                cur_slot = bucket_find_or_add_slot(cur_bucket, obi);
                cur_da = slot_find_or_add_darray(cur_slot, stride);
                cur_stride = stride;
            }
            else if (cur_stride != stride || cur_da->num_verts + stride > 65530) {
                /* ★ другой prim_type или индексы вот-вот переполнят unsigned short */
                cur_da = slot_find_or_add_darray(cur_slot, stride);
                cur_stride = stride;
            }

            emit_vlakren(cur_da, vlr, obr, obi);
        }
    }

    printf("[RASTERIZER] scene_build end: obi_count=%d, vlr_count=%d, hidden_skipped=%d\n",
           obi_count, vlr_count, skipped_hidden);

    /* ★ ОТЛАДКА: хеш содержимого сцены (детерминизм исходных данных) */
    if (getenv("UBF_SDUMP")) {
        unsigned int h = 2166136261u;
        int nverts = 0, nprims = 0;
        for (int i = 0; i < scene->num_buckets; i++) {
            RE_RasterBucket *b = scene->buckets[i];
            for (RE_RasterSlot *sl = b->slots.first; sl; sl = sl->next) {
                for (RE_RasterDisplayArray *da = sl->display_arrays.first; da; da = da->next) {
                    nverts += da->num_verts;
                    nprims += da->num_prims;
                    for (int v = 0; v < da->num_verts; v++) {
                        const unsigned char *p = (const unsigned char *)&da->verts[v];
                        for (unsigned k = 0; k < sizeof(RE_RasterVertex); k++)
                            h = (h ^ p[k]) * 16777619u;
                    }
                    for (int k = 0; k < da->num_indices; k++)
                        h = (h ^ (unsigned int)da->indices[k]) * 16777619u;
                    for (int k = 0; k < da->num_prims; k++)
                        h = (h ^ (unsigned int)da->vlr_indices[k]) * 16777619u;
                }
            }
        }
        printf("[SDUMP] buckets=%d verts=%d prims=%d hash=%08x\n",
               scene->num_buckets, nverts, nprims, h);
    }

	/* debug: посмотрим размеры */
	{
		int total_slots = 0, total_arrays = 0, total_indices = 0;
		for (int i = 0; i < scene->num_buckets; i++) {
			RE_RasterBucket *b = scene->buckets[i];
			for (RE_RasterSlot *sl = b->slots.first; sl; sl = sl->next) {
				total_slots++;
				for (RE_RasterDisplayArray *da = sl->display_arrays.first;
				     da; da = da->next)
				{
					total_arrays++;
					total_indices += da->num_indices;
				}
			}
		}
		printf("[RASTERIZER]   slots=%d, arrays=%d, total_indices=%d\n",
		       total_slots, total_arrays, total_indices);
	}

	return scene;
}

/* ------------------------------------------------------------------------- */

static void free_darray(RE_RasterDisplayArray *da)
{
    if (da->verts)       MEM_freeN(da->verts);
    if (da->indices)     MEM_freeN(da->indices);
    if (da->vlr_indices) MEM_freeN(da->vlr_indices);
    MEM_freeN(da);
}

static void free_slot(RE_RasterSlot *slot)
{
    RE_RasterDisplayArray *da, *dan;
    for (da = slot->display_arrays.first; da; da = dan) {
        dan = da->next;
        free_darray(da);
    }
    MEM_freeN(slot);
}

static void free_bucket(RE_RasterBucket *b)
{
    RE_RasterSlot *slot, *slotn;
    for (slot = b->slots.first; slot; slot = slotn) {
        slotn = slot->next;
        free_slot(slot);
    }
    MEM_freeN(b);
}

void RE_raster_scene_free(RE_RasterScene *scene)
{
    if (!scene) return;
    for (int i = 0; i < scene->num_buckets; i++)
        free_bucket(scene->buckets[i]);
    if (scene->buckets) MEM_freeN(scene->buckets);
    MEM_freeN(scene);
}

/* ------------------------------------------------------------------------- */
/* Visibility — hook для динамических слоёв                                   */
/* ------------------------------------------------------------------------- */

static bool re_layer_is_visible(struct Render *UNUSED(re), ObjectInstanceRen *obi,
                                unsigned int lay, unsigned int lay_zmask, bool all_z)
{
    if (all_z) return true;
    if ((obi->lay & (lay | lay_zmask)) != 0) return true;
    /* ★ как в zbuffer_solid(): объект может быть виден через динамический слой */
    return BKE_object_layer_visible(obi->ob);
}

void RE_raster_scene_update_visibility(RE_RasterScene *scene,
                                       struct Render *re,
                                       struct RenderLayer *rl)
{
    const unsigned int lay = rl->lay;
    const unsigned int lay_zmask = rl->lay_zmask;
    const bool all_z = (rl->layflag & SCE_LAY_ALL_Z) && !(rl->layflag & SCE_LAY_ZMASK);

    for (int i = 0; i < scene->num_buckets; i++) {
        RE_RasterBucket *b = scene->buckets[i];
        for (RE_RasterSlot *slot = b->slots.first; slot; slot = slot->next) {
            slot->visible = re_layer_is_visible(re, slot->obi, lay, lay_zmask, all_z);
            slot->layer_id = slot->obi->lay;
        }
    }
}
