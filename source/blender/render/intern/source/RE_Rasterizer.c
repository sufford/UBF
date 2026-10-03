/*
 * RE_Rasterizer.c — фасад и главный entry point.
 *
 * ПАТЧ (harness):
 *   (1) rasty->winmat / rasty->viewmat больше не остаются нулевыми — это была
 *       причина «серой картинки»: obwinmat = 0-матрица => hoco[3] == 0 =>
 *       testclip()=out => ни один пиксель не попадал в rectp/recto и
 *       shade_samples не вызывался вообще.
 *   (2) RE_rasterizer_render_part теперь воспроизводит семантику
 *       zbuffer_solid(): цикл по OSA-сэмплам + корректное распределение
 *       буферов (последний сэмпл пишет в pa->rect*, остальные — в scratch).
 *   (3) Убраны затирания pa->rectz/rectp/recto (fillrect(pa->rectp,0) убивал
 *       уже накопленные сэмплы в OSA-цикле zbufshadeDA_tile).
 *   (4) Убран нулевой выход при re->osa == 0 (F12 по умолчанию — non-OSA,
 *       раньше растеризатор просто отключался).
 */

#include <string.h>
#include <stdlib.h>
#include <stdio.h>


#include "MEM_guardedalloc.h"
#include "BLI_math.h"
#include "BLI_blenlib.h"
#include "BLI_utildefines.h"

#include "DNA_material_types.h"
#include "DNA_scene_types.h"

#include "render_types.h"
#include "renderdatabase.h"
#include "render_result.h"
#include "shading.h"
#include "RE_Rasterizer.h"
#include "zbuf.h"
#include "RE_Prof.h"     /* ★ PROF — временная обвязка замера */

/* forward */
RE_IStorage *RE_storage_create_scanline(void);

/* ★ НОВОЕ: fallback видимости для динамических слоёв */
#include "BKE_scene.h"

extern struct Render R;

/* ------------------------------------------------------------------------- */

RE_IStorage *RE_storage_create(RE_RasterStorageType type)
{
    switch (type) {
        case RE_STORAGE_EDGE:
        case RE_STORAGE_TILED:
            /* v1: fallback на scanline. Позже добавим отдельные реализации. */
        case RE_STORAGE_SCANLINE:
        default:
            return RE_storage_create_scanline();
    }
}

bool RE_rasterizer_enabled(struct Render *re)
{
    /* ★ ВРЕМЕННО (A/B-тест): UBF_RASTERIZER=0 отключает растеризатор.
     * Убрать после отладки. */
    static int env_ok = -1, env_val = 1;
    if (env_ok < 0) {
        const char *e = getenv("UBF_RASTERIZER");
        env_val = (e && e[0] == '0') ? 0 : 1;
        env_ok = 1;
    }
    if (!env_val) return false;

    if (re == NULL) return false;
    if (re->rasterizer == NULL) return false;
    if (re->r.rasterizer_mode == 0) return false;
    /* Работаем и при re->osa == 0 (штатный F12): render_part сам делает
     * цикл по сэмплам, как это делал zbuffer_solid(). */
    return true;
}

RE_Rasterizer *RE_rasterizer_create(struct Render *re, RE_RasterStorageType type)
{
	RE_Rasterizer *rasty = MEM_callocN(sizeof(*rasty), "RE_Rasterizer");
	rasty->re = re;
	rasty->storage_type = type;
	rasty->storage = RE_storage_create(type);

	if (rasty->storage && rasty->storage->init)
		rasty->storage->init(rasty->storage, rasty);

	return rasty;
}

void RE_rasterizer_free(RE_Rasterizer *rasty)
{
	if (!rasty) return;

	RE_prof_report();    /* ★ PROF — печатает итог до разрушения сцены */

	if (rasty->storage) {
		if (rasty->storage->exit) rasty->storage->exit(rasty->storage);
		MEM_freeN(rasty->storage);
	}
	if (rasty->scene) RE_raster_scene_free(rasty->scene);
	MEM_freeN(rasty);
}

/* ------------------------------------------------------------------------- */
/* Главный entry point                                                        */
/* ------------------------------------------------------------------------- */

void RE_rasterizer_render_part(RE_Rasterizer *rasty,
	struct RenderPart *pa,
	struct RenderLayer *rl,
	ZSpan *zspans_out,
	int *nsamples_out)
{
	ZSpan *zspan;
	struct Render *re;
	int samples, zsample;
	bool neg_zmask;
	/* ★ ШАГ B2-ii: контексты атрибутов, по одному на сэмпл. Стек этой функции
	 * потокобезопасен, а rasty/storage — общие, поэтому только так. */
	RE_RasterAttr attrs[16];

	/* ★ zspans_out живут на стеке вызывающего (zbuffer_solid). Это убирает
	 * и гонку (zspans были общие в rasty), и утечку scratch-буферов. */
	if (!rasty || !rasty->scene || !pa || !zspans_out || !nsamples_out) return;

	*nsamples_out = 0;

	re = rasty->re;
	if (!re) return;

	/* ★ PROF — часть целиком меряется внутри RE_prof_part_end() */
	RE_prof_part_begin();
	RE_prof_note_threads(re->r.threads);

	/* (1) матрицы: ровно как в zbuffer_solid() */
	zbuf_make_winmat(re, rasty->winmat);
	copy_m4_m4(rasty->viewmat, re->viewmat);
	rasty->width = re->winx;
	rasty->height = re->winy;

	(void)rl;

	/* (2) сэмплы: повторяем логику zbuffer_solid() */
	samples = (re->osa ? re->osa : 1);
	samples = MIN2(4, samples - pa->sample);
	if (samples < 1) samples = 1;

	neg_zmask = (rl != NULL) && (rl->layflag & SCE_LAY_ZMASK) && (rl->layflag & SCE_LAY_NEG_ZMASK);

	{	/* ★ PROF */
	double t_span0 = RE_prof_tick();

	for (zsample = 0; zsample < samples; zsample++) {
		zspan = &zspans_out[zsample];

		zbuf_alloc_span(zspan, pa->rectx, pa->recty, re->clipcrop);
		zspan->zmulx = ((float)re->winx) / 2.0f;
		zspan->zmuly = ((float)re->winy) / 2.0f;

		if (re->osa) {
			zspan->zofsx = -pa->disprect.xmin - re->jit[pa->sample + zsample][0];
			zspan->zofsy = -pa->disprect.ymin - re->jit[pa->sample + zsample][1];
		}
		else if (re->i.curblur) {
			zspan->zofsx = -pa->disprect.xmin - re->mblur_jit[re->i.curblur - 1][0];
			zspan->zofsy = -pa->disprect.ymin - re->mblur_jit[re->i.curblur - 1][1];
		}
		else {
			zspan->zofsx = -pa->disprect.xmin;
			zspan->zofsy = -pa->disprect.ymin;
		}
		zspan->zofsx -= 0.5f;
		zspan->zofsy -= 0.5f;

		/* the buffers */
		if (samples == 1 || zsample == samples - 1) {
			zspan->rectp = pa->rectp;
			zspan->recto = pa->recto;
			zspan->rectz = (neg_zmask && pa->rectmask) ? pa->rectmask : pa->rectz;
		}
		else {
			zspan->recto = MEM_mallocN(sizeof(int) * pa->rectx * pa->recty, "recto");
			zspan->rectp = MEM_mallocN(sizeof(int) * pa->rectx * pa->recty, "rectp");
			zspan->rectz = MEM_mallocN(sizeof(int) * pa->rectx * pa->recty, "rectz");
		}

		/* ★ ШАГ B2-ii: атрибуты пишем в gbuf только для ТОГО ЖЕ сэмпла,
		 * чьи rectp/recto/rectz попадут в pa. У промежуточных OSA-сэмплов
		 * буферы scratch-ные, и запись в gbuf оттуда испортила бы результат
		 * (проход-предшественник читал только pa->rectp). */
		attrs[zsample].re = re;
		attrs[zsample].rl = rl;
		attrs[zsample].xmin = pa->disprect.xmin;
		attrs[zsample].ymin = pa->disprect.ymin;
		attrs[zsample].rectx = pa->rectx;
		attrs[zsample].recty = pa->recty;
		if (zspan->rectp == pa->rectp) {
			attrs[zsample].co = pa->gbuf_co;
			attrs[zsample].vn = pa->gbuf_vn;
		}
		else {
			attrs[zsample].co = NULL;
			attrs[zsample].vn = NULL;
		}

		fillrect(zspan->rectz, pa->rectx, pa->recty, 0x7FFFFFFF);
		fillrect(zspan->rectp, pa->rectx, pa->recty, 0);
		fillrect(zspan->recto, pa->rectx, pa->recty, 0);
	}

	RE_prof_span(RE_PROF_SPAN, t_span0);   /* ★ PROF */
	}	/* ★ PROF */

	/* bounds для bbox clip */
	{	/* ★ PROF */
	double t_scene0 = RE_prof_tick();

	{
		float bounds[4];
		bounds[0] = (2 * pa->disprect.xmin - re->winx - 1) / (float)re->winx;
		bounds[1] = (2 * pa->disprect.xmax - re->winx + 1) / (float)re->winx;
		bounds[2] = (2 * pa->disprect.ymin - re->winy - 1) / (float)re->winy;
		bounds[3] = (2 * pa->disprect.ymax - re->winy + 1) / (float)re->winy;

		for (int b = 0; b < rasty->scene->num_buckets; b++) {
			RE_RasterBucket *bucket = rasty->scene->buckets[b];
			rasty->last_material = bucket->material;
			RE_prof_count(RE_PROF_C_BUCKETS, 1);   /* ★ PROF */

			for (RE_RasterSlot *slot = bucket->slots.first; slot; slot = slot->next) {
				RE_prof_count(RE_PROF_C_SLOTS_TESTED, 1);   /* ★ PROF */
				if (!slot->visible) continue;

				ObjectInstanceRen *obi = slot->obi;
				float obwinmat[4][4];

				if (obi->flag & R_TRANSFORMED)
					mul_m4_m4m4(obwinmat, rasty->winmat, obi->mat);
				else
					copy_m4_m4(obwinmat, rasty->winmat);

				if (clip_render_object(obi->obr->boundbox, bounds, obwinmat)) {
					RE_prof_count(RE_PROF_C_SLOTS_CULLED, 1);   /* ★ PROF */
					continue;
				}

				int obi_index = (int)(obi - re->objectinstance);
				RE_prof_count(RE_PROF_C_SLOTS_DRAWN, 1);   /* ★ PROF */

				for (RE_RasterDisplayArray *da = slot->display_arrays.first;
					da; da = da->next)
				{
					RE_prof_count(RE_PROF_C_DARRAYS, 1);   /* ★ PROF */
					for (zsample = 0; zsample < samples; zsample++) {
						rasty->storage->rasterize(rasty->storage, rasty, slot, da,
							&zspans_out[zsample], obi_index, 0,
							(const float(*)[4])obwinmat, bounds,
							&attrs[zsample]);
					}
				}
			}
		}
	}

	RE_prof_span(RE_PROF_SCENE_LOOP, t_scene0);   /* ★ PROF */
	}	/* ★ PROF */

	/* ★ ZSpan'ы НЕ освобождаем здесь: zbuffer_solid вызовет fillfunc на каждый,
	 * потом освободит scratch. Освобождаем только span-массивы. */
	for (zsample = 0; zsample < samples; zsample++)
		zbuf_free_span(&zspans_out[zsample]);

	*nsamples_out = samples;

	/* ★ PROF — печатает строку части (время + счётчики) */
	RE_prof_part_end(pa->disprect.xmin, pa->disprect.ymin, pa->rectx, pa->recty);
}

/* ========================================================================= */
/* ★ ШАГ 3a — G-буфер части (позиция и нормаль на пиксель)                    */
/*                                                                           */
/* Заполняется из pa->rectp/recto/rectz — то есть работает независимо от     */
/* того, кто растеризовал часть (наш растеризатор или BI). Реконструкция     */
/* повторяет последовательность BI:                                          */
/*     shade_input_set_triangle() -> shade_input_set_viewco() ->              */
/*     shade_input_set_uv() -> shade_input_set_normals()                     */
/* но без ShadeInput: снаружи остаётся только чистая математика из           */
/* shadeinput.c / rendercore.c.                                              */
/* ========================================================================= */

/* ★ Этап 2, «diffuse»: подготовка атрибутов на треугольник.
 *
 * Всё, что ниже в этой функции, постоянно на треугольник и раньше считалось на
 * каждый пиксель (см. комментарий к RE_RasterAttrTri в RE_Rasterizer.h).
 * Выражения скопированы из старого пиксельного кода как есть, порядок
 * сохранён: приёмка сверяет результат с BI побитово. */
static bool raster_gbuffer_prepare(RenderLayer *rl, ObjectInstanceRen *obi,
                                   int facenr, RE_RasterAttrTri *tri)
{
	VlakRen *vlr;
	VertRen *va, *vb, *vc;
	int vi;

	if (facenr <= 0 || obi == NULL || obi->obr == NULL) return false;

	vi = (facenr - 1) & RE_QUAD_MASK;
	if (vi < 0 || vi >= obi->obr->totvlak) return false;

	vlr = RE_findOrAddVlak(obi->obr, vi);
	if (!vlr) return false;

	/* shi->mat = shi->mat_override ? shi->mat_override : vlr->mat */
	tri->ma = rl->mat_override ? rl->mat_override : vlr->mat;
	if (!tri->ma) return false;

	/* квад = два треугольника, как в zbuffer_solid(): (v1,v2,v3) и (v1,v3,v4) */
	if (facenr & RE_QUAD_OFFS) { va = vlr->v1; vb = vlr->v3; vc = vlr->v4; }
	else                        { va = vlr->v1; vb = vlr->v2; vc = vlr->v3; }
	if (!va || !vb || !vc) return false;

	tri->smooth      = (vlr->flag & R_SMOOTH) != 0;
	tri->tangent     = (vlr->flag & R_TANGENT) != 0;
	tri->transformed = (obi->flag & R_TRANSFORMED) != 0;
	tri->wire_edge   = (vlr->v2 == vlr->v3);
	tri->need_uv     = tri->smooth
	                   || (tri->ma->texco & NEED_UV)
	                   || (rl->passflag & SCE_PASS_UV);

	/* --- facenor: RE_vlakren_get_normal() --- */
	if (tri->transformed) {
		mul_v3_m3v3(tri->facenor, obi->nmat, vlr->n);
		normalize_v3(tri->facenor);
	}
	else {
		copy_v3_v3(tri->facenor, vlr->n);
	}

	/* --- нормали вершин: shade_input_set_triangle_i() --- */
	if (tri->smooth) {
		copy_v3_v3(tri->n1, va->n);
		copy_v3_v3(tri->n2, vb->n);
		copy_v3_v3(tri->n3, vc->n);
		if (tri->transformed) {
			mul_m3_v3(obi->nmat, tri->n1); normalize_v3(tri->n1);
			mul_m3_v3(obi->nmat, tri->n2); normalize_v3(tri->n2);
			mul_m3_v3(obi->nmat, tri->n3); normalize_v3(tri->n3);
		}
	}

	/* --- мировые координаты вершин: shade_input_set_uv() --- */
	copy_v3_v3(tri->v1, va->co);
	copy_v3_v3(tri->v2, vb->co);
	copy_v3_v3(tri->v3, vc->co);
	if (tri->transformed) {
		mul_m4_v3(obi->mat, tri->v1);
		mul_m4_v3(obi->mat, tri->v2);
		mul_m4_v3(obi->mat, tri->v3);
	}

	/* dface нужен только не-wire ветке, но зависит только от треугольника.
	 * В старом коде стоял ровно здесь — до возможного negate_v3(facenor). */
	tri->dface = dot_v3v3(tri->v1, tri->facenor);

	return true;
}

/* Позиция и нормаль одного пикселя. xs/ys — координаты сэмпла:
 * для не-OSA это ровно пиксель + 0.5, как в shadeinput.c:1434. */
static bool raster_gbuffer_pixel(Render *re,
                                 const RE_RasterAttrTri *tri, int z,
                                 float xs, float ys,
                                 float co_out[3], float vn_out[3])
{
	float facenor[3];
	float view[3], co[3], vn[3];
	float u = 0.0f, v = 0.0f, l;
	bool flippednor = false;

	/* facenor копируется: ниже он может быть перевёрнут, а копия в tri должна
	 * остаться исходной для следующих пикселей этого треугольника. */
	copy_v3_v3(facenor, tri->facenor);

	/* --- co: shade_input_calc_viewco() --- */

	/* calc_view_vector(): вектор НЕ нормализован, это viewplane-координаты.
	 * Нормализация — в самом конце calc_viewco; порядок важен, потому что
	 * set_normals сравнивает facenor именно с нормализованным view. */
	view[2] = -ABS(re->clipsta);

	if (re->r.mode & R_ORTHO) {
		view[0] = view[1] = 0.0f;
	}
	else {
		if (re->r.mode & R_PANORAMA) {
			xs -= re->panodxp;
		}

		view[0] = re->viewplane.xmin + (xs / (float)re->winx) * BLI_rctf_size_x(&re->viewplane);
		view[1] = re->viewplane.ymin + (ys / (float)re->winy) * BLI_rctf_size_y(&re->viewplane);

		if (re->r.mode & R_PANORAMA) {
			float pu = view[0] + re->panodxv;
			float pv = view[2];

			view[0] = re->panoco * pu + re->panosi * pv;
			view[2] = -re->panosi * pu + re->panoco * pv;
		}
	}

	if (tri->ma->material_type == MA_TYPE_WIRE) {
		/* wire: координата восстанавливается по zbuf, менее точно */
		float zco = ((float)z) / 2147483647.0f;

		co[2] = re->winmat[3][2] / (re->winmat[2][3] * zco - re->winmat[2][2]);

		if (re->r.mode & R_ORTHO) {
			float fx = 2.0f / (re->winx * re->winmat[0][0]);
			float fy = 2.0f / (re->winy * re->winmat[1][1]);

			co[0] = (xs - 0.5f * re->winx) * fx - re->winmat[3][0] / re->winmat[0][0];
			co[1] = (ys - 0.5f * re->winy) * fy - re->winmat[3][1] / re->winmat[1][1];
		}
		else {
			float fac = co[2] / view[2];

			co[0] = fac * view[0];
			co[1] = fac * view[1];
		}
	}
	else {
		/* не-wire: пересечение луча зрения с плоскостью грани */
		float dface = tri->dface;

		if (re->r.mode & R_ORTHO) {
			float fx = 2.0f / (re->winx * re->winmat[0][0]);
			float fy = 2.0f / (re->winy * re->winmat[1][1]);

			co[0] = (xs - 0.5f * re->winx) * fx - re->winmat[3][0] / re->winmat[0][0];
			co[1] = (ys - 0.5f * re->winy) * fy - re->winmat[3][1] / re->winmat[1][1];

			if (facenor[2] != 0.0f)
				co[2] = (dface - facenor[0] * co[0] - facenor[1] * co[1]) / facenor[2];
			else
				co[2] = 0.0f;
		}
		else {
			float div = dot_v3v3(facenor, view);
			float fac = (div != 0.0f) ? (dface / div) : 0.0f;

			co[0] = fac * view[0];
			co[1] = fac * view[1];
			co[2] = fac * view[2];
		}
	}

	/* --- uv: shade_input_set_uv() (нужны только для сглаженных граней) --- */
	if (tri->need_uv) {
		if (tri->wire_edge) {
			/* wire render of edge */
			float lend = len_v3v3(tri->v2, tri->v1);
			float lenc = len_v3v3(co, tri->v1);

			u = (lend == 0.0f) ? 0.0f : -(1.0f - lenc / lend);
			v = 0.0f;
		}
		else {
			/* чистый помощник BI без ShadeInput (барицентрика по доминирующей оси) */
			barycentric_differentials_from_position(co, tri->v1, tri->v2, tri->v3, NULL, NULL,
			                                        facenor, false,
			                                        &u, &v, NULL, NULL, NULL, NULL);
			u = -u;
			v = -v;
			CLAMP(u, -2.0f, 1.0f);
			CLAMP(v, -2.0f, 1.0f);
		}
	}

	normalize_v3(view);

	/* --- vn: shade_input_set_normals() --- */
	l = 1.0f + u + v;

	if (!tri->tangent) {
		if (dot_v3v3(facenor, view) < 0.0f) {
			negate_v3(facenor);
			flippednor = true;
		}
	}

	if (tri->smooth) {
		float n1[3], n2[3], n3[3];

		copy_v3_v3(n1, tri->n1);
		copy_v3_v3(n2, tri->n2);
		copy_v3_v3(n3, tri->n3);

		if (flippednor) { negate_v3(n1); negate_v3(n2); negate_v3(n3); }

		vn[0] = l * n3[0] - u * n1[0] - v * n2[0];
		vn[1] = l * n3[1] - u * n1[1] - v * n2[1];
		vn[2] = l * n3[2] - u * n1[2] - v * n2[2];
		normalize_v3(vn);
	}
	else {
		copy_v3_v3(vn, facenor);
	}

	/* финальный flip_normals перепроверяет уже (возможно) перевёрнутый facenor,
	 * поэтому для vn это negate срабатывает только при dot == 0 */
	if (!tri->tangent && dot_v3v3(facenor, view) < 0.0f) {
		negate_v3(vn);
	}

	copy_v3_v3(co_out, co);
	copy_v3_v3(vn_out, vn);
	return true;
}

/* ★ Этап 2, «diffuse»: точка входа для филлера. */
bool RE_raster_attr_prepare(const RE_RasterAttr *attr, int obi_index, int facenr,
                            RE_RasterAttrTri *tri)
{
	if (!attr || !attr->co || !attr->vn) return false;
	if (!tri || obi_index < 0) return false;

	return raster_gbuffer_prepare(attr->rl,
	                              attr->re->objectinstance + obi_index,
	                              facenr, tri);
}

void RE_raster_attr_pixel(const RE_RasterAttr *attr, const RE_RasterAttrTri *tri,
                          int col, int row, int z)
{
	float *co, *vn;
	int idx;

	if (!attr || !attr->co || !attr->vn || !tri) return;
	if (col < 0 || col >= attr->rectx || row < 0 || row >= attr->recty) return;

	idx = row * attr->rectx + col;
	co = attr->co + 3 * idx;
	vn = attr->vn + 3 * idx;

	{	/* ★ PROF */
	double t_attr0 = RE_prof_tick();

	raster_gbuffer_pixel(attr->re, tri, z,
	                     (float)(attr->xmin + col) + 0.5f,
	                     (float)(attr->ymin + row) + 0.5f,
	                     co, vn);

	RE_prof_span(RE_PROF_ATTR, t_attr0);
	RE_prof_count(RE_PROF_C_ATTRPIX, 1);
	}
}

void RE_raster_gbuffer_fill(Render *re, RenderPart *pa, RenderLayer *rl, RE_RasterGBuffer *gb)
{
	RE_RasterAttrTri tri;
	int cache_obi = -1, cache_face = -1, tri_ok = 0;
	int lx, ly, idx = 0;

	if (!re || !pa || !rl || !gb || !gb->co || !gb->vn) return;
	if (!pa->rectp || !pa->recto || !pa->rectz) return;

	for (ly = 0; ly < pa->recty; ly++) {
		const float y = (float)(pa->disprect.ymin + ly);

		for (lx = 0; lx < pa->rectx; lx++, idx++) {
			const float x = (float)(pa->disprect.xmin + lx);
			const int facenr = pa->rectp[idx];
			int obi_index;

			if (facenr <= 0) continue;   /* остаётся NaN — пиксель не заполнен */

			/* ★ кэш на последний треугольник: обход идёт по строкам, и соседние
			 * пиксели почти всегда принадлежат одной грани. */
			obi_index = pa->recto[idx];
			if (facenr != cache_face || obi_index != cache_obi) {
				tri_ok = raster_gbuffer_prepare(rl, re->objectinstance + obi_index,
				                                facenr, &tri) ? 1 : 0;
				cache_face = facenr;
				cache_obi = obi_index;
			}

			if (!tri_ok) continue;

			raster_gbuffer_pixel(re, &tri, pa->rectz[idx],
			                     x + 0.5f, y + 0.5f,
			                     gb->co + 3 * idx, gb->vn + 3 * idx);
		}
	}
}
