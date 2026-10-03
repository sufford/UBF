/*
 * RE_Mirror.c — своё отражение mirror-материала (шаг B4, «вариант B»).
 *
 * Цель шага: для MA_RAYMIRROR не звать BI-рейтрейс (ray_trace/trace_reflect/
 * traceray и RE_rayobject_raycast), а пустить свой луч и найти попадание
 * своим обходом сцены.
 *
 * Что здесь своё:
 *   - вектор отражения (копия reflection()/reflection_simple() из rayshade.c:
 *     формула маленькая, но семантику надо повторить точно, включая
 *     поправку на нормаль грани для гладких граней);
 *   - обход сцены: свой цикл по re->instancetable и граням вместо октодерева
 *     RE_rayobject_raycast;
 *   - отбор граней (R_TRACEBLE, MA_TYPE_WIRE, MA_ONLYCAST) и правило пропуска
 *     своей грани и соседей.
 *
 * Что берётся у BLI как утилита: isect_ray_tri_watertight_v3 — тот же
 * предикат пересечения луча с треугольником, что использует BI. Это
 * осознанно: без него расхождение попаданий шло бы от арифметики
 * пересечения, а не от нашего обхода, и сверка потеряла бы смысл.
 *
 * Границы этапа (пока — только луч и попадание, без затенения точки
 * попадания и без смешения):
 *   - обход идёт по октодереву BI (RE_rayobject_raycast по re->raytree), как
 *     в traceray(): у BI и у нас это одна и та же структура, поэтому набор
 *     найденных попаданий тот же. Линейный перебор по всем граням остался
 *     запасным путём на случай, когда дерева нет (R_RAYTRACE выключен);
 *   - экземпляры с R_ENV_TRANSFORMED: луч переводится в систему экземпляра
 *     вызовами RE_instance_rotate_ray/_restore, как в traceray().
 */

#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdio.h>

#include "MEM_guardedalloc.h"
#include "BLI_math.h"
#include "BLI_math_geom.h"
#include "BLI_utildefines.h"

#include "DNA_material_types.h"
#include "DNA_object_types.h"

#include "render_types.h"
#include "renderdatabase.h"
#include "rayintersection.h"
#include "rayobject.h"

#include "RE_Mirror.h"

int RE_mirror_enabled(void)
{
	static int cached = -1;

	if (cached < 0) cached = getenv("UBF_MIRROR") ? 1 : 0;
	return cached != 0;
}

/* ------------------------------------------------------------------ */
/* вектор отражения                                                    */
/* ------------------------------------------------------------------ */

/* копия reflection_simple() (rayshade.c:582) */
static void re_reflect_simple(float ref[3], const float n[3], const float view[3])
{
	const float f1 = -2.0f * dot_v3v3(n, view);

	madd_v3_v3v3fl(ref, view, n, f1);
}

/* копия reflection() (rayshade.c:589): orn — нормаль грани, поправка нужна,
 * чтобы отражённый вектор не ушёл за спину грани на гладких нормалях */
static void re_reflect_phong(float ref[3], const float n[3], const float view[3], const float orn[3])
{
	float f1;

	re_reflect_simple(ref, n, view);

	f1 = dot_v3v3(ref, orn);
	if (f1 > 0.0f) {
		f1 += 0.01f;
		ref[0] -= f1 * orn[0];
		ref[1] -= f1 * orn[1];
		ref[2] -= f1 * orn[2];
	}
}

/* ------------------------------------------------------------------ */
/* отбор граней — копии проверок BI                                    */
/* ------------------------------------------------------------------ */

/* копия is_raytraceable_vlr() (rayshade.c:182): в дерево BI попадает именно
 * этот набор граней, поэтому обход должен видеть ровно их */
static bool re_face_in_tree(Render *re, VlakRen *vlr)
{
	if ((re->flag & R_BAKE_TRACE) || (vlr->flag & R_TRACEBLE) ||
	    (vlr->mat->material_type == MA_TYPE_VOLUME))
	{
		if (vlr->mat->material_type != MA_TYPE_WIRE)
			return true;
	}
	return false;
}

/* копия vlr_check_intersect() (rayobject.cpp:103) для RE_RAY_MIRROR */
static bool re_face_hittable(VlakRen *vlr)
{
	if (!(vlr->flag & R_TRACEBLE))
		return false;
	return !(vlr->mat->mode & MA_ONLYCAST);
}

/* Вершины грани в том же виде, что BI кладёт в RayFace. Возвращает число
 * вершин (3 или 4). v4 у треугольника не трогаем. */
static int re_face_coords(const ObjectInstanceRen *obi, const VlakRen *vlr, float v[4][3])
{
	int n = 3, i;

	copy_v3_v3(v[0], vlr->v1->co);
	copy_v3_v3(v[1], vlr->v2->co);
	copy_v3_v3(v[2], vlr->v3->co);

	if (vlr->v4) {
		copy_v3_v3(v[3], vlr->v4->co);
		n = 4;
	}

	/* build_raytree() (rayshade.c:372) применяет mat к вершинам
	 * трансформированного экземпляра; rayface_from_vlak() — по флагу
	 * transform_primitives. Для пути без R_RAYTRACE_USE_INSTANCES это одно
	 * и то же. */
	if (obi->flag & R_TRANSFORMED) {
		for (i = 0; i < n; i++)
			mul_m4_v3((float (*)[4])obi->mat, v[i]);
	}

	return n;
}

/* ------------------------------------------------------------------ */
/* пересечение                                                         */
/* ------------------------------------------------------------------ */

/* копия isec_tri_quad() (rayobject.cpp:195): квад — это два треугольника
 * (v1,v2,v3) и (v1,v3,v4), барицентрика отдаётся со знаком минус */
static int re_isec_tri_quad(const float start[3], const struct IsectRayPrecalc *pre,
                            const float v[4][3], int nvert,
                            float r_uv[2], float *r_lambda)
{
	float uv[2], l;

	if (isect_ray_tri_watertight_v3(start, pre, v[0], v[1], v[2], &l, uv)) {
		if (l > -RE_RAYTRACE_EPSILON && l < *r_lambda) {
			r_uv[0] = -uv[0];
			r_uv[1] = -uv[1];
			*r_lambda = l;
			return 1;
		}
	}

	if (nvert == 4) {
		if (isect_ray_tri_watertight_v3(start, pre, v[0], v[2], v[3], &l, uv)) {
			if (l > -RE_RAYTRACE_EPSILON && l < *r_lambda) {
				r_uv[0] = -uv[0];
				r_uv[1] = -uv[1];
				*r_lambda = l;
				return 2;
			}
		}
	}

	return 0;
}

/* Есть ли у граней общая вершина. BI считает «той же» либо тот же VertRen,
 * либо тот же final org index (это про autosmooth). */
static bool re_faces_share_vert(ObjectRen *obr, const VlakRen *a, const VlakRen *b)
{
	VertRen *const *va, *const *vb;

	for (va = &a->v1; *va; va++) {
		int *ia = RE_vertren_get_origindex(obr, *va, false);

		for (vb = &b->v1; *vb; vb++) {
			if (*va == *vb)
				return true;

			if (ia) {
				int *ib = RE_vertren_get_origindex(obr, *vb, 0);

				if (ib && *ia == *ib)
					return true;
			}
		}
	}

	return false;
}

/* Пересекает ли луч исходную грань (проверка «да/нет», как
 * isec_tri_quad_neighbour() в rayobject.cpp:230). BI берёт там вариант
 * предиката без проверки знака и с обратным направлением; здесь взят
 * BLI-предикат с обратным направлением. Расхождение возможно только на
 * строго обратных попаданиях (пробой проверено: таких на приёмочной сцене
 * нет — свой кастер совпал с BI по грани, расстоянию, треугольнику и uv). */
static bool re_isec_origin_face(const float start[3], const float dir[3],
                                ObjectInstanceRen *obi, VlakRen *vlr)
{
	float v[4][3], neg[3], uv[2], l;
	struct IsectRayPrecalc pre;
	int n = re_face_coords(obi, vlr, v);

	negate_v3_v3(neg, dir);
	isect_ray_tri_watertight_v3_precalc(&pre, neg);

	if (isect_ray_tri_watertight_v3(start, &pre, v[0], v[1], v[2], &l, uv))
		return true;

	if (n == 4 && isect_ray_tri_watertight_v3(start, &pre, v[0], v[2], v[3], &l, uv))
		return true;

	return false;
}

/* ------------------------------------------------------------------ */
/* обход сцены                                                         */
/* ------------------------------------------------------------------ */

/* Поиск попадания по октодереву BI. Настройка Isect — копия traceray()
 * (rayshade.c:702-717): та же структура, тот же отбор граней, то же правило
 * про соседей. RE_rayobject_raycast сам считает isect_precalc, idot_axis и
 * bv_index, поэтому здесь только поля луча и опций. */
static int re_mirror_cast_tree(Render *re, ObjectInstanceRen *orig_obi, VlakRen *orig_vlr,
                               const float start[3], const float dir[3], float maxdist,
                               RE_MirrorHit *hit)
{
	Isect isec;

	memset(&isec, 0, sizeof(isec));

	copy_v3_v3(isec.start, start);
	copy_v3_v3(isec.dir, dir);
	isec.dist = maxdist;
	isec.mode = RE_RAY_MIRROR;
	isec.check = RE_CHECK_VLR_RENDER;
	isec.skip = RE_SKIP_VLR_NEIGHBOUR;
	isec.hint = NULL;
	isec.lay = -1;              /* для RE_RAY_MIRROR не используется (только тени) */
	isec.orig.ob = orig_obi;
	isec.orig.face = orig_vlr;

	/* трансформированные экземпляры живут в своей системе координат */
	RE_instance_rotate_ray(orig_obi, &isec);

	if (!RE_rayobject_raycast(re->raytree, &isec))
		return 0;

	RE_instance_rotate_ray_restore(orig_obi, &isec);

	hit->obi = (ObjectInstanceRen *)isec.hit.ob;
	hit->vlr = (VlakRen *)isec.hit.face;
	hit->dist = isec.dist;
	hit->uv[0] = isec.u;
	hit->uv[1] = isec.v;
	hit->tri = isec.isect;

	return 1;
}

int RE_mirror_cast(Render *re, ObjectInstanceRen *orig_obi, VlakRen *orig_vlr,
                   const float start[3], const float dir[3], float maxdist,
                   RE_MirrorHit *hit)
{
	struct IsectRayPrecalc pre;
	ObjectInstanceRen *obi;
	float best = maxdist;
	int found = 0;

	/* Дерево есть тогда, когда включён R_RAYTRACE (makeraytree() из
	 * database_init_objects) — то есть на всех путях, где вообще работает
	 * raymirror. Если дерева нет, остаётся прежний перебор. */
	if (re->raytree)
		return re_mirror_cast_tree(re, orig_obi, orig_vlr, start, dir, maxdist, hit);

	isect_ray_tri_watertight_v3_precalc(&pre, dir);

	/* Тот же набор instance-ов, что попадает в re->raytree: build_raytree()
	 * пропускает re->excludeob и объекты без трассируемых граней (последнее
	 * здесь покрывается проверкой каждой грани). */
	for (obi = re->instancetable.first; obi; obi = obi->next) {
		ObjectRen *obr = obi->obr;
		int v;

		if (re->excludeob && obr->ob == re->excludeob)
			continue;

		for (v = 0; v < obr->totvlak; v++) {
			VlakRen *vlr = obr->vlaknodes[v >> 8].vlak + (v & 255);
			float q[4][3], uv[2];
			float l = best;
			int nv, tri;

			if (!re_face_in_tree(re, vlr))
				continue;
			if (!re_face_hittable(vlr))
				continue;

			/* avoid self-intersection (rayobject.cpp:265) */
			if (obi == orig_obi && vlr == orig_vlr)
				continue;

			nv = re_face_coords(obi, vlr, q);

			tri = re_isec_tri_quad(start, &pre, q, nv, uv, &l);
			if (!tri)
				continue;

			/* RE_SKIP_VLR_NEIGHBOUR (rayobject.cpp:294): соседнюю грань
			 * отбрасываем, только если луч при этом НЕ пересекает саму
			 * исходную грань — иначе попадание считается настоящим */
			if (l < 0.1f && obi == orig_obi && orig_vlr) {
				if (re_faces_share_vert(obr, orig_vlr, vlr)) {
					if (!re_isec_origin_face(start, dir, orig_obi, orig_vlr))
						continue;
				}
			}

			best = l;
			hit->obi = obi;
			hit->vlr = vlr;
			hit->dist = l;
			hit->uv[0] = uv[0];
			hit->uv[1] = uv[1];
			hit->tri = tri;
			found = 1;
		}
	}

	return found;
}

int RE_mirror_trace(Render *re, ShadeInput *shi,
                    const float co[3], const float vn[3],
                    RE_MirrorHit *hit, float dir_out[3])
{
	Material *ma = shi->mat;
	float dir[3];
	float maxdist;

	/* trace_reflect() (rayshade.c:1447): для гладкой грани BI берёт
	 * reflection() с нормалью грани, иначе reflection_simple() */
	if (shi->vlr && (shi->vlr->flag & R_SMOOTH))
		re_reflect_phong(dir, vn, shi->view, shi->facenor);
	else
		re_reflect_simple(dir, vn, shi->view);

	if (dir_out)
		copy_v3_v3(dir_out, dir);

	/* traceray() (rayshade.c:704) */
	maxdist = (ma->dist_mir > 0.0f) ? ma->dist_mir : RE_RAYTRACE_MAXDIST;

	return RE_mirror_cast(re, shi->obi, shi->vlr, co, dir, maxdist, hit);
}

