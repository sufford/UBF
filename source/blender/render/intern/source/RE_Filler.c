/*
 * RE_Filler.c — свой span-филлер (шаг B2, «вариант B»).
 *
 * Заменяет zbuffillGL4() из zbuf.c на пути растеризатора: свой обход рёбер
 * (zbuf_add_to_span), свой расчёт плоскости z в экранных координатах и свой
 * тест глубины в rectz/rectp/recto.
 *
 * Почему это порт семантики, а не «как правильно»: результат работы филлера
 * — это rectp/recto/rectz, и приёмка требует их побитового совпадения с BI.
 * Всё, что видно в этих буферах, нужно повторить точно:
 *   - my0 = ceil(miny), my2 = floor(maxy) — границы строк берутся именно
 *     так, а не округлением к ближайшему;
 *   - выбор левого/правого span по СРАВНЕНИЮ УКАЗАТЕЛЕЙ вершин
 *     (maxv == zspan->minp1), поэтому порядок обхода рёбер (v1-v2, v2-v3,
 *     v3-v1) обязан быть тем же: перестановка меняет, какой span станет
 *     первым, а значит и результат на вырожденных гранях;
 *   - z считается в double по уравнению плоскости, построенному из экранных
 *     координат, и округляется round_db_to_int_clamp — не float-суммой;
 *   - тест глубины строгий (intzverg < *rz), равная глубина НЕ перезаписывает
 *     пиксель: tie-break зависит от порядка отрисовки, и «починить» его
 *     значит разойтись с BI.
 *
 * Интерполяция атрибутов (co/vn) перспективно-корректно прямо здесь —
 * следующий шаг B2-ii; пока филлер заполняет только z/покрытие, а co/vn
 * восстанавливаются отдельным проходом (шаг 3a).
 */

#include <math.h>
#include <string.h>
#include <stdio.h>
#include <float.h>
#include <limits.h>

#include "MEM_guardedalloc.h"
#include "BLI_math.h"
#include "BLI_utildefines.h"

#include "render_types.h"
#include "zbuf.h"           /* ZSpan */
#include "RE_Rasterizer.h"
#include "RE_Prof.h"        /* ★ PROF */

/* Сброс диапазона строк для клиппинга. */
static void re_init_span(ZSpan *zspan)
{
	zspan->miny1 = zspan->miny2 = zspan->recty + 1;
	zspan->maxy1 = zspan->maxy2 = -1;
	zspan->minp1 = zspan->maxp1 = zspan->minp2 = zspan->maxp2 = NULL;
}

/*
 * Ребро -> буфер спанов: для каждой строки записывается x-координата ребра.
 * Два ребра (левое и правое) пишутся в разные массивы span1/span2; какой из
 * них какой — определяется сравнением указателей на вершины, поэтому
 * вызывающий обязан передавать те же указатели, что и BI.
 */
static void re_add_to_span(ZSpan *zspan, const float v1[2], const float v2[2])
{
	const float *minv, *maxv;
	float *span;
	float xx1, dx0, xs0;
	int y, my0, my2;

	if (v1[1] < v2[1]) {
		minv = v1;
		maxv = v2;
	}
	else {
		minv = v2;
		maxv = v1;
	}

	my0 = (int)ceil(minv[1]);
	my2 = (int)floor(maxv[1]);

	if (my2 < 0 || my0 >= zspan->recty) return;

	/* зажимаем по вертикали */
	if (my2 >= zspan->recty) my2 = zspan->recty - 1;
	if (my0 < 0) my0 = 0;

	if (my0 > my2) return;

	xx1 = maxv[1] - minv[1];
	if (xx1 > FLT_EPSILON) {
		dx0 = (minv[0] - maxv[0]) / xx1;
		xs0 = dx0 * (minv[1] - my2) + minv[0];
	}
	else {
		dx0 = 0.0f;
		xs0 = min_ff(minv[0], maxv[0]);
	}

	/* какой массив спанов: пустой — span1; иначе ребро дополняет левый span,
	 * если совпало по указателю с его крайними вершинами */
	if (zspan->maxp1 == NULL) {
		span = zspan->span1;
	}
	else {
		if (maxv == zspan->minp1 || minv == zspan->maxp1) {
			span = zspan->span1;
		}
		else {
			span = zspan->span2;
		}
	}

	if (span == zspan->span1) {
		if (zspan->minp1 == NULL || zspan->minp1[1] > minv[1]) zspan->minp1 = minv;
		if (zspan->maxp1 == NULL || zspan->maxp1[1] < maxv[1]) zspan->maxp1 = maxv;
		if (my0 < zspan->miny1) zspan->miny1 = my0;
		if (my2 > zspan->maxy1) zspan->maxy1 = my2;
	}
	else {
		if (zspan->minp2 == NULL || zspan->minp2[1] > minv[1]) zspan->minp2 = minv;
		if (zspan->maxp2 == NULL || zspan->maxp2[1] < maxv[1]) zspan->maxp2 = maxv;
		if (my0 < zspan->miny2) zspan->miny2 = my0;
		if (my2 > zspan->maxy2) zspan->maxy2 = my2;
	}

	for (y = my2; y >= my0; y--, xs0 += dx0) {
		span[y] = xs0;
	}
}

/*
 * Треугольник -> пиксели. v4 не NULL только для квада (в нашем пути всегда
 * NULL: квады раскалываются вызывающим кодом).
 */
void RE_fill_triangle(ZSpan *zspan, int obi, int zvlnr,
                      const float *v1, const float *v2, const float *v3,
                      const float *v4, const RE_RasterAttr *attr)
{
	double zxd, zyd, zy0, zverg;
	float x0, y0, z0;
	float x1, y1, z1, x2, y2, z2, xx1;
	const float *span1, *span2;
	int *rectoofs, *ro;
	int *rectpofs, *rp;
	const int *rectmaskofs, *rm;
	int *rz, x, y, col;
	int sn1, sn2, rectx, *rectzofs, my0, my2;
	RE_RasterAttrTri tri;                     /* ★ этап 2: атрибуты на треугольник */
	int tri_ok = 0;
	int reached = 0;                          /* ★ PROF */
	double t_fill0 = RE_prof_tick();          /* ★ PROF */

	RE_prof_count(RE_PROF_C_FILLS, 1);        /* ★ PROF */

	re_init_span(zspan);

	re_add_to_span(zspan, v1, v2);
	re_add_to_span(zspan, v2, v3);
	if (v4) {
		re_add_to_span(zspan, v3, v4);
		re_add_to_span(zspan, v4, v1);
	}
	else {
		re_add_to_span(zspan, v3, v1);
	}

	/* второй span не набрался — заполнять нечего */
	if (zspan->minp2 == NULL || zspan->maxp2 == NULL) goto done;

	my0 = max_ii(zspan->miny1, zspan->miny2);
	my2 = min_ii(zspan->maxy1, zspan->maxy2);

	if (my2 < my0) goto done;

	/* уравнение плоскости z(x,y) по экранным координатам */
	x1 = v1[0] - v2[0];
	x2 = v2[0] - v3[0];
	y1 = v1[1] - v2[1];
	y2 = v2[1] - v3[1];
	z1 = v1[2] - v2[2];
	z2 = v2[2] - v3[2];
	x0 = y1 * z2 - z1 * y2;
	y0 = z1 * x2 - x1 * z2;
	z0 = x1 * y2 - y1 * x2;

	if (z0 == 0.0f) goto done;

	reached = 1;   /* ★ PROF: спаны непусты, пиксельный цикл пойдёт */

	/* ★ этап 2, «diffuse»: атрибуты треугольника готовятся ОДИН раз на вызов
	 * филлера, а не на каждый пиксель. Внутри одного вызова obi и zvlnr
	 * постоянны, поэтому кэш не нужен. Вызовов ~42k против ~749k пиксельных
	 * вычислений — по замерам это была самая дорогая часть растеризатора. */
	tri_ok = RE_raster_attr_prepare(attr, obi, zvlnr, &tri) ? 1 : 0;

	xx1 = (x0 * v1[0] + y0 * v1[1]) / z0 + v1[2];

	zxd = -(double)x0 / (double)z0;
	zyd = -(double)y0 / (double)z0;
	zy0 = ((double)my2) * zyd + (double)xx1;

	rectx = zspan->rectx;
	rectzofs = (zspan->rectz + rectx * my2);
	rectpofs = (zspan->rectp + rectx * my2);
	rectoofs = (zspan->recto + rectx * my2);
	rectmaskofs = (zspan->rectmask + rectx * my2);

	/* какой из массивов спанов левый — по значению в строке my0
	 * (не по индексу 0: вне [my0..my2] спаны не заполнены) */
	sn1 = my0;
	if (zspan->span1[sn1] < zspan->span2[sn1]) {
		span1 = zspan->span1 + my2;
		span2 = zspan->span2 + my2;
	}
	else {
		span1 = zspan->span2 + my2;
		span2 = zspan->span1 + my2;
	}

	for (y = my2; y >= my0; y--, span1--, span2--) {
		sn1 = (int)floor(*span1);
		sn2 = (int)floor(*span2);
		sn1++;

		if (sn2 >= rectx) sn2 = rectx - 1;
		if (sn1 < 0) sn1 = 0;

		if (sn2 >= sn1) {
			int intzverg;

			zverg = (double)sn1 * zxd + zy0;
			rz = rectzofs + sn1;
			rp = rectpofs + sn1;
			ro = rectoofs + sn1;
			rm = rectmaskofs + sn1;
			x = sn2 - sn1;
			col = sn1;

			while (x >= 0) {
				intzverg = round_db_to_int_clamp(zverg);

				if (intzverg < *rz) {   /* строго: равная глубина не перезаписывает */
					if (!zspan->rectmask || intzverg > *rm) {
						*rz = intzverg;
						*rp = zvlnr;
						*ro = obi;
						RE_prof_count(RE_PROF_C_PIXELS, 1);   /* ★ PROF */

						/* ★ ШАГ B2-ii: атрибуты считаем здесь же, пока
						 * известен выигравший пиксель, — отдельный проход
						 * по прямоугольнику после растеризации не нужен.
						 * Для пикселей, которые позже перезапишет другой
						 * треугольник, значение будет пересчитано: итог
						 * тот же, что читал бы проход из rectp/recto.
						 * tri подготовлен выше — на треугольник, не на пиксель. */
						if (tri_ok)
							RE_raster_attr_pixel(attr, &tri, col, y, intzverg);
					}
				}
				zverg += zxd;
				rz++;
				rp++;
				ro++;
				rm++;
				col++;
				x--;
			}
		}

		zy0 -= zyd;
		rectzofs -= rectx;
		rectpofs -= rectx;
		rectoofs -= rectx;
		rectmaskofs -= rectx;
	}

done:                                          /* ★ PROF */
	if (!reached) RE_prof_count(RE_PROF_C_FILLS_EMPTY, 1);
	RE_prof_span(RE_PROF_FILL, t_fill0);
}
