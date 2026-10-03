/*
 * RE_Clipper.c — свой клиппер (шаг B1, «вариант B»).
 *
 * Заменяет zbufclip()/zbufclip4() из zbuf.c. Своего здесь — весь код: тесты
 * клиппинга, расчёт lambda, построение новых вершин, порядок выходных
 * треугольников. zbuffillGL4() пока остаётся (это шаг B2).
 *
 * Почему семантика воспроизводится точно, а не «примерно как надо»:
 * порядок, в котором треугольники уходят в филлер, влияет на tie-break
 * глубины при равных z. Любое расхождение в порядке или в арифметике lambda
 * даёт другое изображение на рёбрах, поэтому цель шага — побитовое
 * совпадение покрытия и глубины (rectp/recto/rectz) с путём BI.
 *
 * Структура повторяет BI: клиппинг идёт тремя батчами (сначала z, затем x,
 * затем y), причём после z-батча флаги для x/y пересчитываются по уже
 * отрезанным вершинам. Быстрый путь (флагов нет вообще) не копирует
 * вершины, а сразу делит их перспективно и отдаёт филлеру.
 *
 * Квады сюда не приходят: вызывающий раскалывает квад на два треугольника
 * (v1,v2,v3) и (v1,v3,v4) со битом RE_QUAD_OFFS на втором — так же, как это
 * делает zbuffer_solid() в BI. Поэтому на выходе всегда треугольники.
 */

#include <math.h>
#include <string.h>
#include <float.h>

#include "MEM_guardedalloc.h"
#include "BLI_math.h"
#include "BLI_utildefines.h"

#include "render_types.h"
#include "zbuf.h"          /* только определение ZSpan и RE_QUAD_OFFS */
#include "RE_Rasterizer.h"
#include "RE_Prof.h"       /* ★ PROF */

/* Порядок битов — как в BI, чтобы флаги были взаимозаменяемы: */
#define RE_CLIP_XMIN  1
#define RE_CLIP_XMAX  2
#define RE_CLIP_YMAX  4
#define RE_CLIP_YMIN  8
#define RE_CLIP_ZMIN 16
#define RE_CLIP_ZMAX 32

/*
 * Тест границы пирамиды видимости, вариант Liang&Barsky.
 * Обязан совпадать с testclip() по решениям, иначе клиппер и тест флагов
 * начнут расходиться на вершинах, лежащих ровно на границе (BI об этом
 * предупреждает прямо в комментарии к testclip).
 */
static int re_cliptestf(float a, float b, float c, float d, float *u1, float *u2)
{
	float p = a + b, q = c + d, r;

	if (p < 0.0f) {
		if (q < p) return 0;
		else if (q < 0.0f) {
			r = q / p;
			if (r > *u2) return 0;
			else if (r > *u1) *u1 = r;
		}
	}
	else {
		if (p > 0.0f) {
			if (q < 0.0f) return 0;
			else if (q < p) {
				r = q / p;
				if (r < *u1) return 0;
				else if (r < *u2) *u2 = r;
			}
		}
		else if (q < 0.0f) return 0;
	}
	return 1;
}

/*
 * Флаги клиппинга вершины в HCS. abs4 = |w| + FLT_EPSILON, а не |w|: вершина
 * ровно на границе не должна получать флаг (иначе клиппинг срабатывает на
 * геометрии, которая и так внутри, и покрытие уезжает).
 */
static int re_testclip(const float v[4])
{
	const float abs4 = fabsf(v[3]) + FLT_EPSILON;
	int c = 0;

	if (v[0] < -abs4) c += RE_CLIP_XMIN;
	else if (v[0] > abs4) c += RE_CLIP_XMAX;

	if (v[1] > abs4) c += RE_CLIP_YMAX;
	else if (v[1] < -abs4) c += RE_CLIP_YMIN;

	if (v[2] < -abs4) c += RE_CLIP_ZMIN;
	else if (v[2] > abs4) c += RE_CLIP_ZMAX;

	return c;
}

/* Публичная обёртка: вызывающий код считает флаги клиппинга сам, ДО вызова
 * RE_clip_triangle(). Свой тест, чтобы на пути растеризатора не оставалось
 * обращений к testclip() из zbuf.c. */
int RE_clip_test(const float v[4])
{
	return re_testclip(v);
}

/* HCS -> экранные координаты (перспективное деление). */
static void re_hoco_to_zco(struct ZSpan *zspan, float zco[3], const float hoco[4])
{
	const float div = 1.0f / hoco[3];

	zco[0] = zspan->zmulx * (1.0f + hoco[0] * div) + zspan->zofsx;
	zco[1] = zspan->zmuly * (1.0f + hoco[1] * div) + zspan->zofsy;
	zco[2] = 0x7FFFFFFF * (hoco[2] * div);
}

/*
 * Отрезок v1->v2 против одной пары плоскостей (a: 0=x, 1=y, 2=z).
 * lambda[0]/lambda[1] — параметры входа/выхода; -1.0 означает «граница не
 * пересечена». b2 — «что-то отрезано», b3 — «отрезок вообще виден».
 */
static void re_clippyra(float *lambda, float *v1, float *v2, int *b2, int *b3,
                        int a, float clipcrop)
{
	float da, dw, u1 = 0.0f, u2 = 1.0f;
	float v13;

	lambda[0] = -1.0f;
	lambda[1] = -1.0f;

	da = v2[a] - v1[a];
	/* по x/y клиппинг идёт против clipcrop*w, а не против w: BI расширяет
	 * пирамиду, чтобы OSA-сэмплы на краях не теряли геометрию */
	if (a == 2) {
		dw = (v2[3] - v1[3]);
		v13 = v1[3];
	}
	else {
		dw = clipcrop * (v2[3] - v1[3]);
		v13 = clipcrop * v1[3];
	}

	if (re_cliptestf(-da, -dw, v13, v1[a], &u1, &u2)) {
		if (re_cliptestf(da, -dw, v13, -v1[a], &u1, &u2)) {
			*b3 = 1;
			if (u2 < 1.0f) {
				lambda[1] = u2;
				*b2 = 1;
			}
			else lambda[1] = 1.0;
			if (u1 > 0.0f) {
				lambda[0] = u1;
				*b2 = 1;
			}
			else {
				lambda[0] = 0.0;
			}
		}
	}
}

/*
 * Кладёт в trias[] точки отрезка v1->v2, попавшие внутрь.
 * Новая вершина выделяется из общего пула vez по счётчику *clve —
 * поэтому пул и счётчик общие на весь клиппинг.
 * l1 != 0 означает, что точка входа не совпала с v1 (иначе переиспользуем v1);
 * l2 != 1 — что точка выхода не совпала с v2 (иначе v2 не добавляется).
 */
static void re_makevertpyra(float *vez, float *lambda, float **trias,
                            float *v1, float *v2, int *b1, int *clve)
{
	const float l1 = lambda[0];
	const float l2 = lambda[1];

	if (l1 != -1.0f) {
		if (l1 != 0.0f) {
			float *adr = vez + 4 * (*clve);
			trias[*b1] = adr;
			(*clve)++;
			adr[0] = v1[0] + l1 * (v2[0] - v1[0]);
			adr[1] = v1[1] + l1 * (v2[1] - v1[1]);
			adr[2] = v1[2] + l1 * (v2[2] - v1[2]);
			adr[3] = v1[3] + l1 * (v2[3] - v1[3]);
		}
		else trias[*b1] = v1;

		(*b1)++;
	}
	if (l2 != -1.0f) {
		if (l2 != 1.0f) {
			float *adr = vez + 4 * (*clve);
			trias[*b1] = adr;
			(*clve)++;
			adr[0] = v1[0] + l2 * (v2[0] - v1[0]);
			adr[1] = v1[1] + l2 * (v2[1] - v1[1]);
			adr[2] = v1[2] + l2 * (v2[2] - v1[2]);
			adr[3] = v1[3] + l2 * (v2[3] - v1[3]);
			(*b1)++;
		}
	}
}

/*
 * Возвращает 1, если сработала ветка клиппинга, и 0 для быстрого пути.
 * Нужно вызывающему для счётчика: без него «картинка совпала с BI» ничего
 * не доказывает — на сцене без геометрии на границах пирамиды клиппер
 * может вообще не попасть в клиппинг, и совпадение будет пустым.
 */
static int re_clip_triangle_impl(struct ZSpan *zspan, int obi, int zvlnr,
                          const float f1[4], const float f2[4], const float f3[4],
                          int c1, int c2, int c3,
                          RE_ClipFillFunc fill)
{
	/* vez — пул вершин (100 штук), trias — до 40 ссылок на них,
	 * vlzp — до 31 полигона по 3 ссылки */
	float vez[400];
	float *vlzp[32][3];
	float *trias[40];
	float lambda[3][2];

	if (c1 | c2 | c3) {
		int arg, v, b, clipflag[3], b1, b2, b3, c4;
		int clve = 3, clvlo, clvl = 1;
		float *fp;

		/* целиком снаружи — треугольник не рисуется вовсе */
		if (c1 & c2 & c3) return 1;

		vez[0] = f1[0]; vez[1] = f1[1]; vez[2] = f1[2];  vez[3] = f1[3];
		vez[4] = f2[0]; vez[5] = f2[1]; vez[6] = f2[2];  vez[7] = f2[3];
		vez[8] = f3[0]; vez[9] = f3[1]; vez[10] = f3[2]; vez[11] = f3[3];

		vlzp[0][0] = vez;
		vlzp[0][1] = vez + 4;
		vlzp[0][2] = vez + 8;

		/* Сначала z. Если z-батч что-то отрезал, флаги x/y придётся
		 * считать заново — по новым вершинам, они появятся ниже. */
		clipflag[0] = ((c1 & 48) | (c2 & 48) | (c3 & 48));
		if (clipflag[0] == 0) {
			clipflag[1] = ((c1 & 3) | (c2 & 3) | (c3 & 3));
			clipflag[2] = ((c1 & 12) | (c2 & 12) | (c3 & 12));
		}
		else clipflag[1] = clipflag[2] = 0;

		for (b = 0; b < 3; b++) {
			if (!clipflag[b]) continue;

			clvlo = clvl;   /* фиксируем: полигоны, добавленные в этом батче,
			                 * в нём же не обрабатываются */

			for (v = 0; v < clvlo; v++) {
				if (vlzp[v][0] == NULL) continue;   /* полигон уже снят */

				b2 = b3 = 0;

				if (b == 0) arg = 2;        /* z */
				else if (b == 1) arg = 0;   /* x */
				else arg = 1;               /* y */

				re_clippyra(lambda[0], vlzp[v][0], vlzp[v][1], &b2, &b3, arg, zspan->clipcrop);
				re_clippyra(lambda[1], vlzp[v][1], vlzp[v][2], &b2, &b3, arg, zspan->clipcrop);
				re_clippyra(lambda[2], vlzp[v][2], vlzp[v][0], &b2, &b3, arg, zspan->clipcrop);

				if (b2 == 0 && b3 == 1) {
					/* полигон целиком внутри: переносим его в конец списка
					 * (копируем, а не оставляем на месте — индекс 0 в выдаче
					 * не участвует) */
					vlzp[clvl][0] = vlzp[v][0];
					vlzp[clvl][1] = vlzp[v][1];
					vlzp[clvl][2] = vlzp[v][2];
					vlzp[v][0] = NULL;
					clvl++;
				}
				else if (b3 == 0) {
					vlzp[v][0] = NULL;   /* полигон целиком снаружи */
				}
				else {
					b1 = 0;
					re_makevertpyra(vez, lambda[0], trias, vlzp[v][0], vlzp[v][1], &b1, &clve);
					re_makevertpyra(vez, lambda[1], trias, vlzp[v][1], vlzp[v][2], &b1, &clve);
					re_makevertpyra(vez, lambda[2], trias, vlzp[v][2], vlzp[v][0], &b1, &clve);

					/* после z-батча пересчитываем флаги x/y по новым вершинам */
					if (b == 0) {
						clipflag[1] = clipflag[2] = 0;
						fp = vez;
						for (b3 = 0; b3 < clve; b3++) {
							c4 = re_testclip(fp);
							clipflag[1] |= (c4 & 3);
							clipflag[2] |= (c4 & 12);
							fp += 4;
						}
					}

					vlzp[v][0] = NULL;

					/* веер: (trias[0], trias[k-1], trias[k]) — порядок обхода
					 * сохраняем как у BI, он виден в tie-break глубины */
					if (b1 > 2) {
						for (b3 = 3; b3 <= b1; b3++) {
							vlzp[clvl][0] = trias[0];
							vlzp[clvl][1] = trias[b3 - 2];
							vlzp[clvl][2] = trias[b3 - 1];
							clvl++;
						}
					}
				}
			}
		}

		if (clve > 38 || clvl > 31) {
			/* переполнение клиппинга: у BI это BLI_assert. В релизе лучше
			 * не рисовать ничего, чем записать за границы массивов. */
			BLI_assert(0);
			return 1;
		}

		/* перспективное деление всех получившихся вершин */
		fp = vez;
		for (b = 0; b < clve; b++) {
			re_hoco_to_zco(zspan, fp, fp);
			fp += 4;
		}
		for (b = 1; b < clvl; b++) {
			if (vlzp[b][0])
				fill(zspan, obi, zvlnr, vlzp[b][0], vlzp[b][1], vlzp[b][2], NULL);
		}
		return 1;
	}

	/* быстрый путь: треугольник целиком внутри пирамиды */
	{
		float v1[4], v2[4], v3[4];

		re_hoco_to_zco(zspan, v1, f1);
		re_hoco_to_zco(zspan, v2, f2);
		re_hoco_to_zco(zspan, v3, f3);
		fill(zspan, obi, zvlnr, v1, v2, v3, NULL);
	}

	return 0;
}

/* ★ PROF — обёртка вокруг реализации: меряет клиппер и различает ветки
 * (быстрый путь / клиппинг / «целиком снаружи»). Реализация выше не тронута:
 * её математику приёмка проверяет побитово, и тело менять нельзя. */
int RE_clip_triangle(struct ZSpan *zspan, int obi, int zvlnr,
                     const float f1[4], const float f2[4], const float f3[4],
                     int c1, int c2, int c3,
                     RE_ClipFillFunc fill)
{
	int ret;
	double t0 = RE_prof_tick();

	ret = re_clip_triangle_impl(zspan, obi, zvlnr, f1, f2, f3, c1, c2, c3, fill);

	if (ret) {
		if (c1 & c2 & c3) RE_prof_count(RE_PROF_C_TRIS_OUT, 1);
		else              RE_prof_count(RE_PROF_C_TRIS_CLIPPED, 1);
	}
	else {
		RE_prof_count(RE_PROF_C_TRIS_FAST, 1);
	}

	RE_prof_span(RE_PROF_CLIP, t0);

	return ret;
}
