/*
 * RE_Prof.c — реализация ВРЕМЕННОЙ измерительной обвязки. См. RE_Prof.h.
 *
 * Включается только при UBF_PROF в окружении. Таймер — PIL_check_seconds_timer(),
 * тот же, которым pipeline.c меряет кадры, так что цифры сопоставимы с тем,
 * что Blender печатает сам.
 */

#include <stdio.h>
#include <stdlib.h>

#include "PIL_time.h"

#include "RE_Prof.h"

/* ------------------------------------------------------------------------- */

static int    prof_state = -1;   /* -1 не проверяли, 0 выкл, 1 вкл */
static int    prof_thread_warned = 0;
static int    prof_calibrated = 0;
/* Стоимость одной пары tick+span в наносекундах. Нужна, чтобы читатель мог
 * оценить вклад самой обвязки в per-pixel строки: например, attr меряется
 * двумя вызовами таймера на пиксель, то есть ~2*tick_ns*attrpx секунд из
 * показанного attr — это обвязка, а не работа растеризатора. */
static double prof_tick_ns = 0.0;

/* Аккумулятор текущей части; сбрасывается в part_begin. */
static double prof_acc[RE_PROF_NUM];
static long   prof_cacc[RE_PROF_C_NUM];

/* Итог по всем частям. */
static double prof_tot[RE_PROF_NUM];
static long   prof_ctot[RE_PROF_C_NUM];
/* Суммарное wall-clock время частей: слоты вложены друг в друга, поэтому
 * складывать их нельзя — эталон для сверки даёт только сумма частей. */
static double prof_tot_part_wall = 0.0;

/* Вне частей (build/visibility) — печатается отдельной строкой. */
static double prof_glob[RE_PROF_NUM];

static int    prof_part_no = 0;
static double prof_part_t0 = 0.0;

bool RE_prof_active(void)
{
	if (prof_state < 0) {
		const char *e = getenv("UBF_PROF");
		prof_state = (e && e[0] && e[0] != '0') ? 1 : 0;
	}
	return prof_state == 1;
}

/* ------------------------------------------------------------------------- */

double RE_prof_tick(void)
{
	return RE_prof_active() ? PIL_check_seconds_timer() : 0.0;
}

void RE_prof_span(int slot, double t0)
{
	if (!RE_prof_active()) return;
	prof_acc[slot] += PIL_check_seconds_timer() - t0;
}

void RE_prof_glob_span(int slot, double t0)
{
	if (!RE_prof_active()) return;
	prof_glob[slot] += PIL_check_seconds_timer() - t0;
}

/* ------------------------------------------------------------------------- */

void RE_prof_add(int slot, double dt)
{
	if (!RE_prof_active()) return;
	prof_acc[slot] += dt;
}

void RE_prof_add_global(int slot, double dt)
{
	if (!RE_prof_active()) return;
	prof_glob[slot] += dt;
}

void RE_prof_count(int counter, long n)
{
	if (!RE_prof_active()) return;
	prof_cacc[counter] += n;
}

void RE_prof_note_threads(int threads)
{
	if (!RE_prof_active()) return;
	if (threads > 1 && !prof_thread_warned) {
		prof_thread_warned = 1;
		printf("[PROF] ВНИМАНИЕ: потоков рендера %d, аккумуляторы не защищены — "
		       "цифры рваные. Для корректного замера ставьте UBF_T1=1.\n", threads);
	}
}

/* ------------------------------------------------------------------------- */

/* ★ PROF — калибровка: сколько стоит один RE_prof_tick(). Греется и меряется
 * на 200k вызовов; результат печатается в итоговой строке. Печатается один
 * раз за процесс, потому что цена вызова таймера от рендера не зависит. */
static void prof_calibrate(void)
{
	const int N = 200000;
	double t0, t1, acc = 0.0;
	int i;

	if (prof_calibrated) return;
	prof_calibrated = 1;

	for (i = 0; i < 1000; i++) acc += PIL_check_seconds_timer();   /* прогрев */

	t0 = PIL_check_seconds_timer();
	for (i = 0; i < N; i++) acc += RE_prof_tick();
	t1 = PIL_check_seconds_timer();

	prof_tick_ns = (t1 - t0) / (double)N * 1e9;
	if (acc == 0.0) printf("[PROF] (калибровка: сумма не должна быть нулевой)\n");
}

/* ------------------------------------------------------------------------- */

void RE_prof_part_begin(void)
{
	int i;

	if (!RE_prof_active()) return;

	prof_calibrate();

	for (i = 0; i < RE_PROF_NUM; i++) prof_acc[i] = 0.0;
	for (i = 0; i < RE_PROF_C_NUM; i++) prof_cacc[i] = 0;

	prof_part_t0 = PIL_check_seconds_timer();
}

void RE_prof_part_end(int xmin, int ymin, int rectx, int recty)
{
	double part;
	int i;

	if (!RE_prof_active()) return;

	part = PIL_check_seconds_timer() - prof_part_t0;
	prof_part_no++;
	prof_tot_part_wall += part;

	/* копим итог */
	for (i = 0; i < RE_PROF_NUM; i++) {
		if (RE_PROF_IS_GLOBAL(i)) continue;
		prof_tot[i] += prof_acc[i];
	}
	for (i = 0; i < RE_PROF_C_NUM; i++) prof_ctot[i] += prof_cacc[i];
	prof_ctot[RE_PROF_C_PARTS]++;

	/* ---- исключающие времена ---------------------------------------------
	 * Считаем так, чтобы строки складывались в PART и ни один вклад не был
	 * посчитан дважды. Проценты — от измеренного PART, не от prof_acc[PART]
	 * (его никто не заполняет: часть целиком меряется здесь же).
	 */
	{
		double t_span   = prof_acc[RE_PROF_SPAN];
		double t_scene  = prof_acc[RE_PROF_SCENE_LOOP];
		double t_proj   = prof_acc[RE_PROF_PROJECT];
		double t_clip   = prof_acc[RE_PROF_CLIP];
		double t_fill   = prof_acc[RE_PROF_FILL];
		double t_attr   = prof_acc[RE_PROF_ATTR];

		double ex_span  = t_span;
		double ex_slot  = t_scene - t_proj;      /* обход buckets/slots и bbox */
		double ex_proj  = t_proj - t_clip;       /* projectvert + тесты */
		double ex_clip  = t_clip - t_fill;       /* клиппер без филлера */
		double ex_fill  = t_fill - t_attr;       /* спаны и z-тест */
		double ex_attr  = t_attr;                /* интерполяция co/vn */
		double ex_other = part - (ex_span + ex_slot + ex_proj + ex_clip + ex_fill + ex_attr);

		double inv = (part > 0.0) ? (100.0 / part) : 0.0;

		printf("[PROF] part=%d xy=(%d,%d) rect=%dx%d total=%.4f ms"
		       " | span=%.4f (%.1f%%) slot=%.4f (%.1f%%) proj=%.4f (%.1f%%)"
		       " clip=%.4f (%.1f%%) fill=%.4f (%.1f%%) attr=%.4f (%.1f%%) other=%.4f (%.1f%%)\n",
		       prof_part_no, xmin, ymin, rectx, recty, part * 1000.0,
		       ex_span * 1000.0, ex_span * inv,
		       ex_slot * 1000.0, ex_slot * inv,
		       ex_proj * 1000.0, ex_proj * inv,
		       ex_clip * 1000.0, ex_clip * inv,
		       ex_fill * 1000.0, ex_fill * inv,
		       ex_attr * 1000.0, ex_attr * inv,
		       ex_other * 1000.0, ex_other * inv);

		printf("[PROF] part=%d counts slots=%ld culled=%ld drawn=%ld darrays=%ld"
		       " prims=%ld prim_culled=%ld tris_fast=%ld tris_clip=%ld tris_out=%ld"
		       " fills=%ld fills_empty=%ld px=%ld attrpx=%ld\n",
		       prof_part_no,
		       prof_cacc[RE_PROF_C_SLOTS_TESTED], prof_cacc[RE_PROF_C_SLOTS_CULLED],
		       prof_cacc[RE_PROF_C_SLOTS_DRAWN], prof_cacc[RE_PROF_C_DARRAYS],
		       prof_cacc[RE_PROF_C_PRIMS], prof_cacc[RE_PROF_C_PRIMS_PARTCLIP],
		       prof_cacc[RE_PROF_C_TRIS_FAST], prof_cacc[RE_PROF_C_TRIS_CLIPPED],
		       prof_cacc[RE_PROF_C_TRIS_OUT],
		       prof_cacc[RE_PROF_C_FILLS], prof_cacc[RE_PROF_C_FILLS_EMPTY],
		       prof_cacc[RE_PROF_C_PIXELS], prof_cacc[RE_PROF_C_ATTRPIX]);

		(void)t_span; (void)t_scene; (void)t_proj; (void)t_clip;
		(void)t_fill; (void)t_attr;
	}
}

/* ------------------------------------------------------------------------- */

void RE_prof_report(void)
{
	int i;

	if (!RE_prof_active()) return;
	/* Раньше здесь проверялись только слоты нашего растеризатора. На пути BI
	 * (rasterizer_mode=OFF) все они нулевые, и отчёт молча не печатался —
	 * поэтому добавлены слоты BI. */
	if (prof_part_no == 0 &&
	    prof_glob[RE_PROF_BUILD] == 0.0 && prof_glob[RE_PROF_VISIBILITY] == 0.0 &&
	    prof_glob[RE_PROF_SHADE] == 0.0 && prof_glob[RE_PROF_BI_RASTER] == 0.0 &&
	    prof_glob[RE_PROF_BI_SHADE] == 0.0 && prof_glob[RE_PROF_RAYTRACE] == 0.0 &&
	    prof_glob[RE_PROF_RAYTREE] == 0.0 && prof_glob[RE_PROF_SHADBUF] == 0.0)
		return;

	printf("[PROF] ИТОГ частей=%d buckets/part=%.1f darrays/part=%.1f tick=%.1f ns"
	       " (обвязка на attrpx=%ld это ~%.1f ms)\n",
	       prof_part_no,
	       prof_part_no ? (double)prof_ctot[RE_PROF_C_BUCKETS] / (double)prof_part_no : 0.0,
	       prof_part_no ? (double)prof_ctot[RE_PROF_C_DARRAYS] / (double)prof_part_no : 0.0,
	       prof_tick_ns,
	       prof_ctot[RE_PROF_C_ATTRPIX],
	       prof_ctot[RE_PROF_C_ATTRPIX] * 2.0 * prof_tick_ns / 1e6);

	printf("[PROF] ИТОГ время по частям, суммарно: span=%.4f slot=%.4f proj=%.4f "
	       "clip=%.4f fill=%.4f attr=%.4f (ms); сумма частей=%.4f s\n",
	       prof_tot[RE_PROF_SPAN] * 1000.0,
	       (prof_tot[RE_PROF_SCENE_LOOP] - prof_tot[RE_PROF_PROJECT]) * 1000.0,
	       (prof_tot[RE_PROF_PROJECT] - prof_tot[RE_PROF_CLIP]) * 1000.0,
	       (prof_tot[RE_PROF_CLIP] - prof_tot[RE_PROF_FILL]) * 1000.0,
	       (prof_tot[RE_PROF_FILL] - prof_tot[RE_PROF_ATTR]) * 1000.0,
	       prof_tot[RE_PROF_ATTR] * 1000.0,
	       prof_tot_part_wall);

	printf("[PROF] ИТОГ вне частей: build=%.4f s visibility=%.4f s shade=%.4f s"
	       " (из него наш шейдер=%.4f s, BI=%.4f s)\n",
	       prof_glob[RE_PROF_BUILD], prof_glob[RE_PROF_VISIBILITY],
	       prof_glob[RE_PROF_SHADE], prof_glob[RE_PROF_SHADE_OURS],
	       prof_glob[RE_PROF_SHADE_BI]);

	printf("[PROF] ИТОГ счётчики: slots=%ld culled=%ld drawn=%ld prims=%ld prim_culled=%ld "
	       "tris_fast=%ld tris_clip=%ld tris_out=%ld fills=%ld fills_empty=%ld "
	       "px=%ld attrpx=%ld\n",
	       prof_ctot[RE_PROF_C_SLOTS_TESTED], prof_ctot[RE_PROF_C_SLOTS_CULLED],
	       prof_ctot[RE_PROF_C_SLOTS_DRAWN], prof_ctot[RE_PROF_C_PRIMS],
	       prof_ctot[RE_PROF_C_PRIMS_PARTCLIP],
	       prof_ctot[RE_PROF_C_TRIS_FAST], prof_ctot[RE_PROF_C_TRIS_CLIPPED],
	       prof_ctot[RE_PROF_C_TRIS_OUT],
	       prof_ctot[RE_PROF_C_FILLS], prof_ctot[RE_PROF_C_FILLS_EMPTY],
	       prof_ctot[RE_PROF_C_PIXELS], prof_ctot[RE_PROF_C_ATTRPIX]);

	/* --- путь BI (rasterizer_mode=OFF). Вложенность:
	 *       SCENE_PREP          (до всего)
	 *       SHADBUF, RAYTREE    (до частей)
	 *       BI_RASTER ⊃ ...     (растеризация части)
	 *       SHADE, BI_SHADE ⊃ RAYTRACE ⊃ REFLECT/REFRACT
	 *     Печатаем и «полные», и исключающие времена, чтобы не гадать. --- */
	if (prof_glob[RE_PROF_SCENE_PREP] > 0.0 || prof_glob[RE_PROF_BI_RASTER] > 0.0 ||
	    prof_glob[RE_PROF_RAYTREE] > 0.0 || prof_glob[RE_PROF_SHADBUF] > 0.0) {
		/* Профиль шейдинга: на пути BI это ЛИБО zbufshadeDA_tile (OSA, слот
		 * BI_SHADE), ЛИБО zbufshade_tile (без OSA, слот SHADE_BI). Складываем:
		 * за один прогон заполняется ровно один из них. */
		double t_shade  = prof_glob[RE_PROF_BI_SHADE] + prof_glob[RE_PROF_SHADE_BI];
		double t_ray    = prof_glob[RE_PROF_RAYTRACE];
		double t_refl   = prof_glob[RE_PROF_REFLECT];
		double t_refr   = prof_glob[RE_PROF_REFRACT];
		double ex_shade = t_shade - t_ray;

		printf("[PROF] ИТОГ путь BI: scene_prep=%.4f s shadowbuf=%.4f s raytree=%.4f s "
		       "bi_raster=%.4f s | shade=%.4f s (из него raytrace=%.4f s, "
		       "refl=%.4f s refr=%.4f s; шейдинг без рейтрейса=%.4f s)\n",
		       prof_glob[RE_PROF_SCENE_PREP], prof_glob[RE_PROF_SHADBUF],
		       prof_glob[RE_PROF_RAYTREE], prof_glob[RE_PROF_BI_RASTER],
		       t_shade, t_ray, t_refl, t_refr, ex_shade);

		/* Счётчики BI печатаем из prof_cacc, а НЕ из prof_ctot: на пути BI
		 * RE_prof_part_begin() не вызывается (части считает BI, а не наш
		 * растеризатор), поэтому переносить acc -> tot некому, и в tot всегда
		 * нули. На пути BI prof_cacc за весь рендер никто не сбрасывает. */
		printf("[PROF] ИТОГ счётчики BI: ray_trace=%ld reflect=%ld refract=%ld "
		       "raycast=%ld miss=%ld shadowbufs=%ld shade_samples=%ld\n",
		       prof_cacc[RE_PROF_C_RAYTRACE], prof_cacc[RE_PROF_C_REFLECT],
		       prof_cacc[RE_PROF_C_REFRACT], prof_cacc[RE_PROF_C_RAYCAST],
		       prof_cacc[RE_PROF_C_RAYCAST_MISS], prof_cacc[RE_PROF_C_SHADBUF],
		       prof_cacc[RE_PROF_C_SHADE_SMP]);
	}

	fflush(stdout);

	/* обнуляем, чтобы второй рендер в том же процессе не суммировался с первым */
	for (i = 0; i < RE_PROF_NUM; i++) { prof_tot[i] = 0.0; prof_glob[i] = 0.0; }
	for (i = 0; i < RE_PROF_C_NUM; i++) prof_ctot[i] = 0;
	prof_part_no = 0;
	prof_tot_part_wall = 0.0;
}
