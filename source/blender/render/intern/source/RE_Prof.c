/*
 * RE_Prof.c — реализация ВРЕМЕННОЙ измерительной обвязки. См. RE_Prof.h.
 *
 * Включается только при UBF_PROF в окружении. Таймер — PIL_check_seconds_timer(),
 * тот же, которым pipeline.c меряет кадры, так что цифры сопоставимы с тем,
 * что Blender печатает сам.
 *
 * ПОТОКИ: каждый поток пишет в СВОЙ блок (ProfBlock). Раньше аккумуляторы были
 * общими: при рендере в несколько потоков цифры рвались (части идут параллельно,
 * а part_begin обнулял общий счётчик у соседа) и вдобавок горячие счётчики били
 * по одной кэш-линии из всех ядер. На aero4 (1920x1080, OSA 8, 16 потоков) это
 * стоило +49 s из 209 s кадра против 160.7 s при UBF_PROF=0. Теперь отчёт
 * суммирует блоки, и цена обвязки — только сам вызов таймера.
 */

#include <stdio.h>
#include <stdlib.h>

#include "PIL_time.h"

#include "RE_Prof.h"

#if defined(_MSC_VER)
#  include <intrin.h>
#  define PROF_TLS __declspec(thread)
#  define PROF_CAS32(p, was, now) \
	(_InterlockedCompareExchange((volatile long *)(p), (long)(now), (long)(was)) == (long)(was))
#else
#  define PROF_TLS __thread
#  define PROF_CAS32(p, was, now) (*(p) == (was) ? (*(p) = (now), 1) : 0)
#endif

/* ------------------------------------------------------------------------- */

static int    prof_state = -1;   /* -1 не проверяли, 0 выкл, 1 вкл */
static int    prof_calibrated = 0;
/* Стоимость одной пары tick+span в наносекундах. Нужна, чтобы читатель мог
 * оценить вклад самой обвязки в per-pixel строки: например, attr меряется
 * двумя вызовами таймера на пиксель, то есть ~2*tick_ns*attrpx секунд из
 * показанного attr — это обвязка, а не работа растеризатора. */
static double prof_tick_ns = 0.0;

/* ★ PROF — блок измерений одного потока. Заводится лениво, при первом
 * обращении потока; отчёт суммирует все заведённые блоки. */
#define PROF_MAX_BLOCKS 64

typedef struct ProfBlock {
	volatile long used;             /* занят потоком (заводится через CAS) */
	int    parts;                   /* сколько частей отмерил этот поток */
	double part_t0;                 /* начало текущей части */
	double part_wall;               /* сумма частей, wall-clock */
	double acc[RE_PROF_NUM];        /* накопитель текущей части */
	long   cacc[RE_PROF_C_NUM];     /* счётчики текущей части */
	double tot[RE_PROF_NUM];        /* итог по частям этого потока */
	long   ctot[RE_PROF_C_NUM];
	double glob[RE_PROF_NUM];       /* слоты вне частей (build/visibility/BI) */
	long   dm[3];                   /* счётчики кэша геометрии */
} ProfBlock;

static ProfBlock prof_blocks[PROF_MAX_BLOCKS];
static PROF_TLS int prof_my_index = -1;

static ProfBlock *prof_my_block(void)
{
	int i;

	if (prof_my_index >= 0)
		return &prof_blocks[prof_my_index];

	for (i = 0; i < PROF_MAX_BLOCKS; i++) {
		if (PROF_CAS32(&prof_blocks[i].used, 0, 1)) {
			prof_my_index = i;
			return &prof_blocks[i];
		}
	}

	/* потоков больше, чем блоков: пишем в нулевой, цифры этого потока смешаются */
	prof_my_index = 0;
	return &prof_blocks[0];
}

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
	ProfBlock *b;

	if (!RE_prof_active()) return;
	b = prof_my_block();
	b->acc[slot] += PIL_check_seconds_timer() - t0;
}

void RE_prof_glob_span(int slot, double t0)
{
	ProfBlock *b;

	if (!RE_prof_active()) return;
	b = prof_my_block();
	b->glob[slot] += PIL_check_seconds_timer() - t0;
}

/* ------------------------------------------------------------------------- */

void RE_prof_add(int slot, double dt)
{
	ProfBlock *b;

	if (!RE_prof_active()) return;
	b = prof_my_block();
	b->acc[slot] += dt;
}

void RE_prof_add_global(int slot, double dt)
{
	ProfBlock *b;

	if (!RE_prof_active()) return;
	b = prof_my_block();
	b->glob[slot] += dt;
}

void RE_prof_count(int counter, long n)
{
	ProfBlock *b;

	if (!RE_prof_active()) return;
	if (counter < 0 || counter >= RE_PROF_C_NUM) return;
	b = prof_my_block();
	b->cacc[counter] += n;
}

/* ★ Счётчики кэша геометрии: отдельное хранилище, см. RE_Prof.h. */
void RE_prof_count_dm(int what, long n)
{
	ProfBlock *b;

	if (!RE_prof_active()) return;
	if (what < 0 || what > 2) return;
	b = prof_my_block();
	b->dm[what] += n;
}

void RE_prof_note_threads(int threads)
{
	if (!RE_prof_active()) return;
	if (threads > 1) {
		printf("[PROF] потоков рендера %d: измерения собираются по потокам "
		       "(блок на поток), строки part печатает каждый поток свои.\n", threads);
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
	ProfBlock *b;
	int i;

	if (!RE_prof_active()) return;

	prof_calibrate();

	b = prof_my_block();
	for (i = 0; i < RE_PROF_NUM; i++) b->acc[i] = 0.0;
	for (i = 0; i < RE_PROF_C_NUM; i++) b->cacc[i] = 0;

	b->part_t0 = PIL_check_seconds_timer();
}

void RE_prof_part_end(int xmin, int ymin, int rectx, int recty)
{
	ProfBlock *b;
	double part;
	int i;

	if (!RE_prof_active()) return;

	b = prof_my_block();

	part = PIL_check_seconds_timer() - b->part_t0;
	b->parts++;
	b->part_wall += part;

	/* копим итог этого потока */
	for (i = 0; i < RE_PROF_NUM; i++) {
		if (RE_PROF_IS_GLOBAL(i)) continue;
		b->tot[i] += b->acc[i];
	}
	for (i = 0; i < RE_PROF_C_NUM; i++) b->ctot[i] += b->cacc[i];
	b->ctot[RE_PROF_C_PARTS]++;

	/* ---- исключающие времена ---------------------------------------------
	 * Считаем так, чтобы строки складывались в PART и ни один вклад не был
	 * посчитан дважды. Проценты — от измеренного PART, не от acc[PART]
	 * (его никто не заполняет: часть целиком меряется здесь же).
	 */
	{
		double t_span   = b->acc[RE_PROF_SPAN];
		double t_scene  = b->acc[RE_PROF_SCENE_LOOP];
		double t_proj   = b->acc[RE_PROF_PROJECT];
		double t_clip   = b->acc[RE_PROF_CLIP];
		double t_fill   = b->acc[RE_PROF_FILL];
		double t_attr   = b->acc[RE_PROF_ATTR];

		double ex_span  = t_span;
		double ex_slot  = t_scene - t_proj;      /* обход buckets/slots и bbox */
		double ex_proj  = t_proj - t_clip;       /* projectvert + тесты */
		double ex_clip  = t_clip - t_fill;       /* клиппер без филлера */
		double ex_fill  = t_fill - t_attr;       /* спаны и z-тест */
		double ex_attr  = t_attr;                /* интерполяция co/vn */
		double ex_other = part - (ex_span + ex_slot + ex_proj + ex_clip + ex_fill + ex_attr);

		double inv = (part > 0.0) ? (100.0 / part) : 0.0;

		printf("[PROF] part=%d поток=%d xy=(%d,%d) rect=%dx%d total=%.4f ms"
		       " | span=%.4f (%.1f%%) slot=%.4f (%.1f%%) proj=%.4f (%.1f%%)"
		       " clip=%.4f (%.1f%%) fill=%.4f (%.1f%%) attr=%.4f (%.1f%%) other=%.4f (%.1f%%)\n",
		       b->parts, (int)(b - prof_blocks),
		       xmin, ymin, rectx, recty, part * 1000.0,
		       ex_span * 1000.0, ex_span * inv,
		       ex_slot * 1000.0, ex_slot * inv,
		       ex_proj * 1000.0, ex_proj * inv,
		       ex_clip * 1000.0, ex_clip * inv,
		       ex_fill * 1000.0, ex_fill * inv,
		       ex_attr * 1000.0, ex_attr * inv,
		       ex_other * 1000.0, ex_other * inv);

		printf("[PROF] part=%d поток=%d counts slots=%ld culled=%ld drawn=%ld darrays=%ld"
		       " prims=%ld prim_culled=%ld tris_fast=%ld tris_clip=%ld tris_out=%ld"
		       " fills=%ld fills_empty=%ld px=%ld attrpx=%ld\n",
		       b->parts, (int)(b - prof_blocks),
		       b->cacc[RE_PROF_C_SLOTS_TESTED], b->cacc[RE_PROF_C_SLOTS_CULLED],
		       b->cacc[RE_PROF_C_SLOTS_DRAWN], b->cacc[RE_PROF_C_DARRAYS],
		       b->cacc[RE_PROF_C_PRIMS], b->cacc[RE_PROF_C_PRIMS_PARTCLIP],
		       b->cacc[RE_PROF_C_TRIS_FAST], b->cacc[RE_PROF_C_TRIS_CLIPPED],
		       b->cacc[RE_PROF_C_TRIS_OUT],
		       b->cacc[RE_PROF_C_FILLS], b->cacc[RE_PROF_C_FILLS_EMPTY],
		       b->cacc[RE_PROF_C_PIXELS], b->cacc[RE_PROF_C_ATTRPIX]);

		(void)t_span; (void)t_scene; (void)t_proj; (void)t_clip;
		(void)t_fill; (void)t_attr;
	}
}

/* ------------------------------------------------------------------------- */

void RE_prof_report(void)
{
	/* Суммы по блокам потоков. Имена совпадают с теми, что были у прежних
	 * общих аккумуляторов, чтобы разбор ниже читался как раньше. */
	double prof_tot[RE_PROF_NUM];
	long   prof_ctot[RE_PROF_C_NUM];
	double prof_glob[RE_PROF_NUM];
	long   prof_dm[3];
	int    prof_part_no = 0;
	double prof_tot_part_wall = 0.0;
	int i, t;

	for (i = 0; i < RE_PROF_NUM; i++) { prof_tot[i] = 0.0; prof_glob[i] = 0.0; }
	for (i = 0; i < RE_PROF_C_NUM; i++) prof_ctot[i] = 0;
	for (i = 0; i < 3; i++) prof_dm[i] = 0;

	for (t = 0; t < PROF_MAX_BLOCKS; t++) {
		ProfBlock *b = &prof_blocks[t];

		if (!b->used) continue;
		for (i = 0; i < RE_PROF_NUM; i++) {
			prof_tot[i] += b->tot[i];
			prof_glob[i] += b->glob[i];
		}
		for (i = 0; i < RE_PROF_C_NUM; i++) prof_ctot[i] += b->ctot[i];
		for (i = 0; i < 3; i++) prof_dm[i] += b->dm[i];
		prof_part_no += b->parts;
		prof_tot_part_wall += b->part_wall;
	}

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

	/* --- подготовка сцены (общая для обоих путей, идёт до всего остального).
	 * Печатается всегда, когда её мерили: на быстром пути это главный
	 * последовательный расход кадра, и без разбивки непонятно, куда целиться. --- */
	if (prof_glob[RE_PROF_SCENE_PREP] > 0.0) {
		double t_v = prof_glob[RE_PROF_MESH_VERT];
		double t_f = prof_glob[RE_PROF_MESH_FACE];
		double t_d = prof_glob[RE_PROF_MESH_DM];
		printf("[PROF] ИТОГ подготовка сцены: всего=%.4f s | депграф=%.4f s "
		       "база(VlakRen)=%.4f s -> DerivedMesh=%.4f s вершины=%.4f s "
		       "грани=%.4f s прочее по объектам=%.4f s | прочее до базы=%.4f s\n",
		       prof_glob[RE_PROF_SCENE_PREP],
		       prof_glob[RE_PROF_SCENE_DEPS],
		       prof_glob[RE_PROF_SCENE_DB], t_d, t_v, t_f,
		       prof_glob[RE_PROF_SCENE_DB] - t_v - t_f - t_d,
		       prof_glob[RE_PROF_SCENE_PREP] - prof_glob[RE_PROF_SCENE_DEPS] -
		       prof_glob[RE_PROF_SCENE_DB]);
	}

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
		/* Счётчики BI: на пути BI RE_prof_part_begin() не вызывается (части
		 * считает BI, а не наш растеризатор), поэтому acc -> tot некому
		 * переносить и всё накопленное лежит в блоковых cacc. */
		long c_raytrace = 0, c_reflect = 0, c_refract = 0, c_raycast = 0;
		long c_miss = 0, c_shadowbufs = 0, c_smp = 0;
		long c_lamp = 0, c_shadow_rays = 0, c_hits = 0;

		for (t = 0; t < PROF_MAX_BLOCKS; t++) {
			ProfBlock *b = &prof_blocks[t];
			if (!b->used) continue;
			c_raytrace    += b->cacc[RE_PROF_C_RAYTRACE] + b->ctot[RE_PROF_C_RAYTRACE];
			c_reflect     += b->cacc[RE_PROF_C_REFLECT] + b->ctot[RE_PROF_C_REFLECT];
			c_refract     += b->cacc[RE_PROF_C_REFRACT] + b->ctot[RE_PROF_C_REFRACT];
			c_raycast     += b->cacc[RE_PROF_C_RAYCAST] + b->ctot[RE_PROF_C_RAYCAST];
			c_miss        += b->cacc[RE_PROF_C_RAYCAST_MISS] + b->ctot[RE_PROF_C_RAYCAST_MISS];
			c_shadowbufs  += b->cacc[RE_PROF_C_SHADBUF] + b->ctot[RE_PROF_C_SHADBUF];
			c_smp         += b->cacc[RE_PROF_C_SHADE_SMP] + b->ctot[RE_PROF_C_SHADE_SMP];
			c_lamp        += b->cacc[RE_PROF_C_SHADOW_LAMPS] + b->ctot[RE_PROF_C_SHADOW_LAMPS];
			c_shadow_rays += b->cacc[RE_PROF_C_SHADOW_RAYS] + b->ctot[RE_PROF_C_SHADOW_RAYS];
			c_hits        += b->cacc[RE_PROF_C_SHADOW_HITS] + b->ctot[RE_PROF_C_SHADOW_HITS];
		}

		printf("[PROF] ИТОГ путь BI: scene_prep=%.4f s shadowbuf=%.4f s raytree=%.4f s "
		       "bi_raster=%.4f s | shade=%.4f s (из него raytrace=%.4f s, "
		       "refl=%.4f s refr=%.4f s; шейдинг без рейтрейса=%.4f s; "
		       "тени ламп=%.4f s)\n",
		       prof_glob[RE_PROF_SCENE_PREP], prof_glob[RE_PROF_SHADBUF],
		       prof_glob[RE_PROF_RAYTREE], prof_glob[RE_PROF_BI_RASTER],
		       t_shade, t_ray, t_refl, t_refr, ex_shade,
		       prof_glob[RE_PROF_RAY_SHADOW]);

		printf("[PROF] ИТОГ счётчики BI: ray_trace=%ld reflect=%ld refract=%ld "
		       "raycast=%ld miss=%ld shadowbufs=%ld shade_samples=%ld "
		       "shadow_lamp=%ld shadow_rays=%ld hits=%ld\n",
		       c_raytrace, c_reflect, c_refract, c_raycast, c_miss,
		       c_shadowbufs, c_smp, c_lamp, c_shadow_rays, c_hits);
	}

	/* --- R4: кэш геометрии. Печатаем всегда, когда считали. --- */
	if (prof_dm[RE_PROF_DM_BUILD] || prof_dm[RE_PROF_DM_REUSE] ||
	    prof_dm[RE_PROF_DM_ALIVE]) {
		printf("[PROF] ИТОГ кэш геометрии: построено=%ld взято из кэша=%ld | "
		       "derivedFinal жив на входе=%ld\n",
		       prof_dm[RE_PROF_DM_BUILD], prof_dm[RE_PROF_DM_REUSE],
		       prof_dm[RE_PROF_DM_ALIVE]);
	}

	fflush(stdout);

	/* обнуляем, чтобы второй рендер в том же процессе не суммировался с первым */
	for (t = 0; t < PROF_MAX_BLOCKS; t++) {
		ProfBlock *b = &prof_blocks[t];
		if (!b->used) continue;
		for (i = 0; i < RE_PROF_NUM; i++) { b->tot[i] = 0.0; b->glob[i] = 0.0; }
		for (i = 0; i < RE_PROF_C_NUM; i++) { b->ctot[i] = 0; b->cacc[i] = 0; }
		for (i = 0; i < RE_PROF_NUM; i++) b->acc[i] = 0.0;
		for (i = 0; i < 3; i++) b->dm[i] = 0;
		b->parts = 0;
		b->part_wall = 0.0;
	}
}
