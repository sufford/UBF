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
#include <math.h>


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

#ifdef WITH_UBF_CUDA
#include "ubf_cuda.h"    /* ★ этап A: марш экранных отражений на GPU */
#endif

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

bool RE_fast_skip_raytree(struct Render *re)
{
	/* ★ Замер (полный кадр 1600x768, боевая сцена, Release):
	 *   сборка октодерева 4.2 с из 8.3 с кадра;
	 *   без дерева зеркала «плоские», но отличаются 0.4% пикселей
	 *   (struct_corr 0.9995, mean 0.196 против того же пути с лучами).
	 * Значит по умолчанию дерево не строим, а лучи включаются явно. */
	static int env = -1;

	if (env < 0) {
		const char *e = getenv("UBF_FAST_RAYMIRROR");
		env = (e && e[0] == '1') ? 0 : 1;   /* 1 = пропускать дерево */
	}

	if (!env) return false;
	if (re == NULL) return false;

	return (re->r.rasterizer_mode == RE_RASTERIZER_FAST);
}

/* ------------------------------------------------------------------------- */
/* ★ R3: КАДРОВЫЙ G-БУФЕР И ЭКРАННЫЕ ЗЕРКАЛА (screen-space reflection)        */
/*                                                                            */
/* Отложенному затенению и отражениям нужен буфер на ВЕСЬ кадр, а не на часть: */
/* отражённый луч из зеркала в одной части попадает в пиксель другой части.    */
/* Что храним (всё в view space, как ShadeInput::co / ::vn):                   */
/*   depth — расстояние вдоль оси взгляда (-z), +INF = пиксель не закрашен;    */
/*   pos   — позиция точки;                                                    */
/*   nrm   — нормаль (та же, что ушла в освещение);                            */
/*   view  — нормированный вектор из камеры в точку;                           */
/*   mir   — i (сила отражения с учётом fresnel) и mirr/mirg/mirb материала.   */
/*                                                                            */
/* Пиксели частей не пересекаются, поэтому запись из потоков безопасна.        */
/* Резолв идёт в главном потоке после сборки кадра: снимает копию цвета с      */
/* Combined-пасса (чтобы луч не читал то, что сам же записал) и для зеркальных */
/* пикселей марширует отражённый луч по буферу глубины. Значения не           */
/* нормализуются и не фильтруются: это приближение, допуск у быстрого пути.    */
/* ------------------------------------------------------------------------- */

typedef struct RE_FastFrame {
	int    w, h;
	float *depth;   /* 1 на пиксель */
	float *pos;     /* 3 */
	float *nrm;     /* 3 */
	float *view;    /* 3 */
	float *mir;     /* 4: i, mirr, mirg, mirb */
	float *col;     /* 4: снимок кадра (RGBA) — источник для лучей */
	float *ssun;    /* 1: доля солнца в цвете пикселя (тени S1), -1 = не писали */
} RE_FastFrame;

static RE_FastFrame re_fast_frame = { 0, 0, NULL, NULL, NULL, NULL, NULL, NULL, NULL };

/* ★ S1: направление НА солнце в видовых координатах — одно на весь кадр
 * (солнце бесконечно далеко), поэтому шейдер просто перезаписывает его. */
static float re_fast_sun_dir[3] = { 0.0f, 0.0f, 1.0f };
static int re_fast_sun_valid = 0;

/* ★ S6: счётчики состава света, которые ведёт RE_Shader.c. */
extern double re_fast_stat_hist[8];
extern double re_fast_stat_amb;
extern double re_fast_stat_tot;
extern double re_fast_stat_sunsum;
extern long re_fast_stat_npx;

/* ★ S1: тени от солнца. Выключатель UBF_FAST_SHADOW=0. */
static bool re_fast_shadow_enabled(void)
{
	static int env = -1;

	if (env < 0) {
		const char *e = getenv("UBF_FAST_SHADOW");
		env = (e && e[0] == '0') ? 0 : 1;
	}
	return env != 0;
}

#define RE_FAST_SHADOW_STEPS 24

/* Экранные зеркала выключаются UBF_FAST_SSR=0: тогда кадровый G-буфер не
 * выделяется вовсе и поведение быстрого пути как в R2 (зеркала «плоские»). */
static bool re_fast_ssr_enabled(void)
{
	static int env = -1;

	if (env < 0) {
		const char *e = getenv("UBF_FAST_SSR");
		env = (e && e[0] == '0') ? 0 : 1;
	}
	return env != 0;
}

/* ★ SSR в BI: выключатель — поле .blend RenderData.ubf_bi_ssr, а переменная
 * окружения UBF_BI_SSR его ПЕРЕОПРЕДЕЛЯЕТ (нужна стендам: приёмка и замеры
 * гоняют один и тот же .blend в разных режимах).
 *
 * «Присутствие и не ноль», а не «отсутствие нуля»: по умолчанию выключатель
 * ВЫКЛ, иначе побитовая приёмка accept.ps1 (BI против эталонов) обязана упасть.
 * Отдельный от UBF_FAST_SSR, потому что у BI свой источник данных в G-буфере
 * (шейдинг BI, а не наш шейдер). */
static int bi_ssr_scene = 0;   /* RenderData.ubf_bi_ssr, ставит RE_bi_ssr_set_scene() */

/* ★ Точность марша в BI: UBF_BI_SSR_MARCH=thick включает тест попадания «с
 * толщиной» (точка за поверхностью не дальше шага по глубине). Замер показал,
 * что он ХУЖЕ исходного (см. HANDOFF_SSR_BI_STEP1 §10), поэтому по умолчанию
 * выключен — оставлен, чтобы опыт воспроизводился одной переменной. */
static int bi_ssr_march_thick(void)
{
	static int env = -2;

	if (env == -2) {
		const char *e = getenv("UBF_BI_SSR_MARCH");
		env = (e != NULL && e[0] == 't') ? 1 : 0;
	}
	return env != 0;
}

/* ★ Точность марша в BI: число шагов (UBF_BI_SSR_STEPS, по умолчанию 256).
 *
 * Гипотеза «экспоненциальный шаг 64 проскакивает отражённые объекты» замером
 * ПОДТВЕРДИЛАСЬ: на зеркальной пробе 256 шагов поднимают долю пикселей, где
 * экранное отражение попало в цвет луча (|SSR-Ray| <= 2 уровня) с 18.0% до
 * 32.4% при том же покрытии (сработал 38.0% -> 40.5%), а на боевой сцене
 * уменьшают расхождение с лучевым BI (707 -> 675 px при чистом SSR, 538 -> 511
 * в гибриде). 1024 шага дают почти то же (34.0%) — насыщение на 256.
 * Времени марш почти не ест: 0.4 с из ~18 с кадра (см. PROF bi_raster). */
static int bi_ssr_steps(void)
{
	static int env = -2;

	if (env == -2) {
		const char *e = getenv("UBF_BI_SSR_STEPS");
		env = e ? atoi(e) : 256;
		if (env < 8) env = 8;
		if (env > 2048) env = 2048;
	}
	return env;
}

/* ★ Точность марша в BI: бисекция момента пересечения (UBF_BI_SSR_BISECT,
 * по умолчанию 8; 0 — выключить, нужно для A/B).
 *
 * Марш берёт цвет с ПЕРВОЙ выборки, оказавшейся за поверхностью. Шаг
 * экспоненциальный: вдали он длинный, и выборка «за» может стоять далеко за
 * настоящим пересечением — тогда цвет берётся чужой (соседний объект, фон).
 * Бисекция сжимает отрезок [«до», «за»] до (tfar-tnear)/2^bisect и отдаёт
 * пиксель уже в самой точке пересечения. Стоит это нескольких проб на
 * ПОПАДАНИЕ (не на шаг), то есть копейки. */
static int bi_ssr_bisect(void)
{
	static int env = -2;

	if (env == -2) {
		const char *e = getenv("UBF_BI_SSR_BISECT");
		env = e ? atoi(e) : 8;
		if (env < 0) env = 0;
		if (env > 24) env = 24;
	}
	return env;
}

#ifdef WITH_UBF_CUDA
/* ★ ЭТАП A: приёмник попаданий от GPU. В режиме defer CPU-массивы уже есть
 * (bi_ssr_hit), а в чистом включении их нет — этот буфер только под GPU.
 * Таблица времён шага считается ЗДЕСЬ, тем же powf, что и CPU-эталон: ядро не
 * должно считать трансцендентные само, иначе совпадение перестанет быть битовым. */
static int *bi_ssr_gpu_hit = NULL;
static int bi_ssr_gpu_w = 0;
static int bi_ssr_gpu_h = 0;
static float bi_ssr_ts[2048];

/* GPU-марш включён? (сборка с CUDA + UBF_BI_SSR_GPU=1 + не быстрый путь) */
static int bi_ssr_use_gpu(void)
{
	static int env = -2;

	if (env == -2) {
		const char *e = getenv("UBF_BI_SSR_GPU");
		env = (e != NULL && e[0] != '0') ? 1 : 0;
	}
	return env != 0;
}
#endif

bool RE_bi_ssr_enabled(void)
{
	static int env = -2;   /* -2 = ещё не читали, -1 = не задана, 0/1 = значение */

	if (env == -2) {
		const char *e = getenv("UBF_BI_SSR");
		env = (e == NULL) ? -1 : ((e[0] == '0') ? 0 : 1);
	}
	if (env >= 0)
		return env != 0;
	return bi_ssr_scene != 0;
}

/* Вызывается из RE_fast_frame_begin(), то есть до шейдинга частей: с этого
 * момента RE_bi_ssr_enabled() видит значение из .blend. */
void RE_bi_ssr_set_scene(struct Render *re)
{
	bi_ssr_scene = (re != NULL && re->r.ubf_bi_ssr) ? 1 : 0;
}

/* ★ Проба марша: точка на луче в момент t. Возвращает индекс пикселя кадра,
 * если тест попадания сработал, иначе -1.
 *
 * th > 0 — тест «с толщиной» (UBF_BI_SSR_MARCH=thick), th == 0 — прежний тест
 * (луч просто оказался за поверхностью). Вынесено в функцию, потому что те же
 * пробы нужны бисекции: она ищет момент пересечения между двумя выборками. */
static int bi_ssr_march_probe(struct Render *re, const float *P0, const float *D,
                              float t, float th, int w, int h)
{
	float P[3], wcl, sx, sy, zd, diff;
	int px, py, j;

	P[0] = P0[0] + t * D[0];
	P[1] = P0[1] + t * D[1];
	P[2] = P0[2] + t * D[2];
	if (P[2] >= -0.001f) return -1;          /* ушли за камеру */

	wcl = -P[2];
	sx = 0.5f * re->winx * ((re->winmat[0][0] * P[0] + re->winmat[3][0]) / wcl + 1.0f);
	sy = 0.5f * re->winy * ((re->winmat[1][1] * P[1] + re->winmat[3][1]) / wcl + 1.0f);

	px = (int)(sx * ((float)w / (float)re->winx));
	py = (int)(sy * ((float)h / (float)re->winy));
	if (px < 0 || py < 0 || px >= w || py >= h) return -1;

	j = py * w + px;
	zd = re_fast_frame.depth[j];
	if (zd >= 1e29f) return -1;              /* небо/пусто — не поверхность */

	diff = wcl - zd;
	if (th > 0.0f) {
		if (diff > 0.0f && diff <= th) return j;
	}
	else if (diff > zd * 0.01f + 0.01f) {
		return j;
	}
	return -1;
}

void RE_fast_frame_begin(struct Render *re)
{
	int i, n;
	const bool fast = RE_shader_fast();

	/* ★ Выключатель SSR в BI берём из .blend (RenderData.ubf_bi_ssr) — здесь,
	 * до шейдинга частей. Env UBF_BI_SSR, если задан, переопределяет его. */
	RE_bi_ssr_set_scene(re);

	if (re_fast_frame.depth) RE_fast_frame_end();
	if (re == NULL) return;
	/* ★ Быстрый путь — свой выключатель (UBF_FAST_SSR), BI — свой
	 * (UBF_BI_SSR). Раньше здесь стоял безусловный выход вне быстрого пути,
	 * из-за чего BI кадровый G-буфер не получал вообще. */
	if (fast) {
		if (!re_fast_ssr_enabled()) return;
	}
	else {
		if (!RE_bi_ssr_enabled()) return;
	}
	if (re->rectx <= 0 || re->recty <= 0) return;

	re_fast_frame.w = re->rectx;
	re_fast_frame.h = re->recty;
	n = re_fast_frame.w * re_fast_frame.h;

	re_fast_frame.depth = MEM_callocN(sizeof(float) * n, "RE fast frame depth");
	re_fast_frame.pos   = MEM_callocN(sizeof(float) * 3 * n, "RE fast frame pos");
	re_fast_frame.nrm   = MEM_callocN(sizeof(float) * 3 * n, "RE fast frame nrm");
	re_fast_frame.view  = MEM_callocN(sizeof(float) * 3 * n, "RE fast frame view");
	re_fast_frame.mir   = MEM_callocN(sizeof(float) * 4 * n, "RE fast frame mir");
	re_fast_frame.col   = MEM_callocN(sizeof(float) * 4 * n, "RE fast frame col");
	re_fast_frame.ssun  = MEM_mallocN(sizeof(float) * n, "RE fast frame ssun");

	/* Пусто = +INF: луч сквозь небо идёт дальше и поверхности там не находит. */
	for (i = 0; i < n; i++) {
		re_fast_frame.depth[i] = 1e30f;
		re_fast_frame.ssun[i] = -1.0f;   /* «долю солнца ещё не писали» */
	}
	re_fast_sun_valid = 0;

	/* ★ S6: сброс счётчиков состава света (см. RE_Shader.c). */
	re_fast_stat_npx = 0;
	re_fast_stat_amb = 0.0;
	re_fast_stat_tot = 0.0;
	re_fast_stat_sunsum = 0.0;
	{
		int bi;

		for (bi = 0; bi < 8; bi++) re_fast_stat_hist[bi] = 0.0;
	}
}

void RE_fast_frame_store(int x, int y, const float co[3], const float vn[3],
                         const float view[3], float mirror_i, const float mirrgb[3])
{
	int idx;

	if (re_fast_frame.depth == NULL) return;
	if (x < 0 || y < 0 || x >= re_fast_frame.w || y >= re_fast_frame.h) return;

	idx = y * re_fast_frame.w + x;

	/* ★ R7: пишем только то, что кто-то прочитает.
	 *
	 * Замер на боевой сцене (1600x768): кадровый G-буфер стоит 0.131 с из
	 * 1.077 с тёплого кадра (12%), а зеркальных пикселей в кадре — 12700 из
	 * 1228800, то есть 1%. При этом старый код писал 14 float НА КАЖДЫЙ
	 * пиксель ради этих 1%.
	 *
	 * Что обязан знать каждый пиксель:
	 *   depth — это z-буфер кадра, resolve читает его в ПРОИЗВОЛЬНЫХ точках
	 *           вдоль луча (строки ~271), а не только в зеркальных пикселях;
	 *   mir[0] — сам признак «пиксель зеркальный» (resolve смотрит i > 0).
	 * Остальное (pos, nrm, view, цвет зеркала) читается только в исходном
	 * пикселе луча, а туда resolve доходит лишь при i > 0 — значит для
	 * незазеркальных пикселей эти 12 float не читает никто.
	 *
	 * Инвариант проверяем: картинка обязана остаться бит-в-бит прежней. */
	re_fast_frame.depth[idx] = -co[2];      /* view space: -z и есть расстояние */
	re_fast_frame.mir[4 * idx + 0] = mirror_i;

	if (mirror_i <= 0.0f) return;

	copy_v3_v3(re_fast_frame.pos + 3 * idx, co);
	copy_v3_v3(re_fast_frame.nrm + 3 * idx, vn);
	copy_v3_v3(re_fast_frame.view + 3 * idx, view);

	re_fast_frame.mir[4 * idx + 1] = mirrgb[0];
	re_fast_frame.mir[4 * idx + 2] = mirrgb[1];
	re_fast_frame.mir[4 * idx + 3] = mirrgb[2];
}

/* ★ S1: доля солнца в цвете пикселя + направление на солнце.
 *
 * Пишется ОДИН раз на пиксель: первым для пикселя всегда идёт первичное
 * затенение, а лучи отражений приходят позже и не должны перебивать долю
 * (иначе тень считалась бы по цвету отражения, а не поверхности).
 * Направление одно на кадр, поэтому его можно перезаписывать всем. */
void RE_fast_frame_store_sun(int x, int y, float share, const float dir[3])
{
	int idx;

	if (re_fast_frame.ssun == NULL) return;

	if (dir) {
		re_fast_sun_dir[0] = dir[0];
		re_fast_sun_dir[1] = dir[1];
		re_fast_sun_dir[2] = dir[2];
		re_fast_sun_valid = 1;
	}

	if (x < 0 || y < 0 || x >= re_fast_frame.w || y >= re_fast_frame.h) return;

	idx = y * re_fast_frame.w + x;
	if (re_fast_frame.ssun[idx] < 0.0f)
		re_fast_frame.ssun[idx] = share;
}

/* ★ ГИБРИД (шаг 5): состояние на кадр.
 *
 * bi_ssr_miss[idx] — марш промахнулся, нужен луч BI (ремонтный проход);
 * bi_ssr_hit[idx]  — idx+1 пикселя, в который попал марш (0 = не попал);
 * bi_ssr_repair    — сейчас идёт ремонтный проход частей;
 * bi_ssr_pending   — марш сделан, смешение попаданий отложено до конца ремонта.
 *
 * Маска живёт в КАДРОВЫХ координатах: слияние части (render_result.c) читает её
 * по абсолютным x,y, а не по координатам внутри части. */
static unsigned char *bi_ssr_miss = NULL;
static int           *bi_ssr_hit = NULL;
static int            bi_ssr_w = 0, bi_ssr_h = 0;
static int            bi_ssr_repair = 0;
static int            bi_ssr_pending = 0;
/* Сколько пикселей ремонтный проход реально зашейдил (диагностика: если ноль,
 * значит маска/пропуск настроены неверно и гибрид молча не работает). */
static long           bi_ssr_repaired = 0;

void RE_bi_ssr_repair_note(int n) { bi_ssr_repaired += n; }

/* Гибрид включён, если включён сам SSR в BI и не выставлен UBF_BI_SSR_NORAY
 * (тот оставляет ЧИСТОЕ включение шага 1 — промах = плоский цвет). */
bool RE_bi_ssr_hybrid(void)
{
	static int state = -1;

	if (!RE_bi_ssr_enabled() || RE_shader_fast())
		return false;

	if (state < 0) {
		const char *e = getenv("UBF_BI_SSR_NORAY");
		/* UBF_BI_SSR_NORAY=0 — гибрид, любое другое значение (в т.ч. пустое)
		 * — чистое включение, как в шаге 1. */
		state = (e && e[0] != '0') ? 0 : 1;
	}
	return state == 1;
}

bool RE_bi_ssr_repair_pending(void) { return bi_ssr_pending != 0; }
bool RE_bi_ssr_repair_pass(void) { return bi_ssr_repair != 0; }
void RE_bi_ssr_repair_set(int on) { bi_ssr_repair = on ? 1 : 0; }

/* Нужно ли ПРОПУСТИТЬ пиксель в ремонтном проходе: да, если он не помечен как
 * промах (у него уже есть готовое значение в кадре, и трогать его нельзя).
 * UBF_BI_SSR_REPAIR_ALL=1 — диагностика: ремонтный проход шейдит ВЕСЬ кадр
 * (проверка, что ремонт и слияние по маске в принципе доносят луч до кадра). */
bool RE_bi_ssr_repair_skip(int x, int y)
{
	static int all = -1;

	if (all < 0) all = (getenv("UBF_BI_SSR_REPAIR_ALL") != NULL) ? 1 : 0;
	if (all) return false;

	if (bi_ssr_miss == NULL) return false;
	if (x < 0 || y < 0 || x >= bi_ssr_w || y >= bi_ssr_h) return true;
	return bi_ssr_miss[y * bi_ssr_w + x] == 0;
}

void RE_fast_frame_resolve(struct Render *re)
{
	RenderLayer *rl;
	RenderPass *rpass = NULL;
	float *rect;
	int w, h, stride, x, y, n, mirrored = 0, hits = 0, misses = 0;

	if (re == NULL || re->result == NULL) return;
	if (re_fast_frame.depth == NULL) return;

	w = re_fast_frame.w;
	h = re_fast_frame.h;
	n = w * h;

	/* ★ Диагностика для замеров SSR в BI (обе ручки по умолчанию выключены и
	 * работают только вместе с включённым SSR):
	 *   UBF_BI_SSR_NORESOLVE=1 — марш считается и счётчики печатаются, но
	 *     смешение в кадр НЕ делается: получается «плоский» кадр (лучи зеркал
	 *     уже отключены), то есть база для метрики «SSR ближе к лучу, чем
	 *     плоский цвет» — ровно та метрика, которой мерялся R3 в быстром пути;
	 *   UBF_BI_SSR_DUMP=<путь.pgm> — выгружает маску зеркальных пикселей
	 *     (mir[0] = i, 8 бит). Без маски метрика по всему кадру размывается:
	 *     в боевой сцене зеркал ~13%. */
	const int noresolve = (getenv("UBF_BI_SSR_NORESOLVE") != NULL);
	const char *dumppath = getenv("UBF_BI_SSR_DUMP");

	/* ★ Точность марша в BI: тест попадания «с толщиной» (UBF_BI_SSR_MARCH=thick)
	 * и число шагов (UBF_BI_SSR_STEPS). Оба — только BI: у быстрого пути свой
	 * допуск в приёмке, его марш не трогаем. */
	const int thick = (RE_bi_ssr_enabled() && !RE_shader_fast()) && bi_ssr_march_thick();
	/* ★ Точность марша: 256 шагов вместо 64 — только для BI. У быстрого пути
	 * свой допуск в приёмке (fast_accept), его марш оставлен как был; иначе
	 * смена качества молча сдвинула бы эталонные цифры приёмки. */
	const int steps = (RE_bi_ssr_enabled() && !RE_shader_fast()) ? bi_ssr_steps() : 64;
	/* ★ Бисекция момента пересечения — тоже только BI. */
	const int bisect = (RE_bi_ssr_enabled() && !RE_shader_fast()) ? bi_ssr_bisect() : 0;

#ifdef WITH_UBF_CUDA
	/* ★ ЭТАП A: марш считает GPU, всё остальное (смешение, ремонтный проход,
	 * слияние частей) остаётся на CPU. Так проверка «CPU == GPU» проверяет
	 * ровно ядро, а не всю систему. Любая ошибка CUDA — тихий откат на CPU. */
	int use_gpu = (RE_bi_ssr_enabled() && !RE_shader_fast()) && bi_ssr_use_gpu();
#else
	const int use_gpu = 0;
#endif

	/* ★ ГИБРИД (шаг 5): попадание марша берём с экрана, промах падаем в луч BI.
	 *
	 * Порядок в кадре получается такой:
	 *   1) части с лучами зеркал ВЫКЛЮЧЕНЫ -> кадр = «плоский» цвет, а в
	 *      кадровый G-буфер ложатся параметры отражения;
	 *   2) здесь марш ТОЛЬКО считает попадания и промахи: индексы попавших
	 *      пикселей запоминаются (цвет отражения возьмём позже), промахи
	 *      помечаются в маске;
	 *   3) ремонтный проход частей (pipeline.c) шейдит ТОЛЬКО помеченные
	 *      пиксели и уже с лучами зеркал — там нет другого способа узнать
	 *      геометрию и материал пикселя, кроме того же пиксельного цикла BI;
	 *   4) RE_fast_frame_resolve_blend() смешивает попадания поверх готового
	 *      кадра из снимка «плоского» цвета (re_fast_frame.col).
	 *
	 * Смешение отложено, потому что ремонтный проход правит кадр на месте:
	 * если смешать раньше, слияние части затёрло бы экранное отражение.
	 * Без гибрида (UBF_BI_SSR_NORAY=1) всё как в шаге 1: марш смешивает сам. */
	const int defer = (RE_bi_ssr_hybrid() && !noresolve);

	bi_ssr_pending = 0;
	bi_ssr_repaired = 0;

	if (defer) {
		if (bi_ssr_miss == NULL || bi_ssr_w != w || bi_ssr_h != h) {
			if (bi_ssr_miss) MEM_freeN(bi_ssr_miss);
			if (bi_ssr_hit)  MEM_freeN(bi_ssr_hit);
			bi_ssr_miss = MEM_callocN(sizeof(unsigned char) * (size_t)n, "RE bi ssr miss");
			bi_ssr_hit  = MEM_callocN(sizeof(int) * (size_t)n, "RE bi ssr hit");
			bi_ssr_w = w;
			bi_ssr_h = h;
		}
		else {
			memset(bi_ssr_miss, 0, sizeof(unsigned char) * (size_t)n);
			memset(bi_ssr_hit, 0, sizeof(int) * (size_t)n);
		}
	}

#ifdef WITH_UBF_CUDA
	/* ★ ЭТАП A: считаем марш на GPU. Таблицу времён шага считаем ЗДЕСЬ — тем же
	 * powf и той же формулой, что CPU-эталон, — и отдаём ядру готовой. */
	if (use_gpu) {
		UBFSSRGpu job;
		const float gt0 = 0.05f;
		float gtmax = re->clipend;
		int j;

		if (gtmax <= gt0 * 4.0f) gtmax = 1000.0f;

		if (bi_ssr_gpu_hit == NULL || bi_ssr_gpu_w != w || bi_ssr_gpu_h != h) {
			if (bi_ssr_gpu_hit) MEM_freeN(bi_ssr_gpu_hit);
			bi_ssr_gpu_hit = MEM_mallocN(sizeof(int) * (size_t)n, "RE bi ssr gpu hit");
			bi_ssr_gpu_w = w;
			bi_ssr_gpu_h = h;
		}
		for (j = 0; j < steps; j++)
			bi_ssr_ts[j] = gt0 * powf(gtmax / gt0, (float)(j + 1) / (float)steps);

		memset(&job, 0, sizeof(job));
		job.pos = re_fast_frame.pos;
		job.nrm = re_fast_frame.nrm;
		job.view = re_fast_frame.view;
		job.mir = re_fast_frame.mir;
		job.depth = re_fast_frame.depth;
		job.winmat = (const float *)re->winmat;
		job.ts = bi_ssr_ts;
		job.w = w;
		job.h = h;
		job.winx = re->winx;
		job.winy = re->winy;
		job.steps = steps;
		job.bisect = bisect;
		job.thick = thick;
		job.defer = defer;
		job.hit = defer ? bi_ssr_hit : bi_ssr_gpu_hit;
		job.miss = bi_ssr_miss;

		if (UBF_cuda_ssr_march(&job) != 0) {
			/* Откат: считаем на CPU, кадр обязан быть верным в любом случае. */
			printf("[UBF-CUDA] марш на GPU не удался — считаем на CPU\n");
			fflush(stdout);
			use_gpu = 0;
		}
	}
#endif

	/* Combined-пасс первого слоя — именно он уходит в файл. */
	for (rl = re->result->layers.first; rl && !rpass; rl = rl->next) {
		RenderPass *rp;
		for (rp = rl->passes.first; rp; rp = rp->next) {
			if (STREQ(rp->name, RE_PASSNAME_COMBINED)) { rpass = rp; break; }
		}
	}
	if (!rpass || rpass->rect == NULL) return;
	if (rpass->rectx < w || rpass->recty < h) return;

	rect = rpass->rect;
	stride = rpass->rectx;

	/* Снимок кадра: резолв пишет в тот же буфер, из которого читают лучи. */
	memcpy(re_fast_frame.col, rect, sizeof(float) * 4 * n);

#ifdef WITH_UBF_CUDA
	/* Готовые попадания от GPU (в defer-режиме это те же bi_ssr_hit). */
	const int *gpu_hit = use_gpu ? (defer ? bi_ssr_hit : bi_ssr_gpu_hit) : NULL;
#endif

	/* ★ Диагностика точности: UBF_BI_SSR_HITDUMP=<путь> выгружает массив
	 * попаданий (int на пиксель, индекс+1, 0 = промах). Нужен, чтобы сравнивать
	 * CPU-марш и GPU-марш ПО ИНДЕКСАМ, а не по картинке: картинка скажет «тут
	 * другой цвет», а массив — «выборка уехала на 8 пикселей», и это разные
	 * диагнозы. */
	int *dump_hits = NULL;
	const char *hitdump = getenv("UBF_BI_SSR_HITDUMP");
	if (hitdump) dump_hits = MEM_callocN(sizeof(int) * (size_t)n, "RE bi ssr hitdump");

	for (y = 0; y < h; y++) {
		for (x = 0; x < w; x++) {
			const int idx = y * w + x;
			const float i = re_fast_frame.mir[4 * idx + 0];
			const float *P0, *N, *V;
			float D[3], t, t0, tmax, dd, tprev, tnear, tfar, thfar;
			int k, b, hit = -1;

			if (i <= 0.0f) continue;
			mirrored++;

#ifdef WITH_UBF_CUDA
			if (use_gpu) {
				/* ★ ЭТАП A: попадание и промах уже посчитаны ядром (ubf_cuda.cu),
				 * маска промахов лежит в bi_ssr_miss. Дальше — общий код ниже:
				 * смешение (или запись индекса для ремонта) как было. */
				hit = gpu_hit[idx] - 1;
				if (hit < 0 && defer && bi_ssr_miss[idx]) misses++;
			}
			else {
#endif
			P0 = re_fast_frame.pos + 3 * idx;
			N  = re_fast_frame.nrm + 3 * idx;
			V  = re_fast_frame.view + 3 * idx;

			/* Отражение: D = V - 2(V·N)N (V из камеры в точку). */
			dd = 2.0f * dot_v3v3(V, N);
			D[0] = V[0] - dd * N[0];
			D[1] = V[1] - dd * N[1];
			D[2] = V[2] - dd * N[2];

			/* Луч, уходящий за камеру, в сцене ничего не встретит. */
			if (D[2] >= 0.0f) {
				if (defer) {
					bi_ssr_miss[idx] = 1;
					misses++;
				}
				continue;
			}

			t0 = 0.05f;
			tmax = re->clipend;
			if (tmax <= t0 * 4.0f) tmax = 1000.0f;

			/* Экспоненциальный шаг: у поверхности нужно точнее, чем вдали. */
			tprev = 0.0f;
			thfar = 0.0f;
			tnear = 0.0f;
			tfar = 0.0f;
			for (k = 1; k <= steps; k++) {
				float stepd;

				t = t0 * powf(tmax / t0, (float)k / (float)steps);
				/* Длина шага в единицах глубины: ею меряется «толщина». */
				stepd = fabsf(t - tprev) * fabsf(D[2]);
				thfar = thick ? (stepd * 2.0f + 0.01f) : 0.0f;

				hit = bi_ssr_march_probe(re, P0, D, t, thfar, w, h);
				if (hit >= 0) {
					tnear = tprev;       /* последняя выборка «до» поверхности */
					tfar = t;            /* первая выборка «за» неё */
					break;
				}
				tprev = t;
			}

			/* ★ Бисекция (только BI): уточняем МОМЕНТ пересечения между
			 * выборкой «до» и выборкой «за». Взятый сразу «за» пиксель — это
			 * пиксель далеко за пересечением (шаг экспоненциальный, вдали он
			 * длинный), и цвет оттуда бывает чужой; после бисекции момент
			 * сжат до (tfar - tnear)/2^bisect и цвет берётся с пикселя в
			 * самой точке пересечения.
			 *
			 * Уточнять нечего, если пересечения не было: без этой проверки на
			 * промахе в [tnear, tfar] лежал МУСОР (переменные заполняются только
			 * при попадании), и бисекция изредка «находила» лишний пиксель —
			 * на зеркальной пробе 256 так нашёлся ровно один лишний пиксель из
			 * 65536. Нашлось это сравнением с GPU-маршем (тот же счёт, но
			 * инициализированный нулём): см. HANDOFF_GPU_RECON.md, этап A1. */
			if (hit >= 0) for (b = 0; b < bisect; b++) {
				const float tm = 0.5f * (tnear + tfar);
				const int jm = bi_ssr_march_probe(re, P0, D, tm, thfar, w, h);

				if (jm >= 0) {
					tfar = tm;
					hit = jm;
				}
				else {
					tnear = tm;
				}
			}
#ifdef WITH_UBF_CUDA
			}
#endif

			if (hit >= 0) {
				const float *flat = re_fast_frame.col + 4 * idx;
				const float *smp  = re_fast_frame.col + 4 * hit;
				float *dst = rect + 4 * ((y * stride) + x);
				int c;

				if (dump_hits) dump_hits[idx] = hit + 1;

				if (defer) {
					/* Смешение отложено до конца ремонта: ремонтный проход
					 * правит кадр на месте, и слияние части затирало бы
					 * экранное отражение. Цвет отражения берём из снимка
					 * «плоского» кадра (re_fast_frame.col). */
					bi_ssr_hit[idx] = hit + 1;
				}
				else if (!noresolve) {
					for (c = 0; c < 3; c++) {
						const float mirc = re_fast_frame.mir[4 * idx + 1 + c];
						dst[c] = (1.0f - i) * flat[c] + i * mirc * smp[c];
					}
				}
				hits++;
			}
			else if (defer && !(use_gpu && bi_ssr_miss[idx])) {
				/* Промах: пиксель уходит в ремонтный проход к лучу BI.
				 * При GPU-марше маска уже заполнена ядром и посчитана выше. */
				bi_ssr_miss[idx] = 1;
				misses++;
			}
		}
	}

	/* ★ Дамп индексов попаданий (UBF_BI_SSR_HITDUMP): n int, индекс+1. */
	if (dump_hits && hitdump) {
		FILE *hf = fopen(hitdump, "wb");

		if (hf) {
			fwrite(dump_hits, sizeof(int), (size_t)n, hf);
			fclose(hf);
			printf("[BI-SSR] дамп попаданий: %s (%d пикселей, %dx%d)\n",
			       hitdump, n, w, h);
			fflush(stdout);
		}
		else {
			printf("[BI-SSR] не удалось открыть дамп попаданий: %s\n", hitdump);
			fflush(stdout);
		}
	}
	if (dump_hits) {
		MEM_freeN(dump_hits);
		dump_hits = NULL;
	}

	/* ★ S1: ТЕНИ ОТ СОЛНЦА.
	 *
	 * Идём по кадру ПОСЛЕ зеркал: к этому моменту re_fast_frame.depth — уже
	 * полный z-буфер кадра, а ssun — доля солнца в цвете каждого пикселя,
	 * которую посчитал шейдер. Направление на солнце одно на весь кадр.
	 *
	 * Логика затенения: из точки поверхности шагаем К СОЛНЦУ; если на пути
	 * встречается геометрия, которая на экране ближе к камере, чем наш шаг,
	 * значит свет до точки не доходит. Тест ровно тот же, что у зеркального
	 * луча (wcl > zd), только направление другое.
	 *
	 * Затенённый пиксель теряет ДОЛЮ СОЛНЦА, а не гаснет целиком: ambient и
	 * точечные лампы остаются. */
	if (re_fast_shadow_enabled() && re_fast_frame.ssun && re_fast_sun_valid) {
		int lit = 0, dark = 0;
		const float *D = re_fast_sun_dir;
		const float t0s = 0.5f;
		float tmaxs = re->clipend;

		if (tmaxs <= t0s * 4.0f) tmaxs = 1000.0f;

		for (y = 0; y < h; y++) {
			for (x = 0; x < w; x++) {
				const int idx = y * w + x;
				const float s = re_fast_frame.ssun[idx];
				const float *P0;
				float sh = 1.0f;
				int k;

				if (s <= 0.02f) continue;                 /* солнца в цвете нет */
				if (re_fast_frame.depth[idx] >= 1e29f) continue;   /* небо */
				lit++;

				P0 = re_fast_frame.pos + 3 * idx;

				for (k = 1; k <= RE_FAST_SHADOW_STEPS; k++) {
					float P[3], wcl, sx, sy, zd;
					int px, py, j;

					/* шаг экспоненциальный: у поверхности точнее, вдали грубее */
					{
						const float t = t0s * powf(tmaxs / t0s,
						                           (float)k / (float)RE_FAST_SHADOW_STEPS);
						P[0] = P0[0] + t * D[0];
						P[1] = P0[1] + t * D[1];
						P[2] = P0[2] + t * D[2];
					}

					if (P[2] >= -0.001f) break;           /* ушли за камеру */
					wcl = -P[2];
					sx = 0.5f * re->winx * ((re->winmat[0][0] * P[0] + re->winmat[3][0]) / wcl + 1.0f);
					sy = 0.5f * re->winy * ((re->winmat[1][1] * P[1] + re->winmat[3][1]) / wcl + 1.0f);

					px = (int)(sx * ((float)w / (float)re->winx));
					py = (int)(sy * ((float)h / (float)re->winy));
					if (px < 0 || py < 0 || px >= w || py >= h) continue;

					j = py * w + px;
					zd = re_fast_frame.depth[j];
					if (zd >= 1e29f) continue;            /* в той точке небо */

					if (wcl > zd * 1.01f + 0.05f) { sh = 0.0f; break; }
				}

				if (sh < 1.0f) {
					float *dst = rect + 4 * ((y * stride) + x);
					int c;

					dark++;
					for (c = 0; c < 3; c++) dst[c] *= (1.0f - s);
				}
			}
		}

		printf("[FAST-SHADOW] солнце: затенено %d пикселей из %d, у которых доля солнца > 2%%\n",
		       dark, lit);
	}

	/* ★ S6: состав света нашего шейдера — сколько даёт ambient, сколько всё
	 * вместе и сколько из этого солнце. */
	if (re_fast_stat_npx > 0) {
		const double np = (double)re_fast_stat_npx;
		int bi;

		printf("[FAST-LIGHT] пикселей=%ld ambient=%.1f всего=%.1f солнце=%.1f (доля солнца %.2f%%)\n",
		       re_fast_stat_npx, re_fast_stat_amb / np, re_fast_stat_tot / np,
		       re_fast_stat_sunsum / np,
		       100.0 * re_fast_stat_sunsum / (re_fast_stat_tot > 1e-9 ? re_fast_stat_tot : 1.0));
		printf("[FAST-LIGHT] доля солнца по корзинам 0-12.5%% ... 87.5-100%%: ");
		for (bi = 0; bi < 8; bi++) printf("%.2f%% ", 100.0 * re_fast_stat_hist[bi] / np);
		printf("\n");
	}

	printf("[%s-SSR] экранных зеркал: %d пикселей, попаданий: %d",
	       RE_shader_fast() ? "FAST" : "BI", mirrored, hits);
	if (defer)
		printf(", в луч уходят: %d%s", misses,
		       (misses > 0) ? " (гибрид: ремонтный проход)" : " (гибрид: промахов нет)");
	printf("\n");

	/* ★ Гибрид: смешение попаданий откладывается до конца ремонтного прохода
	 * (его запускает pipeline.c по RE_bi_ssr_repair_pending()). */
	if (defer && (misses > 0 || getenv("UBF_BI_SSR_REPAIR_ALL") != NULL))
		bi_ssr_pending = 1;

	if (dumppath) {
		FILE *f = fopen(dumppath, "wb");

		if (f) {
			unsigned char v;
			int idx;

			fprintf(f, "P5\n%d %d\n255\n", w, h);
			for (idx = 0; idx < n; idx++) {
				const float i = re_fast_frame.mir[4 * idx + 0];

				v = (i <= 0.0f) ? 0 : (i >= 1.0f ? 255 : (unsigned char)(i * 255.0f + 0.5f));
				fputc((int)v, f);
			}
			fclose(f);
			printf("[BI-SSR] маска зеркал выгружена: %s\n", dumppath);
		}
		else {
			printf("[BI-SSR] маску зеркал НЕ удалось открыть на запись: %s\n", dumppath);
		}
	}
}

/* ★ ГИБРИД (шаг 5), часть 2: смешение ПОПАДАНИЙ после ремонтного прохода.
 *
 * К этому моменту кадр содержит: в попавших пикселях — «плоский» цвет (марш
 * смешение отложил), в промахнувшихся — результат луча BI из ремонтного
 * прохода. Смешивать надо только попадания, и брать «плоский» цвет из снимка
 * (re_fast_frame.col), а не из кадра: снимок — это ровно то, что шейдинг
 * положил до всякого отражения, и он не зависит от порядка ремонта. */
void RE_fast_frame_resolve_blend(struct Render *re)
{
	RenderLayer *rl;
	RenderPass *rpass = NULL;
	float *rect;
	int w, h, stride, x, y, n, blended = 0;

	bi_ssr_pending = 0;
	bi_ssr_repair = 0;

	/* Диагностика: UBF_BI_SSR_NOBLEND=1 — смешение не делается, кадр остаётся
	 * таким, каким его собрал ремонтный проход. Нужна, чтобы отделить «ремонт
	 * не шейдит» от «слияние не доносит»: если кадр при этом равен плоскому,
	 * значит луча в ремонте не было. */
	const int noblend = (getenv("UBF_BI_SSR_NOBLEND") != NULL);

	if (re == NULL || re->result == NULL) return;
	if (re_fast_frame.depth == NULL || bi_ssr_hit == NULL) return;

	w = re_fast_frame.w;
	h = re_fast_frame.h;
	n = w * h;

	for (rl = re->result->layers.first; rl && !rpass; rl = rl->next) {
		RenderPass *rp;
		for (rp = rl->passes.first; rp; rp = rp->next) {
			if (STREQ(rp->name, RE_PASSNAME_COMBINED)) { rpass = rp; break; }
		}
	}
	if (!rpass || rpass->rect == NULL) return;
	if (rpass->rectx < w || rpass->recty < h) return;

	rect = rpass->rect;
	stride = rpass->rectx;

	/* ★ Диагностика: UBF_BI_SSR_DEBUG=1 — что реально лежит в кадре после
	 * ремонтного прохода (сверка с PNG лучевого/плоского варианта). */
	if (getenv("UBF_BI_SSR_DEBUG")) {
		static const int probe[4][2] = { {80,137}, {132,97}, {178,64}, {176,58} };
		int k;

		for (k = 0; k < 4; k++) {
			const int px = probe[k][0], py = probe[k][1];
			const float *d;

			if (px >= w || py >= h) continue;
			d = rect + 4 * ((py * stride) + px);
			printf("[BI-SSR] кадр(%d,%d): %.4f %.4f %.4f  hit=%d mask=%d\n",
			       px, py, d[0], d[1], d[2],
			       bi_ssr_hit[py * w + px] > 0,
			       bi_ssr_miss ? (int)bi_ssr_miss[py * w + px] : -1);
		}
		fflush(stdout);
	}

	if (noblend) {
		printf("[BI-SSR] гибрид: смешение ВЫКЛЮЧЕНО диагностикой "
		       "(шейдинг ремонта — %ld пикселей)\n", bi_ssr_repaired);
		fflush(stdout);
		return;
	}

	for (y = 0; y < h; y++) {
		for (x = 0; x < w; x++) {
			const int idx = y * w + x;
			const int hit = bi_ssr_hit[idx] - 1;
			const float i = re_fast_frame.mir[4 * idx + 0];
			const float *flat, *smp;
			float *dst;
			int c;

			if (hit < 0 || i <= 0.0f) continue;

			flat = re_fast_frame.col + 4 * idx;
			smp  = re_fast_frame.col + 4 * hit;
			dst  = rect + 4 * ((y * stride) + x);

			for (c = 0; c < 3; c++) {
				const float mirc = re_fast_frame.mir[4 * idx + 1 + c];
				dst[c] = (1.0f - i) * flat[c] + i * mirc * smp[c];
			}
			blended++;
		}
	}

	printf("[BI-SSR] гибрид: экранное отражение смешано в %d пикселей "
	       "(остальные зеркала ушли в луч BI; шейдинг ремонта — %ld пикселей)\n",
	       blended, bi_ssr_repaired);
	fflush(stdout);
}

void RE_fast_frame_end(void)
{
	if (re_fast_frame.depth) MEM_freeN(re_fast_frame.depth);
	if (re_fast_frame.pos)   MEM_freeN(re_fast_frame.pos);
	if (re_fast_frame.nrm)   MEM_freeN(re_fast_frame.nrm);
	if (re_fast_frame.view)  MEM_freeN(re_fast_frame.view);
	if (re_fast_frame.mir)   MEM_freeN(re_fast_frame.mir);
	if (re_fast_frame.col)   MEM_freeN(re_fast_frame.col);
	if (re_fast_frame.ssun)  MEM_freeN(re_fast_frame.ssun);

	re_fast_frame.depth = NULL;
	re_fast_frame.pos   = NULL;
	re_fast_frame.nrm   = NULL;
	re_fast_frame.view  = NULL;
	re_fast_frame.mir   = NULL;
	re_fast_frame.col   = NULL;
	re_fast_frame.ssun  = NULL;

	/* ★ Гибрид: маска и индексы живут ровно один кадр. */
	if (bi_ssr_miss) { MEM_freeN(bi_ssr_miss); bi_ssr_miss = NULL; }
	if (bi_ssr_hit)  { MEM_freeN(bi_ssr_hit);  bi_ssr_hit = NULL; }
	bi_ssr_w = bi_ssr_h = 0;
	bi_ssr_pending = 0;
	bi_ssr_repair = 0;
	re_fast_frame.w = re_fast_frame.h = 0;
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
