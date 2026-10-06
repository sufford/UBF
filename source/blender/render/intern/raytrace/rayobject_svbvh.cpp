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
 * along with this program; if not, write to the Free Software Foundation,
 * Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 *
 * The Original Code is Copyright (C) 2009 Blender Foundation.
 * All rights reserved.
 */

/** \file blender/render/intern/raytrace/rayobject_svbvh.cpp
 *  \ingroup render
 */


#include "MEM_guardedalloc.h"

#include "BLI_utildefines.h"
#include "BLI_task.h"               /* ★ C1c: параллельная сборка дерева */

#include "vbvh.h"
#include "svbvh.h"
#include "reorganize.h"

#include <stdlib.h>                 /* ★ PROF: getenv для UBF_BVH_PLAIN */
#include "../source/RE_Prof.h"      /* ★ PROF: разложение сборки дерева */

#ifdef __SSE__

#define DFS_STACK_SIZE  256

struct SVBVHTree {
	RayObject rayobj;

	SVBVHNode *root;
	MemArena *node_arena;

	float cost;
	RTBuilder *builder;
};

/*
 * Cost to test N childs
 */
struct PackCost {
	float operator()(int n)
	{
		return (n / 4) + ((n % 4) > 2 ? 1 : n % 4);
	}
};


/* ★ C1c: ПАРАЛЛЕЛЬНАЯ СБОРКА ДВОИЧНОГО VBVH.
 *
 * Замер (C1): сборка дерева боевой сцены — 2.74 s из 3.96 s фазы raytree и из
 * 10.0 s кадра, и это чистая рекурсия по 2 млн граней В ОДНОМ ПОТОКЕ
 * (rtbuild 0.67 s, упаковка 0.15 s, реорганизация 0.09 s — остальное).
 *
 * Как делим работу: верхние уровни дерева строим последовательно до среза
 * ПО УРОВНЮ (BFS), пока кусков не станет >= запрошенного. Срез по уровню важен
 * тем, что связка готовых кусков («хребет») тогда в точности повторяет верхние
 * уровни последовательной сборки: те же узлы, те же дети, те же bb. Куски
 * собираются параллельно, каждый в своей арене и со своей заготовкой свипа
 * (общая заготовка здесь была бы гонкой). Форма дерева получается той же,
 * поэтому картинка обязана совпасть битово — это и проверяется (max=0).
 *
 * Арены кусков живут до реорганизации (узлы из них читает упаковщик), поэтому
 * вызывающий получает их списком и освобождает сам.
 *
 * Ручка: UBF_BVH_PAR=N, N>1 — сколько кусков хотим (степень двойки даёт ровный
 * срез). По умолчанию 32 (боевое значение по решению человека); UBF_BVH_PAR=0
 * возвращает прежнюю последовательную сборку. */
/* ★ UBF: значения из .blend (см. RE_rayobject_ubf_bvh_set в rayobject_internal.h).
 * ubf_bvh_par == 0 в файле означает «как раньше»: 32 куска. Это же значение
 * получают файлы, сохранённые до появления поля. Геттеры ниже: переменная
 * окружения важнее файла — этого требуют стенды, приёмка и замеры. */
static int ubf_bvh_par_file = 32;
static int ubf_bvh_rtsort_file = 1;
static int ubf_bvh_cutpar_file = 1;

void RE_rayobject_ubf_bvh_set(int par_chunks, int rtsort, int cutpar)
{
	ubf_bvh_par_file = par_chunks;
	ubf_bvh_rtsort_file = rtsort;
	ubf_bvh_cutpar_file = cutpar;
}

int RE_rayobject_ubf_bvh_rtsort(void)
{
	const char *e = getenv("UBF_BVH_RTSORT");

	if (e)
		return (e[0] == '0') ? 0 : 1;
	return ubf_bvh_rtsort_file;
}

static int ubf_bvh_par_env(void)
{
	const char *e = getenv("UBF_BVH_PAR");
	int env;

	if (!e)
		return (ubf_bvh_par_file > 0) ? ubf_bvh_par_file : 32;

	env = atoi(e);
	if (env < 0) env = 0;
	if (env > 4096) env = 4096;
	return env;
}

static void ubf_par_thread_account(double *t_thread, const ParallelRangeTLS *tls, double dt);

struct ParBuildData {
	RTBuilder *chunks;
	OVBVHNode **roots;
	MemArena **arenas;
	RayObjectControl *control;
	double *t_thread;
};

static void par_build_one(void *userdata, const int i, const ParallelRangeTLS *tls)
{
	ParBuildData *d = (ParBuildData *)userdata;
	double t0 = RE_prof_tick();
	BuildBinaryVBVH<OVBVHNode> builder(d->arenas[i], d->control);

	d->roots[i] = builder.transform(&d->chunks[i]);

	ubf_par_thread_account(d->t_thread, tls, RE_prof_tick() - t0);
}

/* ★ Диагностика C1f: сколько времени потоки реально проводят в работе.
 *
 * Нужна, чтобы отличить «потоки простаивают на аллокаторе» от «потоки работают, но
 * упираются в память»: в первом случае сумма по потокам близка к стене, во втором —
 * заметно больше. Считается только под UBF_PROF (и стоит копейки). */
#define PAR_THREADS 128

static void ubf_par_thread_account(double *t_thread, const ParallelRangeTLS *tls, double dt)
{
	int id = tls ? tls->thread_id : -1;

	if (t_thread && id >= 0 && id < PAR_THREADS)
		t_thread[id] += dt;
}

static void ubf_par_threads_report(const char *what, double wall, double *t_thread)
{
	double sum = 0.0, tmax = 0.0;
	int i, n = 0;

	if (!RE_prof_active() || !t_thread)
		return;

	for (i = 0; i < PAR_THREADS; i++) {
		sum += t_thread[i];
		if (t_thread[i] > tmax)
			tmax = t_thread[i];
		if (t_thread[i] > 0.0)
			n++;
	}

	printf("[PROF] ИТОГ BVH-потоки (%s): стена=%.4f s сумма по %d потокам=%.4f s -> "
	       "занято в среднем %.1f потоков (самый занятый %.4f s)\n",
	       what, wall, n, sum, wall > 0.0 ? sum / wall : 0.0, tmax);
}

/* ★ C1e: разбиение узла ОДНОГО уровня — своя задача.
 *
 * Узлы уровня независимы (диапазоны в отсортированных массивах не пересекаются), а
 * стоимость уровня примерно постоянна, поэтому уровень можно делить между потоками.
 * Корневое разбиение (один узел) остаётся серийным — его распараллелить нечем.
 *
 * Заготовку свипа каждая задача берёт свою: ставим maxsize и обнуляем указатель,
 * чтобы сработала ленивая ветка C1a, а после разбиения освобождаем её и ОБНУЛЯЕМ у
 * детей — иначе дети унаследуют уже освобождённый указатель (rtbuild_get_child его
 * копирует).
 *
 * Ручка: UBF_BVH_CUTPAR (по умолчанию ВКЛ — параллельный срез, битово тот же;
 * UBF_BVH_CUTPAR=0 возвращает прежний серийный срез). */
struct ParCutData {
	RTBuilder *in;
	RTBuilder *out;
	int *status;
	double *t_thread;
};

static int ubf_bvh_cutpar_env(void)
{
	const char *e = getenv("UBF_BVH_CUTPAR");

	if (e)
		return (e[0] == '0') ? 0 : 1;
	return ubf_bvh_cutpar_file;
}

static void ubf_bvh_cut_one(void *userdata, const int i, const ParallelRangeTLS *tls)
{
	ParCutData *d = (ParCutData *)userdata;
	RTBuilder *b = &d->in[i];
	double t0 = RE_prof_tick();
	int ok = 0;

	d->status[i] = 0;

	if (rtbuild_size(b) > 1) {
		RTBuilder a, c;

		b->primitives.maxsize = rtbuild_size(b);
		b->sweep_scratch = NULL;
		b->sweep_scratch_size = 0;

		if (rtbuild_heuristic_object_split(b, 2) == 2) {
			rtbuild_get_child(b, 0, &a);
			rtbuild_get_child(b, 1, &c);

			a.sweep_scratch = NULL;
			a.sweep_scratch_size = 0;
			c.sweep_scratch = NULL;
			c.sweep_scratch_size = 0;

			d->out[2 * i] = a;
			d->out[2 * i + 1] = c;
			ok = 1;
		}
	}

	if (b->sweep_scratch) {
		MEM_freeN(b->sweep_scratch);
		b->sweep_scratch = NULL;
		b->sweep_scratch_size = 0;
	}

	d->status[i] = ok;
	ubf_par_thread_account(d->t_thread, tls, RE_prof_tick() - t0);
}

/* «Хребет»: ровно те узлы верхних уровней, что построила бы рекурсия. */
static OVBVHNode *par_build_spine(MemArena *arena, OVBVHNode **roots, int n)
{
	OVBVHNode *node, *a, *b;
	int mid;

	if (n == 1)
		return roots[0];

	mid = n / 2;
	a = par_build_spine(arena, roots, mid);
	b = par_build_spine(arena, roots + mid, n - mid);

	node = (OVBVHNode *)BLI_memarena_alloc(arena, sizeof(OVBVHNode));
	assert(RE_rayobject_isAligned(node));

	node->sibling = NULL;
	node->child = a;
	a->sibling = b;
	b->sibling = NULL;

	INIT_MINMAX(node->bb, node->bb + 3);
	DO_MIN(a->bb, node->bb);
	DO_MAX(a->bb + 3, node->bb + 3);
	DO_MIN(b->bb, node->bb);
	DO_MAX(b->bb + 3, node->bb + 3);

	return node;
}

/* Возвращает корень или NULL (тогда вызывающий собирает последовательно).
 * В *r_arenas / *r_narenas отдаёт арены кусков — их освобождает вызывающий
 * ПОСЛЕ реорганизации (до неё узлы из них ещё читаются). */
static OVBVHNode *par_build_binary_vbvh(MemArena *arena1, RayObjectControl *control,
                                        RTBuilder *root_builder, int want_chunks,
                                        MemArena ***r_arenas, int *r_narenas)
{
	RTBuilder *cur, *next;
	MemArena **arenas;
	OVBVHNode **roots;
	ParBuildData data;
	ParCutData cut;
	ParallelRangeSettings settings;
	int n_cur, i, depth = 0;
	int target;                                        /* кусков, округлённых до 2^n */
	int *status;
	int cutpar;
	double *t_thread;
	double t_cutpar = 0.0;
	double t0, t_serial, t_par;

	*r_arenas = NULL;
	*r_narenas = 0;

	/* Кусков должно быть степенью двойки: срез делается только по ПОЛНОМУ уровню,
	 * иначе связка перестаёт повторять верхние уровни последовательной сборки. */
	target = 1;
	while (target < want_chunks)
		target <<= 1;

	cur = (RTBuilder *)MEM_mallocN(sizeof(RTBuilder) * target, "svbvh par level");
	next = (RTBuilder *)MEM_mallocN(sizeof(RTBuilder) * target, "svbvh par level2");
	arenas = (MemArena **)MEM_mallocN(sizeof(MemArena *) * target, "svbvh par arenas");
	roots = (OVBVHNode **)MEM_mallocN(sizeof(OVBVHNode *) * target, "svbvh par roots");
	status = (int *)MEM_mallocN(sizeof(int) * target, "svbvh par cut status");
	t_thread = (double *)MEM_callocN(sizeof(double) * PAR_THREADS, "svbvh par thread time");

	cutpar = ubf_bvh_cutpar_env();
	cut.in = NULL;
	cut.out = NULL;
	cut.status = status;
	cut.t_thread = t_thread;

	BLI_parallel_range_settings_defaults(&settings);
	settings.use_threading = true;

	/* 1. Последовательный срез по уровню: делим ВСЕ куски уровня, потом идём ниже.
	 *
	 * Только полный уровень: если на очередном уровне уже есть готовые листья, то
	 * узлов на нём меньше 2^depth и связка кусков перестала бы повторять верхние
	 * уровни последовательной сборки — дерево вышло бы ДРУГИМ. Это не теория: на
	 * сцене зеркала (приёмка, fb\accept.ps1) дерево неполное, и срез «как попало»
	 * дал max=255 против эталона при max=0 на боевой сцене. Поэтому при первом же
	 * неполном уровне останавливаемся на предыдущем. */
	t0 = RE_prof_tick();
	cur[0] = *root_builder;
	n_cur = 1;
	while (n_cur < target && depth < 24) {
		int n_next = 0;

		for (i = 0; i < n_cur; i++)
			if (rtbuild_size(&cur[i]) <= 1)
				break;
		if (i < n_cur)
			break;

		if (cutpar && n_cur > 1) {
			/* ★ C1e: узлы уровня — параллельно, у каждой задачи своя заготовка */
			double tc = RE_prof_tick();

			cut.in = cur;
			cut.out = next;

			BLI_task_parallel_range(0, n_cur, &cut, ubf_bvh_cut_one, &settings);
			t_cutpar += RE_prof_tick() - tc;

			for (i = 0; i < n_cur; i++)
				if (!status[i])
					break;
			if (i < n_cur)
				break;                 /* неполный уровень — стоп на предыдущем */

			n_next = 2 * n_cur;
		}
		else {
			for (i = 0; i < n_cur; i++) {
				RTBuilder a, b;

				if (rtbuild_heuristic_object_split(&cur[i], 2) != 2)
					break;
				rtbuild_get_child(&cur[i], 0, &a);
				rtbuild_get_child(&cur[i], 1, &b);
				next[n_next++] = a;
				next[n_next++] = b;
			}

			if (n_next != 2 * n_cur)
				break;
		}

		{ RTBuilder *t = cur; cur = next; next = t; }
		n_cur = n_next;
		depth++;
	}
	t_serial = RE_prof_tick() - t0;

	if (n_cur < 2) {
		MEM_freeN(cur);
		MEM_freeN(next);
		MEM_freeN(arenas);
		MEM_freeN(roots);
		MEM_freeN(status);
		MEM_freeN(t_thread);
		return NULL;
	}

	/* 2. Кускам — свои арены и свои заготовки свипа (общая была бы гонкой). */
	for (i = 0; i < n_cur; i++) {
		arenas[i] = BLI_memarena_new(BLI_MEMARENA_STD_BUFSIZE, "svbvh par arena");
		BLI_memarena_use_malloc(arenas[i]);

		cur[i].primitives.maxsize = rtbuild_size(&cur[i]);
		cur[i].sweep_scratch = NULL;
		cur[i].sweep_scratch_size = 0;
	}

	/* 3. Куски — параллельно. */
	data.chunks = cur;
	data.roots = roots;
	data.arenas = arenas;
	data.control = control;
	data.t_thread = t_thread;

	BLI_parallel_range_settings_defaults(&settings);
	settings.use_threading = true;

	t0 = RE_prof_tick();
	BLI_task_parallel_range(0, n_cur, &data, par_build_one, &settings);
	t_par = RE_prof_tick() - t0;

	/* 4. Заготовки свипа кусков больше не нужны (у корневого билдера своя). */
	for (i = 0; i < n_cur; i++) {
		if (cur[i].sweep_scratch) {
			MEM_freeN(cur[i].sweep_scratch);
			cur[i].sweep_scratch = NULL;
		}
	}

	/* 5. Хребет. */
	{
		OVBVHNode *root = NULL;
		int ok = 1;

		for (i = 0; i < n_cur; i++)
			if (!roots[i])
				ok = 0;

		if (ok)
			root = par_build_spine(arena1, roots, n_cur);

		if (RE_prof_active())
			printf("[PROF] ИТОГ BVH-пара: кусков=%d срез(послед.)=%.4f s куски(парал.)=%.4f s\n",
			       n_cur, t_serial, t_par);

		ubf_par_threads_report("срез+куски", t_cutpar + t_par, t_thread);

		MEM_freeN(cur);
		MEM_freeN(next);
		MEM_freeN(roots);
		MEM_freeN(status);
		MEM_freeN(t_thread);

		if (ok) {
			*r_arenas = arenas;
			*r_narenas = n_cur;
			return root;
		}

		for (i = 0; i < n_cur; i++)
			BLI_memarena_free(arenas[i]);
		MEM_freeN(arenas);
		return NULL;
	}
}

template<>
void bvh_done<SVBVHTree>(SVBVHTree *obj)
{
	/* ★ PROF: разложение сборки дерева по фазам + переключатель «простой» ветки.
	 * По умолчанию (UBF_BVH_PLAIN не задан) поведение прежнее — оптимальная
	 * упаковка. Простая ветка существовала в коде за `if (0)` и не вызывалась. */
	static int plain_env = -1;
	double t0, t_rt, t_build = 0.0, t_pack = 0.0, t_push = 0.0, t_reorg = 0.0;

	if (plain_env < 0)
		plain_env = getenv("UBF_BVH_PLAIN") ? 1 : 0;

	t0 = RE_prof_tick();                                   /* ★ PROF */
	rtbuild_done(obj->builder, &obj->rayobj.control);
	t_rt = RE_prof_tick() - t0;                            /* ★ PROF */

	//TODO find a away to exactly calculate the needed memory
	MemArena *arena1 = BLI_memarena_new(BLI_MEMARENA_STD_BUFSIZE, "svbvh arena");
	BLI_memarena_use_malloc(arena1);

	MemArena *arena2 = BLI_memarena_new(BLI_MEMARENA_STD_BUFSIZE, "svbvh arena2");
	BLI_memarena_use_malloc(arena2);
	BLI_memarena_use_align(arena2, 16);

	//Build and optimize the tree
	if (plain_env) {
		VBVHNode *root;

		t0 = RE_prof_tick();                               /* ★ PROF */
		root = BuildBinaryVBVH<VBVHNode>(arena1, &obj->rayobj.control).transform(obj->builder);
		t_build = RE_prof_tick() - t0;                      /* ★ PROF */

		if (RE_rayobjectcontrol_test_break(&obj->rayobj.control)) {
			BLI_memarena_free(arena1);
			BLI_memarena_free(arena2);
			return;
		}

		t0 = RE_prof_tick();                               /* ★ PROF */
		reorganize(root);
		remove_useless(root, &root);
		bvh_refit(root);
		t_pack = RE_prof_tick() - t0;                       /* ★ PROF */

		t0 = RE_prof_tick();                               /* ★ PROF */
		pushup(root);
		pushdown(root);
		pushup_simd<VBVHNode, 4>(root);
		t_push = RE_prof_tick() - t0;                       /* ★ PROF */

		t0 = RE_prof_tick();                               /* ★ PROF */
		obj->root = Reorganize_SVBVH<VBVHNode>(arena2).transform(root);
		t_reorg = RE_prof_tick() - t0;                      /* ★ PROF */
	}
	else {
		//Finds the optimal packing of this tree using a given cost model
		//TODO this uses quite a lot of memory, find ways to reduce memory usage during building
		OVBVHNode *root;
		MemArena **par_arenas = NULL;                      /* ★ C1c */
		int n_par_arenas = 0;                              /* ★ C1c */
		int par_chunks = ubf_bvh_par_env();                /* ★ C1c */

		t0 = RE_prof_tick();                               /* ★ PROF */
		if (par_chunks > 1) {
			root = par_build_binary_vbvh(arena1, &obj->rayobj.control, obj->builder,
			                             par_chunks, &par_arenas, &n_par_arenas);
			/* Не вышло (делить нечего) — откат на прежнюю сборку. */
			if (!root && !RE_rayobjectcontrol_test_break(&obj->rayobj.control))
				root = BuildBinaryVBVH<OVBVHNode>(arena1, &obj->rayobj.control).transform(obj->builder);
		}
		else {
			root = BuildBinaryVBVH<OVBVHNode>(arena1, &obj->rayobj.control).transform(obj->builder);
		}
		t_build = RE_prof_tick() - t0;                      /* ★ PROF */

		if (RE_rayobjectcontrol_test_break(&obj->rayobj.control)) {
			for (int i = 0; i < n_par_arenas; i++)         /* ★ C1c */
				BLI_memarena_free(par_arenas[i]);
			if (par_arenas)
				MEM_freeN(par_arenas);
			BLI_memarena_free(arena1);
			BLI_memarena_free(arena2);
			return;
		}

		if (root) {
			t0 = RE_prof_tick();                           /* ★ PROF */
			VBVH_optimalPackSIMD<OVBVHNode, PackCost>(PackCost()).transform(root);
			t_pack = RE_prof_tick() - t0;                   /* ★ PROF */

			t0 = RE_prof_tick();                           /* ★ PROF */
			obj->root = Reorganize_SVBVH<OVBVHNode>(arena2).transform(root);
			t_reorg = RE_prof_tick() - t0;                  /* ★ PROF */
		}
		else
			obj->root = NULL;

		/* ★ C1c: узлы кусков уже скопированы в arena2 — арены кусков не нужны. */
		for (int i = 0; i < n_par_arenas; i++)
			BLI_memarena_free(par_arenas[i]);
		if (par_arenas)
			MEM_freeN(par_arenas);
	}

	if (RE_prof_active()) {                                /* ★ PROF */
		printf("[PROF] ИТОГ BVH: ветка=%s rtbuild=%.4f s построение=%.4f s "
		       "упаковка=%.4f s проходы=%.4f s реорганизация=%.4f s\n",
		       plain_env ? "простая" : "оптимальная",
		       t_rt, t_build, t_pack, t_push, t_reorg);
	}

	//Free data
	BLI_memarena_free(arena1);

	obj->node_arena = arena2;
	obj->cost = 1.0;

	rtbuild_free(obj->builder);
	obj->builder = NULL;
}

template<int StackSize>
static int intersect(SVBVHTree *obj, Isect *isec)
{
	//TODO renable hint support
	if (RE_rayobject_isAligned(obj->root)) {
		if (isec->mode == RE_RAY_SHADOW)
			return svbvh_node_stack_raycast<StackSize, true>(obj->root, isec);
		else
			return svbvh_node_stack_raycast<StackSize, false>(obj->root, isec);
	}
	else
		return RE_rayobject_intersect( (RayObject *) obj->root, isec);
}

template<class Tree>
static void bvh_hint_bb(Tree *tree, LCTSHint *hint, float *UNUSED(min), float *UNUSED(max))
{
	//TODO renable hint support
	{
		hint->size = 0;
		hint->stack[hint->size++] = (RayObject *)tree->root;
	}
}
/* the cast to pointer function is needed to workarround gcc bug: http://gcc.gnu.org/bugzilla/show_bug.cgi?id=11407 */
template<class Tree, int STACK_SIZE>
static RayObjectAPI make_api()
{
	static RayObjectAPI api =
	{
		(RE_rayobject_raycast_callback) ((int   (*)(Tree *, Isect *)) & intersect<STACK_SIZE>),
		(RE_rayobject_add_callback)     ((void  (*)(Tree *, RayObject *)) & bvh_add<Tree>),
		(RE_rayobject_done_callback)    ((void  (*)(Tree *))       & bvh_done<Tree>),
		(RE_rayobject_free_callback)    ((void  (*)(Tree *))       & bvh_free<Tree>),
		(RE_rayobject_merge_bb_callback)((void  (*)(Tree *, float *, float *)) & bvh_bb<Tree>),
		(RE_rayobject_cost_callback)    ((float (*)(Tree *))      & bvh_cost<Tree>),
		(RE_rayobject_hint_bb_callback) ((void  (*)(Tree *, LCTSHint *, float *, float *)) & bvh_hint_bb<Tree>)
	};

	return api;
}

template<class Tree>
static RayObjectAPI *bvh_get_api(int maxstacksize)
{
	static RayObjectAPI bvh_api256 = make_api<Tree, 1024>();

	if (maxstacksize <= 1024) return &bvh_api256;
	assert(maxstacksize <= 256);
	return NULL;
}

RayObject *RE_rayobject_svbvh_create(int size)
{
	return bvh_create_tree<SVBVHTree, DFS_STACK_SIZE>(size);
}

#else

RayObject *RE_rayobject_svbvh_create(int UNUSED(size))
{
	puts("WARNING: SSE disabled at compile time\n");
	return NULL;
}

#endif
