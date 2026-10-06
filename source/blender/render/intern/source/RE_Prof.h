/*
 * RE_Prof.h — ВРЕМЕННАЯ измерительная обвязка (этап 2, подэтап «diffuse»).
 *
 * ЗАЧЕМ: прежде чем что-то оптимизировать, надо знать, куда уходит время.
 * Штатных таймеров внутри растеризатора нет: прошлые замеры были внешними,
 * по wall-clock (fb_time*.py), а они не разделяют проекцию, клиппинг,
 * заполнение спанов и интерполяцию атрибутов.
 *
 * ПОЧЕМУ ОТДЕЛЬНЫЙ ФАЙЛ: обвязка обязана удаляться одним движением.
 * Удаление = снести RE_Prof.c + RE_Prof.h, строку в render/CMakeLists.txt и
 * все вызовы RE_prof_* (помечены ★ PROF).
 *
 * СТОИМОСТЬ ПРИ ВЫКЛЮЧЕННОМ РЕЖИМЕ: ноль. Проверка окружения кэшируется в
 * статике, при UBF_PROF в окружении нет — ни один таймер не вызывается.
 *
 * ПОТОКИ: измерения копятся в блоке на поток (ProfBlock), отчёт суммирует блоки,
 * поэтому мерить можно и в несколько потоков. Раньше аккумуляторы были общими:
 * числа рвались, а горячие счётчики били по одной кэш-линии из всех ядер — на
 * aero4 (16 потоков) это стоило +49 s из 209 s кадра против 160.7 s без профиля.
 *
 * ВЛОЖЕННОСТЬ (критично для чтения отчёта):
 *     PART ⊃ { SPAN, SCENE_LOOP }
 *     SCENE_LOOP ⊃ PROJECT
 *     PROJECT ⊃ CLIP
 *     CLIP ⊃ FILL              (клиппер сам зовёт филлер)
 *     FILL ⊃ ATTR
 * RE_prof_part_end печатает ИСКЛЮЧАЮЩИЕ времена, то есть вклад уровня без
 * вложенных в него, — сумма строк даёт PART.
 */

#ifndef __RE_PROF_H__
#define __RE_PROF_H__

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Времена (вложенность см. в шапке файла). */
typedef enum RE_ProfSlot {
	RE_PROF_SPAN = 0,      /* выделение/заливка span-буферов части */
	RE_PROF_SCENE_LOOP,    /* обход buckets -> slots -> display arrays */
	RE_PROF_PROJECT,       /* цикл по примитивам: projectvert + culling */
	RE_PROF_CLIP,          /* RE_clip_triangle */
	RE_PROF_FILL,          /* RE_fill_triangle */
	RE_PROF_ATTR,          /* RE_raster_attr_pixel (co/vn пикселя) */

	/* вне частей: считаются на весь рендер-слой, печатаются отдельной строкой.
	 * В PART они не попадают: часть закрывается раньше, чем zbuffer_solid
	 * позовёт fillfunc (то есть шейдинг), а build/visibility вообще идут до
	 * первой части. */
	RE_PROF_BUILD,         /* RE_raster_scene_build */
	RE_PROF_VISIBILITY,    /* RE_raster_scene_update_visibility */
	RE_PROF_SHADE,         /* блок шейдинга solid в zbufshade_tile */
	RE_PROF_SHADE_OURS,    /* из него: RE_shader_shade_samples (наш шейдер) */
	RE_PROF_SHADE_BI,      /* из него: shade_samples (BI) */

	/* --- профиль ЧУЖОГО (BI) пути: когда scene.render.rasterizer_mode=OFF,
	 * наш растеризатор не вызывается вообще, и все слоты выше пусты.
	 * Эти шесть заполняются только на пути BI. --- */
	RE_PROF_BI_RASTER,     /* zbuffer_solid(), ветка BI: растеризация части */
	RE_PROF_BI_SHADE,      /* шейдинг OSA-пути: zbufshadeDA_tile */
	RE_PROF_RAYTRACE,      /* ray_trace() — зеркало и преломление */
	RE_PROF_REFLECT,       /* из него: trace_reflect() */
	RE_PROF_REFRACT,       /* из него: trace_refract() */
	RE_PROF_RAY_SHADOW,    /* ray_shadow() — теневые лучи ламп (этап B1-замер) */
	RE_PROF_RAYTREE,       /* build_raytree() — октодерево */
	RE_PROF_SHADBUF,       /* создание shadow-буферов ламп */
	RE_PROF_SCENE_PREP,    /* convertblender: сцена -> база рендера */
	RE_PROF_SCENE_DEPS,    /* из него: BKE_scene_update_for_newframe (депграф) */
	RE_PROF_SCENE_DB,      /* из него: database_init_objects (сцена -> VlakRen) */
	RE_PROF_MESH_VERT,     /* из базы: копирование вершин (init_render_dm) */
	RE_PROF_MESH_FACE,     /* из базы: цикл грани/материалы (init_render_dm) */
	RE_PROF_MESH_DM,       /* из базы: mesh_create_derived_render (по объекту) */

	RE_PROF_NUM
} RE_ProfSlot;

#define RE_PROF_IS_GLOBAL(slot) \
	((slot) == RE_PROF_BUILD || (slot) == RE_PROF_VISIBILITY || \
	 (slot) == RE_PROF_SHADE || (slot) == RE_PROF_SHADE_OURS || \
	 (slot) == RE_PROF_SHADE_BI || (slot) == RE_PROF_BI_RASTER || \
	 (slot) == RE_PROF_BI_SHADE || (slot) == RE_PROF_RAYTRACE || \
	 (slot) == RE_PROF_REFLECT || (slot) == RE_PROF_REFRACT || \
	 (slot) == RE_PROF_RAY_SHADOW || \
	 (slot) == RE_PROF_RAYTREE || (slot) == RE_PROF_SHADBUF || \
	 (slot) == RE_PROF_SCENE_PREP || (slot) == RE_PROF_SCENE_DEPS || \
	 (slot) == RE_PROF_SCENE_DB || (slot) == RE_PROF_MESH_VERT || \
	 (slot) == RE_PROF_MESH_FACE || (slot) == RE_PROF_MESH_DM)

/* Счётчики — чтобы отличать «дорого потому что много работы» от
 * «дорого потому что работа плохая». */
typedef enum RE_ProfCounter {
	RE_PROF_C_PARTS = 0,
	RE_PROF_C_BUCKETS,         /* суммарно по частям */
	RE_PROF_C_SLOTS_TESTED,    /* слотов просмотрено в цикле части */
	RE_PROF_C_SLOTS_CULLED,    /* из них отсечено clip_render_object */
	RE_PROF_C_SLOTS_DRAWN,     /* из них дошло до растеризации */
	RE_PROF_C_DARRAYS,         /* вызовов storage->rasterize */
	RE_PROF_C_PRIMS,           /* примитивов просмотрено */
	RE_PROF_C_PRIMS_PARTCLIP,  /* отсечено тестом части */
	RE_PROF_C_TRIS_FAST,       /* треугольников по быстрому пути клиппера */
	RE_PROF_C_TRIS_CLIPPED,    /* треугольников через ветку клиппинга */
	RE_PROF_C_TRIS_OUT,        /* из них целиком снаружи (не рисуются) */
	RE_PROF_C_FILLS,           /* треугольников дошло до заполнения спанов */
	RE_PROF_C_FILLS_EMPTY,     /* из них вышло пустым (нет второго спана) */
	RE_PROF_C_PIXELS,          /* пикселей выиграло тест глубины */
	RE_PROF_C_ATTRPIX,         /* пикселей с посчитанными co/vn */

	/* --- путь BI --- */
	RE_PROF_C_RAYTRACE,        /* вызовов ray_trace() */
	RE_PROF_C_REFLECT,         /* вызовов trace_reflect() */
	RE_PROF_C_REFRACT,         /* вызовов trace_refract() */
	RE_PROF_C_RAYCAST,         /* RE_rayobject_raycast в trace_reflect */
	RE_PROF_C_RAYCAST_MISS,    /* из них мимо (луч ушёл в пустоту) */
	RE_PROF_C_SHADBUF,         /* построено теневых буферов */
	RE_PROF_C_SHADE_SMP,       /* затенённых сэмплов (OSA) */
	RE_PROF_C_SHADOW_LAMPS,    /* вызовов ray_shadow() = лампа x сэмпл */
	RE_PROF_C_SHADOW_RAYS,     /* теневых лучей (raycast в теневых функциях) */
	RE_PROF_C_SHADOW_HITS,     /* из них попали (луч в тени, обход короткий) */

	RE_PROF_C_NUM
} RE_ProfCounter;

bool RE_prof_active(void);

/* ★ PROF — метка времени. При выключенном профиле возвращает 0.0 и НЕ зовёт
 * таймер, поэтому вызывающему не нужно ветвиться: пара tick/span при
 * выключенном профиле стоит ровно один вызов RE_prof_active().
 * PIL_time.h наружу не тянется — весь таймер живёт в RE_Prof.c. */
double RE_prof_tick(void);
void   RE_prof_span(int slot, double t0);
void   RE_prof_glob_span(int slot, double t0);

/* ★ PROF — начало/конец части. end печатает строку [PROF] part ... */
void RE_prof_part_begin(void);
void RE_prof_part_end(int xmin, int ymin, int rectx, int recty);

/* ★ PROF — накопление. RE_prof_add только для слотов части. */
void RE_prof_add(int slot, double dt);
void RE_prof_add_global(int slot, double dt);
void RE_prof_count(int counter, long n);

/* ★ Счётчики кэша геометрии держим ОТДЕЛЬНО от prof_cacc: они считаются на
 * подготовке сцены, то есть ДО частей, а RE_prof_part_begin() обнуляет cacc на
 * старте каждой части — в общей куче эти числа просто терялись. */
#define RE_PROF_DM_BUILD  0   /* построено DerivedMesh заново */
#define RE_PROF_DM_REUSE  1   /* взято из кэша объекта */
#define RE_PROF_DM_ALIVE  2   /* ob->derivedFinal был жив на входе */
void RE_prof_count_dm(int what, long n);

/* ★ PROF — предупреждение о многопоточности (зовётся из render_part). */
void RE_prof_note_threads(int threads);

/* ★ PROF — итоговая строка; зовётся из RE_rasterizer_free(). */
void RE_prof_report(void);

#ifdef __cplusplus
}
#endif

#endif /* __RE_PROF_H__ */
