/*
 * RE_GPURenderer.h — GPU deferred-растеризатор (mini-EEVEE) для Blender Internal.
 * Custom fork.
 *
 * Этап M1: геометрия BI-сцены (VlakRen) -> GPU-ready массивы, затем G-буфер.
 *
 * Двухфазная модель (VlakRen живёт в рендер-джобе, GL рисует главный поток):
 *   - RE_gpu_renderer_scene_build(): рендер-джоба, БЕЗ GL. Строит CPU-массивы
 *     вершин/индексов из re->instancetable (VlakRen).
 *   - RE_gpu_renderer_draw(): главный поток (вьюпорт), с GL-контекстом.
 *     Загружает массивы в VBO/IBO, рисует G-буфер, выводит результат.
 */

#ifndef __RE_GPURENDERER_H__
#define __RE_GPURENDERER_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "DNA_listBase.h"

struct Render;
struct Material;
struct ObjectInstanceRen;
struct ObjectRen;
struct VlakRen;

/* ------------------------------------------------------------------------- */
/* CPU-представление геометрии (строится без GL). */

typedef struct RE_GPUVertex {
	float co[3];      /* объектные координаты */
	float n[3];       /* объектная нормаль */
	float uv[2];
	float rgba[4];    /* 0..1 */
	float emit;       /* 0..1 — эмиссия материала (ma->emit), в alpha нормали G-буфера */
	float tangent[4]; /* M8: касательный базис для normal map. xyz — T (объектное
	                   * пространство), w — знак битангента (B = cross(N,T)*w). */
	float orco[3];    /* M11: ORCO (BI `VertRen->orco`) — «original coordinates»,
	                   * нормированные в [-1,1] по texspace объекта. Нужны картам с
	                   * mtex->texco==TEXCO_ORCO (jolly/something2/disk/cave): BI
	                   * сэмплит их по `do_2d_mapping` FLAT => uv=(orco.xy+1)/2,
	                   * а НЕ по UV-развёртке. */
} RE_GPUVertex;

/* M14: максимум слотов диффузных карт (GL 2.1 => >=16 текстурных юнитов;
 * G-буфер использует 4 (слот0), 5 (нормаль), 6 (альфа) и 7..9 (слоты 1..3)). */
#define GPU_MAX_TEX_SLOTS 4

/* Один непрерывный список треугольников одного материала одного инстанса. */
typedef struct RE_GPUMesh {
	struct RE_GPUMesh *next;

	struct Material *ma;
	float alpha;              /* прозрачность материала [0,1]; 1.0 если BI-прозрачность выключена */

	/* ★ M5: параметры блика материала (BI: ma->specr/g/b и ma->har). Живут на
	 * МЕШЕ, а не на вершине, потому что меш — это «один материал одного
	 * инстанса»: внутри него материал один и тот же, значит и блик один.
	 * Передаются в G-буфер униформами, 4-я цель MRT. */
	float spec_col[3];        /* цвет блика (ma->specr/g/b), по умолчанию белый */
	float hard;               /* hardness (ma->har, 1..511); на GPU едет как hard/511 */

	float model[4][4];        /* object->view (obi->mat) */
	float normal_mat[3][3];   /* inverse-transpose object->view (obi->nmat) */
	float obmat[4][4];        /* object->world (obi->obmat), для shadow map */

	float bb_min[3];          /* мировой AABB меша (для отсечения по фрустуму) */
	float bb_max[3];

	int num_verts;
	int num_indices;

	/* ★ Ёмкости буферов (отдельно от num_*): mesh_emit_face рос массивы
	 * MEM_reallocN на КАЖДУЮ грань — для меша со 100k+ граней это O(N²)
	 * копирований и рендер «висит» (mapFULL: грань 138k). Растём экспоненциально. */
	int verts_alloc;
	int indices_alloc;
	int line_indices_alloc;

	RE_GPUVertex *verts;
	unsigned int *indices;    /* 32-битные индексы (GL_UNSIGNED_INT) */

	/* GL-ресурсы (создаются на главном потоке) */
	unsigned int vbo;
	unsigned int ibo;
	bool uploaded;

	/* ★ M16 (раунд 67): материал типа WIRE. BI кодирует ребро ВЫРОЖДЕННОЙ
	 * VlakRen (v3 == v2, v4 == NULL) — см. convertblender.c:3555, поэтому рёбра
	 * уже приходят в этот меш. Но триангуляция делала из них треугольник
	 * НУЛЕВОЙ площади, и проволока не рисовалась вообще. Такие рёбра собираем в
	 * отдельный индексный буфер и рисуем GL_LINES (в проходе G-буфера, поэтому
	 * деферред-освещение достаётся даром). */
	unsigned int *line_indices;
	int num_line_indices;
	unsigned int lbo;

	/* M5: диффузная текстура материала.
	 * ima — Image* (взят из mtex слота с MAP_COL / TEX_IMAGE), только CPU-джоба.
	 * tex / tex_tried — GL-текстура, создаётся на главном потоке. */
	void *ima;
	unsigned int tex;
	int tex_tried;
	/* M9: как BI СМЕШИВАЕТ диффузную карту с базовым цветом материала
	 * (`texture_rgb_blend`, render_texture.c): blend_type (MTEX_*, 0=MIX) и
	 * colfac (mtex->colfac). При MIX/colfac=1 карта ЗАМЕНЯЕТ базовый цвет —
	 * поэтому простое умножение (base*=tex) затемняло текстуры и делало чёрные
	 * материалы (base=0) чёрными. */
	int tex_blend;
	float colfac;
	/* ★ M24 (раунд 81): MIP-карта диффузного слота 0.
	 * BI строит мип-уровни, когда у текстуры включён TEX_MIPMAP (ibuf->mipmap[],
	 * imagetexture.c:883) и сэмплит трилинейно; раньше GPU-растеризатор грузил
	 * только уровень 0 с GL_LINEAR, поэтому сильно минифицированные текстуры
	 * (глаза персонажей) выглядели резче/шумнее, чем в scanline. tex_mipmap=1 —
	 * грузить цепочку мипов BI и ставить GL_LINEAR_MIPMAP_LINEAR.
	 * tex_gauss — TEX_GAUSS_MIP (гауссов фильтр вместо box), как в BI. */
	int tex_mipmap;
	int tex_gauss;
	/* ★★ M25 (раунд 82): shi->refl = ma->ref (RNA `diffuse_intensity`).
	 * BI умножает ВКЛАД ЛАМП и ambient на shi->refl (shadeoutput.c:1455
	 * `i*= visifac*shi->refl`, :1099 `+= shi->ao*shi->r*shi->refl*f`), а GPU-
	 * растеризатор этот множитель вообще не применял. У 105 из 109 материалов
	 * mapFull он равен 0.8, то есть сцена была стабильно на 25% ярче scanline.
	 * Запекаем его в albedo G-буфера (там же, где BI применяет его к shi->rcol):
	 * albedo = rcol*refl, поэтому и диффуз, и ambient получают ровно тот же
	 * множитель. NB: эмиссия в BI считается от shi->r без refl — при emit>0
	 * G-буфер занизит её на этот множитель (в mapFull emit=0). */
	float refl;
	/* M11: откуда брать координаты для диффузной карты. BI: mtex->texco
	 * (TEXCO_UV => UV-развёртка, TEXCO_ORCO => ORCO + `do_2d_mapping` FLAT:
	 * uv=(orco.xy+1)/2, затем translate/scale `size*(t-0.5)+ofs+0.5`).
	 * tex_orco=1 => брать orco; иначе UV. Прочие источники (GLOB/OBJECT/NORM/
	 * REFL/TANGENT/STRESS/WINDOW) и mapping CUBE/TUBE/SPHERE пока не умеем —
	 * там остаётся UV (см. TODO). */
	int tex_orco;
	float tex_size[2];
	float tex_ofs[2];
	/* M27: texco==GLOB — использовать мировую позицию (v_wpos) вместо v_orco.
	 * Для GLOB tex_orco=1 (идём в ветку маппинга), tex_glob=1 (координата = v_wpos). */
	int tex_glob;
	/* M16: тип do_2d_mapping для ORCO-координат: 0=FLAT, 1=CUBE, 2=TUBE, 3=SPHERE.
	 * Для UV-texco не используется (там FLAT — тождество). */
	int tex_map;

	/* M14: многослотовая цепочка диффузных карт (до GPU_MAX_TEX_SLOTS). Слот 0 —
	 * основной (поля tex_* выше), слоты 1..3 — дополнительные. BI применяет их
	 * подряд: shi->r = texture_rgb_blend(shi->r, tcol_i, shi->r, tin_i, colfac_i, bt_i)
	 * (render_texture.c:2554). Нужно для mono/alone `Face` (10-11 COL-слотов, MIX). */
	int n_tex;                       /* 1..GPU_MAX_TEX_SLOTS */
	/* ★ раунд 83-fix3: MTEX_RGBTOINT для образных диффузных слотов. BI берёт в
	 * texture_rgb_blend фактор `tin` (render_texture.c:2440): для картинки это
	 * 1.0, а яркость — ТОЛЬКО при включённом `use_rgb_to_intensity`. Раньше
	 * шейдер подставлял на место tin альфу картинки, поэтому у карт с частичной
	 * альфой модуляция выходила в разы слабее scanline. */
	int tex_rgbtoint;
	void *tex_ima2[GPU_MAX_TEX_SLOTS - 1];
	int tex_blend2[GPU_MAX_TEX_SLOTS - 1];
	int tex_rgbtoint2[GPU_MAX_TEX_SLOTS - 1];
	float colfac2[GPU_MAX_TEX_SLOTS - 1];
	int tex_orco2[GPU_MAX_TEX_SLOTS - 1];
	int tex_map2[GPU_MAX_TEX_SLOTS - 1];
	int tex_glob2[GPU_MAX_TEX_SLOTS - 1];
	float tex_size2[GPU_MAX_TEX_SLOTS - 1][2];
	float tex_ofs2[GPU_MAX_TEX_SLOTS - 1][2];
	unsigned int tex2[GPU_MAX_TEX_SLOTS - 1];
	int tex2_tried;

	/* M8: нормальная карта (TANGENT-space, образная). nima — Image* (CPU-джоба),
	 * ntex/n_tex_tried — GL-текстура, n_has — есть ли образная карта нормалей
	 * (иначе 0 — шейдер не сэмплит, нормаль геометрическая). */
	void *nima;
	unsigned int ntex;
	int n_tex_tried;
	int n_has;
	/* M26: BI применяет карту нормалей ДВУМЯ путями (render_texture.c:1783):
	 *  - образ с флагом TEX_NORMALMAP -> ТАНГЕНЦИАЛЬНАЯ карта нормалей;
	 *  - иначе -> height-bump по яркости карты (compatible_bump_compute).
	 * n_is_height=1 — второй случай (карты mapFull *_nor_gl_*.exr идут БЕЗ
	 * TEX_NORMALMAP, т.е. BI трактует их как height, а не как нормаль).
	 * n_norfac — mtex->norfac этого слота (масштаб/знак bump). */
	int n_is_height;
	float n_norfac;
	/* M28: источник координат карты нормалей (как у диффузной): orco=1 (ORCO),
	 * glob=1 (GLOB — мировая позиция), map — do_2d_mapping, size/ofs. */
	int n_orco;
	int n_glob;
	int n_map;
	float n_size[2];
	float n_ofs[2];
	/* M10: альфа-карта (MAP_ALPHA). BI: `shi->alpha` (стартует из ma->alpha)
	 * модулируется картой через `texture_value_blend(def_var, shi->alpha, tin,
	 * alphafac, blendtype)`, где tin = ta (если Use Alpha) либо яркость (render_texture.c:2712).
	 * Во ВСЕХ сценах карта — та же картинка, что и цвет, а материал имеет
	 * ma->alpha=0 + Z_TRANSPARENCY, поэтому эффективная прозрачность = альфа-канал
	 * картинки. Без этого такие меши были ПОЛНОСТЬЮ невидимы (screen-door по 0). */
	void *aima;
	unsigned int atex;
	int a_tex_tried;
	int a_has;
	int a_blend;      /* mtex->blendtype */
	int a_use_a;      /* tin = альфа картинки (Use Alpha), иначе её яркость */
	float a_fac;      /* mtex->alphafac */
	float a_def;      /* mtex->def_var */
	int a_orco;       /* M11: координаты альфа-карты (как tex_orco) */
	int a_glob;       /* M27: alpha-карта по GLOB (мировая позиция) */
	int a_map;        /* M16: do_2d_mapping альфа-карты (0=FLAT,1=CUBE,2=TUBE,3=SPHERE) */
	float a_size[2];
	float a_ofs[2];

	/* M12: карты блика/жёсткости (MAP_SPEC / MAP_HAR). В сценах они лежат на ТОМ
	 * ЖЕ слоте, что и диффузная карта (jolly/something2: slot 0 = COL|SPEC|HAR|…),
	 * поэтому переиспользуем сэмпл диффузной текстуры и её tin. BI модулирует:
	 *   shi->spec = texture_value_blend(def_var, spec, tin, specfac, blendtype)
	 *   shi->har  = 128*texture_value_blend(def_var, har/128, tin, hardfac, bt)
	 * tin = альфа картинки (Use Alpha) либо её яркость (как MAP_ALPHA). */
	int spec_has;     /* MAP_SPEC mapto на диффузном слоте */
	int spec_blend;   /* mtex->blendtype */
	float spec_fac;   /* mtex->specfac */
	float spec_def;   /* mtex->def_var */
	int har_has;      /* MAP_HAR mapto */
	int har_blend;
	float har_fac;    /* mtex->hardfac */
	float har_def;    /* mtex->def_var */
	int sv_use_a;     /* tin = альфа (Use Alpha), иначе яркость — общий для spec/har */

	/* M13: карта цвета блика (MAP_COLSPEC). На том же слоте, что и диффузная карта
	 * (jolly Material/Material.001: slot 0 = COL|SPEC|HAR|COLSPEC|REFL). BI модулирует
	 *   shi->specr = texture_rgb_blend(specr, tcol, specr, tin, colspecfac, bt),
	 *   где tcol = цвет текстуры, tin = texres.ta (альфа картинки, render_texture.c:2532). */
	int colspec_has;
	int colspec_blend;
	float colspec_fac;   /* mtex->colspecfac */

	/* M17: MAP_REFL (отражение). BI: shi->refl стартует из ma->ref и модулируется
	 *   shi->refl = texture_value_blend(def_var, shi->refl, tin, difffac, blendtype)
	 *   (render_texture.c:2719). tin — как у spec/har (Use Alpha -> альфа, иначе яркость).
	 *   Пока 5-я цель MRT доступна, GB_FS пишет refl в gl_FragData[4], а LIGHT_FS
	 *   использует его как силу подмешивания SSR (вместо фиксированного 0.35). */
	int refl_has;
	int refl_blend;
	float refl_fac;      /* mtex->difffac */
	float refl_def;      /* mtex->def_var */
	float refl_base;     /* база силы отражения: ma->ray_mirror при включённой галке
	                        Mirror (ma->mode & MA_RAYMIRROR), иначе 0. Раньше здесь
	                        был ma->ref (= diffuse_intensity, по умолчанию 0.8),
	                        из-за чего зеркалили все материалы. */

	/* ★ M19: ПРОЦЕДУРНАЯ карта цвета (TEX_CLOUDS / TEX_BLEND) в слоте MAP_COL.
	 * BI считает её так (render_texture.c: texco_mapping -> texfunc -> BRICONT
	 * -> colorband):  texvec = size*(co + ofs);  tin = <texfunc>(texvec);
	 * tin = (tin-0.5)*contrast + bright - 0.5 (clamp);  tcol = colorband(tin)
	 * либо (tin,tin,tin) если колорбэнда нет. Дальше — обычный
	 * texture_rgb_blend(base, tcol, base, tin, colfac, blendtype), поэтому в
	 * шейдере достаточно подменить vec4 tc (rgb=tcol, a=tin) для слота 0.
	 * Диагностика в сценах: metro Material (BLEND, MAP_COL) и disk Material.002
	 * (CLOUDS, MAP_COL). */
	int proc_type;        /* 0 = нет, 1 = TEX_CLOUDS, 5 = TEX_BLEND */
	int proc_stype;       /* tex->stype: TEX_LIN/QUAD/EASE/DIAG/SPHERE/HALO/RAD */
	int proc_flip;        /* tex->flag & TEX_FLIPBLEND */
	float proc_noisesize; /* tex->noisesize (CLOUDS) */
	int proc_noisedepth;  /* tex->noisedepth (CLOUDS), обрезан до 0..8 */
	int proc_basis;       /* tex->noisebasis */
	int proc_hard;        /* tex->noisetype != TEX_NOISESOFT */
	float proc_bright;    /* tex->bright (BRICONT) */
	float proc_contrast;  /* tex->contrast (BRICONT) */
	int proc_noclamp;     /* tex->flag & TEX_NO_CLAMP */
	int proc_has_ramp;    /* BKE_colorband_evaluate вернула цвет */
	unsigned char proc_ramp[256 * 4];  /* запечённый колорбэнд, RGBA8 */
	int proc_orco;        /* 1 = texco ORCO, 0 = UV (процедурные карты маппинг
	                        * FLAT/CUBE/TUBE/SPHERE НЕ применяют — texco_mapping
	                        * зовёт do_2d_mapping только для TEX_IMAGE) */
	float proc_size[2];   /* mtex->size[0..1] */
	float proc_ofs[2];    /* mtex->ofs[0..1] */
	int proc_col;         /* M20: 1 = процедурная текстура даёт MAP_COL (модулирует
	                       * base). Если MAP_COL нет (напр. только MAP_SPEC), текстура
	                       * считается ради побочных каналов, но базовый цвет не трогает. */

	/* ★ M22: ВТОРАЯ процедурная текстура материала, лежащая НЕ в слоте 0
	 * (в сценах — jolly: слот 0 образная MAP_COL, слот 1 CLOUDS с
	 * MAP_NORM|MAP_SPEC|MAP_HAR, basis=VORONOI_CRACKLE). Даёт значение `tin`
	 * для MAP_SPEC/MAP_HAR. Таблицы шума (hash/hashv/hashp) общие с первой;
	 * свой только колорбэнд. */
	int has_proc2;
	int proc2_type, proc2_stype, proc2_flip, proc2_noisedepth, proc2_basis, proc2_hard;
	int proc2_noclamp, proc2_has_ramp, proc2_orco;
	float proc2_noisesize, proc2_bright, proc2_contrast, proc2_size[2], proc2_ofs[2];
	unsigned char proc2_ramp[256 * 4];
	int proc2_spec, proc2_har;                 /* каким mapto соответствует */
	int proc2_spec_blend, proc2_har_blend;
	float proc2_spec_fac, proc2_spec_def, proc2_har_fac, proc2_har_def;

	/* ★ M23: MAP_NORM от процедурной карты (bump). Источник выбирается после
	 * разбора всех слотов: вторая карта приоритетнее, т.к. в сценах именно она
	 * несёт вортоновский рельеф (jolly: слот 1, mapto=0x122). */
	int proc_norm, proc2_norm;                 /* слот 0 / вторая дают MAP_NORM */
	float proc_norfac, proc2_norfac;           /* mtex->norfac */
	int bump_src;                              /* 0 = нет, 1 = слот 0, 2 = вторая */
	float bump_norfac;

	/* M15: полупрозрачный меш (эффективная alpha < 1). Такие меши рисуются
	 * НЕ в G-буфер, а forward-проходом (FORWARD_FS) с alpha-блендингом,
	 * чтобы дать честную прозрачность вместо screen-door. Критерий:
	 * есть альфа-карта (a_has) или ma->alpha < 1. */
	int transparent;
} RE_GPUMesh;

/* ★ M16 (раунд 67): один спрайт-хало (BI HaloRen, renderdatabase.c:RE_inithalo).
 * Позиция переведена в МИР (как и вершины мешей), чтобы биллборд строился теми
 * же матрицами, что и всё остальное. */
typedef struct RE_GPUHalo {
	float pos[3];       /* мир (из har->co, которая у BI в пространстве камеры) */
	float rad;          /* hasize — радиус пятна в мировых единицах */
	float col[3];       /* har->r/g/b */
	float alfa;         /* har->alfa */
	float hard;         /* har->hard (1..511, как у блика) */
	float add;          /* har->add (0..255): аддитивная составляющая */
	int type;           /* har->type: HA_XALPHA, HA_VECT, HA_FLARE... */
	int starpoints, linec, ringc, flarec;
	int seed;           /* har->seed: смещение в hashvectf для колец/лучей */
	int xalpha;         /* har->type & HA_XALPHA: alpha = falloff^2 */
	float spec_col[3];  /* ma->specr/g/b — цвет лучей (linef) */
	float mir_col[3];   /* ma->mirr/g/b — цвет колец (ringf) */
	struct Material *ma;
} RE_GPUHalo;

typedef struct RE_GPUScene {
	RE_GPUMesh *meshes;
	int num_meshes;

	/* ★ M16 (раунд 67): материалы типа HALO. BI рисует их не треугольниками, а
	 * спрайтами-биллбордами (HaloRen, rendercore.c:renderhalo_*): каждая вершина
	 * с таким материалом даёт светящееся пятно радиуса hasize с радиальным
	 * затуханием (1-r)^hard, цветом материала и alpha. Здесь — то, что нужно
	 * биллборду; собирается в CPU-джобе (re->sortedhalos), GL-ресурсов не
	 * требует. */
	RE_GPUHalo *halos;
	int num_halos;
} RE_GPUScene;

/* ------------------------------------------------------------------------- */

/* Свет для деферред-шейдинга. Поля — ровно то, из чего BI считает вклад лампы
 * в lamp_get_visibility() (shadeoutput.c), чтобы затухание совпадало с BI.
 * ★ Потолок задан не производительностью, а текстурными юнитами: на каждую лампу
 *   в шейдере висит ПАРА сэмплеров — 2D-тень (юнит 6+i) и кубическая
 *   (6+GPU_MAX_LIGHTS+i) в проходе света, 10+i и 10+GPU_MAX_LIGHTS+i в forward.
 *   Итого нужно 10 + 2*GPU_MAX_LIGHTS юнитов (на NVIDIA обычно 32 → до 11 ламп).
 *   При нехватке линковка шейдера падает, и проход молча отменяется (кадр чёрный),
 *   поэтому лимит виден в диагностике [GPU-CAPS] max_texture_image_units. */
#define GPU_MAX_LIGHTS 10

enum {
	GPU_LIGHT_SUN = 0,    /* направленный (и LA_HEMI) */
	GPU_LIGHT_POINT = 1,
	GPU_LIGHT_SPOT = 2,
	GPU_LIGHT_AREA = 3
};

enum {
	GPU_FALLOFF_CONSTANT = 0,
	GPU_FALLOFF_INVLINEAR = 1,
	GPU_FALLOFF_INVSQUARE = 2
};

typedef struct RE_GPULight {
	int type;         /* GPU_LIGHT_* */
	int falloff;      /* GPU_FALLOFF_* (SLIDERS/INVCOEFFICIENTS/CURVE сведены к INVSQUARE) */
	int sphere;       /* lar->mode & LA_SPHERE: обнулять свет за пределами dist */
	float pos[3];     /* view-space позиция (point/spot/area) */
	float dir[3];     /* view-space направление, куда светит (sun/spot) */
	float color[3];   /* цвет лампы ВМЕСТЕ с energy: BI кладёт в LampRen lar->r = energy*la->r
	                   * (convertblender.c:3837), и освещение берёт именно lar->r */
	float dist;       /* lar->dist */
	float spotsi;     /* cos(spotsize/2) */
	float spotbl;     /* (1-spotsi)*spotblend */

	/* ★ Теневая карта этой лампы. Матрицы считаются на CPU (gpu_compute_shadow_mats);
	 * карту рисует отдельный проход, а проход освещения сэмплит её по u_lvp[i].
	 * Тень умеют SUN (ортопроекция по AABB сцены), SPOT (перспектива по углу конуса),
	 * POINT и AREA (кубическая карта, 6 граней в face_view[]). SUN/SPOT — те типы, для
	 * которых теневой буфер строит и сам BI (convertblender.c:3946); солнце BI
	 * затеняет трассировкой лучей, которой у деферред-растеризатора нет — поэтому
	 * мы делаем ему теневую карту. Точечная и AREA в BI тоже затеняются только
	 * трассировкой (буфера нет), поэтому у нас кубическая карта. Карта
	 * индексируется НОМЕРОМ лампы (текстура создаётся на все GPU_MAX_LIGHTS),
	 * а не отдельным слотом. */
	int shadows;           /* 1 — лампа даёт тень (и матрицы ниже заполнены) */
	float soft;            /* lar->soft: мягкость тени (0..100, по умолч. 3). В BI это
	                        * fac в shadbuf.c — масштаб области фильтра PCF. Мы шлём его
	                        * в шейдер и расширяем радиус выборок (см. u_lsoft). */
	float clipsta, clipend;/* lar->clipsta/clipend: ближняя/дальняя плоскость (spot) */
	float view[4][4];      /* world -> light (инверсия lar->lampmat, как в BI) */
	float proj[4][4];      /* ортопроекция (sun) или перспектива по конусу (spot) */
	float vp[4][4];        /* proj * view: world -> light-clip */
	float face_view[6][4][4]; /* point: 6 видов куба (world -> face-view). Рендер
	                           * граней + textureCube(dir) — согласованы
	                           * (verify_cubemap.py). У sun/spot не используется. */
	float wpos[3];         /* point: мировая позиция света (центр куба) */
	float wdir[3];         /* ★ МИРОВОЕ направление лампы (локальная -Z в мире).
	                        * pos/dir выше — КАМЕРНЫЕ и сняты в момент сборки базы,
	                        * поэтому при движении камеры они устаревают и свет
	                        * «прилипает» к экрану. На отрисовке считаем камерные
	                        * величины из wpos/wdir текущим re->viewmat. */
	float pz[2];           /* point: (A,B) пересчёта window-глубины по расстоянию d:
	                        * sc.z=(A + B/d)*0.5+0.5 (та же проекция, что в проходе) */
} RE_GPULight;

/* ------------------------------------------------------------------------- */

typedef struct RE_GPURenderer {
	struct Render *re;

	/* CPU-сцена (строится джобой, читается главным потоком). */
	RE_GPUScene *scene;

	/* ★ Двойной буфер сцены. Рендер-джоба собирает НОВУЮ сцену и публикует её,
	 * а предыдущую кладёт сюда — НЕ освобождая: её ещё может читать главный поток.
	 * Главный поток освобождает scene_old (GL-ресурсы + память) в начале кадра. */
	RE_GPUScene *scene_old;

	/* 1 — база рендера пересобралась (правки геометрии, перемещение объектов):
	 * сцену нужно собрать заново. Ставит главный поток, гасит джоба. */
	int scene_dirty;

	/* Свет (собирается джобой из сцены, используется в шейдинге).
	 * Каждая лампа несёт свои теневые матрицы (см. RE_GPULight.shadows). */
	RE_GPULight lights[GPU_MAX_LIGHTS];
	int num_lights;
	int num_shadow_lights;  /* сколько ламп реально дают тень (0..GPU_MAX_LIGHTS) */

	float light_dir[3];    /* направление на свет (view-space) */
	float light_color[3];
	float light_view[4][4]; /* world->light-view первой солнцевой лампы (диагностика) */
	float light_proj[4][4]; /* её проекция (диагностика) */
	int has_sun;            /* 1 — нашли sun-лампу; 0 — свет по умолчанию */

	/* ★ Мир сцены (re->wrld): фон и ambient. Раньше фон был плоским серым 0.19,
	 * а ambient — захардкоженной полусферой; настройки World сцены не читались. */
	float world_hor[3];     /* horizon_color */
	float world_zen[3];     /* zenith_color */
	float world_amb[3];     /* ambient_color */
	int world_blend;        /* WO_SKYBLEND: фон = mix(hor, zen, |0.5+view.y|) */
	/* ★★ M26 (раунд 82): "Environment Lighting" мира (wrld->mode & WO_ENV_LIGHT,
	 * energy = wrld->ao_env_energy). ВАЖНО: horizon/zenith — это ЦВЕТ ФОНА, а не
	 * свет. BI делает их ambient-светом ТОЛЬКО при включённом Environment Lighting
	 * (shadeoutput.c:1127 `shr->combined += shi->env * shi->r * shi->refl * f`);
	 * без галки ambient материала считается исключительно из ambient_color
	 * (`shi->ambr = ma->amb * wrld.ambr`, shadeoutput.c:2000). GPU-растеризатор
	 * подмешивал mix(hor, zen) как ambient ВСЕГДА — на mapFull (zen=0.01, галка
	 * выключена) это пересвечивало все неосвещённые поверхности: в самых тёмных
	 * 48% пикселей кадр был в 87 раз ярче scanline, тогда как освещённые 2.3%
	 * сходились с точностью 1.12x. */
	int world_env;          /* 1 — Environment Lighting включён */
	float world_env_energy; /* wrld->ao_env_energy */
	/* Экспозиция/диапазон мира — BI wrld_exposure_correct (shadeoutput.c:1745).
	 * linfac/logfac уже посчитаны в convertblender.c:4198. */
	float linfac, logfac;
	int expo_on;            /* 1, если exp!=0 или range!=1 (иначе BI не применяет) */

	/* GL-ресурсы (создаются на главном потоке). */
	void *gpu_res;   /* RE_GPUGPURes; скрыт в .c */
} RE_GPURenderer;

/* --- создание/освобождение --- */
RE_GPURenderer *RE_gpu_renderer_create(struct Render *re);
void RE_gpu_renderer_free(RE_GPURenderer *gr);

/* ★ Сборка CPU-сцены из VlakRen в рендер-джобе (без GL). Пересоздаёт сцену;
 * GL-ресурсы не трогает (они на главном потоке). */
void RE_gpu_renderer_build_render(struct Render *re);

/* ★ Пометить GPU-сцену устаревшей: база рендера пересобралась (правки геометрии,
 * перемещение/удаление объектов, смена материалов). Вызывается ГЛАВНЫМ потоком
 * сразу после RE_Database_FromScene. Пересборку сделает рендер-джоба, а старую
 * сцену освободит главный поток в начале следующего кадра. */
void RE_gpu_renderer_invalidate(struct Render *re);

/* --- фаза 1: сборка CPU-массивов из VlakRen (БЕЗ GL, из рендер-джобы) --- */
void RE_gpu_renderer_scene_build(RE_GPURenderer *gr);
void RE_gpu_renderer_scene_free(RE_GPUScene *scene);

/* --- фаза 2: GL-отрисовка (главный поток, вьюпорт) --- */
/* Инициализация GL-ресурсов (VBO/IBO, G-буфер FBO, шейдеры). width/height —
 * размер кадра в пикселях. */
int RE_gpu_renderer_gl_init(RE_GPURenderer *gr, int width, int height);
/* Отрисовка кадра: геометрия -> G-буфер, затем вывод. */
void RE_gpu_renderer_draw(RE_GPURenderer *gr);
void RE_gpu_renderer_gl_free(RE_GPURenderer *gr);
/* ★ Удалить GL-ресурсы (VBO/IBO) мешей одной сцены. Вызывается главным потоком
 * при свопе сцены (см. RE_gpu_renderer_draw_render) и при teardown. */
void RE_gpu_renderer_gl_free_scene(RE_GPUScene *scene);

/* ★ F12 (полный рендер): отрисовать кадр в offscreen-буфер и заполнить им
 * re->result. Только с ГЛАВНОГО потока (нужен GL-контекст). */
int RE_gpu_renderer_render_to_result(struct Render *re, int width, int height);

/* --- диагностика (без GL, для headless-проверки сборки) --- */
/* Печатает число мешей, вершин, индексов и хеш геометрии. */
void RE_gpu_renderer_dump(const RE_GPURenderer *gr);

/* Отладочный хук (UBF_GPU_DEBUG): собрать GPU-сцену и вывести статистику.
 * Без GL; вызывается из RE_TileProcessor. */
void RE_gpu_renderer_debug_build(struct Render *re);

#ifdef __cplusplus
}
#endif

#endif /* __RE_GPURENDERER_H__ */
