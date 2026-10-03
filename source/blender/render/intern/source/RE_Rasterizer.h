/*
 * RE_Rasterizer.h — Software rasterizer frontend for Blender Internal.
 * Custom fork.
 */

#ifndef __RE_RASTERIZER_H__
#define __RE_RASTERIZER_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "DNA_listBase.h"

struct Render;
struct RenderPart;
struct RenderLayer;
struct ObjectInstanceRen;
struct ObjectRen;
struct VlakRen;
struct Material;
struct ZSpan;
struct ShadeInput;
struct ShadeSample;
struct PixStr;

/* ------------------------------------------------------------------------- */

typedef enum RE_RasterStorageType {
    RE_STORAGE_SCANLINE = 0,
    RE_STORAGE_EDGE,
    RE_STORAGE_TILED,
    RE_STORAGE_NUM
} RE_RasterStorageType;

/* ------------------------------------------------------------------------- */

typedef struct RE_RasterVertex {
    float co[3];
    float n[3];
    float uv[2];
    unsigned char rgba[4];
    int   orig_index;
    short flags;
    short pad;
} RE_RasterVertex;

typedef struct RE_RasterDisplayArray {
    struct RE_RasterDisplayArray *next, *prev;

    struct RE_RasterVertex *verts;
    unsigned short         *indices;
    int                    *vlr_indices;   /* ★ индекс VlakRen + 1 для каждого примитива */
    int                     num_verts;
    int                     num_indices;
    int                     num_prims;
    int                     prim_type;
    int                     cap_verts;
    int                     cap_indices;
    int                     cap_prims;
} RE_RasterDisplayArray;

typedef struct RE_RasterSlot {
    struct RE_RasterSlot *next, *prev;          /* ListBase links */

    struct ObjectInstanceRen *obi;
    int                       layer_id;
    bool                      visible;
    bool                      object_color;
    float                     rgba[4];
    ListBase                  display_arrays;
} RE_RasterSlot;

typedef struct RE_RasterBucket {
    struct Material *material;
    ListBase         slots;
} RE_RasterBucket;

typedef struct RE_RasterScene {
    struct Render           *re;
    struct RE_RasterBucket **buckets;
    int                       num_buckets;
    int                       cap_buckets;
} RE_RasterScene;

/* ------------------------------------------------------------------------- */

typedef struct RE_IStorage RE_IStorage;

/* ★ ШАГ B2-ii: контекст интерполяции атрибутов.
 *
 * Раньше co/vn считались отдельным проходом ПОСЛЕ растеризации
 * (RE_raster_gbuffer_fill читал rectp/recto/rectz и восстанавливал пиксели).
 * Теперь те же вычисления зовутся из филлера прямо в момент, когда пиксель
 * выиграл тест глубины, поэтому отдельный проход не нужен.
 *
 * Контекст живёт на стеке RE_rasterizer_render_part и передаётся вниз
 * ПАРАМЕТРОМ: RE_Rasterizer и storage общие для потоков, хранить в них
 * изменяемое состояние нельзя (это была бы гонка). */
typedef struct RE_RasterAttr {
    struct Render      *re;
    struct RenderLayer *rl;
    int                 xmin, ymin;   /* начало прямоугольника части в кадре */
    int                 rectx, recty;
    float              *co;           /* 3 float на пиксель; NULL — не заполнять */
    float              *vn;
} RE_RasterAttr;

struct RE_IStorage {
    bool (*init)(RE_IStorage *self, struct RE_Rasterizer *rasty);
    void (*exit)(RE_IStorage *self);
    void (*rasterize)(RE_IStorage *self, struct RE_Rasterizer *rasty,
                      RE_RasterSlot *slot, RE_RasterDisplayArray *da,
                      struct ZSpan *zspan, int obi_index, int vlr_index_base,
                      const float obwinmat[4][4], const float bounds[4],
                      const RE_RasterAttr *attr);
    void *user_data;
};

/* ------------------------------------------------------------------------- */

/* ★ ШАГ 3a: G-буфер части                                                    */
/*                                                                            */
/* Позиция (co) и нормаль (vn) в view space для каждого пикселя части — те же */
/* величины, что BI кладёт в ShadeInput::co / ShadeInput::vn на шейдинге.     */
/* Восстанавливаются из rectp/recto/rectz БЕЗ ShadeInput, standalone, той же  */
/* математикой, что в shadeinput.c — чтобы будущий свой шейдер не зависел от  */
/* легаси-шейдинга BI.                                                        */
/*                                                                            */
/* Буфер живёт в RenderPart (per-thread, как recto/rectp/rectz) — общей       */
/* памяти между потоками нет по построению.                                   */
/*                                                                            */
/* Координаты сэмпла берутся как «пиксель + 0.5» — соглашение не-OSA           */
/* (shadeinput.c:1434). OSA и motion blur требуют прохода на каждый сэмпл и    */
/* здесь пока не поддержаны. Заполнение вызывается только из не-OSA пути      */
/* (zbufshade_tile).                                                          */
/* ------------------------------------------------------------------------- */

typedef struct RE_RasterGBuffer {
    float *co;   /* 3 float на пиксель, part-local; NaN = пиксель не заполнен */
    float *vn;   /* 3 float на пиксель, part-local; NaN = пиксель не заполнен */
} RE_RasterGBuffer;

typedef struct RE_Rasterizer {
    struct Render *re;
    int width, height;

    float viewmat[4][4];
    float winmat[4][4];

    struct Material *last_material;

    RE_RasterStorageType  storage_type;
    RE_IStorage          *storage;

    struct RE_RasterScene *scene;
} RE_Rasterizer;

/* ------------------------------------------------------------------------- */
/* API                                                                       */
/* ------------------------------------------------------------------------- */

RE_Rasterizer *RE_rasterizer_create(struct Render *re, RE_RasterStorageType type);
void           RE_rasterizer_free(RE_Rasterizer *rasty);

RE_RasterScene *RE_raster_scene_build(RE_Rasterizer *rasty);
void            RE_raster_scene_free(RE_RasterScene *scene);
void            RE_raster_scene_update_visibility(RE_RasterScene *scene,
                                                  struct Render *re,
                                                  struct RenderLayer *rl);

void RE_rasterizer_render_part(RE_Rasterizer *rasty,
	struct RenderPart *pa,
	struct RenderLayer *rl,
	struct ZSpan *zspans_out,
	int *nsamples_out);

RE_IStorage   *RE_storage_create(RE_RasterStorageType type);
RE_IStorage   *RE_storage_create_scanline(void);

bool RE_rasterizer_enabled(struct Render *re);

/* ★ ШАГ 3a — G-буфер части. */
void RE_raster_gbuffer_fill(struct Render *re, struct RenderPart *pa,
                            struct RenderLayer *rl, struct RE_RasterGBuffer *gb);

/* ★ ШАГ B3 — свой шейдер. RE_shader_lambert() возвращает false и причину
 * в *why, если материал/сцена используют то, что шейдер пока не умеет:
 * тогда вызывающий обязан оставить результат BI.
 * RE_shader_shade_samples() делает setup и красит своим шейдером, а
 * неподдержанное отдаёт BI сам (семантика как у shade_samples()):
 * setup при этом выполняется ровно один раз. */
bool RE_shader_enabled(void);
bool RE_shader_lambert(struct Render *re, struct ShadeInput *shi,
                       const float co[3], const float vn[3],
                       float out[3], const char **why);
int RE_shader_shade_samples(struct Render *re, struct ShadeSample *ssamp,
                            struct PixStr *ps, int x, int y,
                            const float *co, const float *vn, const char **why);

/* ★ ШАГ B1 — свой клиппер.
 *
 * Замена zbufclip()/zbufclip4(). Принимает треугольник в HCS плюс флаги
 * клиппинга (из testclip) и отдаёт получившиеся треугольники в fill —
 * всегда по три вершины, в уже поделённых перспективно экранных
 * координатах. Фиксация семантики (порядок выдачи) важна: он влияет на
 * tie-break глубины, то есть на побитовое совпадение с путём BI.
 *
 * fill вызывается с v4 == NULL; квады раскалываются вызывающим кодом.
 * Возвращает 1, если сработала ветка клиппинга, и 0 для быстрого пути —
 * вызывающему это нужно для счётчика покрытия веток. */
typedef void (*RE_ClipFillFunc)(struct ZSpan *zspan, int obi, int zvlnr,
                                const float v1[4], const float v2[4],
                                const float v3[4], const float v4[4]);

int RE_clip_triangle(struct ZSpan *zspan, int obi, int zvlnr,
                     const float f1[4], const float f2[4], const float f3[4],
                     int c1, int c2, int c3, RE_ClipFillFunc fill);

/* Флаги клиппинга вершины (RE_CLIP_XMIN..RE_CLIP_ZMAX) — свой тест вместо
 * testclip() из zbuf.c. Вызывающий считает их сам и передаёт в
 * RE_clip_triangle(). */
int RE_clip_test(const float v[4]);

/* ★ ШАГ B2 — свой span-филлер.
 *
 * Замена zbuffillGL4(): свой обход рёбер, свой расчёт плоскости z и свой
 * строгий тест глубины в rectz/rectp/recto. Сигнатура совпадает с
 * RE_ClipFillFunc, поэтому подходит клипперу как колбэк напрямую — но
 * вызывающий обычно оборачивает его, чтобы подменить индекс VlakRen.
 * v4 != NULL только для квада; в нашем пути всегда NULL.
 * attr != NULL — филлер сам считает co/vn для выигравшего пиксель (шаг B2-ii),
 * заменяя отдельный проход RE_raster_gbuffer_fill(). */
void RE_fill_triangle(struct ZSpan *zspan, int obi, int zvlnr,
                      const float *v1, const float *v2, const float *v3,
                      const float *v4, const RE_RasterAttr *attr);

/* ★ Этап 2, «diffuse»: подготовленные атрибуты ОДНОГО треугольника.
 *
 * Раньше raster_gbuffer_pixel() считал на каждый пиксель то, что постоянно на
 * треугольник: facenor (mul_v3_m3v3 + normalize_v3), три мировые координаты
 * вершин (mul_m4_v3), три нормали вершин (mul_m3_v3 + normalize_v3) и dface.
 * На плотной сцене это 749 259 вычислений вместо 42 134 — ~18× лишней работы,
 * и по замерам это 44% времени растеризатора.
 *
 * Готовится один раз на треугольник: в филлере — на весь вызов (при клиппинге
 * вызовов больше, по одному на кусок, но obi/zvlnr внутри вызова постоянны,
 * поэтому кэш не нужен); в проходе-предшественнике — с кэшем на последний
 * треугольник.
 *
 * ВАЖНО: арифметика не менялась. Те же выражения в том же порядке, просто
 * считаются реже; приёмка сверяет rectp/recto/rectz и картинку с BI побитово,
 * поэтому «упрощать» формулы здесь нельзя. */
typedef struct RE_RasterAttrTri {
    struct Material *ma;
    bool  smooth;       /* vlr->flag & R_SMOOTH */
    bool  tangent;      /* vlr->flag & R_TANGENT */
    bool  transformed;  /* obi->flag & R_TRANSFORMED */
    bool  need_uv;      /* smooth || (ma->texco & NEED_UV) || (rl->passflag & SCE_PASS_UV) */
    bool  wire_edge;    /* vlr->v2 == vlr->v3 — у BI это «wire render of edge» */

    float facenor[3];   /* «сырой» facenor: пиксель копирует его себе и может перевернуть */
    float n1[3], n2[3], n3[3];
    float v1[3], v2[3], v3[3];
    float dface;        /* dot_v3v3(v1, facenor) — до возможного negate */
} RE_RasterAttrTri;

/* Подготовка атрибутов треугольника. false — считать нечего (нет G-буфера,
 * пиксель вне vlr, нет материала): тогда атрибуты не пишутся вообще, как и
 * раньше, когда raster_gbuffer_pixel() возвращал false. */
bool RE_raster_attr_prepare(const RE_RasterAttr *attr, int obi_index, int facenr,
                            RE_RasterAttrTri *tri);

/* Атрибуты одного выигравшего пикселя. col/row — координаты внутри
 * прямоугольника части, z — значение из rectz. */
void RE_raster_attr_pixel(const RE_RasterAttr *attr, const RE_RasterAttrTri *tri,
                          int col, int row, int z);

#ifdef __cplusplus
}
#endif

#endif /* __RE_RASTERIZER_H__ */
