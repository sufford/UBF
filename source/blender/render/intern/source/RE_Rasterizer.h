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

struct RE_IStorage {
    bool (*init)(RE_IStorage *self, struct RE_Rasterizer *rasty);
    void (*exit)(RE_IStorage *self);
    void (*rasterize)(RE_IStorage *self, struct RE_Rasterizer *rasty,
                      RE_RasterSlot *slot, RE_RasterDisplayArray *da,
                      struct ZSpan *zspan, int obi_index, int vlr_index_base,
                      const float obwinmat[4][4], const float bounds[4]);
    void *user_data;
};

/* ------------------------------------------------------------------------- */

typedef struct RE_Rasterizer {
    struct Render *re;
    int width, height;

    float viewmat[4][4];
    float winmat[4][4];

    struct Material *last_material;

    RE_RasterStorageType  storage_type;
    RE_IStorage          *storage;

    struct RE_RasterScene *scene;

    /* ★ ZSpan'ы последнего RE_rasterizer_render_part(). Нужны, чтобы
     * zbuffer_solid() вызвал fillfunc() ПО ОДНОМУ РАЗУ НА СЭМПЛ с настоящим
     * ZSpan — как это делает встроенный код. Передавать NULL нельзя:
     * fillfunc-и (например make_pixelstructs в OSA-пути) его разыменовывают.
     * Гарантированно валидны сразу после RE_rasterizer_render_part(). */
    struct ZSpan *zspans;
    int           num_zspans;
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

void            RE_rasterizer_render_part(RE_Rasterizer *rasty,
                                          struct RenderPart *pa,
                                          struct RenderLayer *rl);

RE_IStorage   *RE_storage_create(RE_RasterStorageType type);
RE_IStorage   *RE_storage_create_scanline(void);

bool RE_rasterizer_enabled(struct Render *re);

#ifdef __cplusplus
}
#endif

#endif /* __RE_RASTERIZER_H__ */