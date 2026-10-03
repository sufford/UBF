/*
 * RE_StorageScanline.c — scanline стратегия.
 *
 * ★ ШАГ B1/B2: клиппинг свой — RE_clip_triangle() + RE_clip_test() из
 * RE_Clipper.c, заполнение спанов своё — RE_fill_triangle() из RE_Filler.c.
 * Ни zbufclip(), ни zbuffillGL4(), ни testclip() из zbuf.c на этом пути
 * больше не вызываются.
 *
 * ПАТЧ (harness):
 *   (1) Возвращаем 4-ю вершину квада при клиппинге. Раньше адаптер филлера
 *       прокидывал v4 как есть, а клиппер при клиппинге зовёт филлер с
 *       v4 == NULL — квад вырождался в треугольник.
 *   (2) clipcrop берётся из zspan (его выставил RE_rasterizer_render_part),
 *       больше не зависим от глобального R.
 *   (3) Точка врезки одна: RE_rasterizer_render_part вызывает rasterize()
 *       по одному разу на ZSpan, а не на каждый сэмпл.
 */

#include <math.h>
#include <string.h>
#include <float.h>
#include <limits.h>
#include <stdio.h>

#include "MEM_guardedalloc.h"
#include "BLI_math.h"
#include "BLI_utildefines.h"

#include "render_types.h"
#include "renderdatabase.h"
#include "RE_Rasterizer.h"
#include "zbuf.h"
#include "RE_Prof.h"     /* ★ PROF */

/* ★ part-bbox отсечение: точная копия логики zbuf_part_project() из zbuf.c.
 * Нужно потому, что zbuf_add_to_span() НЕ клампит x — вершина, улетевшая
 * далеко влево/вправо, даёт запись пикселей вне rect с заворотом в соседние
 * строки (полосы/диагонали на картинке). */
static int raster_part_clip(const float ho[4], const float bounds[4])
{
    const float wco = ho[3];
    int c = 0;

    if (ho[0] < bounds[0] * wco) c |= 1;
    else if (ho[0] > bounds[1] * wco) c |= 2;
    if (ho[1] > bounds[3] * wco) c |= 4;
    else if (ho[1] < bounds[2] * wco) c |= 8;

    return c;
}


typedef struct ScanlineFillCtx {
    int          vlr_index_base;
    const float *quad_v4;    /* ★ НОВОЕ: 4-я вершина исходного квада */
    const RE_RasterAttr *attr;  /* ★ ШАГ B2-ii: интерполяция атрибутов */
} ScanlineFillCtx;

/* ★ ШАГ B1/B2: адаптер между своим клиппером и своим филлером.
 *  - подставляет индекс VlakRen, соответствующий данному примитиву;
 *  - сохраняет бит RE_QUAD_OFFS, который выставил вызывающий код. */
static void scanline_fill(struct ZSpan *zspan, int obi, int zvlnr,
                          const float v1[4], const float v2[4],
                          const float v3[4], const float v4[4])
{
    ScanlineFillCtx *ctx = (ScanlineFillCtx *)zspan->sss_handle;
    int new_zvlnr = ctx->vlr_index_base;

    if (zvlnr & 0x8000000)   /* RE_QUAD_OFFS */
        new_zvlnr |= 0x8000000;

    /* v4 всегда NULL: квады расщепляются на два треугольника ДО вызова,
     * как это делает zbuffer_solid() в BI. attr — контекст интерполяции
     * атрибутов (шаг B2-ii); он передаётся вниз как параметр, а не через
     * общее состояние: storage общий для потоков. */
    (void)v4;
    RE_fill_triangle(zspan, obi, new_zvlnr, v1, v2, v3, NULL, ctx->attr);
}

/* ------------------------------------------------------------------------- */
/* Storage                                                                    */
/* ------------------------------------------------------------------------- */

typedef struct ScanlineStorage {
    int unused;   /* ★ storage общий для всех потоков: изменяемого состояния
                   * здесь хранить нельзя (была гонка на общем ctx). */
} ScanlineStorage;

static bool scanline_init(RE_IStorage *self, RE_Rasterizer *UNUSED(rasty))
{
    ScanlineStorage *s = MEM_callocN(sizeof(*s), "ScanlineStorage");
    self->user_data = s;
    return true;
}

static void scanline_exit(RE_IStorage *self)
{
    if (self->user_data) MEM_freeN(self->user_data);
    self->user_data = NULL;
}

static void scanline_rasterize(RE_IStorage *self, RE_Rasterizer *rasty,
                               RE_RasterSlot *UNUSED(slot),
                               RE_RasterDisplayArray *da,
                               struct ZSpan *zspan,
                               int obi_index, int vlr_index_base,
                               const float obwinmat[4][4], const float bounds[4],
                               const RE_RasterAttr *attr)
{
    ScanlineStorage *s = (ScanlineStorage *)self->user_data;
    const int stride = da->prim_type;
    const int num_prims = (stride > 0) ? (da->num_indices / stride) : 0;
    /* ★ ЛОКАЛЬНЫЙ контекст: storage общий для всех потоков, хранить в нём
     * изменяемое состояние нельзя (была гонка на общем s->ctx). */
    ScanlineFillCtx ctx;
    int p;

    (void)s;

    void *orig_sss_handle = zspan->sss_handle;

    ctx.attr = attr;      /* ★ ШАГ B2-ii */
    zspan->sss_handle = &ctx;

    RE_prof_count(RE_PROF_C_PRIMS, num_prims);   /* ★ PROF */
    {	/* ★ PROF — весь цикл по примитивам: projectvert + culling */
    double t_proj0 = RE_prof_tick();

    for (p = 0; p < num_prims; p++) {
        const int base = p * stride;
        float hoco[4][4];
        int   cflags[4], pclip[4];
        int   i, and_mask, partclip;

        ctx.vlr_index_base = da->vlr_indices[p];
        ctx.quad_v4 = (stride == 4) ? da->verts[da->indices[base + 3]].co : NULL;

        partclip = 0;
        for (i = 0; i < stride; i++) {
            const RE_RasterVertex *v = &da->verts[da->indices[base + i]];
            projectvert(v->co, (float (*)[4])obwinmat, hoco[i]);
            cflags[i] = RE_clip_test(hoco[i]);   /* ★ B2: свой тест, не testclip() из zbuf.c */
            pclip[i] = raster_part_clip(hoco[i], bounds);
            if (i == 0) partclip = pclip[i];
            else partclip &= pclip[i];
        }

        /* плитка целиком не покрывает грань (как zbuf_part_project в BI) */
        if (partclip) {
            RE_prof_count(RE_PROF_C_PRIMS_PARTCLIP, 1);   /* ★ PROF */
            continue;
        }

        and_mask = cflags[0];
        for (i = 1; i < stride; i++) and_mask &= cflags[i];
        if (and_mask) continue;

        /* ★ как в zbuffer_solid(): quad = два треугольника (v1,v2,v3) и
         * (v1,v3,v4), второй — с битом RE_QUAD_OFFS. Это даёт ту же
         * интерполяцию z и ту же логику span'ов, что и в BI. */
        if (stride == 4) {
            RE_clip_triangle(zspan, obi_index, p + 1,
                             hoco[0], hoco[1], hoco[2],
                             cflags[0], cflags[1], cflags[2],
                             scanline_fill);
            RE_clip_triangle(zspan, obi_index, p + 1 + RE_QUAD_OFFS,
                             hoco[0], hoco[2], hoco[3],
                             cflags[0], cflags[2], cflags[3],
                             scanline_fill);
        }
        else {
            RE_clip_triangle(zspan, obi_index, p + 1,
                             hoco[0], hoco[1], hoco[2],
                             cflags[0], cflags[1], cflags[2],
                             scanline_fill);
        }
    }

    RE_prof_span(RE_PROF_PROJECT, t_proj0);   /* ★ PROF */
    }	/* ★ PROF */

    zspan->sss_handle = orig_sss_handle;
}

RE_IStorage *RE_storage_create_scanline(void)
{
    RE_IStorage *s = MEM_callocN(sizeof(*s), "RE_IStorage scanline");
    s->init = scanline_init;
    s->exit = scanline_exit;
    s->rasterize = scanline_rasterize;
    return s;
}
