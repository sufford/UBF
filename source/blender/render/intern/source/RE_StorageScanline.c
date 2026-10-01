/*
 * RE_StorageScanline.c — scanline стратегия.
 *
 * Использует zbufclip() из zbuf.c через подмену zspan->zbuffunc.
 * Это даёт клиппинг near-plane и edge-walking без переписывания clippyra.
 *
 * ПАТЧ (harness):
 *   (1) Возвращаем 4-ю вершину квада при клиппинге. Раньше scanline_zbuffunc
 *       прокидывал v4 как есть, а zbufclip4()/zbufclip() при клиппинге зовут
 *       zbuffunc с v4 == NULL — квад вырождался в треугольник.
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
} ScanlineFillCtx;

/* Временный zbuffunc — вызывает zbuffillGL4 напрямую, но:
 *  - подставляет индекс VlakRen, соответствующий данному примитиву;
 *  - сохраняет бит RE_QUAD_OFFS, который выставил вызывающий код;
 *  - восстанавливает v4, если zbufclip() отдал NULL при клиппинге. */
static void scanline_zbuffunc(struct ZSpan *zspan, int obi, int zvlnr,
                              const float v1[4], const float v2[4],
                              const float v3[4], const float v4[4])
{
    ScanlineFillCtx *ctx = (ScanlineFillCtx *)zspan->sss_handle;
    int new_zvlnr = ctx->vlr_index_base;

    if (zvlnr & 0x8000000)   /* RE_QUAD_OFFS */
        new_zvlnr |= 0x8000000;

    /* v4 всегда NULL: квады расщепляются на два треугольника ДО вызова,
     * как это делает zbuffer_solid() в BI. */
    (void)v4;
    zbuffillGL4(zspan, obi, new_zvlnr, v1, v2, v3, NULL);
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
    /* ★ ОТЛАДКА: включить дамп tie-случаев (один раз, до старта потоков) */
    zbuf_debug_set_tiedump(getenv("UBF_TIE") != NULL);
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
                               const float obwinmat[4][4], const float bounds[4])
{
    ScanlineStorage *s = (ScanlineStorage *)self->user_data;
    const int stride = da->prim_type;
    const int num_prims = (stride > 0) ? (da->num_indices / stride) : 0;
    /* ★ ЛОКАЛЬНЫЙ контекст: storage общий для всех потоков, хранить в нём
     * изменяемое состояние нельзя (была гонка на общем s->ctx). */
    ScanlineFillCtx ctx;
    int p;

    (void)s;

    void (*orig_zbuffunc)(struct ZSpan *, int, int,
                          const float *, const float *, const float *, const float *) = zspan->zbuffunc;
    void *orig_sss_handle = zspan->sss_handle;

    zspan->sss_handle = &ctx;
    zspan->zbuffunc = scanline_zbuffunc;

    /* ★ ОТЛАДКА: хеш вершин display array и obwinmat В МОМЕНТ растеризации */
    if (getenv("UBF_CDUMP")) {
        static int cdbg = 0;
        if (cdbg < 6) {
            unsigned int hv = 2166136261u, hm = 2166136261u;
            const unsigned char *pm = (const unsigned char *)obwinmat;
            int k;
            for (k = 0; k < 64; k++) hm = (hm ^ pm[k]) * 16777619u;
            for (k = 0; k < da->num_verts * (int)sizeof(RE_RasterVertex); k++)
                hv = (hv ^ ((const unsigned char *)da->verts)[k]) * 16777619u;
            printf("[CDUMP] zofs=(%.1f,%.1f) verts=%d prims=%d obwinmat=%08x vhash=%08x "
                   "co0=(%.9g,%.9g,%.9g) id0=%d\n",
                   zspan->zofsx, zspan->zofsy, da->num_verts, da->num_prims, hm, hv,
                   (double)da->verts[0].co[0], (double)da->verts[0].co[1],
                   (double)da->verts[0].co[2], da->verts[0].orig_index);
            cdbg++;
        }
    }

    for (p = 0; p < num_prims; p++) {
        const int base = p * stride;
        float hoco[4][4];
        int   cflags[4], pclip[4];
        int   i, and_mask, partclip;

        ctx.vlr_index_base = da->vlr_indices[p];
        ctx.quad_v4 = (stride == 4) ? da->verts[da->indices[base + 3]].co : NULL;

        /* ★ ОТЛАДКА: сырые вершины примитива (поиск недетерминизма) */
        if (getenv("UBF_VDUMP")) {
            static int vdbg = 0;
            if (vdbg < 8 && stride == 4 && da->num_prims == 1) {
                printf("[VDUMP] prim p=%d vlridx=%d co:", p, da->vlr_indices[p]);
                for (i = 0; i < stride; i++) {
                    const RE_RasterVertex *v = &da->verts[da->indices[base + i]];
                    printf(" (%.6f,%.6f,%.6f)", v->co[0], v->co[1], v->co[2]);
                }
                printf("\n");
                vdbg++;
            }
        }

        partclip = 0;
        for (i = 0; i < stride; i++) {
            const RE_RasterVertex *v = &da->verts[da->indices[base + i]];
            projectvert(v->co, (float (*)[4])obwinmat, hoco[i]);
            cflags[i] = testclip(hoco[i]);
            pclip[i] = raster_part_clip(hoco[i], bounds);
            if (i == 0) partclip = pclip[i];
            else partclip &= pclip[i];
        }

        /* плитка целиком не покрывает грань (как zbuf_part_project в BI) */
        if (partclip && !getenv("UBF_NO_PARTCLIP")) {
            if (num_prims == 1) {   /* плоскость: 1 примитив */
                printf("[PARTCLIP-PLANE] ymin=%.1f partclip=0x%x bounds=(%.3f %.3f %.3f %.3f)\n",
                       -zspan->zofsy - 0.5f, partclip,
                       bounds[0], bounds[1], bounds[2], bounds[3]);
                for (i = 0; i < stride; i++)
                    printf("   v%d: ho=(%.3f %.3f %.3f w=%.3f) clip=0x%x\n",
                           i, hoco[i][0], hoco[i][1], hoco[i][2], hoco[i][3], pclip[i]);
            }
            continue;
        }

        and_mask = cflags[0];
        for (i = 1; i < stride; i++) and_mask &= cflags[i];
        if (and_mask) continue;

        /* ★ ОТЛАДКА: hash hoco для плоскости на конкретной плитке */
        if (getenv("UBF_HDUMP") && stride == 4 && da->num_prims == 1) {
            unsigned int hh = 2166136261u;
            for (i = 0; i < 4; i++) {
                const unsigned char *b = (const unsigned char *)hoco[i];
                for (int j = 0; j < 16; j++) hh = (hh ^ b[j]) * 16777619u;
            }
            printf("[HDUMP] tile zofs=(%.1f,%.1f) chash=%08x c0=%d c1=%d c2=%d c3=%d\n",
                   zspan->zofsx, zspan->zofsy, hh,
                   cflags[0], cflags[1], cflags[2], cflags[3]);
        }

        /* ★ ОТЛАДКА: подробности по примитиву (ищем недетерминизм) */
        if (getenv("UBF_PDUMP")) {
            printf("[PDUMP] zofs=(%.1f,%.1f) b=(%.5f,%.5f,%.5f,%.5f) obi=%d stride=%d p=%d/%d vlridx=%d "
                   "v0=(%.4f,%.4f,%.4f,%.4f) c=%d,%d,%d,%d partclip=0x%x\n",
                   zspan->zofsx, zspan->zofsy,
                   bounds[0], bounds[1], bounds[2], bounds[3],
                   obi_index, stride, p, num_prims,
                   da->vlr_indices[p],
                   hoco[0][0], hoco[0][1], hoco[0][2], hoco[0][3],
                   cflags[0], cflags[1], cflags[2], cflags[3], partclip);
        }

        /* ★ как в zbuffer_solid(): quad = два треугольника (v1,v2,v3) и
         * (v1,v3,v4), второй — с битом RE_QUAD_OFFS. Это даёт ту же
         * интерполяцию z и ту же логику span'ов, что и в BI. */
        if (stride == 4) {
            if (getenv("UBF_MARK")) {
                printf("[MARK] zbufclip A p=%d/%d stride=%d hoco0=(%.3f,%.3f,%.3f,%.3f)\n",
                       p, num_prims, stride, hoco[0][0], hoco[0][1], hoco[0][2], hoco[0][3]);
                fflush(stdout);
            }
            zbufclip(zspan, obi_index, p + 1,
                     hoco[0], hoco[1], hoco[2], cflags[0], cflags[1], cflags[2]);
            if (getenv("UBF_MARK")) {
                printf("[MARK] zbufclip B p=%d\n", p);
                fflush(stdout);
            }
            zbufclip(zspan, obi_index, p + 1 + RE_QUAD_OFFS,
                     hoco[0], hoco[2], hoco[3], cflags[0], cflags[2], cflags[3]);
            if (getenv("UBF_MARK")) {
                printf("[MARK] zbufclip done p=%d\n", p);
                fflush(stdout);
            }
        }
        else {
            zbufclip(zspan, obi_index, p + 1,
                     hoco[0], hoco[1], hoco[2], cflags[0], cflags[1], cflags[2]);
        }
    }

    /* ★ ОТЛАДКА: сколько пикселей в этом ZSpan реально заполнено.
     * Считаем только первые 8 вызовов, чтобы не есть CPU на больших сценах.
     * ВАЖНО: не читаем rasty->last_material — это общее поле, его пишет
     * render_part из других потоков (была гонка в самом зонде). */
    if (num_prims) {
        static int dbg = 0;
        if (dbg < 8) {
            const int npix = zspan->rectx * zspan->recty;
            int hits = 0;
            for (int k = 0; k < npix; k++)
                if (zspan->rectp[k]) hits++;
            printf("[SCAN] prims=%d filled_px=%d/%d (rect %dx%d)\n",
                   num_prims, hits, npix, zspan->rectx, zspan->recty);
            dbg++;
        }
    }

    zspan->zbuffunc = orig_zbuffunc;
    zspan->sss_handle = orig_sss_handle;

    /* ★ ОТЛАДКА: hash буфера rectp после растеризации этой display array */
    if (getenv("UBF_HDUMP")) {
        unsigned int hh = 2166136261u;
        const int np = zspan->rectx * zspan->recty;
        int hits = 0;
        for (int k = 0; k < np; k++) {
            hh = (hh ^ (unsigned int)zspan->rectp[k]) * 16777619u;
            if (zspan->rectp[k]) hits++;
        }
        printf("[HDUMP] AFTER zofs=(%.1f,%.1f) prims=%d hits=%d hash=%08x v0=%d v1000=%d v4000=%d z0=%d z1000=%d\n",
               zspan->zofsx, zspan->zofsy, num_prims, hits, hh,
               zspan->rectp[0], zspan->rectp[1000], zspan->rectp[4000],
               zspan->rectz[0], zspan->rectz[1000]);
    }
}

RE_IStorage *RE_storage_create_scanline(void)
{
    RE_IStorage *s = MEM_callocN(sizeof(*s), "RE_IStorage scanline");
    s->init = scanline_init;
    s->exit = scanline_exit;
    s->rasterize = scanline_rasterize;
    return s;
}
