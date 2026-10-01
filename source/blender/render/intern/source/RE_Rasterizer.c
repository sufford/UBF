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


#include "MEM_guardedalloc.h"
#include "BLI_math.h"
#include "BLI_utildefines.h"

#include "render_types.h"
#include "renderdatabase.h"
#include "render_result.h"
#include "RE_Rasterizer.h"
#include "zbuf.h"

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

RE_Rasterizer *RE_rasterizer_create(struct Render *re, RE_RasterStorageType type)
{
    RE_Rasterizer *rasty = MEM_callocN(sizeof(*rasty), "RE_Rasterizer");
    rasty->re = re;
    rasty->storage_type = type;
    rasty->storage = RE_storage_create(type);
    /* ★ ZSpan'ы на весь OSA-диапазон: их читает zbuffer_solid() после
     * render_part, чтобы вызвать fillfunc по одному разу на сэмпл */
    rasty->zspans = MEM_callocN(sizeof(ZSpan) * 16, "RE_Rasterizer zspans");
    rasty->num_zspans = 0;

    if (rasty->storage && rasty->storage->init)
        rasty->storage->init(rasty->storage, rasty);

    return rasty;
}

void RE_rasterizer_free(RE_Rasterizer *rasty)
{
    if (!rasty) return;

    /* ★ освободить буферы незакрытых ZSpan'ов (последний вызов render_part
     * мог оставить scratch-буферы для промежуточных сэмплов) */
    if (rasty->zspans) {
        int i;
        for (i = 0; i < rasty->num_zspans; i++) {
            ZSpan *zs = &rasty->zspans[i];
            if (!(rasty->num_zspans == 1 || i == rasty->num_zspans - 1)) {
                if (zs->rectz) MEM_freeN(zs->rectz);
                if (zs->rectp) MEM_freeN(zs->rectp);
                if (zs->recto) MEM_freeN(zs->recto);
            }
            zbuf_free_span(zs);
        }
        MEM_freeN(rasty->zspans);
        rasty->zspans = NULL;
        rasty->num_zspans = 0;
    }

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
                               struct RenderLayer *rl)
{
    /* ★ zspans живут в самом RE_Rasterizer, а не на стеке: их должен увидеть
     * вызывающий (zbuffer_solid) сразу после возврата, чтобы вызвать fillfunc
     * для каждого сэмпла с настоящим ZSpan. Потокобезопасно: каждый RenderPart
     * обрабатывается одним потоком, render_part и последующие fillfunc идут
     * подряд в одном потоке. */
    ZSpan *zspans = rasty ? rasty->zspans : NULL;
    ZSpan *zspan;
    struct Render *re;
    int samples, zsample;
    bool neg_zmask;

    if (!rasty || !rasty->scene || !pa) return;

    re = rasty->re;
    if (!re) return;

    rasty->num_zspans = 0;
    fflush(stdout); printf("[RASTERIZER] render_part enter (part %d, %dx%d, scene=%p, osa=%d) "
           "disprect=(%d,%d)-(%d,%d) win=(%d,%d)\n",
           pa->nr, pa->rectx, pa->recty, (void *)rasty->scene, re->osa,
           pa->disprect.xmin, pa->disprect.ymin, pa->disprect.xmax, pa->disprect.ymax,
           re->winx, re->winy);

    /* ★ ОТЛАДКА: хеш матриц вида, которые видит растеризатор */
    if (getenv("UBF_MDUMP")) {
        static int mdbg = 0;
        if (mdbg < 3) {
            unsigned int hv = 2166136261u, hw = 2166136261u;
            const unsigned char *pv = (const unsigned char *)re->viewmat;
            const unsigned char *pw = (const unsigned char *)re->winmat;
            int k;
            for (k = 0; k < 64; k++) {
                hv = (hv ^ pv[k]) * 16777619u;
                hw = (hw ^ pw[k]) * 16777619u;
            }
            printf("[MDUMP] render_part part=%d viewmat=%08x winmat=%08x viewmat00=%.9g winmat00=%.9g\n",
                   pa->nr, hv, hw,
                   (double)((const float *)re->viewmat)[0],
                   (double)((const float *)re->winmat)[0]);
            mdbg++;
        }
    }



    /* ★ (1) матрицы: ровно как в zbuffer_solid() */
    zbuf_make_winmat(re, rasty->winmat);
    copy_m4_m4(rasty->viewmat, re->viewmat);
    rasty->width  = re->winx;
    rasty->height = re->winy;

    /* ★ update_visibility вызывается ОДИН РАЗ из do_render_3d, до старта
     * потоков: здесь её быть не должно (гонка на slot->visible). */
    (void)rl;

    /* ★ (2) сэмплы: повторяем логику zbuffer_solid() */
    samples = (re->osa ? re->osa : 1);
    samples = MIN2(4, samples - pa->sample);
    if (samples < 1) samples = 1;

    /* ★ ОТЛАДКА: состояние буферов плитки (ищем краш в превью) */
    if (getenv("UBF_PBUF")) {
        printf("[PBUF] part=%d sample=%d osa=%d samples=%d rect=%dx%d "
               "rectz=%p rectp=%p recto=%p mask=%p\n",
               pa->nr, pa->sample, re->osa, samples, pa->rectx, pa->recty,
               (void *)pa->rectz, (void *)pa->rectp, (void *)pa->recto,
               (void *)pa->rectmask);
        fflush(stdout);
    }

    neg_zmask = (rl != NULL) && (rl->layflag & SCE_LAY_ZMASK) && (rl->layflag & SCE_LAY_NEG_ZMASK);

    for (zsample = 0; zsample < samples; zsample++) {
        zspan = &zspans[zsample];

        zbuf_alloc_span(zspan, pa->rectx, pa->recty, re->clipcrop);
        /* needed for transform from hoco to zbuffer co */
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
        /* to center the sample position */
        zspan->zofsx -= 0.5f;
        zspan->zofsy -= 0.5f;

        /* the buffers:
         *  - non-OSA (samples == 1): всё пишем прямо в pa->rect* — shade_samples
         *    читает именно их (zbufshade_tile);
         *  - OSA: промежуточные сэмплы в scratch, финальный — в pa->rect*. */
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

        fillrect(zspan->rectz, pa->rectx, pa->recty, 0x7FFFFFFF);
        fillrect(zspan->rectp, pa->rectx, pa->recty, 0);
        fillrect(zspan->recto, pa->rectx, pa->recty, 0);
    }

    /* bounds для bbox clip */
    {
        float bounds[4];
        bounds[0] = (2 * pa->disprect.xmin - re->winx - 1) / (float)re->winx;
        bounds[1] = (2 * pa->disprect.xmax - re->winx + 1) / (float)re->winx;
        bounds[2] = (2 * pa->disprect.ymin - re->winy - 1) / (float)re->winy;
        bounds[3] = (2 * pa->disprect.ymax - re->winy + 1) / (float)re->winy;

        /* Обход buckets → slots → display_arrays */
        for (int b = 0; b < rasty->scene->num_buckets; b++) {
            RE_RasterBucket *bucket = rasty->scene->buckets[b];

            /* state switch (v1: только кэш последнего материала) */
            rasty->last_material = bucket->material;

            for (RE_RasterSlot *slot = bucket->slots.first; slot; slot = slot->next) {
                if (!slot->visible) continue;

                ObjectInstanceRen *obi = slot->obi;
                float obwinmat[4][4];

                if (obi->flag & R_TRANSFORMED)
                    mul_m4_m4m4(obwinmat, rasty->winmat, obi->mat);
                else
                    copy_m4_m4(obwinmat, rasty->winmat);

                if (clip_render_object(obi->obr->boundbox, bounds, obwinmat))
                    continue;

                int obi_index = (int)(obi - re->objectinstance);

                for (RE_RasterDisplayArray *da = slot->display_arrays.first;
                     da; da = da->next)
                {
                    if (getenv("UBF_MARK")) {
                        printf("[MARK] rasterize call b=%d sl=%p da=%p verts=%d prims=%d obi=%d recto=%p\n",
                               b, (void *)slot, (void *)da, da->num_verts, da->num_prims,
                               obi_index, (void *)zspans[0].recto);
                        fflush(stdout);
                    }
                    for (zsample = 0; zsample < samples; zsample++) {
                        rasty->storage->rasterize(rasty->storage, rasty, slot, da,
                                                  &zspans[zsample], obi_index, 0,
                                                  (const float (*)[4])obwinmat, bounds);
                    }
                    if (getenv("UBF_MARK")) {
                        printf("[MARK] rasterize returned da=%p\n", (void *)da);
                        fflush(stdout);
                    }
                }
            }
        }
    }

    /* ★ ОТЛАДКА: хеш финального z-буфера плитки (детерминизм растеризации).
     * pa->rect* — конечный буфер (в non-OSA пишем прямо в него). */
    if (getenv("UBF_SUM")) {
        unsigned int h = 2166136261u;
        const int np = pa->rectx * pa->recty;
        for (int k = 0; k < np; k++) {
            h = (h ^ (unsigned int)pa->rectp[k]) * 16777619u;
            h = (h ^ (unsigned int)pa->recto[k]) * 16777619u;
            h = (h ^ (unsigned int)pa->rectz[k]) * 16777619u;
        }
        printf("[SUM] part %d (xmin=%d ymin=%d) hash=%08x p0=%d p1=%d p2=%d z0=%d\n",
               pa->nr, pa->disprect.xmin, pa->disprect.ymin, h,
               pa->rectp[0], pa->rectp[1], pa->rectp[2], pa->rectz[0]);
    }

    /* ★ ZSpan'ы НЕ освобождаем здесь: вызывающий (zbuffer_solid) сразу после
     * возврата обходит их и вызывает fillfunc по одному на сэмпл. Освобождение —
     * в следующем render_part и в RE_rasterizer_free(). Но span-массивы
     * zbuf_free_span() освобождаем теперь, они больше не нужны. */
    for (zsample = 0; zsample < samples; zsample++)
        zbuf_free_span(&zspans[zsample]);

    rasty->num_zspans = samples;
}
