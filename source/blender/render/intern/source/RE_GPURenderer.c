/*
 * RE_GPURenderer.c — GPU deferred-растеризатор (mini-EEVEE) для Blender Internal.
 * Custom fork.
 *
 * Этап M1: геометрия BI-сцены (VlakRen) -> GPU-ready массивы, затем G-буфер.
 *
 * Двухфазная модель (VlakRen живёт в рендер-джобе, GL рисует главный поток):
 *   - RE_gpu_renderer_scene_build(): рендер-джоба, БЕЗ GL. Строит CPU-массивы
 *     вершин/индексов из re->instancetable (VlakRen).
 *   - RE_gpu_renderer_gl_init / RE_gpu_renderer_draw(): главный поток (вьюпорт),
 *     с GL-контекстом. Загружает массивы в VBO/IBO, рисует G-буфер, выводит.
 *
 * На текущем шаге (M1) реализована и проверяется фаза 1 (сборка CPU-массивов)
 * и диагностика. GL-фаза — каркас, заполняется следом.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <float.h>

#include "MEM_guardedalloc.h"
#include "BLI_math.h"
#include "BLI_utildefines.h"

#include "DNA_meshdata_types.h"
#include "DNA_material_types.h"
#include "DNA_scene_types.h"
#include "DNA_lamp_types.h"
#include "DNA_texture_types.h"
#include "DNA_node_types.h"

#include "BKE_scene.h"
#include "BKE_node.h"
#include "BKE_image.h"
#include "BKE_colorband.h"   /* M19: BKE_colorband_evaluate — запекание колорбэнда */
#include "IMB_imbuf_types.h"
#include "IMB_imbuf.h"       /* M24: IMB_makemipmap — мип-цепочка BI для текстуры */
#include "IMB_colormanagement.h" /* M26: colorspace_srgb — декод sRGB->linear у образных карт */
#include "BLI_threads.h"     /* M24: LOCK_IMAGE вокруг генерации мипов, как в BI */

#include "render_types.h"
#include "renderdatabase.h"
#include "RE_GPURenderer.h"

/* ★ GL-фаза: используем raw OpenGL через GPU_glew.h (как BGE/GPU-модуль).
 * Контекст GL активен на главном потоке во вьюпорте. */
#include "GPU_glew.h"

/* ------------------------------------------------------------------------- */
/* CPU-сцена                                                                  */
/* ------------------------------------------------------------------------- */

/* M11: откуда слот берёт координаты и его translate/scale.
 * Поддерживаем TEXCO_UV (orco=0, uv как есть) и TEXCO_ORCO при mapping=FLAT
 * (orco=1: uv=(orco.xy+1)/2, затем size*(t-0.5)+ofs+0.5 — как do_2d_mapping
 * + texco_mapping в render_texture.c). UV в BI заранее масштабирован в [-1,1]
 * (shadeinput.c:1202), поэтому FLAT для UV — тождество, и наш UV-путь верен. */
static void gpu_slot_texco(MTex *mtex, int *r_orco, int *r_map, int *r_glob, float r_size[2], float r_ofs[2])
{
	*r_orco = 0;
	*r_glob = 0;
	*r_map = 0;
	r_size[0] = r_size[1] = 1.0f;
	r_ofs[0] = r_ofs[1] = 0.0f;
	if (!mtex) return;
	r_size[0] = mtex->size[0];
	r_size[1] = mtex->size[1];
	r_ofs[0] = mtex->ofs[0];
	r_ofs[1] = mtex->ofs[1];
	if (mtex->texco == TEXCO_ORCO) {
		/* M16: do_2d_mapping для ORCO — FLAT/CUBE/TUBE/SPHERE (0..3). */
		*r_orco = 1;
		*r_map = (int)mtex->mapping;
		if (*r_map < 0 || *r_map > 3) *r_map = 0;
		if (getenv("UBF_GPU_DEBUG") && *r_map != 0) {
			printf("[GPU-TEXMAP] texco=ORCO map=%d (0=FLAT,1=CUBE,2=TUBE,3=SPHERE)\n", *r_map);
		}
	}
	else if (mtex->texco == TEXCO_GLOB) {
		/* M27: texco==GLOB — BI использует МИРОВУЮ позицию (shi->gl) с тем же
		 * do_2d_mapping FLAT/CUBE/TUBE/SPHERE. orco=1 (идём в ветку маппинга),
		 * glob=1 (берём v_wpos вместо v_orco). */
		*r_orco = 1;
		*r_glob = 1;
		*r_map = (int)mtex->mapping;
		if (*r_map < 0 || *r_map > 3) *r_map = 0;
		if (getenv("UBF_GPU_DEBUG")) {
			printf("[GPU-TEXMAP] texco=GLOB map=%d (0=FLAT,1=CUBE,2=TUBE,3=SPHERE)\n", *r_map);
		}
	}
	else if (mtex->texco != TEXCO_UV) {
		/* не умеем (OBJECT и др.): остаёмся на UV, чтобы не рисовать мусор */
		if (getenv("UBF_GPU_DEBUG")) {
			printf("[GPU-TEXCO] mtex->texco=%d mapping=%d не поддержан -> UV\n",
			       (int)mtex->texco, (int)mtex->mapping);
		}
	}
}

/* M12: «Use Alpha» у образной карты — как BI решает, брать ли tin из альфы
 * (texres.talpha, imagetexture.c:215): TEX_USEALPHA включён, альфа не игнорируется
 * и не вычисляется. Используется и для MAP_ALPHA (a_use_a), и для MAP_SPEC/MAP_HAR. */
static int gpu_use_a(MTex *mtex)
{
	if (!mtex || !mtex->tex || mtex->tex->type != TEX_IMAGE || !mtex->tex->ima)
		return 0;
	Image *ima = (Image *)mtex->tex->ima;
	int flag = mtex->tex->imaflag;
	return ((flag & TEX_USEALPHA) && !(ima->flag & IMA_IGNORE_ALPHA) &&
	        !(flag & TEX_CALCALPHA)) ? 1 : 0;
}

/* ★ M19: запекаем колорбэнд текстуры в 256 отсчётов RGBA8 САМИМ BI
 * (BKE_colorband_evaluate). Тогда GLSL не нужно повторять ни интерполяцию
 * CONSTANT/LINEAR/EASE/CARDINAL/B_SPLINE, ни раскладку стопов — только
 * выбрать отсчёт. Возвращает 1, если колорбэнд реально вернул цвет
 * (в BI это флаг TEX_COLORBAND -> TEX_RGB, render_texture.c:1214). */
static int gpu_proc_bake_ramp(Tex *tex, unsigned char ramp[256 * 4])
{
	float col[4];
	int i, ok = 0;
	if (!tex || !tex->coba) return 0;
	for (i = 0; i < 256; i++) {
		float in = (float)i / 255.0f;
		col[0] = col[1] = col[2] = in;
		col[3] = 1.0f;
		if (BKE_colorband_evaluate(tex->coba, in, col)) ok = 1;
		/* CLAMP() в blenlib — операторный макрос (с блоком), поэтому здесь CLAMPIS */
		ramp[i * 4 + 0] = (unsigned char)(CLAMPIS(col[0], 0.0f, 1.0f) * 255.0f + 0.5f);
		ramp[i * 4 + 1] = (unsigned char)(CLAMPIS(col[1], 0.0f, 1.0f) * 255.0f + 0.5f);
		ramp[i * 4 + 2] = (unsigned char)(CLAMPIS(col[2], 0.0f, 1.0f) * 255.0f + 0.5f);
		ramp[i * 4 + 3] = (unsigned char)(CLAMPIS(col[3], 0.0f, 1.0f) * 255.0f + 0.5f);
	}
	return ok;
}

/* ★ M19: процедурная карта ЦВЕТА (TEX_CLOUDS / TEX_BLEND) в слоте MAP_COL.
 * BI (render_texture.c: texco_mapping -> texfunc -> BRICONT -> colorband):
 *   co     = texco (ORCO или UV(+z=0));  для процедурных do_2d_mapping НЕ зовётся
 *   texvec = size[i]*(co[i] + ofs[i])   (projx/y/z = 1/2/3 по умолчанию)
 *   tin    = <texfunc>(texvec)
 *   tin    = (tin-0.5)*contrast + bright - 0.5, clamp [0,1] если !TEX_NO_CLAMP
 *   tcol   = colorband(tin)  либо (tin,tin,tin)
 * Дальше обычный texture_rgb_blend(base, tcol, base, tin, colfac, blendtype) —
 * поэтому в GB_FS достаточно подменить vec4 tc (rgb=tcol, a=tin).
 * Возвращает 1, если слот опознан как процедурная карта цвета. */
static int gpu_proc_extract(RE_GPUMesh *m, MTex *mtex)
{
	Tex *tex;
	if (!mtex || !(tex = mtex->tex)) return 0;
	if (tex->type != TEX_CLOUDS && tex->type != TEX_BLEND && tex->type != TEX_WOOD && tex->type != TEX_DISTNOISE) return 0;
	/* ★ M20: раньше принимали только MAP_COL. Теперь та же процедурная текстура
	 * может влиять на блик/жёсткость/цвет блика/отражение (BI: один сэмпл
	 * `texres` питает все каналы сразу). Наличие MAP_COL отдельно запоминаем:
	 * только он модулирует базовый цвет (texture_rgb_blend по render_texture.c). */
	if (!(mtex->mapto & (MAP_COL | MAP_SPEC | MAP_HAR | MAP_COLSPEC | MAP_REF | MAP_NORM))) return 0;

	m->proc_type = (int)tex->type;
	m->proc_col = (mtex->mapto & MAP_COL) ? 1 : 0;
	m->proc_stype = (int)tex->stype;
	m->proc_flip = (tex->flag & TEX_FLIPBLEND) ? 1 : 0;
	m->proc_noisesize = tex->noisesize;
	m->proc_noisedepth = (int)tex->noisedepth;
	if (m->proc_noisedepth < 0) m->proc_noisedepth = 0;
	if (m->proc_noisedepth > 8) m->proc_noisedepth = 8;   /* потолок GLSL-цикла */
	m->proc_basis = (int)tex->noisebasis;
	m->proc_wf = (int)tex->noisebasis2;
	m->proc_dist = tex->dist_amount;
	m->proc_hard = (tex->noisetype != TEX_NOISESOFT) ? 1 : 0;
	m->proc_bright = tex->bright;
	m->proc_contrast = tex->contrast;
	m->proc_noclamp = (tex->flag & TEX_NO_CLAMP) ? 1 : 0;
	m->proc_has_ramp = gpu_proc_bake_ramp(tex, m->proc_ramp);

	/* blending — ровно как у образной диффузной карты (M9/M14) */
	m->tex_blend = mtex->blendtype;
	m->colfac = mtex->colfac;

	/* texco процедурной карты: ORCO или UV (маппинг FLAT/CUBE/TUBE/SPHERE
	 * применяется BI только к TEX_IMAGE — см. texco_mapping) */
	m->tex_orco = (mtex->texco == TEXCO_ORCO) ? 1 : 0;
	m->tex_map = 0;
	m->tex_size[0] = mtex->size[0];
	m->tex_size[1] = mtex->size[1];
	m->tex_ofs[0] = mtex->ofs[0];
	m->tex_ofs[1] = mtex->ofs[1];
	/* ★ M27: источник координат ПРОЦЕДУРНОЙ карты. BI (render_texture.c:2306-2352)
	 * для процедурных не зовёт do_2d_mapping, а берёт co напрямую:
	 *   ORCO   -> shi->lo (нормированный ORCO, [-1,1])        => porco=1
	 *   GLOB   -> shi->gl (МИРОВАЯ позиция)                    => porco=2
	 *   OBJECT -> ob->imat_ren * shi->co; в mapFull у всех OBJECT-слотов
	 *             mtex->object == NULL, и BI падает на shi->co,
	 *             а shi->co — КАМЕРНОЕ пространство (shadeinput.c:417) => porco=3
	 *   UV     -> (uv, 0)                                       => porco=0 */
	m->proc_orco = (mtex->texco == TEXCO_ORCO) ? 1
	             : ((mtex->texco == TEXCO_GLOB) ? 2
	             : ((mtex->texco == TEXCO_OBJECT) ? 3 : 0));
	m->proc_size[0] = m->tex_size[0];
	m->proc_size[1] = m->tex_size[1];
	m->proc_ofs[0] = m->tex_ofs[0];
	m->proc_ofs[1] = m->tex_ofs[1];
	if (mtex->texco != TEXCO_ORCO && mtex->texco != TEXCO_UV &&
	    mtex->texco != TEXCO_GLOB && mtex->texco != TEXCO_OBJECT) {
		if (getenv("UBF_GPU_DEBUG")) {
			printf("[GPU-TEXCO] proc texco=%d не поддержан -> UV\n", (int)mtex->texco);
		}
	}

	/* M20: побочные каналы той же процедурной текстуры. В шейдере они берут
	 * `tc`/`sv_tin` (результат gpu_p_tex) — ровно как образные карты в M12/M13/M17.
	 * BI на процедурных текстурах tin = значение (не альфа), поэтому sv_use_a
	 * не важен: tc.a = tin и dot(tc.rgb) = tin. */
	m->sv_use_a = 0;
	/* M23: MAP_NORM — рельеф от процедурной карты (bump в GB_FS). */
	if (mtex->mapto & MAP_NORM) {
		m->proc_norm = 1;
		m->proc_norfac = mtex->norfac;
	}
	if (mtex->mapto & MAP_SPEC) {
		m->spec_has = 1;
		m->spec_blend = mtex->blendtype;
		m->spec_fac = mtex->specfac;
		m->spec_def = mtex->def_var;
	}
	if (mtex->mapto & MAP_HAR) {
		m->har_has = 1;
		m->har_blend = mtex->blendtype;
		m->har_fac = mtex->hardfac;
		m->har_def = mtex->def_var;
	}
	if (mtex->mapto & MAP_COLSPEC) {
		m->colspec_has = 1;
		m->colspec_blend = mtex->blendtype;
		m->colspec_fac = mtex->colspecfac;
	}
	if (mtex->mapto & MAP_REF) {
		m->refl_has = 1;
		m->refl_blend = mtex->blendtype;
		m->refl_fac = mtex->difffac;
		m->refl_def = mtex->def_var;
	}
	return 1;
}

/* ★ M22: ВТОРАЯ процедурная текстура материала (слот != 0) как источник
 * MAP_SPEC/MAP_HAR. В сценах это jolly: слот 0 — образная `Concrete.jpg`
 * (MAP_COL), слот 1 — CLOUDS с mapto=0x122 (NORM|SPEC|HAR), basis=8 (CRACKLE).
 * Параметры blend/fac/def складываем в proc2_*, а в spec_has/har_has переносим
 * ПОЗЖЕ — после общего блока `if (found)`, иначе сброс M20 их затрёт. */
static int gpu_proc_extract2(RE_GPUMesh *m, MTex *mtex)
{
	Tex *tex;
	if (!mtex || !(tex = mtex->tex)) return 0;
	if (tex->type != TEX_CLOUDS && tex->type != TEX_BLEND && tex->type != TEX_WOOD && tex->type != TEX_DISTNOISE) return 0;
	if (!(mtex->mapto & (MAP_SPEC | MAP_HAR | MAP_NORM))) return 0;

	m->has_proc2 = 1;
	m->proc2_type = (int)tex->type;
	m->proc2_stype = (int)tex->stype;
	m->proc2_flip = (tex->flag & TEX_FLIPBLEND) ? 1 : 0;
	m->proc2_noisesize = tex->noisesize;
	m->proc2_noisedepth = (int)tex->noisedepth;
	if (m->proc2_noisedepth < 0) m->proc2_noisedepth = 0;
	if (m->proc2_noisedepth > 8) m->proc2_noisedepth = 8;   /* потолок GLSL-цикла */
	m->proc2_basis = (int)tex->noisebasis;
	m->proc2_wf = (int)tex->noisebasis2;
	m->proc2_dist = tex->dist_amount;
	m->proc2_hard = (tex->noisetype != TEX_NOISESOFT) ? 1 : 0;
	m->proc2_bright = tex->bright;
	m->proc2_contrast = tex->contrast;
	m->proc2_noclamp = (tex->flag & TEX_NO_CLAMP) ? 1 : 0;
	m->proc2_has_ramp = gpu_proc_bake_ramp(tex, m->proc2_ramp);
	/* texco: как и у первой, маппинг FLAT/CUBE/TUBE/SPHERE BI зовёт только
	 * для TEX_IMAGE, поэтому здесь ORCO (1), GLOB (2, мир), OBJECT (3, камера) либо UV (0). */
	m->proc2_orco = (mtex->texco == TEXCO_ORCO) ? 1
	              : ((mtex->texco == TEXCO_GLOB) ? 2
	              : ((mtex->texco == TEXCO_OBJECT) ? 3 : 0));
	m->proc2_size[0] = mtex->size[0];
	m->proc2_size[1] = mtex->size[1];
	m->proc2_ofs[0] = mtex->ofs[0];
	m->proc2_ofs[1] = mtex->ofs[1];

	if (mtex->mapto & MAP_SPEC) {
		m->proc2_spec = 1;
		m->proc2_spec_blend = mtex->blendtype;
		m->proc2_spec_fac = mtex->specfac;
		m->proc2_spec_def = mtex->def_var;
	}
	if (mtex->mapto & MAP_HAR) {
		m->proc2_har = 1;
		m->proc2_har_blend = mtex->blendtype;
		m->proc2_har_fac = mtex->hardfac;
		m->proc2_har_def = mtex->def_var;
	}
	if (mtex->mapto & MAP_NORM) {   /* M23: рельеф */
		m->proc2_norm = 1;
		m->proc2_norfac = mtex->norfac;
	}
	return 1;
}

static RE_GPUMesh *gpu_mesh_new(RE_GPUScene *scene, Material *ma,
                                float model[4][4], float nmat[3][3],
                                float obmat[4][4])
{
	RE_GPUMesh *m = MEM_callocN(sizeof(*m), "RE_GPUMesh");
	m->ma = ma;
	/* Прозрачность. BI учитывает ma->alpha ТОЛЬКО при MA_TRANSP (иначе shadeoutput.c
	 * принудительно ставит alpha=1.0) — повторяем это правило, чтобы материал с
	 * alpha<1 но выключенной прозрачностью оставался непрозрачным.
	 * Дальше BI расходится на два пути: Z-прозрачность (MA_ZTRANSP, обычный
	 * z-буфер) и RayTransp (MA_RAYTRANSP, только через raytracer). У нас
	 * растеризатор, поэтому берём ОБА: иначе стекло, сделанное через RayTransp
	 * (так в mono), молча рендерилось бы непрозрачным. */
	m->alpha = 1.0f;
	if (ma && (ma->mode & MA_TRANSP) && (ma->mode & (MA_ZTRANSP | MA_RAYTRANSP))) {
		m->alpha = ma->alpha;
		if (m->alpha < 0.0f) m->alpha = 0.0f;
		if (m->alpha > 1.0f) m->alpha = 1.0f;
	}
	copy_m4_m4(m->model, model);
	copy_m3_m3(m->normal_mat, nmat);
	if (obmat) copy_m4_m4(m->obmat, obmat);

	/* ★ M5: блик материала. BI берёт его из ma->specr/g/b (цвет) и ma->har
	 * (hardness, 1..511 — «жёсткость» блика): shadeoutput.c:1574/1598-1610.
	 * Значения по умолчанию BI (белый блик, hardness 50) ставим сами, потому
	 * что массив рендера может прийти без материала. */
	m->spec_col[0] = m->spec_col[1] = m->spec_col[2] = 1.0f;
	m->hard = 50.0f;
	if (ma) {
		m->spec_col[0] = ma->specr;
		m->spec_col[1] = ma->specg;
		m->spec_col[2] = ma->specb;
		m->hard = (float)ma->har;
		if (m->hard < 1.0f) m->hard = 1.0f;
		if (m->hard > 511.0f) m->hard = 511.0f;
		if (getenv("UBF_GPU_DEBUG"))
			printf("[GPU-SPEC] ma='%s' spec=%.3f specr=%.3f specg=%.3f specb=%.3f har=%d\n",
			       ma->id.name + 2, ma->spec, ma->specr, ma->specg, ma->specb, (int)ma->har);
	}

	/* ★ SSR: БАЗОВАЯ сила отражения. BI включает зеркало ТОЛЬКО при включённой
	 * галке Mirror (ma->mode & MA_RAYMIRROR) и ненулевом ma->ray_mirror, а силой
	 * служит ma->ray_mirror (в BI умноженный на fresnel, здесь без него).
	 * Раньше база была ma->ref — а это diffuse_intensity (по умолчанию 0.8)!
	 * Поэтому зеркалили ВСЕ материалы, даже без галки Mirror, и с неверной силой. */
	m->refl_base = (ma && (ma->mode & MA_RAYMIRROR) && ma->ray_mirror > 0.0f)
	               ? ma->ray_mirror : 0.0f;

	/* M5/M9: диффузная карта-изображение материала. Предпочитаем слот, который
	 * реально влияет на цвет (MAP_COL), иначе берём первую попавшуюся
	 * карту изображения. Сами пиксели грузятся на главном потоке (gl_upload).
	 * Вместе с картой запоминаем, КАК её смешивать с базовым цветом
	 * (`texture_rgb_blend`: blend_type + mtex->colfac). */
	m->tex_blend = MTEX_BLEND;   /* 0 = MIX (умолчание BI: карта заменяет цвет) */
	m->colfac = 1.0f;
	/* M24: мипы по умолчанию выключены (BI: только при TEX_MIPMAP) */
	m->tex_mipmap = 0;
	m->tex_gauss = 0;
	/* ★ M25 (раунд 82): shi->refl = ma->ref (RNA `diffuse_intensity`). BI множит
	 * на него вклад ламп и ambient; без него сцена ярче scanline на 1/0.8.
	 * Дефолт BI для ma->ref — 0.8, поэтому берём именно его, а не 1.0. */
	m->refl = ma ? ma->ref : 0.8f;
	if (ma) {
		MTex *found = NULL;
		int found_proc = 0;
		int found_proc2 = 0;
		for (int i = 0; i < MAX_MTEX; i++) {
			MTex *mtex = ma->mtex[i];
			if (!mtex || !mtex->tex) continue;
			if (mtex->tex->type != TEX_IMAGE || !mtex->tex->ima) {
				/* ★ M19: процедурная карта цвета (CLOUDS/BLEND) в слоте MAP_COL.
				 * Берём её ТОЛЬКО как слот 0 (пока образной диффузной карты нет):
				 * в сценах процедурная карта в материале всегда одна. */
				if (getenv("UBF_GPU_DEBUG") && mtex->mapto) {
					printf("[GPU-PROC] ma='%s' slot=%d tex='%s' type=%d stype=%d mapto=0x%03X noise_scale=%.3f depth=%d basis=%d soft=%d\n",
					       ma->id.name + 2, i, mtex->tex->id.name + 2,
					       (int)mtex->tex->type, (int)mtex->tex->stype, (int)mtex->mapto,
					       mtex->tex->noisesize, mtex->tex->noisedepth,
					       (int)mtex->tex->noisebasis, mtex->tex->noisetype == TEX_NOISESOFT);
				}
				if (!found && !found_proc && gpu_proc_extract(m, mtex)) {
					found_proc = 1;
					m->n_tex = 1;
					if (getenv("UBF_GPU_DEBUG")) {
						printf("[GPU-PROCOK] ma='%s' slot=%d type=%d stype=%d ramp=%d basis=%d depth=%d scale=%.3f orco=%d colfac=%.2f bt=%d col=%d spec=%d har=%d cs=%d refl=%d\n",
						       ma->id.name + 2, i, m->proc_type, m->proc_stype,
						       m->proc_has_ramp, m->proc_basis, m->proc_noisedepth,
						       m->proc_noisesize, m->proc_orco, m->colfac, m->tex_blend,
						       m->proc_col, m->spec_has, m->har_has, m->colspec_has, m->refl_has);
						printf("[GPU-SPECCFG] ma='%s' spec_has=%d spec_fac=%.3f spec_def=%.3f spec_blend=%d\n",
						       ma->id.name + 2, m->spec_has, m->spec_fac, m->spec_def, m->spec_blend);
					}
				}
				/* ★ M22: та же процедурная текстура, но на ДРУГОМ слоте — берём её
				 * как источник MAP_SPEC/MAP_HAR (свой конфиг + свой колорбэнд). */
				else if (!found_proc2 && gpu_proc_extract2(m, mtex)) {
					found_proc2 = 1;
					if (getenv("UBF_GPU_DEBUG")) {
						printf("[GPU-PROC2OK] ma='%s' slot=%d type=%d stype=%d ramp=%d basis=%d depth=%d scale=%.3f orco=%d spec=%d har=%d norm=%d\n",
						       ma->id.name + 2, i, m->proc2_type, m->proc2_stype,
						       m->proc2_has_ramp, m->proc2_basis, m->proc2_noisedepth,
						       m->proc2_noisesize, m->proc2_orco, m->proc2_spec, m->proc2_har,
						       m->proc2_norm);
					}
				}
				continue;
			}
			if (mtex->mapto & MAP_COL) {
				if (!found) {
					m->ima = mtex->tex->ima;
					found = mtex;
					m->n_tex = 1;
				}
				/* M14: дополнительные диффузные слоты (цепочка, как в BI) */
				else if (m->n_tex < GPU_MAX_TEX_SLOTS) {
					int k = m->n_tex - 1;   /* индекс в tex_ima2[] */
					m->tex_ima2[k] = mtex->tex->ima;
					m->tex_blend2[k] = mtex->blendtype;
					m->tex_rgbtoint2[k] = (mtex->texflag & MTEX_RGBTOINT) ? 1 : 0;
					m->colfac2[k] = mtex->colfac;
					gpu_slot_texco(mtex, &m->tex_orco2[k], &m->tex_map2[k], &m->tex_glob2[k], m->tex_size2[k], m->tex_ofs2[k]);
					m->n_tex++;
				}
			}
			/* нет MAP_COL: берём первую образную карту как слот 0 */
			else if (!found) {
				m->ima = mtex->tex->ima;
				found = mtex;
				m->n_tex = 1;
			}
		}
		if (found) {
			if (found_proc) {
				/* В материале есть и образная, и процедурная карта цвета.
				 * BI применил бы обе (слоты по порядку), но образную мы
				 * поддерживаем полнее — она и остаётся слотом 0. */
				if (getenv("UBF_GPU_DEBUG")) {
					printf("[GPU-PROCIMG] ma='%s': есть и образная, и процедурная карта — берём образную\n",
					       ma->id.name + 2);
				}
				m->proc_type = 0;
				m->proc_has_ramp = 0;
				/* M20: сбрасываем и побочные каналы процедурной текстуры — иначе
				 * её spec/har-факторы применились бы к сэмплу образной карты. */
				m->proc_col = 0;
				m->spec_has = 0;
				m->har_has = 0;
				m->colspec_has = 0;
				m->refl_has = 0;
				m->proc_norm = 0;      /* M23: то же для рельефа */
				m->proc_norfac = 0.0f;
			}
			m->tex_blend = found->blendtype;
			m->colfac = found->colfac;
			/* ★ раунд 83-fix3: BI берёт фактором `tin` (render_texture.c:2440) —
			 * для картинки это 1.0, и лишь при MTEX_RGBTOINT яркость. */
			m->tex_rgbtoint = (found->texflag & MTEX_RGBTOINT) ? 1 : 0;
			/* ★ M24: BI включает мипы только при TEX_MIPMAP у Tex-слота
			 * (imagetexture.c:883 image_mipmap_test), фильтр — TEX_GAUSS_MIP. */
			if (found->tex) {
				m->tex_mipmap = (found->tex->imaflag & TEX_MIPMAP) ? 1 : 0;
				m->tex_gauss = (found->tex->imaflag & TEX_GAUSS_MIP) ? 1 : 0;
			}
			gpu_slot_texco(found, &m->tex_orco, &m->tex_map, &m->tex_glob, m->tex_size, m->tex_ofs);
			/* M17: MAP_REF может лежать на ЛЮБОМ слоте, не только на диффузном.
			 * Для диагностики просканируем все слоты. */
			if (getenv("UBF_GPU_DEBUG") && ma) {
				for (int si = 0; si < MAX_MTEX; si++) {
					MTex *mt = ma->mtex[si];
					if (mt && (mt->mapto & MAP_REF)) {
						printf("[GPU-REFLOK] ma='%s' REFL на слоте %d tex=%s mapto=0x%03X fac=%.2f def=%.2f\n",
						       ma->id.name + 2, si,
						       mt->tex ? mt->tex->id.name + 2 : "<none>",
						       (int)mt->mapto, mt->difffac, mt->def_var);
					}
				}
			}
			/* M12: MAP_SPEC/MAP_HAR. В сценах они на том же слоте, что и диффузная
			 * карта (jolly/something2: slot 0 = COL|SPEC|HAR|…), поэтому берём
			 * из found и переиспользуем сэмпл диффузной текстуры. Если блик-карта
			 * лежит на ДРУГОМ слоте — не умеем (в сценах таких нет, см. probe). */
			if (found->mapto & MAP_SPEC) {
				m->spec_has = 1;
				m->spec_blend = found->blendtype;
				m->spec_fac = found->specfac;
				m->spec_def = found->def_var;
				m->sv_use_a = gpu_use_a(found);
			}
			if (found->mapto & MAP_HAR) {
				m->har_has = 1;
				m->har_blend = found->blendtype;
				m->har_fac = found->hardfac;
				m->har_def = found->def_var;
			}
			/* M13: MAP_COLSPEC (цвет блика) — на том же слоте, переиспользуем tc */
			if (found->mapto & MAP_COLSPEC) {
				m->colspec_has = 1;
				m->colspec_blend = found->blendtype;
				m->colspec_fac = found->colspecfac;
			}
			/* M17: MAP_REFL (отражение) — модулирует силу зеркального отражения.
			 * shi->refl стартует из ma->ref и меняется texture_value_blend.
			 * База уже стоит из gpu_mesh_new (ma->ref); тут лишь модуляция. */
			if (found->mapto & MAP_REF) {
				m->refl_has = 1;
				m->refl_blend = found->blendtype;
				m->refl_fac = found->difffac;
				m->refl_def = found->def_var;
				m->sv_use_a = gpu_use_a(found);   /* tin = альфа (Use Alpha), иначе яркость */
			}
		}
		/* ★ M22: вторую процедурную текстуру применяем ПОСЛЕ образной — иначе
		 * сброс M20 (когда в материале есть и образная, и процедурная карта)
		 * затрёт её spec/har. BI обрабатывает слоты по порядку, и обе карты
		 * влияют на свои каналы. */
		if (m->has_proc2) {
			if (m->proc2_spec) {
				m->spec_has = 1;
				m->spec_blend = m->proc2_spec_blend;
				m->spec_fac = m->proc2_spec_fac;
				m->spec_def = m->proc2_spec_def;
			}
			if (m->proc2_har) {
				m->har_has = 1;
				m->har_blend = m->proc2_har_blend;
				m->har_fac = m->proc2_har_fac;
				m->har_def = m->proc2_har_def;
			}
			m->sv_use_a = 0;   /* процедурная: tin = значение (tc2.a), не альфа */
		}
		/* ★ M23: источник рельефа MAP_NORM. Вторая карта приоритетнее: в сценах
		 * именно она несёт вортоновский рельеф (jolly, слот 1, mapto=0x122). */
		if (m->proc2_norm) {
			m->bump_src = 2;
			m->bump_norfac = m->proc2_norfac;
		}
		else if (m->proc_norm) {
			m->bump_src = 1;
			m->bump_norfac = m->proc_norfac;
		}
		else {
			m->bump_src = 0;
			m->bump_norfac = 0.0f;
		}
		if (getenv("UBF_GPU_DEBUG") && ma && m->bump_src != 0) {
			printf("[GPU-PROCBUMP] ma='%s' src=%d norfac=%.3f\n",
			       ma->id.name + 2, m->bump_src, m->bump_norfac);
		}
	}
	/* Если mtex-слота с картой нет — ищем ноду Image Texture. В нод-материалах
	 * (например, motorcycle: mtex-текстур нет вовсе) картинка живёт только там,
	 * у ноды SH_NODE_TEX_IMAGE изображение лежит в node->id. */
	if (!m->ima && ma->nodetree) {
		for (bNode *node = ma->nodetree->nodes.first; node; node = node->next) {
			if (node->type == SH_NODE_TEX_IMAGE && node->id) {
				m->ima = node->id;
				break;
			}
		}
	}

	/* ★ M8: нормальная карта (image, TANGENT-space). Только образная: процедурные
	 * карты (CLOUDS и т.п.) BI считает на лету — их мы не умеем, пропускаем.
	 * normapspace==MTEX_NSPACE_TANGENT — касательное пространство (обычный случай);
	 * иначе (WORLD/OBJECT/CAMERA) карта задаёт нормаль в другом пространстве —
	 * тоже не умеем, пропускаем. */
	if (ma) {
		for (int i = 0; i < MAX_MTEX; i++) {
			MTex *mtex = ma->mtex[i];
			if (!mtex || !mtex->tex) continue;
			if (mtex->tex->type != TEX_IMAGE || !mtex->tex->ima) continue;
			if ((mtex->mapto & MAP_NORM)) {
				/* ★ M28: раньше требовали normapspace==TANGENT, из-за чего
				 * normal-слои стен/пола mapFull (normapspace=CAMERA, т.е. 0)
				 * молча пропускались -> не было bump. BI применяет height-bump
				 * (без TEX_NORMALMAP) в ЛЮБОМ пространстве (compatible_bump_compute
				 * работает по экранным производным яркости), поэтому пропускаем
				 * только ТАНГЕНЦИАЛЬНУЮ нормаль не в TANGENT-пространстве. */
				if ((mtex->tex->imaflag & TEX_NORMALMAP) &&
				    mtex->normapspace != MTEX_NSPACE_TANGENT)
					continue;
				m->nima = mtex->tex->ima;
				m->n_has = 1;
				/* M26: BI (render_texture.c:1783) считает height-bump ТОЛЬКО
				 * когда у образа НЕТ TEX_NORMALMAP; с флагом — тангенциальная
				 * карта нормалей. Повторяем это различие. */
				m->n_is_height = (mtex->tex->imaflag & TEX_NORMALMAP) ? 0 : 1;
				m->n_norfac = mtex->norfac;
				/* M28: координаты карты нормалей (как у диффузной). */
				gpu_slot_texco(mtex, &m->n_orco, &m->n_map, &m->n_glob, m->n_size, m->n_ofs);
				if (getenv("UBF_GPU_DEBUG")) {
					printf("[GPU-NORM] ma='%s' img=%s is_height=%d norfac=%.3f "
					       "nsp=%d orco=%d glob=%d map=%d\n",
					       ma->id.name + 2, mtex->tex->ima->id.name,
					       m->n_is_height, m->n_norfac, (int)mtex->normapspace,
					       m->n_orco, m->n_glob, m->n_map);
				}
				break;
			}
		}
	}

	/* ★ M10: альфа-карта (MAP_ALPHA, image). Даёт ПОПИКСЕЛЬНУЮ прозрачность и
	 * спасает материалы с ma->alpha=0 (иначе весь меш отбрасывался). Запоминаем
	 * параметры смешивания (`texture_value_blend`) и способ получения значения
	 * карты: tin = альфа картинки, если включён Use Alpha (TEX_USEALPHA) и альфа
	 * не игнорируется/не вычисляется; иначе — яркость (render_texture.c:2712). */
	m->a_def = 1.0f;
	m->a_fac = 1.0f;
	m->a_blend = MTEX_BLEND;
	if (ma) {
		for (int i = 0; i < MAX_MTEX; i++) {
			MTex *mtex = ma->mtex[i];
			if (!mtex || !mtex->tex) continue;
			if (mtex->tex->type != TEX_IMAGE || !mtex->tex->ima) continue;
			if (mtex->mapto & MAP_ALPHA) {
				m->aima = mtex->tex->ima;
				m->a_has = 1;
				m->a_blend = mtex->blendtype;
				m->a_fac = mtex->alphafac;
				m->a_def = mtex->def_var;
				m->a_use_a = gpu_use_a(mtex);
				gpu_slot_texco(mtex, &m->a_orco, &m->a_map, &m->a_glob, m->a_size, m->a_ofs);
				break;
			}
		}
	}

	if (scene->meshes == NULL) {
		scene->meshes = m;
	}
	else {
		RE_GPUMesh *last = scene->meshes;
		while (last->next) last = last->next;
		last->next = m;
	}
	scene->num_meshes++;
	/* M15: полупрозрачный меш — рисуем forward-проходом. Эффективная alpha < 1
	 * при наличии альфа-карты или ma->alpha<1 (в сценах у таких мешей ma->alpha=0
	 * + Z_TRANSPARENCY, прозрачность берётся из карты — см. M10). */
	m->transparent = (m->a_has || (ma && ma->alpha < 0.999f)) ? 1 : 0;

	/* ★ M16 (раунд 67): материал VOLUME. В BI сканирующий растеризатор НЕ умеет
	 * объёмный рендер (он есть только у raytrace-ветки, rayshade.c) — вместо
	 * этого convertblender.c:729 принудительно ставит такому материалу MA_TRANSP
	 * и снимает MA_SHADBUF, то есть объёмный материал рисуется как ОБЫЧНАЯ
	 * ПРОЗРАЧНАЯ поверхность с alpha материала. Повторяем это ровно: иначе у
	 * объёмного материала с alpha=1 получался бы непрозрачный ящик, тогда как в
	 * BI он всё равно идёт по прозрачному конвейеру. */
	if (ma && ma->material_type == MA_TYPE_VOLUME)
		m->transparent = 1;
	return m;
}

/* ★★ r59: ГЛАВНАЯ ОШИБКА ПОЗИЦИОНИРОВАНИЯ ★★
 * Blender Internal хранит вершины (VertRen.co) и нормали УЖЕ В ПРОСТРАНСТВЕ
 * КАМЕРЫ: база рендера конвертируется с учётом вида (именно поэтому
 * программный растеризатор ходит обратно через object->imat_ren). Растеризатор
 * же весь построен на МИРОВЫХ координатах: u_obmat = объект->мир (шейдер тени:
 * u_lightvp * u_obmat * a_co; forward: v_wpos = u_obmat * a_co), мировой AABB
 * меша, мировые позиции ламп.
 * Раньше mesh_emit_face копировал v->co как есть, т.е. в буфере лежали
 * КАМЕРНЫЕ координаты, и вид применялся ВТОРОЙ раз (u_model = obmat * viewmat).
 * Отсюда всё сразу: объекты уезжали и выглядели повёрнутыми, «сцена во
 * вьюпорте сильно повёрнута», а для одиночного куба в начале координат это
 * выглядело просто как смещение. Возвращаем вершины в мир через re->viewinv,
 * а obmat меша делаем единичным — тогда все существующие потребители
 * (тень, forward, AABB, u_model) становятся корректными без правок. */
static float g_gpu_gather_viewinv[4][4];
static int   g_gpu_gather_viewinv_ok = 0;

static void mesh_emit_face(RE_GPUMesh *m, VlakRen *vlr, ObjectRen *obr)
{
	int num_verts = vlr->v4 ? 4 : 3;
	/* ★ M16 (раунд 67): материал WIRE. BI кодирует ребро ВЫРОЖДЕННОЙ VlakRen
	 * (v3 == v2, v4 == NULL) — convertblender.c:3555. Отличить ребро от обычного
	 * треугольника можно именно по v3 == v2 (у треугольника все три вершины
	 * различны). Такое ребро надо рисовать ЛИНИЕЙ: триангуляция давала
	 * треугольник нулевой площади, и проволока была не видна вообще. */
	const int is_wire = (num_verts == 3 && vlr->v3 == vlr->v2);
	const int num_tri_idx = is_wire ? 0 : ((num_verts == 4) ? 6 : 3);   /* квад -> 2 треугольника */
	const int base = m->num_verts;

	MTFace *mtface = NULL;
	MCol   *mcol   = NULL;

	if (is_wire) num_verts = 2;   /* у ребра только две вершины */

	if (obr->actmtface >= 0)
		mtface = RE_vlakren_get_tface(obr, vlr, obr->actmtface, NULL, 0);
	if (obr->actmcol >= 0)
		mcol = RE_vlakren_get_mcol(obr, vlr, obr->actmcol, NULL, 0);

	/* ★ Растём ЭКСПОНЕНЦИАЛЬНО (а не на каждую грань): раньше MEM_reallocN на
	 * каждую грань давал O(N²) копий и «зависал» на больших мешах (mapFULL: 138k
	 * граней). Минимальный стартовый размер 16 — чтобы не дробить по мелочи. */
	{
		int nalloc;
		if (m->num_verts + num_verts > m->verts_alloc) {
			nalloc = m->verts_alloc ? m->verts_alloc : 16;
			while (nalloc < m->num_verts + num_verts) nalloc *= 2;
			m->verts = MEM_reallocN(m->verts, sizeof(RE_GPUVertex) * nalloc);
			m->verts_alloc = nalloc;
		}
		if (num_tri_idx && m->num_indices + num_tri_idx > m->indices_alloc) {
			nalloc = m->indices_alloc ? m->indices_alloc : 16;
			while (nalloc < m->num_indices + num_tri_idx) nalloc *= 2;
			m->indices = MEM_reallocN(m->indices, sizeof(unsigned int) * nalloc);
			m->indices_alloc = nalloc;
		}
		if (is_wire && m->num_line_indices + 2 > m->line_indices_alloc) {
			nalloc = m->line_indices_alloc ? m->line_indices_alloc : 16;
			while (nalloc < m->num_line_indices + 2) nalloc *= 2;
			m->line_indices = MEM_reallocN(m->line_indices, sizeof(unsigned int) * nalloc);
			m->line_indices_alloc = nalloc;
		}
	}

	VertRen *vv[4] = { vlr->v1, vlr->v2, vlr->v3, vlr->v4 };

	for (int i = 0; i < num_verts; i++) {
		RE_GPUVertex *rv = &m->verts[m->num_verts++];
		VertRen *v = vv[i];

		copy_v3_v3(rv->co, v->co);
		/* ★ r59: из пространства камеры в мир (см. комментарий выше). */
		if (g_gpu_gather_viewinv_ok)
			mul_m4_v3(g_gpu_gather_viewinv, rv->co);

		if (vlr->flag & R_SMOOTH)
			copy_v3_v3(rv->n, v->n);
		else
			copy_v3_v3(rv->n, vlr->n);

		/* ★ Вырожденная нормаль вершины (в random2 таких 2 шт.) дала бы в шейдере
		 * normalize(vec3(0)) = NaN -> чёрные/мусорные пиксели. Падаем на нормаль
		 * грани, а если и она нулевая — на (0,0,1). */
		{
			float nl2 = rv->n[0] * rv->n[0] + rv->n[1] * rv->n[1] + rv->n[2] * rv->n[2];
			if (nl2 < 1e-12f) {
				copy_v3_v3(rv->n, vlr->n);
				nl2 = rv->n[0] * rv->n[0] + rv->n[1] * rv->n[1] + rv->n[2] * rv->n[2];
				if (nl2 < 1e-12f) { rv->n[0] = 0.0f; rv->n[1] = 0.0f; rv->n[2] = 1.0f; }
			}
		}

		/* ★ r59: нормали у BI тоже в пространстве камеры — переводим в мир
		 * (только 3x3, перенос к направлениям не применяется). */
		if (g_gpu_gather_viewinv_ok)
			mul_mat3_m4_v3(g_gpu_gather_viewinv, rv->n);

		if (mtface) {
			rv->uv[0] = mtface->uv[i][0];
			rv->uv[1] = mtface->uv[i][1];
		}
		else {
			rv->uv[0] = rv->uv[1] = 0.0f;
		}

		/* ★ M11: ORCO вершины (BI `VertRen->orco`) — для карт с texco==TEXCO_ORCO.
		 * Аллоцируется только если материал просит ORCO (convertblender.c:3180),
		 * поэтому проверяем указатель; без него карта сядет в ORCO=(0,0,0) => uv
		 * (0.5,0.5), что лучше мусора (BI в этом случае тоже берёт lo=0). */
		if (v->orco) {
			rv->orco[0] = v->orco[0];
			rv->orco[1] = v->orco[1];
			rv->orco[2] = v->orco[2];
		}
		else {
			rv->orco[0] = rv->orco[1] = rv->orco[2] = 0.0f;
		}

		if (mcol) {
			rv->rgba[0] = mcol[i].r / 255.0f;
			rv->rgba[1] = mcol[i].g / 255.0f;
			rv->rgba[2] = mcol[i].b / 255.0f;
			rv->rgba[3] = mcol[i].a / 255.0f;
		}
		else {
			/* базовый цвет материала как albedo (нет вершинных цветов) */
			Material *ma = m->ma;
			if (ma) {
				rv->rgba[0] = ma->r;
				rv->rgba[1] = ma->g;
				rv->rgba[2] = ma->b;
				rv->rgba[3] = ma->alpha;
			}
			else {
				rv->rgba[0] = rv->rgba[1] = rv->rgba[2] = rv->rgba[3] = 1.0f;
			}
		}
		/* M5: в alpha albedo едет интенсивность блика (ma->spec, диапазон [0,1]).
		 * ma->alpha тут не используется: прозрачность в однослойном деферреде
		 * не поддержана, а alpha albedo в LIGHT_FS идёт только в выходную альфу. */
		rv->rgba[3] = m->ma ? m->ma->spec : 0.5f;
		/* эмиссия материала (M5): для всех вершин меша одинакова.
		 * Диапазон Emit в BI — [0,2]; в 8-битную alpha нормали кладём emit/2,
		 * шейдер умножает обратно на 2 (иначе 2.0 клампилось бы к 1.0). */
		{
			float e = m->ma ? m->ma->emit : 0.0f;
			if (e < 0.0f) e = 0.0f;
			if (e > 2.0f) e = 2.0f;
			rv->emit = e * 0.5f;
		}
	}

	/* ★ M8: касательный базис для normal map. Считаем по первому треугольнику
	 * грани (vv[0..2]) из объектных координат и UV; знак битангента — от нормали
	 * грани. На GPU шейдер ортонормирует T против нормали вершины (smooth/flat).
	 * Если UV/треугольник вырождены — T=0 (шейдер откатится на геометрию). */
	{
		float T[3] = { 0.0f, 0.0f, 0.0f };
		float sign = 1.0f;
		if (mtface && num_verts >= 3) {
			const float *p0 = vv[0]->co, *p1 = vv[1]->co, *p2 = vv[2]->co;
			float dp1[3] = { p1[0] - p0[0], p1[1] - p0[1], p1[2] - p0[2] };
			float dp2[3] = { p2[0] - p0[0], p2[1] - p0[1], p2[2] - p0[2] };
			float duv1[2] = { mtface->uv[1][0] - mtface->uv[0][0],
			                  mtface->uv[1][1] - mtface->uv[0][1] };
			float duv2[2] = { mtface->uv[2][0] - mtface->uv[0][0],
			                  mtface->uv[2][1] - mtface->uv[0][1] };
			float det = duv1[0] * duv2[1] - duv1[1] * duv2[0];
			if (fabsf(det) > 1e-12f) {
				float r = 1.0f / det;
				T[0] = (dp1[0] * duv2[1] - dp2[0] * duv1[1]) * r;
				T[1] = (dp1[1] * duv2[1] - dp2[1] * duv1[1]) * r;
				T[2] = (dp1[2] * duv2[1] - dp2[2] * duv1[1]) * r;
				float B[3] = { (dp2[0] * duv1[0] - dp1[0] * duv2[0]) * r,
				               (dp2[1] * duv1[0] - dp1[1] * duv2[0]) * r,
				               (dp2[2] * duv1[0] - dp1[2] * duv2[0]) * r };
				float cx = vlr->n[1] * T[2] - vlr->n[2] * T[1];
				float cy = vlr->n[2] * T[0] - vlr->n[0] * T[2];
				float cz = vlr->n[0] * T[1] - vlr->n[1] * T[0];
				if (cx * B[0] + cy * B[1] + cz * B[2] < 0.0f) sign = -1.0f;
			}
		}
		for (int i = 0; i < num_verts; i++) {
			RE_GPUVertex *rv = &m->verts[base + i];
			copy_v3_v3(rv->tangent, T);
			rv->tangent[3] = sign;
		}
	}

	/* триангуляция: треугольник = 3 индекса, квад = 2 треугольника (6 индексов);
	 * ребро материала WIRE — 2 индекса в отдельный буфер ЛИНИЙ */
	if (is_wire) {
		m->line_indices[m->num_line_indices++] = base + 0;
		m->line_indices[m->num_line_indices++] = base + 1;
	}
	else if (num_verts == 3) {
		m->indices[m->num_indices++] = base + 0;
		m->indices[m->num_indices++] = base + 1;
		m->indices[m->num_indices++] = base + 2;
	}
	else {
		m->indices[m->num_indices++] = base + 0;
		m->indices[m->num_indices++] = base + 1;
		m->indices[m->num_indices++] = base + 2;
		m->indices[m->num_indices++] = base + 0;
		m->indices[m->num_indices++] = base + 2;
		m->indices[m->num_indices++] = base + 3;
	}
}

/* Заполнить RE_GPULight из LampRen (все величины — те, из которых BI считает
 * вклад лампы в lamp_get_visibility(), shadeoutput.c). */
static void gpu_fill_light(RE_GPULight *L, LampRen *lar)
{
	const int is_sun = (lar->type == LA_SUN || lar->type == LA_HEMI);

	/* ★ Цвет КАК ЕСТЬ: lar->r/g/b уже включают energy (convertblender.c:3837
	 * «lar->r = lar->energy * la->r»), и BI берёт именно их — shadeoutput.c:1347
	 * «lacol[0]= lar->r». Раньше здесь energy ДЕЛИЛАСЬ, то есть картинка выходила
	 * ярче BI: у disk оба солнца имеют energy 0.24, а светили как 1.0. Теперь
	 * яркость честная и соотношение ламп сохранено (в cave point 0.24 против
	 * spot 1.0). */
	L->color[0] = lar->r;
	L->color[1] = lar->g;
	L->color[2] = lar->b;

	copy_v3_v3(L->dir, lar->vec);
	normalize_v3(L->dir);
	copy_v3_v3(L->pos, lar->co);
	/* мировая позиция лампы (столбец переноса lampmat = ob->obmat) — центр
	 * кубической карты point-лампы. Точечная лампа изотропна, поэтому грани куба
	 * ориентируем по МИРОВЫМ осям и поворот лампы не учитываем. */
	copy_v3_v3(L->wpos, lar->lampmat[3]);
	/* ★ Мировое направление лампы = её локальная -Z, переведённая в мир (строка 2
	 * матрицы при строчном соглашении Blender). lar->vec/lar->co — КАМЕРНЫЕ: BI
	 * переводит их viewmat-ом ещё при сборке базы (convertblender.c:3722), так что
	 * они верны только для того вида, при котором собрана база. Держим мировые
	 * величины и переводим в камеру на отрисовке — иначе свет «едет» за вьюпортом. */
	{
		float z[3];
		z[0] = lar->lampmat[2][0];
		z[1] = lar->lampmat[2][1];
		z[2] = lar->lampmat[2][2];
		if (normalize_v3(z) > 1e-8f) {
			L->wdir[0] = -z[0];
			L->wdir[1] = -z[1];
			L->wdir[2] = -z[2];
		}
		else {
			/* вырожденная матрица — берём то, что дал BI (камерное) */
			L->wdir[0] = L->dir[0];
			L->wdir[1] = L->dir[1];
			L->wdir[2] = L->dir[2];
		}
	}
	L->pz[0] = L->pz[1] = 0.0f;
	L->dist = lar->dist;
	L->sphere = (lar->mode & LA_SPHERE) ? 1 : 0;
	L->spotsi = lar->spotsi;
	L->spotbl = lar->spotbl;
	L->soft = lar->soft;    /* мягкость тени (0..100) — в BI это fac в shadbuf.c */

	/* ★ Тень — как в BI: лампа даёт её, если включён теневой флаг.
	 * SUN — ортопроекция, SPOT — перспектива по конусу (те же типы, для которых
	 * BI строит теневой буфер). POINT в BI затеняется ТОЛЬКО трассировкой лучей
	 * (буфера нет), поэтому у нас для неё кубическая карта (6 граней) — тот же
	 * флаг LA_SHAD_RAY. Семантика rna_use_shadow_get (rna_lamp.c:86): для SUN/HEMI
	 * это ТОЛЬКО LA_SHAD_RAY (у disk солнца стоит LA_SHAD_BUF без RAY — тени НЕТ),
	 * для SPOT — LA_SHAD_BUF | LA_SHAD_RAY, для POINT — LA_SHAD_RAY.
	 * lar->mode = la->mode (convertblender.c:3744); бит LA_SHAD_RAY снимается,
	 * только если в сцене выключена трассировка (convertblender.c:3935). */
	if (is_sun) {
		L->shadows = (lar->mode & LA_SHAD_RAY) ? 1 : 0;
	}
	else if (lar->type == LA_SPOT) {
		L->shadows = (lar->mode & (LA_SHAD_BUF | LA_SHAD_RAY)) ? 1 : 0;
	}
	else if (lar->type == LA_LOCAL) {
		L->shadows = (lar->mode & LA_SHAD_RAY) ? 1 : 0;
	}
	else if (lar->type == LA_AREA) {
		/* ★ AREA в BI затеняется ТОЛЬКО трассировкой лучей (LA_SHAD_RAY), как и
		 * point — теневого буфера у неё нет (rna_use_shadow_get: не-SPOT => RAY).
		 * У нас карты нет совсем; даём кубическую карту (как point). Тень выйдет
		 * ЖЁСТКОЙ (в BI у area она мягкая, из размера), но хоть какая-то. */
		L->shadows = (lar->mode & LA_SHAD_RAY) ? 1 : 0;
	}
	else {
		L->shadows = 0;   /* не должно случиться: остальные типы уже разобраны */
	}

	/* world -> light-view: BI инвертирует lampmat (= ob->obmat, convertblender.c:3725),
	 * и светит лампа вдоль своей локальной -Z. Для SPOT это и есть вид из лампы;
	 * для SUN gpu_compute_shadow_mats дополнительно сдвинет его по центру сцены. */
	invert_m4_m4(L->view, lar->lampmat);
	L->clipsta = lar->clipsta;
	L->clipend = lar->clipend;
	unit_m4(L->proj);
	unit_m4(L->vp);

	/* BI раскладывает затухание на 6 типов; наблюдаемые в сценах —
	 * INVERSE_SQUARE. SLIDERS/INVCOEFFICIENTS/CURVE сводим к INVSQUARE
	 * (мини-EEVEE, «картинка может отличаться от битово-точного BI»). */
	if (lar->falloff_type == LA_FALLOFF_CONSTANT)
		L->falloff = GPU_FALLOFF_CONSTANT;
	else if (lar->falloff_type == LA_FALLOFF_INVLINEAR)
		L->falloff = GPU_FALLOFF_INVLINEAR;
	else
		L->falloff = GPU_FALLOFF_INVSQUARE;

	if (is_sun) L->type = GPU_LIGHT_SUN;
	else if (lar->type == LA_SPOT) L->type = GPU_LIGHT_SPOT;
	else if (lar->type == LA_AREA) L->type = GPU_LIGHT_AREA;
	else L->type = GPU_LIGHT_POINT;
}

/* Сборка света из сцены (CPU, джоба). */
/* ★ Перевести МИРОВЫЕ позицию/направление лампы в КАМЕРНЫЕ текущим видом.
 * Нужно именно на отрисовке: re->viewmat меняется при движении камеры, а pos/dir
 * в RE_GPULight сняты при сборке базы и без этого свет «прилипает» к экрану.
 * mul_m4_v3 — позиция (с переносом), mul_mat3_m4_v3 — направление (только поворот). */
static void gpu_light_view(const RE_GPULight *L, const float viewmat[4][4],
                           float out_pos[3], float out_dir[3])
{
	copy_v3_v3(out_pos, L->wpos);
	mul_m4_v3((float (*)[4])viewmat, out_pos);
	copy_v3_v3(out_dir, L->wdir);
	mul_mat3_m4_v3((float (*)[4])viewmat, out_dir);
	normalize_v3(out_dir);
}

/* Сборка света из сцены (CPU, джоба). */
/* ★ M16 (раунд 67): собрать спрайт-хало (материалы типа HALO) из базы рендера.
 * BI держит их в re->sortedhalos (HaloRen*, отсортированы по z —
 * convertblender.c:1965), а har->co у BI в ПРОСТРАНСТВЕ КАМЕРЫ (RE_inithalo
 * проецирует её сразу через re->winmat, renderdatabase.c:977) — поэтому
 * переводим в мир тем же обратным видом, что и вершины мешей. */
/* ★ Halo SHADE (раунд 76): MA_HALO_SHADE — хало подсвечивается лампами.
 * Повторяет render_lighting_halo (pixelshading.c:63) в мини-EEVEE виде:
 * только рассеянный вклад (без теневых буферов/трассировки), в камерных
 * координатах (har->co и L->pos — камерные). Умножает col на накопленный свет.
 * Свет отсутствует — цвет не трогаем (в BI shadeHaloFloat так же: вызывается
 * только при R.lights.first, иначе цвет остаётся как есть). */
static void gpu_halo_shade(const RE_GPULight *lights, int num_lights,
                           const float hco[3], const float vn[3], float col[3])
{
	float ir = 0.0f, ig = 0.0f, ib = 0.0f;

	if (num_lights <= 0) return;

	for (int i = 0; i < num_lights; i++) {
		const RE_GPULight *L = &lights[i];
		float lv[3], lampdist, inpr, inp;

		if (L->type == GPU_LIGHT_SUN) {
			copy_v3_v3(lv, L->dir);      /* камерное направление солнца (куда светит) */
			lampdist = 1.0f;
		}
		else {
			lv[0] = hco[0] - L->pos[0];
			lv[1] = hco[1] - L->pos[1];
			lv[2] = hco[2] - L->pos[2];
			float ld = len_v3(lv);
			if (ld < 1e-8f) continue;
			lv[0] /= ld; lv[1] /= ld; lv[2] /= ld;
			/* простое затухание (мини-EEVEE: сводим к линейному виду) */
			lampdist = L->dist / (L->dist + ld);
		}

		/* SPOT: конус — как в BI, точки вне конуса не освещают */
		if (L->type == GPU_LIGHT_SPOT) {
			inpr = dot_v3v3(lv, L->dir);
			if (inpr <= L->spotsi) continue;
			lampdist *= inpr;
		}

		/* «диффуз» хало: максимум, когда вектор хало перпендикулярен свету */
		inp = 1.0f - fabsf(dot_v3v3(vn, lv));
		inp *= lampdist;
		if (inp > 0.0f) {
			ir += inp * L->color[0];
			ig += inp * L->color[1];
			ib += inp * L->color[2];
		}
	}

	if (ir < 0.0f) ir = 0.0f;
	if (ig < 0.0f) ig = 0.0f;
	if (ib < 0.0f) ib = 0.0f;

	col[0] *= ir;
	col[1] *= ig;
	col[2] *= ib;
}

static void gpu_gather_halos(RE_GPURenderer *gr, RE_GPUScene *scene)
{
	Render *re = gr->re;
	int i;

	if (!re || !scene) return;
	if (!re->sortedhalos || re->tothalo <= 0) return;

	scene->halos = (RE_GPUHalo *)MEM_callocN(sizeof(RE_GPUHalo) * (size_t)re->tothalo,
	                                         "RE_GPUHalo");
	scene->num_halos = 0;
	int nvec = 0, nshade = 0;

	for (i = 0; i < re->tothalo; i++) {
		HaloRen *har = re->sortedhalos[i];
		RE_GPUHalo *h;
		if (!har || har->hasize <= 0.0f) continue;

		h = &scene->halos[scene->num_halos++];
		copy_v3_v3(h->pos, har->co);
		if (g_gpu_gather_viewinv_ok)
			mul_m4_v3(g_gpu_gather_viewinv, h->pos);
		h->rad = har->hasize;
		h->col[0] = har->r; h->col[1] = har->g; h->col[2] = har->b;
		/* ★ Halo SHADE (раунд 76): MA_HALO_SHADE + вектор (HA_VECT) — подсвечиваем лампами */
		if (har->mat && (har->mat->mode & MA_HALO_SHADE) && (har->type & HA_VECT)) {
			gpu_halo_shade(gr->lights, gr->num_lights, har->co, har->no, h->col);
			nshade++;
		}
		if (har->type & HA_VECT) nvec++;
		h->alfa = har->alfa;
		h->hard = har->hard;
		h->add = (float)har->add;
		h->type = har->type;
		h->starpoints = har->starpoints;
		h->linec = har->linec;
		h->ringc = har->ringc;
		h->flarec = har->flarec;
		h->seed = har->seed;
		h->xalpha = (har->type & HA_XALPHA) ? 1 : 0;
		if (har->mat) {
			/* цвета лучей (linef -> ma->spec*) и колец (ringf -> ma->mir*),
			 * как в конце shadeHaloFloat (pixelshading.c:449-476) */
			h->spec_col[0] = har->mat->specr;
			h->spec_col[1] = har->mat->specg;
			h->spec_col[2] = har->mat->specb;
			h->mir_col[0] = har->mat->mirr;
			h->mir_col[1] = har->mat->mirg;
			h->mir_col[2] = har->mat->mirb;
		}
		h->ma = har->mat;
	}

	printf("[GPU] halos: n=%d (re->tothalo=%d)\n", scene->num_halos, re->tothalo);
	printf("[GPU-HALOSHADE] vect=%d shaded=%d\n", nvec, nshade);
	fflush(stdout);   /* диагностика: при падении буферизованный stdout теряется */
}

static void gpu_gather_lights(RE_GPURenderer *gr)
{
	Render *re = gr->re;

	/* ★ По умолчанию — солнце сверху (world -Z), белое. ВАЖНО: light_view
	 * обязан быть валидным ВСЕГДА. Раньше он заполнялся только при найденной
	 * sun-лампе; в сцене без солнца (обычный point-лампа по умолчанию) он
	 * оставался нулевой матрицей из calloc, gpu_compute_light_proj получал
	 * вырожденный AABB, и shadow map помечал тенью ВСЮ сцену → чёрная картинка. */
	unit_m4(gr->light_view);
	gr->light_color[0] = gr->light_color[1] = gr->light_color[2] = 1.0f;
	{
		float dz[3] = {0.0f, 0.0f, -1.0f};   /* world -Z = «сверху вниз» */
		mul_mat3_m4_v3(re->viewmat, dz);     /* world -> view (только поворот) */
		normalize_v3(dz);
		copy_v3_v3(gr->light_dir, dz);
	}
	gr->has_sun = 0;
	gr->num_lights = 0;

	/* Теневую карту строит ПЕРВОЕ солнце (теней от point/spot нет — это
	 * потребовало бы cubemap или дополнительных карт). Ищем по ВСЕМУ списку,
	 * независимо от того, попало ли солнце в отобранные слоты. */
	{
		for (LampRen *lar = re->lampren.first; lar; lar = lar->next) {
			if (lar->type == LA_SUN || lar->type == LA_HEMI) {
				RE_GPULight tmp;
				gpu_fill_light(&tmp, lar);
				copy_v3_v3(gr->light_dir, tmp.dir);
				copy_v3_v3(gr->light_color, tmp.color);
				/* world->light-view для shadow map (lampmat = object->world) */
				invert_m4_m4(gr->light_view, lar->lampmat);
				gr->has_sun = 1;
				break;
			}
		}
	}

	/* ре-лампы собраны в re->lampren (convertblender.c): vec = направление света
	 * в view-space (куда светит, локальная -Z лампа), co = позиция в view-space,
	 * r/g/b = цвет * energy.
	 *
	 * Раньше учитывалось ТОЛЬКО солнце: сцены с point/spot-лампами (а таких
	 * большинство — tut1, cave, traffic, jolly, alone, motorcycle) освещались
	 * фейковым белым светом «сверху вниз». Теперь собираем все лампы, но шейдер
	 * держит лишь GPU_MAX_SHADOW_LIGHTS — поэтому отбираем самые значимые. */
	{
		int total = 0;
		LampRen *lar;

		for (lar = re->lampren.first; lar; lar = lar->next) total++;

		if (getenv("UBF_GPU_DEBUG")) {
			printf("[GPU-LAMPS] lampren total=%d\n", total);
			for (lar = re->lampren.first; lar; lar = lar->next)
				printf("[GPU-LAMPS]   type=%d pos=(%.2f %.2f %.2f) col=(%.2f %.2f %.2f)\n",
				       (int)lar->type, lar->co[0], lar->co[1], lar->co[2],
				       lar->r, lar->g, lar->b);
		}

		if (total > 0) {
			RE_GPULight *pool = MEM_callocN(sizeof(RE_GPULight) * total, "gpu light pool");
			float *score = MEM_callocN(sizeof(float) * total, "gpu light score");
			int n = 0;

			for (lar = re->lampren.first; lar; lar = lar->next, n++) {
				gpu_fill_light(&pool[n], lar);
				/* Направленные — впереди: иначе сцена с 5+ лампами могла бы выкинуть
				 * солнце вместе с его тенями. Дальше — по суммарной яркости (сумма
				 * квадратов, чтобы не тянуть fabsf). */
				float bright = pool[n].color[0] * pool[n].color[0]
				             + pool[n].color[1] * pool[n].color[1]
				             + pool[n].color[2] * pool[n].color[2];
				score[n] = (pool[n].type == GPU_LIGHT_SUN) ? 1e6f : 0.0f;
				/* ★ Релевантность виду. Раньше порядок задавала ТОЛЬКО яркость, и в
				 * сцене из десятков ОДИНАКОВЫХ лампочек (равная яркость) отбор брал
				 * первые по списку — часто далёкие и вне кадра, а половина кадра
				 * оставалась тёмной. Близкая к камере лампа с большей вероятностью
				 * освещает видимую геометрию, поэтому яркость делим на камерное
				 * расстояние в квадрате. Яркость при этом продолжает различать лампы
				 * разной мощности. Знак view-z не учитываем: в наблюдаемых сценах
				 * лампы висят на высоте камеры (z≈0), и штраф «позади» ошибочно
				 * задвигал ближние лампы за далёкие. */
				if (pool[n].type != GPU_LIGHT_SUN) {
					float lv[3], ld[3], d2;
					gpu_light_view(&pool[n], re->viewmat, lv, ld);
					d2 = lv[0] * lv[0] + lv[1] * lv[1] + lv[2] * lv[2];
					score[n] += bright / (1.0f + d2 * 0.002f);
				}
			}

			/* ★ Раунд 16: берём ВСЕ лампы сцены (раньше — только
			 * GPU_MAX_SHADOW_LIGHTS самых значимых). Порядок всё ещё важен:
			 * первые GPU_MAX_SHADOW_LIGHTS получают теневые карты (их число
			 * ограничено текстурными юнитами), остальные светят без тени. */
			if (gr->lights_cap < n) {
				if (gr->lights) MEM_freeN(gr->lights);
				gr->lights = MEM_callocN(sizeof(RE_GPULight) * n, "gpu lights");
				gr->lights_cap = n;
			}
			gr->num_lights = 0;

			for (int k = 0; k < n; k++) {
				int bi = -1;
				float bs = -1.0f;

				for (int i = 0; i < n; i++) {
					if (score[i] > bs) { bs = score[i]; bi = i; }
				}
				if (bi < 0) break;

				gr->lights[gr->num_lights] = pool[bi];
				gr->num_lights++;
				score[bi] = -1.0f;   /* -1 недостижим (score >= 0) — метка «взято» */
			}

			if (total > GPU_MAX_SHADOW_LIGHTS) {
				printf("[GPU] ламп в сцене %d: светят ВСЕ, тень дают первые %d "
				       "(самые релевантные виду: направленные, затем яркость × "
				       "близость к камере)\n",
				       total, GPU_MAX_SHADOW_LIGHTS);
			}

			MEM_freeN(score);
			MEM_freeN(pool);
		}
	}
}

/* Ortho-проекция (GL, NDC z в [-1,1]). mat[col][row]. */
static void gpu_ortho_m4(float m[4][4], float l, float r, float b, float t,
                         float n, float f)
{
	memset(m, 0, sizeof(float) * 16);
	m[0][0] = 2.0f / (r - l);
	m[1][1] = 2.0f / (t - b);
	m[2][2] = -2.0f / (f - n);
	m[3][0] = -(r + l) / (r - l);
	m[3][1] = -(t + b) / (t - b);
	m[3][2] = -(f + n) / (f - n);
	m[3][3] = 1.0f;
}

/* Перспективная проекция (GL, NDC z в [-1,1], как glFrustum). */
static void gpu_perspective_m4(float m[4][4], float fovy, float aspect,
                               float n, float f)
{
	memset(m, 0, sizeof(float) * 16);
	float t = 1.0f / tanf(fovy * 0.5f);
	m[0][0] = t / aspect;
	m[1][1] = t;
	m[2][2] = (f + n) / (n - f);
	m[3][2] = (2.0f * f * n) / (n - f);
	m[2][3] = -1.0f;
}

/* ★ Ортопроекция SUN по AABB сцены в light-view лампы (та же подгонка, что
 * раньше, только пишем в конкретную лампу L). Мутирует L->view (сдвиг по центру
 * сцены) и заполняет L->proj. */
static void gpu_compute_sun_shadow(RE_GPUScene *scene, RE_GPULight *L)
{
	float min[3], max[3];
	copy_v3_fl(min, FLT_MAX);
	copy_v3_fl(max, -FLT_MAX);

	/* --- 1) мировой AABB сцены --- */
	float wmin[3], wmax[3];
	copy_v3_fl(wmin, FLT_MAX);
	copy_v3_fl(wmax, -FLT_MAX);
	for (RE_GPUMesh *m = scene->meshes; m; m = m->next) {
		for (int i = 0; i < m->num_verts; i++) {
			float w[3];
			copy_v3_v3(w, m->verts[i].co);      /* obj-space */
			mul_m4_v3(m->obmat, w);             /* obj -> world */
			minmax_v3v3_v3(wmin, wmax, w);
		}
	}

	if (wmin[0] > wmax[0]) {   /* пустая сцена */
		unit_m4(L->view);
		gpu_ortho_m4(L->proj, -1, 1, -1, 1, 0.1f, 10.0f);
		return;
	}

	/* --- 2) AABB в light-view --- */
	for (RE_GPUMesh *m = scene->meshes; m; m = m->next) {
		for (int i = 0; i < m->num_verts; i++) {
			float w[3];
			copy_v3_v3(w, m->verts[i].co);      /* obj-space */
			mul_m4_v3(m->obmat, w);             /* obj -> world */
			mul_m4_v3(L->view, w);              /* world -> light-view */
			minmax_v3v3_v3(min, max, w);
		}
	}

	/* x/y: симметричная квадратная область вокруг центра сцены */
	float ex = (max[0] - min[0]) * 0.5f;
	float ey = (max[1] - min[1]) * 0.5f;
	float extent = (ex > ey) ? ex : ey;
	extent *= 1.2f;
	if (extent < 1.0f) extent = 1.0f;

	/* сдвинуть light_view так, чтобы центр сцены был на оси света (x/y).
	 * ВАЖНО: translate_m4 меняет матрицу IN-PLACE, поэтому t сначала = identity. */
	float cx = (min[0] + max[0]) * 0.5f;
	float cy = (min[1] + max[1]) * 0.5f;
	float t[4][4];
	unit_m4(t);
	translate_m4(t, -cx, -cy, 0.0f);
	mul_m4_m4m4(L->view, t, L->view);

	/* z: свет смотрит по -Z, поэтому ближайшая точка = -max[2], дальняя = -min[2] */
	float zn = -max[2] - 1.0f;
	float zf = -min[2] + 2.0f;
	if (zn < 0.01f) zn = 0.01f;              /* near обязан быть > 0 */
	if (zf <= zn) zf = zn + 2.0f;

	gpu_ortho_m4(L->proj, -extent, extent, -extent, extent, zn, zf);
}

/* ★ Кубическая карта POINT-лампы: 6 видов куба + перспектива 90°. near/far берём
 * из clipsta/clipend, но far расширяем до макс. расстояния от света до сцены (по
 * мировым AABB мешей), чтобы карта покрыла всех кастеров. Грани ориентируем по
 * МИРОВЫМ осям (точечная лампа изотропна, поворот не важен) — тогда в шейдере
 * dir = wp - wpos без лишней матрицы. Ориентации граней — стандартные OpenGL
 * (проверено verify_cubemap.py: согласованы с textureCube). */
static void gpu_compute_point_shadow(RE_GPUScene *scene, RE_GPULight *L)
{
	/* far — макс. расстояние от света до сцены (по мировым AABB мешей), чтобы
	 * карта покрыла всех кастеров. */
	float f = 0.0f;
	for (RE_GPUMesh *m = scene->meshes; m; m = m->next) {
		for (int c = 0; c < 8; c++) {
			float d3[3];
			d3[0] = ((c & 1) ? m->bb_max[0] : m->bb_min[0]) - L->wpos[0];
			d3[1] = ((c & 2) ? m->bb_max[1] : m->bb_min[1]) - L->wpos[1];
			d3[2] = ((c & 4) ? m->bb_max[2] : m->bb_min[2]) - L->wpos[2];
			float d = len_v3(d3);
			if (d > f) f = d;
		}
	}
	if (f < 0.2f) f = 0.2f;

	/* ★ near — 0.1% от far (не меньше 0.1). BI трассирует тень point-лампы ИЗ
	 * позиции света, поэтому «правильный» near близок к нулю; но window-глубина
	 * при огромном far/near теряет точность вдали (упакованные 24 бита), а
	 * сравнение идёт в расстояниях — поэтому near поднимаем пропорционально far.
	 * Так геометрия ближе ~0.1% сцены от света в карту не попадает: у лампы,
	 * упёршейся в поверхность, её собственная тень теряется (в BI была бы). */
	float n = f * 0.001f;
	if (n < 0.1f) n = 0.1f;
	if (n > f * 0.5f) n = f * 0.5f;

	gpu_perspective_m4(L->proj, 90.0f * (float)M_PI / 180.0f, 1.0f, n, f);

	/* window-глубина в проходе = gl_FragCoord.z = ((A + B/d)*0.5 + 0.5), где
	 * d — расстояние от света. A и B кладём в L->pz — шейдер посчитает то же
	 * самое из одного d, без передачи матрицы проекции. */
	L->pz[0] = -(f + n) / (n - f);
	L->pz[1] = (2.0f * f * n) / (n - f);

	/* стандартные оси граней куба: right, up, fwd (см. verify_cubemap.py) */
	static const float face_axes[6][3][3] = {
		{{ 0, 0,-1},{ 0,-1, 0},{ 1, 0, 0}},   /* +X */
		{{ 0, 0, 1},{ 0,-1, 0},{-1, 0, 0}},   /* -X */
		{{ 1, 0, 0},{ 0, 0, 1},{ 0, 1, 0}},   /* +Y */
		{{ 1, 0, 0},{ 0, 0,-1},{ 0,-1, 0}},   /* -Y */
		{{ 1, 0, 0},{ 0,-1, 0},{ 0, 0, 1}},   /* +Z */
		{{-1, 0, 0},{ 0,-1, 0},{ 0, 0,-1}},   /* -Z */
	};
	for (int k = 0; k < 6; k++) {
		const float *r = face_axes[k][0], *u = face_axes[k][1], *fv = face_axes[k][2];
		float rot[4][4], tr[4][4];
		unit_m4(rot);
		/* столбцы rot: right, up, -forward (камера смотрит -Z в face-view) */
		rot[0][0] = r[0]; rot[1][0] = r[1]; rot[2][0] = r[2]; rot[3][0] = 0.0f;
		rot[0][1] = u[0]; rot[1][1] = u[1]; rot[2][1] = u[2]; rot[3][1] = 0.0f;
		rot[0][2] = -fv[0]; rot[1][2] = -fv[1]; rot[2][2] = -fv[2]; rot[3][2] = 0.0f;
		rot[0][3] = rot[1][3] = rot[2][3] = 0.0f; rot[3][3] = 1.0f;
		/* грани смотрят из центра куба: сдвиг мировой позиции света */
		unit_m4(tr);
		tr[3][0] = -L->wpos[0]; tr[3][1] = -L->wpos[1]; tr[3][2] = -L->wpos[2];
		mul_m4_m4m4(L->face_view[k], rot, tr);
	}
	/* vp для диагностики = proj * face_view[0] */
	mul_m4_m4m4(L->vp, L->proj, L->face_view[0]);
}

/* ★ Мировой AABB меша (по вершинам, obj->world через obmat). Для отсечения
 * по фрустуму камеры и света. Считается один раз при сборке сцены. */
static void gpu_mesh_world_bbox(RE_GPUMesh *m)
{
	float mn[3], mx[3];
	copy_v3_fl(mn, FLT_MAX);
	copy_v3_fl(mx, -FLT_MAX);
	for (int i = 0; i < m->num_verts; i++) {
		float w[3];
		copy_v3_v3(w, m->verts[i].co);
		mul_m4_v3(m->obmat, w);          /* obj -> world */
		minmax_v3v3_v3(mn, mx, w);
	}
	if (mn[0] > mx[0]) {                  /* пустой меш */
		copy_v3_fl(mn, 0.0f);
		copy_v3_fl(mx, 0.0f);
	}
	copy_v3_v3(m->bb_min, mn);
	copy_v3_v3(m->bb_max, mx);
}

/* ★ Плоскости фрустума (Gribb-Hartmann) из clip-матрицы m (world->clip).
 * m столбцово-главная (m[col][row]), поэтому строка i = {m[0][i],m[1][i],m[2][i],m[3][i]}.
 * Плоскость (a,b,c,d): точка внутри, если a*x+b*y+c*z+d >= 0. */
static void gpu_frustum_planes(const float m[4][4], float pl[6][4])
{
	for (int p = 0; p < 6; p++) {
		int row = p / 2;                  /* 0=x,1=y,2=z */
		float sign = (p % 2 == 0) ? 1.0f : -1.0f;
		for (int j = 0; j < 4; j++)
			pl[p][j] = m[j][3] + sign * m[j][row];
	}
}

/* ★ AABB [mn,mx] целиком вне фрустума, если для какой-то плоскости самая
 * «плюсовая» вершина AABB лежит на отрицательной стороне. */
static int gpu_aabb_outside(const float pl[6][4], const float mn[3], const float mx[3])
{
	for (int p = 0; p < 6; p++) {
		float px = (pl[p][0] >= 0.0f) ? mx[0] : mn[0];
		float py = (pl[p][1] >= 0.0f) ? mx[1] : mn[1];
		float pz = (pl[p][2] >= 0.0f) ? mx[2] : mn[2];
		if (pl[p][0] * px + pl[p][1] * py + pl[p][2] * pz + pl[p][3] < 0.0f)
			return 1;
	}
	return 0;
}

/* ★ Теневые матрицы КАЖДОЙ лампы. SUN — ортопроекция по AABB сцены; SPOT —
 * перспектива по углу конуса (2*acos(spotsi) с запасом, чтобы мягкий край конуса
 * не резало по кромке карты); POINT и AREA — кубическая карта (6 видов в
 * face_view[], у AREA тень жёсткая — приближение, в BI она только трассируется).
 * Заполняет L->vp (у point/area — proj*face_view[0], для диагностики), а
 * gr->light_view/light_proj зеркалит первую солнцеву лампу. */
static void gpu_compute_shadow_mats(RE_GPURenderer *gr, RE_GPUScene *scene)
{
	int nshadow = 0;
	int mirrored = 0;
	for (int i = 0; i < gr->num_lights; i++) {
		RE_GPULight *L = &gr->lights[i];
		/* ★ Раунд 16: теневых карт ровно GPU_MAX_SHADOW_LIGHTS — их число задано
		 * текстурными юнитами (пара сэмплеров на лампу). Лампы дальше по списку
		 * СВЕТЯТ, но без тени: гасим флаг, и шейдер для них получит fl.w=0. */
		if (i >= GPU_MAX_SHADOW_LIGHTS) L->shadows = 0;
		if (!L->shadows) continue;
		if (L->type == GPU_LIGHT_SUN) {
			gpu_compute_sun_shadow(scene, L);
			/* ★ vp обязан считаться ЗДЕСЬ: gpu_compute_sun_shadow мутирует L->view
			 * (сдвиг по центру сцены), так что proj*view нужен после неё. */
			mul_m4_m4m4(L->vp, L->proj, L->view);
			if (!mirrored) {
				copy_m4_m4(gr->light_view, L->view);
				copy_m4_m4(gr->light_proj, L->proj);
				mirrored = 1;
			}
		}
		else if (L->type == GPU_LIGHT_SPOT) {
			/* полный угол конуса = 2*acos(spotsi); запас 12% на мягкий край */
			float fovy = 2.0f * acosf(L->spotsi) * 1.12f;
			if (fovy < 0.01f) fovy = 0.01f;
			float n = (L->clipsta > 0.01f) ? L->clipsta : 0.01f;
			float f = (L->clipend > n) ? L->clipend : (n + 10.0f);
			/* ★★ Раунд 83 — ГЛАВНАЯ причина «теней не как в scanline»: BI НЕ
			 * ограничивает теневую карту спота полем clipend. Она подгоняет far
			 * под фактическую геометрию в конусе:
			 *     shadbuf.c:704   lar->shb->clipend = maxz + delta*0.1f;
			 *     shadbuf.c:777   perspective_m4(..., shb->d, shb->clipend);
			 * (clipend у BI идёт только в масштаб bias, shadbuf.c:707).
			 * Мы же отдавали clipend в проекцию напрямую. В mapFull у ламп
			 * clip_end=30, distance=34.5, INVERSE_SQUARE — а свет на 50 юнитах
			 * ещё 37% от света на 30 (visifac = dist/(dist+d^2)). Всё, что дальше
			 * far, шейдер объявлял ОСВЕЩЁННЫМ (ранний выход `sc.z > 1.0`), и в
			 * «чёрной» зоне scanline (39.9% кадра) GPU светил 2.83/255 вместо
			 * 0.02. Проверено замером: тот же кадр с clip_end=500 даёт 0.178 —
			 * утечка падает в 16 раз, mean|diff| 9.22 -> 8.03.
			 * maxz меряем как у point-лампы (gpu_compute_point_shadow): максимум
			 * -view_z по углам мировых AABB мешей. Стоимость — 8*N_mesh на лампу
			 * на пересборку сцены. */
			{
				float fmax = f;
				for (RE_GPUMesh *m = scene->meshes; m; m = m->next) {
					for (int c = 0; c < 8; c++) {
						float p[3];
						p[0] = (c & 1) ? m->bb_max[0] : m->bb_min[0];
						p[1] = (c & 2) ? m->bb_max[1] : m->bb_min[1];
						p[2] = (c & 4) ? m->bb_max[2] : m->bb_min[2];
						mul_m4_v3(L->view, p);          /* -> view-пространство лампы */
						if (-p[2] > fmax) fmax = -p[2];
					}
				}
				/* запас 2% + 1 юнит: кастер на самой границе не должен обрезаться */
				if (fmax > f) f = fmax * 1.02f + 1.0f;
				if (f > 5000.0f) f = 5000.0f;       /* страховка для упакованной глубины */
			}
			gpu_perspective_m4(L->proj, fovy, 1.0f, n, f);
			mul_m4_m4m4(L->vp, L->proj, L->view);
			/* ★★ Раунд 83: (A,B) пересчёта window-глубины в РАССТОЯНИЕ вдоль оси
			 * света. Это ровно те же коэффициенты, что gpu_perspective_m4 кладёт
			 * в m[2][2]/m[3][2]: z_ndc = (f+n)/(f-n) - 2fn/((f-n)d) = A + B/d.
			 * Шейдер по ним восстанавливает дистанцию окклюдера и сравнивает её с
			 * дистанцией фрагмента — вместо сравнения window-глубин, где bias
			 * приходилось задавать в window-единицах и он раздувался как d^2.
			 * Ортопроекция солнца эти поля НЕ заполняет (остаётся 0,0) — по этому
			 * признаку gpu_occludes выбирает старую ветку. */
			L->pz[0] = (f + n) / (f - n);
			L->pz[1] = -2.0f * f * n / (f - n);
		}
		else if (L->type == GPU_LIGHT_POINT || L->type == GPU_LIGHT_AREA) {
			/* кубическая карта: face_view[6] + proj; vp заполняется внутри.
			 * AREA — та же схема (в BI у неё только трассировка, см. gpu_fill_light). */
			gpu_compute_point_shadow(scene, L);
		}
		else {
			L->shadows = 0;   /* не должно случиться */
		}
		nshadow++;
	}
	gr->num_shadow_lights = nshadow;
}

/* Сборка мира сцены (CPU, джоба). re->wrld — копия scene->world, которую делает
 * convertblender.c:4171; для сцены без World она обнулена, а exp/range выставлены
 * в 0/1 (convertblender.c:4188), то есть фон будет чёрным — как и в BI. */
static void gpu_gather_world(RE_GPURenderer *gr)
{
	Render *re = gr->re;
	const World *w = &re->wrld;

	gr->world_hor[0] = w->horr; gr->world_hor[1] = w->horg; gr->world_hor[2] = w->horb;
	gr->world_zen[0] = w->zenr; gr->world_zen[1] = w->zeng; gr->world_zen[2] = w->zenb;
	gr->world_amb[0] = w->ambr; gr->world_amb[1] = w->ambg; gr->world_amb[2] = w->ambb;
	gr->world_blend = (w->skytype & WO_SKYBLEND) ? 1 : 0;
	/* ★★ M26: horizon/zenith — ЦВЕТ ФОНА, а не свет. BI делает их ambient-светом
	 * только при включённом Environment Lighting, иначе ambient = ma->amb *
	 * wrld.ambient_color (shadeoutput.c:2000). */
	gr->world_env = (w->mode & WO_ENV_LIGHT) ? 1 : 0;
	gr->world_env_energy = w->ao_env_energy;

	/* linfac/logfac посчитаны в convertblender.c:4198-4199 из wrld.exp и wrld.range;
	 * BI применяет поправку только когда exp!=0 или range!=1 (shadeoutput.c:1960). */
	gr->linfac = w->linfac;
	gr->logfac = w->logfac;
	gr->expo_on = (w->exp != 0.0f || w->range != 1.0f) ? 1 : 0;
}

void RE_gpu_renderer_scene_build(RE_GPURenderer *gr)
{
	Render *re = gr->re;
	RE_GPUScene *scene = MEM_callocN(sizeof(*scene), "RE_GPUScene");
	/* ★ gr->scene присваиваем ТОЛЬКО в самом конце: главный поток читает
	 * gr->scene в RE_gpu_renderer_draw и не должен увидеть недостроенную сцену. */

	gpu_gather_lights(gr);
	gpu_gather_world(gr);

	/* ★ r59: запоминаем обратный вид — им переводим вершины VertRen.co из
	 * пространства камеры (как их хранит BI) в мир (как ждёт растеризатор). */
	copy_m4_m4(g_gpu_gather_viewinv, re->viewinv);
	g_gpu_gather_viewinv_ok = 1;

	/* ★ M16: халы переводим в мир тем же обратным видом, поэтому собираем их
	 * ПОСЛЕ того, как он запомнен. ВАЖНО: передаём ЛОКАЛЬНУЮ scene — gr->scene
	 * присваивается только в самом конце сборки (см. комментарий выше). */
	gpu_gather_halos(gr, scene);

	for (ObjectInstanceRen *obi = re->instancetable.first;
	     obi; obi = obi->next)
	{
		ObjectRen *obr = obi->obr;
		RE_GPUMesh *cur = NULL;
		Material *cur_ma = NULL;

		for (int v = 0; v < obr->totvlak; v++) {
			VlakRen *vlr = RE_findOrAddVlak(obr, v);

			if (!vlr) continue;
			if (vlr->flag & R_HIDDEN) continue;

			if (vlr->mat != cur_ma || cur == NULL) {
				cur = gpu_mesh_new(scene, vlr->mat, obi->mat, obi->nmat, obi->obmat);
				cur_ma = vlr->mat;
				/* ★ r59: вершины уже в МИРОВЫХ координатах (см.
				 * g_gpu_gather_viewinv), поэтому obmat меша обязан быть
				 * единичным — его применяют тени (u_lightvp*u_obmat*a_co),
				 * forward (v_wpos = u_obmat*a_co), мировой AABB и u_model.
				 * Иначе матрица объекта применилась бы второй раз. */
				unit_m4(cur->obmat);
			}
			mesh_emit_face(cur, vlr, obr);
		}
	}

	/* ★ Мировые AABB мешей — для отсечения по фрустуму камеры (G-буфер) и света
	 * (теневые карты). Считается один раз здесь. */
	for (RE_GPUMesh *m = scene->meshes; m; m = m->next)
		gpu_mesh_world_bbox(m);

	gpu_compute_shadow_mats(gr, scene);

	/* ★ публикуем готовую сцену одним присваиванием. Предыдущую НЕ освобождаем —
	 * её ещё может читать главный поток; отдаём её в scene_old, и главный поток
	 * освободит её (GL-ресурсы + память) в начале следующего кадра. */
	gr->scene_old = gr->scene;
	gr->scene = scene;
}

void RE_gpu_renderer_scene_free(RE_GPUScene *scene)
{
	if (!scene) return;

	RE_GPUMesh *m = scene->meshes;
	while (m) {
		RE_GPUMesh *n = m->next;
		if (m->verts)   MEM_freeN(m->verts);
		if (m->indices) MEM_freeN(m->indices);
		if (m->line_indices) MEM_freeN(m->line_indices);   /* M16: рёбра WIRE */
		MEM_freeN(m);
		m = n;
	}
	if (scene->halos) MEM_freeN(scene->halos);   /* M16: спрайты HALO */
	MEM_freeN(scene);
}

/* ------------------------------------------------------------------------- */
/* Жизненный цикл                                                             */
/* ------------------------------------------------------------------------- */

RE_GPURenderer *RE_gpu_renderer_create(Render *re)
{
	RE_GPURenderer *gr = MEM_callocN(sizeof(*gr), "RE_GPURenderer");
	gr->re = re;
	return gr;
}

/* ★ Сборка CPU-сцены из VlakRen. Вызывается из рендер-джобы (БЕЗ GL).
 * Пересоздаёт сцену; GL-ресурсы не трогает — они на главном потоке. */
void RE_gpu_renderer_build_render(Render *re)
{
	if (!re) return;

	if (re->gpu_renderer == NULL)
		re->gpu_renderer = RE_gpu_renderer_create(re);

	RE_GPURenderer *gr = re->gpu_renderer;

	/* ★ КРИТИЧНО (был краш при движении камеры). Эта функция вызывается из
	 * рендер-джобы на КАЖДЫЙ кадр вьюпорта — в том числе когда меняется только
	 * положение камеры, а геометрия та же. Раньше здесь сцена освобождалась и
	 * собиралась заново, а главный поток в это время рисовал её же в
	 * RE_gpu_renderer_draw → use-after-free и падение. Геометрия от камеры не
	 * зависит, поэтому сцена собирается один раз и пересобирается ТОЛЬКО когда
	 * главный поток пометит её устаревшей (scene_dirty) — то есть когда реально
	 * пересобралась база рендера. Свежие матрицы obj->view и нормалей считаются
	 * на главном потоке в draw из актуального re->viewmat (его обновляет
	 * RE_DataBase_IncrementalView при повороте камеры). */
	if (gr->scene && !gr->scene_dirty) return;

	/* ждём, пока главный поток освободит предыдущую сцену — иначе утечка и
	 * (хуже) потеря указателя на неё */
	if (gr->scene_old) return;

	gr->scene_dirty = 0;
	const int was_rebuild = (gr->scene != NULL);
	RE_gpu_renderer_scene_build(gr);

	printf("[GPU] rasterizer_mode=GPU: scene %s (meshes=%d) has_sun=%d "
	       "light_dir=(%.2f %.2f %.2f) light_color=(%.2f %.2f %.2f)\n",
	       was_rebuild ? "REBUILT" : "built",
	       gr->scene ? gr->scene->num_meshes : 0, gr->has_sun,
	       gr->light_dir[0], gr->light_dir[1], gr->light_dir[2],
	       gr->light_color[0], gr->light_color[1], gr->light_color[2]);

	/* ★ Собранные лампы (проверяется headless). Раньше печатать было нечего:
	 * учитывалось одно солнце. type: 0=sun 1=point 2=spot 3=area;
	 * falloff: 0=const 1=invlinear 2=invsquare. */
	{
		static const char *tn[4] = { "sun", "point", "spot", "area" };
		printf("[GPU] lights: n=%d\n", gr->num_lights);
		for (int i = 0; i < gr->num_lights && i < GPU_MAX_SHADOW_LIGHTS; i++) {
			const RE_GPULight *L = &gr->lights[i];
			printf("[GPU-LIGHT] [%d] %-5s color=(%.3f %.3f %.3f) pos=(%.2f %.2f %.2f) "
			       "dir=(%.2f %.2f %.2f) dist=%.1f falloff=%d sphere=%d spotsi=%.3f soft=%.2f\n",
			       i, tn[L->type < 0 || L->type > 3 ? 0 : L->type],
			       L->color[0], L->color[1], L->color[2],
			       L->pos[0], L->pos[1], L->pos[2],
			       L->dir[0], L->dir[1], L->dir[2],
			       L->dist, L->falloff, L->sphere, L->spotsi, L->soft);
		}
	}

	/* ★ Мир сцены: фон и ambient (см. раунд 25). */
	printf("[GPU] world: hor=(%.3f %.3f %.3f) zen=(%.3f %.3f %.3f) amb=(%.3f %.3f %.3f) "
	       "skyblend=%d expo_on=%d\n",
	       gr->world_hor[0], gr->world_hor[1], gr->world_hor[2],
	       gr->world_zen[0], gr->world_zen[1], gr->world_zen[2],
	       gr->world_amb[0], gr->world_amb[1], gr->world_amb[2],
	       gr->world_blend, gr->expo_on);
}

/* --- extern-точка входа (RE_pipeline.h) для view_draw во вьюпорте --- */

bool RE_gpu_renderer_is_active(struct Render *re)
{
	return (re && re->gpu_renderer);
}

/* ★ Пометить сцену устаревшей. Вызывается главным потоком сразу после
 * RE_Database_FromScene: изменилась геометрия/объекты/материалы, значит VlakRen
 * пересозданы и старые CPU-массивы больше не соответствуют сцене. */
void RE_gpu_renderer_invalidate(struct Render *re)
{
	if (re && re->gpu_renderer)
		re->gpu_renderer->scene_dirty = 1;
}

void RE_gpu_renderer_draw_render(struct Render *re, int width, int height)
{
	if (!re || !re->gpu_renderer) return;
	RE_GPURenderer *gr = re->gpu_renderer;

	/* ★ Освободить сцену, которую джоба уже заменила новой. Делаем это ЗДЕСЬ:
	 * главный поток, GL-контекст активен, и кадр ещё не начат — сцену никто
	 * не читает. VBO/IBO принадлежат мешам именно старой сцены. */
	if (gr->scene_old) {
		RE_gpu_renderer_gl_free_scene(gr->scene_old);
		RE_gpu_renderer_scene_free(gr->scene_old);
		gr->scene_old = NULL;
	}

	/* ★ gl_init в конце своей работы привязывает FBO 0 (сбрасывает состояние).
	 * Если вьюпорт рисует не в дефолтный FBO, его привязка потерялась бы, и
	 * draw() сохранил бы saved_fb=0 -> результат ушёл бы не туда. Сохраняем
	 * привязку ДО gl_init и возвращаем ПОСЛЕ. */
	GLint fb_before_init = 0;
	glGetIntegerv(GL_FRAMEBUFFER_BINDING_EXT, &fb_before_init);

	RE_gpu_renderer_gl_init(gr, width, height);

	glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, (GLuint)fb_before_init);
	RE_gpu_renderer_draw(gr);
}

/* ★ Кадр растеризатора — в ДИСПЛЕЙНОМ пространстве: вьюпорт рисует его в окно
 * как есть, без color management. re->result же хранит ЛИНЕЙНЫЕ значения, и при
 * показе/записи Blender применяет к ним view transform (по умолчанию sRGB).
 * Без обратного преобразования кадр F12 выходит пересвеченным («выцветшим»)
 * относительно вьюпорта. */
static float gpu_srgb_to_linear(float c)
{
	if (c <= 0.04045f) return c / 12.92f;
	return (float)pow((c + 0.055f) / 1.055f, 2.4f);
}

/* ★ F12 (полный рендер): отрисовать GPU-кадр в НЕЗАВИСИМЫЙ offscreen-буфер и
 * заполнить им re->result (проход Combined). Вызывается ТОЛЬКО с ГЛАВНОГО
 * потока, где есть GL-контекст: оператор F12 считает кадр синхронно, потому что
 * рендер-джоба идёт в фоновом потоке без GL, а в 2.79 нет и создания фонового
 * GL-контекста (GHOST_CreateSystemBackgroundContext отсутствует).
 * Свой FBO нужен, чтобы кадр не зависел от размера окна вьюпорта и не портил
 * его содержимое. Возвращает 1, если кадр отрисован и результат заполнен. */
int RE_gpu_renderer_render_to_result(Render *re, int width, int height)
{
	GLuint fbo = 0, tex = 0;
	GLint saved_fb = 0;
	GLint saved_vp[4] = { 0, 0, 0, 0 };
	unsigned char *pix;
	int ok = 0, x, y;

	if (!re || !re->gpu_renderer || !re->result) return 0;
	if (width <= 0 || height <= 0) return 0;

	pix = (unsigned char *)MEM_mallocN((size_t)width * (size_t)height * 4, "GPU f12 pixels");
	if (!pix) return 0;

	glGetIntegerv(GL_FRAMEBUFFER_BINDING_EXT, &saved_fb);
	glGetIntegerv(GL_VIEWPORT, saved_vp);

	glGenTextures(1, &tex);
	glBindTexture(GL_TEXTURE_2D, tex);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, width, height, 0, GL_RGBA, GL_UNSIGNED_BYTE, NULL);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);

	glGenFramebuffersEXT(1, &fbo);
	glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, fbo);
	glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT, GL_TEXTURE_2D, tex, 0);

	if (glCheckFramebufferStatusEXT(GL_FRAMEBUFFER_EXT) == GL_FRAMEBUFFER_COMPLETE_EXT) {
		RenderResult *rr = re->result;
		RenderLayer *rl;
		float *lin;

		glViewport(0, 0, width, height);
		/* draw_render сохраняет/возвращает ТЕКУЩУЮ привязку FBO, поэтому
		 * нарисованное уйдёт именно в наш offscreen-буфер. */
		RE_gpu_renderer_draw_render(re, width, height);
		glReadPixels(0, 0, width, height, GL_RGBA, GL_UNSIGNED_BYTE, pix);

		/* Декодируем sRGB→линейное. Переворачивать НЕ надо: и glReadPixels, и
		 * re->result хранят кадр СНИЗУ вверх (строка 0 — нижняя), запись файла
		 * сама разворачивает его для PNG. Из-за лишнего переворота кадр F12
		 * выходил вверх ногами (небо внизу, земля сверху). */
		lin = (float *)MEM_mallocN((size_t)width * (size_t)height * 4 * sizeof(float),
		                           "GPU f12 linear");
		if (lin) {
			for (y = 0; y < height; y++) {
				const unsigned char *src = pix + (size_t)y * (size_t)width * 4;
				float *dst = lin + (size_t)y * (size_t)width * 4;
				for (x = 0; x < width; x++) {
					dst[x * 4 + 0] = gpu_srgb_to_linear(src[x * 4 + 0] / 255.0f);
					dst[x * 4 + 1] = gpu_srgb_to_linear(src[x * 4 + 1] / 255.0f);
					dst[x * 4 + 2] = gpu_srgb_to_linear(src[x * 4 + 2] / 255.0f);
					dst[x * 4 + 3] = src[x * 4 + 3] / 255.0f;
				}
			}

			for (rl = rr->layers.first; rl; rl = rl->next) {
				RenderPass *rp;
				for (rp = rl->passes.first; rp; rp = rp->next) {
					int cw, ch;
					if (!rp->rect || strcmp(rp->name, RE_PASSNAME_COMBINED) != 0) continue;
					cw = rp->rectx < width ? rp->rectx : width;
					ch = rp->recty < height ? rp->recty : height;
					for (y = 0; y < ch; y++)
						memcpy(rp->rect + (size_t)y * (size_t)rp->rectx * 4,
						       lin + (size_t)y * (size_t)width * 4,
						       (size_t)cw * 4 * sizeof(float));
					ok = 1;
				}
			}
			if (rr->rectf) {
				int cw = rr->rectx < width ? rr->rectx : width;
				int ch = rr->recty < height ? rr->recty : height;
				for (y = 0; y < ch; y++)
					memcpy(rr->rectf + (size_t)y * (size_t)rr->rectx * 4,
					       lin + (size_t)y * (size_t)width * 4,
					       (size_t)cw * 4 * sizeof(float));
				ok = 1;
			}
			MEM_freeN(lin);
		}
	}
	else {
		printf("[GPU-F12] offscreen FBO неполный — кадр не отрисован\n");
	}

	glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, (GLuint)saved_fb);
	glViewport(saved_vp[0], saved_vp[1], saved_vp[2], saved_vp[3]);
	if (fbo) glDeleteFramebuffersEXT(1, &fbo);
	if (tex) glDeleteTextures(1, &tex);
	MEM_freeN(pix);

	if (ok)
		printf("[GPU-F12] кадр %dx%d отрисован растеризатором и записан в результат\n",
		       width, height);
	return ok;
}

void RE_gpu_renderer_free_render(struct Render *re)
{
	if (!re || !re->gpu_renderer) return;
	RE_gpu_renderer_free(re->gpu_renderer);
	re->gpu_renderer = NULL;
}

void RE_gpu_renderer_free(RE_GPURenderer *gr)
{
	if (!gr) return;
	/* ★ Сначала GL-ресурсы: gl_free перебирает меши сцены, чтобы удалить
	 * VBO/IBO, поэтому сцены можно освобождать только ПОСЛЕ. Раньше было
	 * наоборот — use-after-free на teardown. */
	RE_gpu_renderer_gl_free(gr);
	/* scene_old могла не успеть освободиться в draw (превью закрыли сразу после
	 * правки геометрии) — удаляем её GL-буферы тоже, иначе утечка. */
	if (gr->scene_old) {
		RE_gpu_renderer_gl_free_scene(gr->scene_old);
		RE_gpu_renderer_scene_free(gr->scene_old);
	}
	if (gr->scene) RE_gpu_renderer_scene_free(gr->scene);
	/* ★ раунд 16: динамический массив ламп (число ламп не ограничено) */
	if (gr->lights) MEM_freeN(gr->lights);
	MEM_freeN(gr);
}

/* ------------------------------------------------------------------------- */
/* Диагностика (без GL)                                                       */
/* ------------------------------------------------------------------------- */

void RE_gpu_renderer_dump(const RE_GPURenderer *gr)
{
	if (!gr || !gr->scene) {
		printf("[GPU] no scene\n");
		return;
	}

	int num_meshes = 0, num_verts = 0, num_indices = 0;
	unsigned int hash = 0;

	for (RE_GPUMesh *m = gr->scene->meshes; m; m = m->next) {
		num_meshes++;
		num_verts   += m->num_verts;
		num_indices += m->num_indices;

		for (int i = 0; i < m->num_indices; i++)
			hash = hash * 33u + m->indices[i];

		for (int i = 0; i < m->num_verts; i++) {
			const float *c = m->verts[i].co;
			unsigned int h = (unsigned int)(c[0] * 1000.0f) * 31u +
			                 (unsigned int)(c[1] * 1000.0f) * 17u +
			                 (unsigned int)(c[2] * 1000.0f);
			hash = hash * 33u + h;
		}
	}

	printf("[GPU] meshes=%d verts=%d indices=%d hash=%u\n",
	       num_meshes, num_verts, num_indices, hash);
	printf("[GPU] light_dir=(%.3f %.3f %.3f) light_color=(%.3f %.3f %.3f)\n",
	       gr->light_dir[0], gr->light_dir[1], gr->light_dir[2],
	       gr->light_color[0], gr->light_color[1], gr->light_color[2]);
	printf("[GPU] light_proj row0=(%.2f %.2f %.2f %.2f) row1=(%.2f %.2f %.2f %.2f)\n",
	       gr->light_proj[0][0], gr->light_proj[0][1], gr->light_proj[0][2], gr->light_proj[0][3],
	       gr->light_proj[1][0], gr->light_proj[1][1], gr->light_proj[1][2], gr->light_proj[1][3]);

	/* ★ Собранные лампы (headless-проверка; раньше печатать было нечего —
	 * учитывалось одно солнце). type: 0=sun 1=point 2=spot 3=area;
	 * falloff: 0=const 1=invlinear 2=invsquare. Цвет — уже с energy. */
	{
		static const char *tn[4] = { "sun", "point", "spot", "area" };
		printf("[GPU] lights: n=%d (shadow=%d)\n", gr->num_lights, gr->num_shadow_lights);
		{
			int n2d = 0, ncube = 0;
			for (int i = 0; i < gr->num_lights && i < GPU_MAX_SHADOW_LIGHTS; i++) {
				const RE_GPULight *L = &gr->lights[i];
				if (!L->shadows) continue;
				if (L->type == GPU_LIGHT_POINT || L->type == GPU_LIGHT_AREA) ncube++; else n2d++;
			}
			printf("[GPU] shadow maps: 2d=%d cube=%d\n", n2d, ncube);
		}
		for (int i = 0; i < gr->num_lights && i < GPU_MAX_SHADOW_LIGHTS; i++) {
			const RE_GPULight *L = &gr->lights[i];
			printf("[GPU-LIGHT] [%d] %-5s color=(%.3f %.3f %.3f) pos=(%.2f %.2f %.2f) "
			       "dir=(%.2f %.2f %.2f) dist=%.1f falloff=%d sphere=%d spotsi=%.3f "
			       "shadows=%d soft=%.2f clip=(%.2f %.2f)\n",
			       i, tn[(L->type < 0 || L->type > 3) ? 0 : L->type],
			       L->color[0], L->color[1], L->color[2],
			       L->pos[0], L->pos[1], L->pos[2],
			       L->dir[0], L->dir[1], L->dir[2],
			       L->dist, L->falloff, L->sphere, L->spotsi,
			       L->shadows, L->soft, L->clipsta, L->clipend);
			/* ★ POINT/AREA-тень (кубическая карта): центр куба и (A,B) пересчёта
			 * window-глубины. Проверка headless: wpos — мировая позиция, pz конечны
			 * и B<0 (f>n). */
			if ((L->type == GPU_LIGHT_POINT || L->type == GPU_LIGHT_AREA) && L->shadows) {
				printf("[GPU-POINTSHADOW] [%d] %s wpos=(%.3f %.3f %.3f) pz=(%.6f %.6f) "
				       "f0=(%.2f %.2f %.2f %.2f)\n", i,
				       (L->type == GPU_LIGHT_AREA) ? "area" : "point",
				       L->wpos[0], L->wpos[1], L->wpos[2], L->pz[0], L->pz[1],
				       L->face_view[0][0][0], L->face_view[0][0][1],
				       L->face_view[0][0][2], L->face_view[0][3][0]);
			}
		}
		/* ★ Теневые матрицы. Печатаем ЧИСТУЮ проекцию L->proj (не vp): по ней
		 * видно тип — у солнца орто (m23=0, m33=1), у прожектора перспектива
		 * (m23=-1, m32=(2fn)/(n-f)). Плюс vp row3 — проверка конечности
		 * результирующей world->light-clip матрицы. */
		for (int i = 0; i < gr->num_lights && i < GPU_MAX_SHADOW_LIGHTS; i++) {
			const RE_GPULight *L = &gr->lights[i];
			if (!L->shadows) continue;
			printf("[GPU-SHADOW] [%d] %s proj row0=(%.4f %.4f %.4f %.4f) "
			       "row2=(%.4f %.4f %.4f %.4f) row3=(%.4f %.4f %.4f %.4f) "
			       "m23=%.4f m33=%.4f | vp row3=(%.4f %.4f %.4f %.4f)\n",
			       i, tn[(L->type < 0 || L->type > 3) ? 0 : L->type],
			       L->proj[0][0], L->proj[0][1], L->proj[0][2], L->proj[0][3],
			       L->proj[2][0], L->proj[2][1], L->proj[2][2], L->proj[2][3],
			       L->proj[3][0], L->proj[3][1], L->proj[3][2], L->proj[3][3],
			       L->proj[2][3], L->proj[3][3],
			       L->vp[3][0], L->vp[3][1], L->vp[3][2], L->vp[3][3]);
		}
	}

	/* ★ Мир сцены (проверяется headless против world_probe.py). */
	printf("[GPU-WORLD] hor=(%.3f %.3f %.3f) zen=(%.3f %.3f %.3f) amb=(%.3f %.3f %.3f) "
	       "blend=%d linfac=%.4f logfac=%.6f expo_on=%d\n",
	       gr->world_hor[0], gr->world_hor[1], gr->world_hor[2],
	       gr->world_zen[0], gr->world_zen[1], gr->world_zen[2],
	       gr->world_amb[0], gr->world_amb[1], gr->world_amb[2],
	       gr->world_blend, gr->linfac, gr->logfac, gr->expo_on);

	/* ★ Проверка инвариантов CPU-экстракции (вход G-буфера). */
	int bad_norm = 0, bad_alpha = 0, bad_index = 0, bad_tri = 0, bad_emit = 0;
	float emit_min = 1e30f, emit_max = -1e30f;
	float spec_min = 1e30f, spec_max = -1e30f;
	for (RE_GPUMesh *m = gr->scene->meshes; m; m = m->next) {
		for (int i = 0; i < m->num_verts; i++) {
			const RE_GPUVertex *v = &m->verts[i];
			float nl = v->n[0] * v->n[0] + v->n[1] * v->n[1] + v->n[2] * v->n[2];
			if (nl < 0.9f || nl > 1.1f) bad_norm++;
			if (v->rgba[0] < 0.0f || v->rgba[0] > 1.0f ||
			    v->rgba[1] < 0.0f || v->rgba[1] > 1.0f ||
			    v->rgba[2] < 0.0f || v->rgba[2] > 1.0f ||
			    v->rgba[3] < 0.0f || v->rgba[3] > 1.0f) bad_alpha++;
			if (v->emit < 0.0f || v->emit > 1.0f) bad_emit++;
			if (v->emit < emit_min) emit_min = v->emit;
			if (v->emit > emit_max) emit_max = v->emit;
			if (v->rgba[3] < spec_min) spec_min = v->rgba[3];
			if (v->rgba[3] > spec_max) spec_max = v->rgba[3];
		}
		for (int i = 0; i < m->num_indices; i++) {
			if (m->indices[i] >= (unsigned int)m->num_verts) bad_index++;
		}
		if (m->num_indices % 3 != 0) bad_tri++;
		/* отдельно вывести первый треугольник каждого меша (видно содержимое) */
		if (m->num_indices >= 3) {
			const RE_GPUVertex *v0 = &m->verts[m->indices[0]];
			const RE_GPUVertex *v1 = &m->verts[m->indices[1]];
			const RE_GPUVertex *v2 = &m->verts[m->indices[2]];
			printf("[GPU-MESH] verts=%d idx=%d rgba=(%.2f %.2f %.2f %.2f) "
			       "n0=(%.3f %.3f %.3f) tri0=(%.3f %.3f %.3f)|(%.3f %.3f %.3f)|(%.3f %.3f %.3f)\n",
			       m->num_verts, m->num_indices,
			       v0->rgba[0], v0->rgba[1], v0->rgba[2], v0->rgba[3],
			       v0->n[0], v0->n[1], v0->n[2],
			       v0->co[0], v0->co[1], v0->co[2],
			       v1->co[0], v1->co[1], v1->co[2],
			       v2->co[0], v2->co[1], v2->co[2]);
		}
	}
	printf("[GPU] invariants: bad_norm=%d bad_rgba=%d bad_index=%d bad_tri=%d bad_emit=%d "
	       "emit_min=%.4f emit_max=%.4f spec_min=%.4f spec_max=%.4f "
	       "sizeof(RE_GPUVertex)=%d (stride float=%d, offsets co=0 n=3 uv=6 rgba=8 emit=12 tangent=13 orco=17)\n",
	       bad_norm, bad_alpha, bad_index, bad_tri, bad_emit, emit_min, emit_max,
	       spec_min, spec_max,
	       (int)sizeof(RE_GPUVertex), (int)(sizeof(RE_GPUVertex) / sizeof(float)));

	/* M5: сколько мешей получили диффузную карту (проверяется headless, без GL) */
	{
		int n_with = 0, shown = 0;
		for (RE_GPUMesh *m = gr->scene->meshes; m; m = m->next) {
			if (!m->ima) continue;
			n_with++;
			if (shown < 5) {
				Image *im = (Image *)m->ima;
				printf("[GPU-TEX] mesh: ma='%s' ima='%s' blend=%d colfac=%.2f\n",
				       m->ma ? m->ma->id.name + 2 : "<none>", im->id.name + 2,
				       m->tex_blend, m->colfac);
				shown++;
			}
		}
		printf("[GPU] textures: meshes_with_image=%d of %d\n",
		       n_with, gr->scene->num_meshes);
	}
	/* M8: меши с нормальной картой (проверяется headless, без GL) */
	{
		int n_nrm = 0;
		for (RE_GPUMesh *m = gr->scene->meshes; m; m = m->next)
			if (m->n_has) n_nrm++;
		printf("[GPU] normalmaps: meshes_with_normal_map=%d of %d\n",
		       n_nrm, gr->scene->num_meshes);
	}

	/* M5: прозрачность — сколько мешей реально полупрозрачные (без GL) */
	{
		int n_tr = 0, shown = 0;
		float amin = 1.0f, amax = 1.0f;
		for (RE_GPUMesh *m = gr->scene->meshes; m; m = m->next) {
			if (m->alpha < amin) amin = m->alpha;
			if (m->alpha > amax) amax = m->alpha;
			if (m->alpha < 0.999f) {
				n_tr++;
				if (shown < 5) {
					printf("[GPU-ALPHA] mesh: ma='%s' alpha=%.3f amap=%d "
					       "blend=%d fac=%.2f def=%.2f use_a=%d same_img=%d\n",
					       m->ma ? m->ma->id.name + 2 : "<none>", m->alpha,
					       m->a_has, m->a_blend, m->a_fac, m->a_def, m->a_use_a,
					       (m->aima && m->aima == m->ima) ? 1 : 0);
					shown++;
				}
			}
		}
		printf("[GPU] alpha: transparent_meshes=%d of %d alpha_min=%.3f alpha_max=%.3f\n",
		       n_tr, gr->scene->num_meshes, amin, amax);
		/* ★ M10: сколько мешей спасает альфа-карта (ma->alpha=0 + MAP_ALPHA:
		 * без карты они были бы полностью невидимы). */
		{
			int am = 0, am_zero = 0;
			for (RE_GPUMesh *m = gr->scene->meshes; m; m = m->next) {
				if (m->a_has) {
					am++;
					if (m->alpha < 0.001f) am_zero++;
				}
			}
			printf("[GPU] alphamap: meshes_with_alpha_map=%d (of them ma->alpha==0: %d)\n",
			       am, am_zero);
		}
	}

	/* ★ M11: ORCO-координаты. Считаем, у скольких мешей диффузная/альфа-карта
	 * берётся из ORCO (texco==TEXCO_ORCO), а не из UV — это те слоты, которые
	 * раньше (без M11) сэмплились неправильно (jolly/something2/disk/cave). */
	{
		int n_orco = 0, n_aorco = 0, shown = 0;
		for (RE_GPUMesh *m = gr->scene->meshes; m; m = m->next) {
			if (m->tex_orco) n_orco++;
			if (m->a_orco) n_aorco++;
			if ((m->tex_orco || m->a_orco) && shown < 6) {
				printf("[GPU-ORCO-SLOT] ma='%s' diff_orco=%d size=(%.2f %.2f) ofs=(%.2f %.2f)"
				       " a_orco=%d a_size=(%.2f %.2f)\n",
				       m->ma ? m->ma->id.name + 2 : "<none>",
				       m->tex_orco, m->tex_size[0], m->tex_size[1],
				       m->tex_ofs[0], m->tex_ofs[1],
				       m->a_orco, m->a_size[0], m->a_size[1]);
				shown++;
			}
		}
		printf("[GPU] orco: diff_orco=%d a_orco=%d of %d\n",
		       n_orco, n_aorco, gr->scene->num_meshes);
	}

	/* ★ M14: многослотовые диффузные карты. Максимум и число мешей с >1 слотом. */
	{
		int max_slots = 0, n_multi = 0, shown = 0;
		for (RE_GPUMesh *m = gr->scene->meshes; m; m = m->next) {
			if (m->n_tex > max_slots) max_slots = m->n_tex;
			if (m->n_tex > 1) n_multi++;
			if (m->n_tex > 1 && shown < 4) {
				printf("[GPU-NTEX] ma='%s' n=%d slot0='%s' blend=%d fac=%.2f",
				       m->ma ? m->ma->id.name + 2 : "<none>", m->n_tex,
				       m->ima ? ((Image *)m->ima)->id.name + 2 : "<none>",
				       m->tex_blend, m->colfac);
				for (int k = 0; k < m->n_tex - 1 && k < 3; k++) {
					printf(" | s%d='%s' blend=%d fac=%.2f",
					       k + 1, ((Image *)m->tex_ima2[k])->id.name + 2,
					       m->tex_blend2[k], m->colfac2[k]);
				}
				printf("\n");
				shown++;
			}
		}
		printf("[GPU] ntex: max_slots=%d multi=%d of %d\n",
		       max_slots, n_multi, gr->scene->num_meshes);
	}

	/* ★ M15: полупрозрачные меши (для будущего forward-прохода). Показывает, сколько
	 * мешей классифицированы как полупрозрачные (альфа-карта или ma->alpha<1). */
	{
		int ntrans = 0;
		for (RE_GPUMesh *m = gr->scene->meshes; m; m = m->next)
			if (m->transparent) ntrans++;
		printf("[GPU] transparent: %d of %d\n", ntrans, gr->scene->num_meshes);
	}

	/* ★ M5: блик материала. Показывает, у скольких мешей он ОТЛИЧАЕТСЯ от
	 * дефолтов BI (белый, hardness 50) — то есть ради скольких мешей вообще
	 * стоит держать 4-ю цель G-буфера. Без GL, проверяется headless. */
	{
		int n_custom = 0, shown = 0;
		float hmin = -1.0f, hmax = -1.0f;
		for (RE_GPUMesh *m = gr->scene->meshes; m; m = m->next) {
			if (hmin < 0.0f) hmin = hmax = m->hard;
			if (m->hard < hmin) hmin = m->hard;
			if (m->hard > hmax) hmax = m->hard;
			const int white = (m->spec_col[0] > 0.999f && m->spec_col[1] > 0.999f &&
			                   m->spec_col[2] > 0.999f);
			const int h50 = (m->hard > 49.5f && m->hard < 50.5f);
			if (white && h50) continue;
			n_custom++;
			if (shown < 5) {
				printf("[GPU-SPEC] mesh: ma='%s' spec=(%.3f %.3f %.3f) hard=%.0f\n",
				       m->ma ? m->ma->id.name + 2 : "<none>",
				       m->spec_col[0], m->spec_col[1], m->spec_col[2], m->hard);
				shown++;
			}
		}
		if (hmin < 0.0f) hmin = hmax = 0.0f;
		printf("[GPU] specular: non_default_meshes=%d of %d hard_min=%.0f hard_max=%.0f\n",
		       n_custom, gr->scene->num_meshes, hmin, hmax);
	}

	/* ★ M12/M13: карты блика/жёсткости/цвета блика. Считаем меши, у которых
	 * MAP_SPEC/MAP_HAR/MAP_COLSPEC активны на диффузном слоте. */
	{
		int n_spec = 0, n_har = 0, n_colspec = 0, n_refl = 0, shown = 0;
		for (RE_GPUMesh *m = gr->scene->meshes; m; m = m->next) {
			if (m->spec_has) n_spec++;
			if (m->har_has) n_har++;
			if (m->colspec_has) n_colspec++;
			if (m->refl_has) n_refl++;
			if ((m->spec_has || m->har_has || m->colspec_has || m->refl_has) && shown < 5) {
				printf("[GPU-SPECHAR] ma='%s' spec=%d(blend=%d fac=%.2f def=%.2f)"
				       " har=%d(blend=%d fac=%.2f def=%.2f) use_a=%d colspec=%d(blend=%d fac=%.2f)"
				       " refl=%d(blend=%d fac=%.2f def=%.2f base=%.2f)\n",
				       m->ma ? m->ma->id.name + 2 : "<none>",
				       m->spec_has, m->spec_blend, m->spec_fac, m->spec_def,
				       m->har_has, m->har_blend, m->har_fac, m->har_def,
				       m->sv_use_a,
				       m->colspec_has, m->colspec_blend, m->colspec_fac,
				       m->refl_has, m->refl_blend, m->refl_fac, m->refl_def, m->refl_base);
				shown++;
			}
		}
		printf("[GPU] specharmaps: spec=%d har=%d colspec=%d refl=%d of %d\n",
		       n_spec, n_har, n_colspec, n_refl, gr->scene->num_meshes);
	}

	/* ★ AABB мешей и отсечение по фрустуму камеры (headless-проверка).
	 * bad_bb — сколько мешей имеют вырожденный AABB (должно быть 0). Считаем и
	 * AABB всей сцены по мешам (для сверки с солнцевой тенью), и число мешей вне
	 * кадра (в 64x64-дампе камера видит всю сцену, поэтому outside должно быть
	 * небольшим; для больших сцен часть мешей отсечётся — это и есть отсечение). */
	{
		int bad_bb = 0, inside = 0, outside = 0;
		float smn[3], smx[3];
		copy_v3_fl(smn, FLT_MAX);
		copy_v3_fl(smx, -FLT_MAX);
		for (RE_GPUMesh *m = gr->scene->meshes; m; m = m->next) {
			for (int k = 0; k < 3; k++)
				if (m->bb_min[k] > m->bb_max[k]) bad_bb++;
			minmax_v3v3_v3(smn, smx, m->bb_min);
			minmax_v3v3_v3(smn, smx, m->bb_max);
		}
		if (gr->re && gr->re->viewmat) {
			float cm[4][4], cpl[6][4];
			mul_m4_m4m4(cm, gr->re->winmat, gr->re->viewmat);
			gpu_frustum_planes(cm, cpl);
			for (RE_GPUMesh *m = gr->scene->meshes; m; m = m->next) {
				if (gpu_aabb_outside(cpl, m->bb_min, m->bb_max)) outside++;
				else inside++;
			}
			/* ★ Матрицы камеры + вердикты отсечения по первым мешам: позволяют
			 * ПЕРЕПРОВЕРИТЬ отсечение независимо (verify_cull.py строит плоскости
			 * Грибба-Хартмана по этим же числам и повторяет тест «плюсовой» вершины).
			 * Без этого строка `[GPU] cull:` недоказуема. */
			printf("[GPU-CAMMAT] view r0=(%.5f %.5f %.5f %.5f) r1=(%.5f %.5f %.5f %.5f) "
			       "r2=(%.5f %.5f %.5f %.5f) r3=(%.5f %.5f %.5f %.5f) "
			       "win r0=(%.5f %.5f %.5f %.5f) r1=(%.5f %.5f %.5f %.5f) "
			       "r2=(%.5f %.5f %.5f %.5f) r3=(%.5f %.5f %.5f %.5f)\n",
			       gr->re->viewmat[0][0], gr->re->viewmat[0][1], gr->re->viewmat[0][2], gr->re->viewmat[0][3],
			       gr->re->viewmat[1][0], gr->re->viewmat[1][1], gr->re->viewmat[1][2], gr->re->viewmat[1][3],
			       gr->re->viewmat[2][0], gr->re->viewmat[2][1], gr->re->viewmat[2][2], gr->re->viewmat[2][3],
			       gr->re->viewmat[3][0], gr->re->viewmat[3][1], gr->re->viewmat[3][2], gr->re->viewmat[3][3],
			       gr->re->winmat[0][0], gr->re->winmat[0][1], gr->re->winmat[0][2], gr->re->winmat[0][3],
			       gr->re->winmat[1][0], gr->re->winmat[1][1], gr->re->winmat[1][2], gr->re->winmat[1][3],
			       gr->re->winmat[2][0], gr->re->winmat[2][1], gr->re->winmat[2][2], gr->re->winmat[2][3],
			       gr->re->winmat[3][0], gr->re->winmat[3][1], gr->re->winmat[3][2], gr->re->winmat[3][3]);
			int nbbs = 0;
			for (RE_GPUMesh *m = gr->scene->meshes; m && nbbs < 16; m = m->next, nbbs++) {
				printf("[GPU-CAMBB] %d mn=(%.4f %.4f %.4f) mx=(%.4f %.4f %.4f) out=%d\n",
				       nbbs, m->bb_min[0], m->bb_min[1], m->bb_min[2],
				       m->bb_max[0], m->bb_max[1], m->bb_max[2],
				       gpu_aabb_outside(cpl, m->bb_min, m->bb_max));
			}
		}

		/* ★ Проверка корректности отсечения: у первой ТЕНЕВОЙ лампы фрустум
		 * подогнан под сцену, поэтому меши вне него — это те, что позади света
		 * или в стороне (они не отбрасывают тень вперёд). Если бы их было много,
		 * это указывало бы на неверный AABB/плоскости. */
		int sun_culled = 0;
		for (int i = 0; i < gr->num_lights && i < GPU_MAX_SHADOW_LIGHTS; i++) {
			const RE_GPULight *L = &gr->lights[i];
			if (!L->shadows) continue;
			float spl[6][4];
			gpu_frustum_planes(L->vp, spl);
			for (RE_GPUMesh *m = gr->scene->meshes; m; m = m->next) {
				if (gpu_aabb_outside(spl, m->bb_min, m->bb_max)) sun_culled++;
			}
			break;   /* достаточно первой теневой лампы */
		}

		printf("[GPU] cull: bad_bb=%d inside=%d outside=%d sun_culled=%d "
		       "scene_bb=(%.2f %.2f %.2f)|(%.2f %.2f %.2f)\n",
		       bad_bb, inside, outside, sun_culled,
		       smn[0], smn[1], smn[2], smx[0], smx[1], smx[2]);
	}
}

/* ------------------------------------------------------------------------- */
/* Отладочный хук (UBF_GPU_DEBUG): собрать GPU-сцену и вывести статистику.   */
/* Вызывается из RE_TileProcessor; работает без GL, для headless-проверки.   */
/* ------------------------------------------------------------------------- */

void RE_gpu_renderer_debug_build(Render *re)
{
	const char *dbg = getenv("UBF_GPU_DEBUG");
	if (!dbg) return;

	RE_GPURenderer *gr = RE_gpu_renderer_create(re);
	RE_gpu_renderer_scene_build(gr);
	RE_gpu_renderer_dump(gr);
	RE_gpu_renderer_free(gr);
}

/* ------------------------------------------------------------------------- */
/* GL-фаза                                                                    */
/* ------------------------------------------------------------------------- */

/* Размер shadow map. Задан здесь ОДНИМ местом: из него же берётся шаг выборок
 * PCF (u_shadow_texel), иначе при смене размера шаг бы разъехался с текстурой. */
#define GPU_SHADOW_SIZE 4096

typedef struct RE_GPUGPURes {
	GLuint fb;          /* G-буфер FBO */
	GLuint tex_albedo;  /* RGBA8 */
	GLuint tex_nrm;     /* RGBA8 (view-space normal в RGB) */
	GLuint tex_vpos;    /* RGBA8: rgb = упакованная линейная view-глубина, a = маска геометрии */
	GLuint tex_spec;    /* RGBA8: rgb = цвет блика, a = hardness/511 (4-я цель MRT, M5) */
	GLuint rb_depth;    /* depth renderbuffer (GL_DEPTH_COMPONENT24) — для depth-теста */
	int width, height;
	bool inited;
	/* Кэш теневых карт. Тени зависят ТОЛЬКО от геометрии и ламп (не от камеры),
	 * поэтому их можно не пересчитывать при движении камеры. 1 — пересчитать,
	 * 0 — использовать кэш. Ставится в 1 при создании ресурса и когда gl_init
	 * заливает НОВЫЕ меши (признак пересборки сцены). */
	bool shadow_dirty;

	/* Есть ли 4-я цель MRT. Если GL_MAX_COLOR_ATTACHMENTS_EXT < 4, G-буфер
	 * остаётся 3-целевым, GB_FS компилируется БЕЗ записи в gl_FragData[3],
	 * а вместо tex_spec в проход освещения подставляется tex_spec_def. */
	bool spec_target;

	/* Есть ли 5-я цель MRT (MAP_REFL). Нужна для пер-пиксельной силы отражения;
	 * если нет, LIGHT_FS/SSR используют фиксированный микс 0.35. */
	bool refl_target;
	GLuint tex_refl;    /* R8: сила отражения (ma->ref, модулированная MAP_REFL) */
	GLuint tex_refl_def;/* заглушка для tex_refl, если refl_target=false */

	/* ★ M19: процедурные карты цвета (CLOUDS/BLEND).
	 * tex_hash  — 512x1 LUMINANCE8: таблица `hash[512]` из noise.c (значение/255).
	 * tex_hashv — 256x1 RGBA8: `hashvectf[768]` по одному градиенту в тексель
	 *             (компоненты закодированы как (v+1)/2).
	 * tex_ramp  — 256x1 RGBA8: колорбэнд текущего материала (запечён на CPU
	 *             через BKE_colorband_evaluate); обновляется glTexSubImage2D. */
	GLuint tex_hash, tex_hashv, tex_ramp;
	GLuint tex_hashp;   /* M20: 256x1 RGB8 = hashpntf[] (Voronoi feature points) */
	GLuint tex_ramp2;   /* M22: колорбэнд ВТОРОЙ процедурной текстуры материала */

	/* G-буфер шейдер */
	GLuint prog_gb;
	GLint gb_u_proj, gb_u_model, gb_u_normmat, gb_u_pack;
	GLint gb_a_co, gb_a_n, gb_a_rgba, gb_a_emit, gb_a_uv;
	GLint gb_a_tangent;   /* M8: касательная (T + знак) */
	GLint gb_a_orco;      /* M11: ORCO вершины */
	GLint gb_u_tex, gb_u_has_tex, gb_u_alpha;
	GLint gb_u_colfac, gb_u_tex_blend;   /* M9: смешивание диффузной карты */
	GLint gb_u_refl;                     /* M25: shi->refl (ma->ref) */
	GLint gb_u_tex_orco, gb_u_tex_size, gb_u_tex_ofs;   /* M11: источник координат карты */
	GLint gb_u_tex_glob;   /* M27: GLOB -> v_wpos */
	GLint gb_u_tex_map;                                  /* M16: do_2d_mapping (0..3) */
	/* ★ M19: процедурная карта цвета (CLOUDS/BLEND) */
	GLint gb_u_proc_type, gb_u_proc_stype, gb_u_proc_flip;
	GLint gb_u_proc_col;   /* M20 */
	GLint gb_u_proc_noisesize, gb_u_proc_noisedepth, gb_u_proc_basis, gb_u_proc_wf, gb_u_proc_hard;
	GLint gb_u_proc_dist;
	GLint gb_u_proc_bright, gb_u_proc_contrast, gb_u_proc_noclamp;
	GLint gb_u_proc_has_ramp, gb_u_proc_ramp, gb_u_proc_hash, gb_u_proc_hashv;
	GLint gb_u_proc_hashp;   /* M20: Voronoi feature points (hashpntf) */
	GLint gb_u_proc_orco, gb_u_proc_size, gb_u_proc_ofs;
	/* ★ M22: вторая процедурная текстура (слот != 0) — источник MAP_SPEC/MAP_HAR */
	GLint gb_u_has_proc2;
	GLint gb_u_proc2_type, gb_u_proc2_stype, gb_u_proc2_flip;
	GLint gb_u_proc2_noisesize, gb_u_proc2_noisedepth, gb_u_proc2_basis, gb_u_proc2_wf, gb_u_proc2_hard;
	GLint gb_u_proc2_dist;
	GLint gb_u_proc2_bright, gb_u_proc2_contrast, gb_u_proc2_noclamp;
	GLint gb_u_proc2_has_ramp, gb_u_proc2_ramp;
	GLint gb_u_proc2_orco, gb_u_proc2_size, gb_u_proc2_ofs;
	/* ★ M23: bump (MAP_NORM) от процедурной карты */
	GLint gb_u_has_proc_nrm, gb_u_proc_nrm_src, gb_u_proc_norfac;
	/* M14: дополнительные диффузные слоты */
	GLint gb_u_tex1, gb_u_tex1_blend, gb_u_tex1_fac, gb_u_tex1_orco, gb_u_tex1_size, gb_u_tex1_ofs;
	GLint gb_u_tex1_glob;
	GLint gb_u_tex1_map;
	GLint gb_u_tex2, gb_u_tex2_blend, gb_u_tex2_fac, gb_u_tex2_orco, gb_u_tex2_size, gb_u_tex2_ofs;
	GLint gb_u_tex2_glob;
	GLint gb_u_tex2_map;
	GLint gb_u_tex3, gb_u_tex3_blend, gb_u_tex3_fac, gb_u_tex3_orco, gb_u_tex3_size, gb_u_tex3_ofs;
	GLint gb_u_tex3_glob;
	GLint gb_u_tex3_map;
	GLint gb_u_n_tex;
	/* ★ раунд 83-fix3: MTEX_RGBTOINT диффузных слотов (фактор tin в BI) */
	GLint gb_u_tex_rgbtoint, gb_u_tex1_rgbtoint, gb_u_tex2_rgbtoint, gb_u_tex3_rgbtoint;
	GLint gb_u_atex, gb_u_has_atex;      /* M10: альфа-карта */
	GLint gb_u_a_fac, gb_u_a_def, gb_u_a_blend, gb_u_a_use_a;
	GLint gb_u_a_orco, gb_u_a_size, gb_u_a_ofs;   /* M11: источник координат альфа-карты */
	GLint gb_u_a_glob;   /* M27: GLOB -> v_wpos */
	GLint gb_u_a_map;                             /* M16: do_2d_mapping альфа-карты */
	GLint gb_u_has_spec, gb_u_spec_blend, gb_u_spec_fac, gb_u_spec_def;   /* M12: карта блика */
	GLint gb_u_has_har, gb_u_har_blend, gb_u_har_fac, gb_u_har_def;       /* M12: карта жёсткости */
	GLint gb_u_sv_use_a;   /* M12: tin = альфа/яркость для spec/har */
	GLint gb_u_has_colspec, gb_u_colspec_blend, gb_u_colspec_fac;   /* M13: цвет блика */
	GLint gb_u_refl_base, gb_u_has_refl, gb_u_refl_blend, gb_u_refl_fac, gb_u_refl_def;   /* M17: отражение */
	GLint gb_u_nrm, gb_u_has_nrm;   /* M8: нормальная карта */
	GLint gb_u_nrm_is_height, gb_u_nrm_norfac;   /* M26: height-bump vs tangent */
	GLint gb_u_nrm_orco, gb_u_nrm_glob, gb_u_nrm_map;   /* M28: координаты карты нормалей */
	GLint gb_u_nrm_size, gb_u_nrm_ofs;
	GLint gb_u_bump_scale;
	GLint gb_u_spec_col, gb_u_hard;   /* M5: блик материала (нужны только при spec_target) */
	GLuint tex_white;   /* 1x1 белая — подставляется, когда у материала нет карты */
	GLuint tex_spec_def;/* 1x1: белый блик + hardness по умолчанию BI (50) — когда 4-й цели нет */

	/* ★ Теневые карты: по одной на каждую лампу (все GPU_MAX_SHADOW_LIGHTS, даже если
	 * тень дают не все — так нет зависимости от порядка сбора света). Упакованная
	 * window-глубина в color + depth renderbuffer. Лампа i использует tex_shadow[i].
	 * Программа одна (prog_shadow), её uniform u_lightvp ставится на каждый проход. */
	GLuint shadow_fb[GPU_MAX_SHADOW_LIGHTS];
	GLuint tex_shadow[GPU_MAX_SHADOW_LIGHTS];
	GLuint rb_shadow[GPU_MAX_SHADOW_LIGHTS];
	GLuint prog_shadow;
	GLint shadow_a_co, shadow_u_lightvp, shadow_u_obmat;
	/* ★ POINT: кубическая карта на каждую лампу (6 граней в одной текстуре).
	 * shadow_fb[si]/rb_shadow[si] переиспользуются — перед рендером грани k
	 * color-attachment переключается на GL_TEXTURE_CUBE_MAP_*_k. tex_cube_white —
	 * заглушка 1x1 для не-point ламп (samplerCube обязан быть привязан). */
	GLuint tex_shadow_cube[GPU_MAX_SHADOW_LIGHTS];
	GLuint tex_cube_white;

	/* ★★ Раунд 16: данные ВСЕХ ламп одной float-текстурой — вместо per-light
	 * юниформов u_lpos/u_ldir/u_lcol/u_lfl (те кончались на 10 лампах, потому что
	 * упирались в лимит uniform-компонентов). Формат — RGBA32F, ширина = число
	 * ламп, высота GPU_LIGHT_DATA_ROWS (4):
	 *   строка 0: pos.x pos.y pos.z type
	 *   строка 1: dir.x dir.y dir.z dist
	 *   строка 2: col.r col.g col.b falloff
	 *   строка 3: sphere spotsi spotbl soft
	 * Шейдер читает её через gpu_ltap(i, row) с GL_NEAREST по центрам текселей,
	 * а u_light_texel = (1/ширина, 1/высота) передаётся юниформом. */
	GLuint tex_lights;
	int tex_lights_w;      /* текущая ширина (в лампах); 0 — текстура не создана */
	float *light_buf;      /* CPU-буфер выгрузки (переиспользуется между кадрами) */
	int light_buf_cap;

	/* шейдер деферред-освещения */
	GLuint prog_light;
	GLint light_a_pos, light_a_uv;
	GLint light_u_albedo, light_u_nrm;
	GLint light_u_num_lights, light_u_exposure;
	GLint light_u_lightdata, light_u_lighttexel;   /* ★ раунд 16 */
	GLint light_u_world_hor, light_u_world_zen, light_u_world_amb;
	GLint light_u_world_blend, light_u_amb_fill;
	GLint light_u_env_light, light_u_env_energy;   /* M26: Environment Lighting */
	GLint light_u_linfac, light_u_logfac, light_u_expo_on;
	GLint light_u_depth, light_u_invproj, light_u_invview;
	GLint light_u_shadow[GPU_MAX_SHADOW_LIGHTS];   /* теневая карта каждой лампы */
	GLint light_u_lvp[GPU_MAX_SHADOW_LIGHTS];      /* world->light-clip каждой лампы */
	GLint light_u_shadow_cube[GPU_MAX_SHADOW_LIGHTS]; /* point: кубическая карта лампы */
	GLint light_u_lwpos[GPU_MAX_SHADOW_LIGHTS];    /* point: мировая позиция света */
	GLint light_u_lpz[GPU_MAX_SHADOW_LIGHTS];      /* point: (A,B) window-глубины */
	/* ★ раунд 16: light_u_lsoft убран — soft едет в строке 4 u_light_data */
	GLint light_u_spec;    /* M5: цвет блика + hardness */
	GLint light_u_refl;    /* M17: сила отражения (MAP_REFL) */
	GLint light_u_pack;
	GLint light_u_ortho;   /* раунд 83-fix: 1 = ортокамера */
	GLint light_u_up;
	GLint light_u_proj, light_u_ssr;
	GLint light_u_texel, light_u_ao;
	GLint light_u_shadow_texel, light_u_pcf;
	GLint light_u_shadow_bias;   /* раунд 83: множитель мирового bias теней */
	GLint light_u_debug;   /* 0=освещение, 1=albedo, 2=нормаль, 3=глубина */

	/* M15: forward-проход полупрозрачных мешей (alpha-блендинг). */
	GLuint prog_forward;
	GLint fw_a_co, fw_a_n, fw_a_rgba, fw_a_emit, fw_a_uv, fw_a_tangent, fw_a_orco;
	GLint fw_u_proj, fw_u_model, fw_u_obmat, fw_u_normmat;
	GLint fw_u_alpha, fw_u_exposure, fw_u_expo_on, fw_u_linfac, fw_u_logfac;
	GLint fw_u_amb_fill, fw_u_up, fw_u_world_hor, fw_u_world_zen, fw_u_world_amb;
	GLint fw_u_world_blend, fw_u_num_lights;
	GLint fw_u_env_light, fw_u_env_energy;         /* M26: Environment Lighting */
	GLint fw_u_lightdata, fw_u_lighttexel;   /* ★ раунд 16: та же текстура ламп */
	GLint fw_u_shadow[GPU_MAX_SHADOW_LIGHTS], fw_u_lvp[GPU_MAX_SHADOW_LIGHTS];
	GLint fw_u_shadow_cube[GPU_MAX_SHADOW_LIGHTS], fw_u_lwpos[GPU_MAX_SHADOW_LIGHTS];
	GLint fw_u_lpz[GPU_MAX_SHADOW_LIGHTS];   /* ★ раунд 16: fw_u_lsoft убран */
	GLint fw_u_pcf, fw_u_shadow_texel, fw_u_depth, fw_u_resolution, fw_u_pack;
	GLint fw_u_shadow_bias;   /* раунд 83: множитель мирового bias теней */
	GLint fw_u_ortho;         /* раунд 83-fix: 1 = ортокамера */
	GLint fw_u_albedo, fw_u_inv_proj, fw_u_texel, fw_u_ssr, fw_u_ao;
	GLint fw_u_tex, fw_u_has_tex, fw_u_colfac, fw_u_tex_blend, fw_u_refl;
	GLint fw_u_tex_orco, fw_u_tex_size, fw_u_tex_ofs, fw_u_tex_map;
	GLint fw_u_tex_glob;
	/* ★ M19: процедурная карта цвета и в forward-проходе (disk Material.002 —
	 * прозрачный материал с CLOUDS-картой цвета). Порядок полей как у gb_u_proc_*. */
	GLint fw_u_proc_type, fw_u_proc_stype, fw_u_proc_flip;
	GLint fw_u_proc_col;   /* M20 */
	GLint fw_u_tex_rgbtoint;   /* ★ раунд 83-fix3 */
	GLint fw_u_proc_noisesize, fw_u_proc_noisedepth, fw_u_proc_basis, fw_u_proc_wf, fw_u_proc_hard;
	GLint fw_u_proc_dist;
	GLint fw_u_proc_bright, fw_u_proc_contrast, fw_u_proc_noclamp;
	GLint fw_u_proc_has_ramp, fw_u_proc_ramp, fw_u_proc_hash, fw_u_proc_hashv;
	GLint fw_u_proc_hashp;   /* M20: Voronoi feature points (hashpntf) */
	GLint fw_u_proc_orco, fw_u_proc_size, fw_u_proc_ofs;
	GLint fw_u_atex, fw_u_has_atex, fw_u_a_fac, fw_u_a_def, fw_u_a_blend, fw_u_a_use_a;
	GLint fw_u_a_orco, fw_u_a_size, fw_u_a_ofs, fw_u_a_map;
	GLint fw_u_a_glob;
	GLint fw_u_nrm, fw_u_has_nrm, fw_u_spec_col, fw_u_hard;

	/* ★ M16: спрайты-хало (материалы типа HALO). */
	GLuint prog_halo;
	GLint halo_a_corner, halo_a_uv;
	GLint halo_u_proj, halo_u_view, halo_u_pos, halo_u_right, halo_u_up, halo_u_rad;
	GLint halo_u_col, halo_u_alfa, halo_u_hard, halo_u_add, halo_u_zcenter;
	GLint halo_u_depth, halo_u_pack, halo_u_resolution;
	GLint halo_u_hashv, halo_u_seed, halo_u_ringc, halo_u_linec, halo_u_starpoints;
	GLint halo_u_xalpha, halo_u_spec_col, halo_u_mir_col;

	/* полноэкранный quad */
	GLuint quad_vbo, quad_ibo;
} RE_GPUGPURes;

/* --- диагностика: расшифровка статуса FBO --- */
static const char *gl_fbo_status_str(GLenum st)
{
	switch (st) {
		case GL_FRAMEBUFFER_COMPLETE_EXT:                       return "COMPLETE";
		case GL_FRAMEBUFFER_INCOMPLETE_ATTACHMENT_EXT:          return "INCOMPLETE_ATTACHMENT";
		case GL_FRAMEBUFFER_INCOMPLETE_MISSING_ATTACHMENT_EXT:  return "INCOMPLETE_MISSING_ATTACHMENT";
#ifdef GL_FRAMEBUFFER_INCOMPLETE_DUPLICATE_ATTACHMENT_EXT
		case GL_FRAMEBUFFER_INCOMPLETE_DUPLICATE_ATTACHMENT_EXT:return "INCOMPLETE_DUPLICATE_ATTACHMENT";
#endif
		case GL_FRAMEBUFFER_INCOMPLETE_DRAW_BUFFER_EXT:         return "INCOMPLETE_DRAW_BUFFER";
		case GL_FRAMEBUFFER_INCOMPLETE_READ_BUFFER_EXT:         return "INCOMPLETE_READ_BUFFER";
		case GL_FRAMEBUFFER_UNSUPPORTED_EXT:                    return "UNSUPPORTED";
#ifdef GL_FRAMEBUFFER_INCOMPLETE_DIMENSIONS_EXT
		case GL_FRAMEBUFFER_INCOMPLETE_DIMENSIONS_EXT:          return "INCOMPLETE_DIMENSIONS";
#endif
#ifdef GL_FRAMEBUFFER_INCOMPLETE_FORMATS_EXT
		case GL_FRAMEBUFFER_INCOMPLETE_FORMATS_EXT:             return "INCOMPLETE_FORMATS";
#endif
		default:                                                return "UNKNOWN";
	}
}

/* --- возможности GPU: определяют, валиден ли 3-MRT G-буфер и какие форматы --- */
static void gl_print_caps(void)
{
	const char *ver = (const char *)glGetString(GL_VERSION);
	const char *ren = (const char *)glGetString(GL_RENDERER);
	const char *ven = (const char *)glGetString(GL_VENDOR);
	const char *glsl = (const char *)glGetString(GL_SHADING_LANGUAGE_VERSION);
	GLint maxct = 0, maxdb = 0, maxtex = 0;
	glGetIntegerv(GL_MAX_COLOR_ATTACHMENTS_EXT, &maxct);
	glGetIntegerv(GL_MAX_DRAW_BUFFERS, &maxdb);
	glGetIntegerv(GL_MAX_TEXTURE_IMAGE_UNITS, &maxtex);
	printf("[GPU-CAPS] GL_VERSION=%s\n", ver ? ver : "(null)");
	printf("[GPU-CAPS] GL_RENDERER=%s | GL_VENDOR=%s\n", ren ? ren : "(null)", ven ? ven : "(null)");
	printf("[GPU-CAPS] GLSL=%s\n", glsl ? glsl : "(null)");
	printf("[GPU-CAPS] max_color_attachments=%d max_draw_buffers=%d "
	       "ARB_texture_float=%d EXT_framebuffer_object=%d ARB_draw_buffers=%d\n",
	       maxct, maxdb,
	       (int)GLEW_ARB_texture_float,
	       (int)GLEW_EXT_framebuffer_object,
	       (int)GLEW_ARB_draw_buffers);
	/* ★ Сэмплеры на лампу: 2D-тень (юнит 6+i) + кубическая (6+GPU_MAX_SHADOW_LIGHTS+i) в
	 * проходе света; в forward — 10+i и 10+GPU_MAX_SHADOW_LIGHTS+i. Нужно
	 * 6 + 2*GPU_MAX_SHADOW_LIGHTS и 10 + 2*GPU_MAX_SHADOW_LIGHTS юнитов соответственно: при
	 * нехватке линковка шейдера падает и проход молча отменяется (кадр чёрный). */
	printf("[GPU-CAPS] max_texture_image_units=%d (нужно %d для света, %d для forward "
	       "при GPU_MAX_SHADOW_LIGHTS=%d)%s\n",
	       maxtex, 6 + 2 * GPU_MAX_SHADOW_LIGHTS, 10 + 2 * GPU_MAX_SHADOW_LIGHTS, GPU_MAX_SHADOW_LIGHTS,
	       (maxtex < 10 + 2 * GPU_MAX_SHADOW_LIGHTS)
	           ? " — НЕ ХВАТАЕТ, часть ламп не получит теней/света!" : "");
}

/* --- компиляция шейдера --- */
static GLuint gl_compile(GLenum type, const char *src)
{
	GLuint sh = glCreateShader(type);
	glShaderSource(sh, 1, &src, NULL);
	glCompileShader(sh);
	GLint ok = 0;
	glGetShaderiv(sh, GL_COMPILE_STATUS, &ok);
	if (!ok) {
		char log[2048];
		glGetShaderInfoLog(sh, sizeof(log), NULL, log);
		printf("[GPU] shader compile error: %s\n", log);
		glDeleteShader(sh);
		return 0;
	}
	return sh;
}

static GLuint gl_link(GLuint vs, GLuint fs)
{
	if (!vs || !fs) return 0;
	GLuint p = glCreateProgram();
	glAttachShader(p, vs);
	glAttachShader(p, fs);
	glLinkProgram(p);
	GLint ok = 0;
	glGetProgramiv(p, GL_LINK_STATUS, &ok);
	if (!ok) {
		char log[2048];
		glGetProgramInfoLog(p, sizeof(log), NULL, log);
		printf("[GPU] program link error: %s\n", log);
		glDeleteProgram(p);
		glDeleteShader(vs);
		glDeleteShader(fs);
		return 0;
	}
	/* шейдеры можно удалить после линковки */
	glDeleteShader(vs);
	glDeleteShader(fs);
	return p;
}

/* --- G-буфер vertex shader: позиция + нормаль + цвет (M2) --- */
static const char *GB_VS =
	"attribute vec3 a_co;\n"
	"attribute vec3 a_n;\n"
	"attribute vec4 a_rgba;\n"
	"attribute float a_emit;\n"
	"attribute vec2 a_uv;\n"
	"attribute vec4 a_tangent;\n"   /* M8: T.xyz (объектное), w = знак битангента */
	"attribute vec3 a_orco;\n"      /* M11: ORCO (BI VertRen->orco), в [-1,1] */
	"uniform mat4 u_proj;\n"
	"uniform mat4 u_model;\n"
	"uniform mat3 u_normmat;\n"
	"varying vec3 v_nrm;\n"
	"varying vec4 v_rgba;\n"
	"varying float v_emit;\n"
	"varying vec2 v_uv_tex;\n"
	"varying vec3 v_tangent;\n"     /* view-space касательный (T) */
	"varying float v_tang_sign;\n"
	"varying float v_linz;\n"      /* линейная глубина во view-space (положительная) */
	"varying vec3 v_orco;\n"
	"varying vec3 v_vpos;\n"        /* M23: view-space позиция — нужна для bump (экранные производные) */
	"varying vec3 v_wpos;\n"        /* M27: мировая позиция — для texco==TEXCO_GLOB (BI shi->gl) */
	"varying vec3 v_w_nrm;\n"       /* M29: МИРОВАЯ нормаль — для CUBE-маппинга (BI cubemap_glob) */
	"void main(){\n"
	"  vec3 nn = u_normmat * a_n;\n"
	"  float nl = length(nn);\n"
	"  v_nrm = (nl > 1e-6) ? nn / nl : vec3(0.0, 0.0, 1.0);\n"   /* защита от normalize(0) = NaN */
	"  v_w_nrm = normalize(a_n);\n"   /* M29: a_n уже мировая (rv->n прошёл viewinv, line 744) */
	"  v_rgba = a_rgba;\n"
	"  v_emit = a_emit;\n"
	"  v_uv_tex = a_uv;\n"
	"  v_tangent = mat3(u_model) * a_tangent.xyz;\n"   /* касательная — направление: модель, не инв-трансп */
	"  v_tang_sign = a_tangent.w;\n"
	"  v_orco = a_orco;\n"
	"  v_wpos = a_co;\n"           /* M27: a_co уже в мировых координатах (viewinv в rv->co) */
	"  vec4 vp = u_model * vec4(a_co, 1.0);\n"
	"  v_linz = -vp.z;\n"
	"  v_vpos = vp.xyz;\n"
	"  gl_Position = u_proj * vp;\n"
	"}\n";

/* --- G-буфер fragment shader: albedo(0) + view-space normal(1) + view-глубина(2) --- */
static const char *GB_FS =
	"varying vec3 v_nrm;\n"
	"varying vec4 v_rgba;\n"
	"varying float v_emit;\n"
	"varying vec2 v_uv_tex;\n"
	"varying vec3 v_tangent;\n"
	"varying float v_tang_sign;\n"
	"varying float v_linz;\n"
	"varying vec3 v_orco;\n"        /* M11: ORCO вершины (BI VertRen->orco) */
	"varying vec3 v_vpos;\n"        /* M23: view-space позиция (для bump по экранным производным) */
	"varying vec3 v_wpos;\n"        /* M27: мировая позиция (для texco==TEXCO_GLOB) */
	"varying vec3 v_w_nrm;\n"       /* M29: МИРОВАЯ нормаль (для CUBE-маппинга) */
	"uniform float u_pack;\n"      /* 1/clipend: нормировка глубины в [0,1) */
	"uniform sampler2D u_tex;\n"   /* M5: диффузная карта материала */
	"uniform float u_has_tex;\n"
	/* ★ M14: дополнительные диффузные слоты (цепочка texture_rgb_blend). */
	"uniform sampler2D u_tex1;\n"
	"uniform int u_tex1_blend;\n"
	"uniform float u_tex1_fac;\n"
	"uniform float u_tex1_orco;\n"
	"uniform float u_tex1_glob;\n"
	"uniform float u_tex1_map;\n"
	"uniform vec2 u_tex1_size;\n"
	"uniform vec2 u_tex1_ofs;\n"
	"uniform sampler2D u_tex2;\n"
	"uniform int u_tex2_blend;\n"
	"uniform float u_tex2_fac;\n"
	"uniform float u_tex2_orco;\n"
	"uniform float u_tex2_glob;\n"
	"uniform float u_tex2_map;\n"
	"uniform vec2 u_tex2_size;\n"
	"uniform vec2 u_tex2_ofs;\n"
	"uniform sampler2D u_tex3;\n"
	"uniform int u_tex3_blend;\n"
	"uniform float u_tex3_fac;\n"
	"uniform float u_tex3_orco;\n"
	"uniform float u_tex3_glob;\n"
	"uniform float u_tex3_map;\n"
	"uniform vec2 u_tex3_size;\n"
	"uniform vec2 u_tex3_ofs;\n"
	"uniform int u_n_tex;\n"        /* число диффузных слотов (1..4) */
	"/* ★ раунд 83-fix3: 1 = MTEX_RGBTOINT у слота (тогда tin = яркость карты,\n"
	"   иначе tin = 1.0). BI: render_texture.c:2440. */\n"
	"uniform float u_tex_rgbtoint, u_tex1_rgbtoint, u_tex2_rgbtoint, u_tex3_rgbtoint;\n"
	"uniform float u_colfac;\n"    /* M9: mtex->colfac */
	"uniform float u_refl;\n"      /* M25: shi->refl = ma->ref; BI множит на него диффуз и ambient */
	"uniform int u_tex_blend;\n"   /* M9: MTEX_* (0=MIX) */
	"uniform float u_tex_orco;\n"  /* M11: 1 => диффузная карта по ORCO, 0 => UV */
	"uniform float u_tex_glob;\n"  /* M27: 1 => диффузная карта по GLOB (мировая позиция) */
	"uniform float u_tex_map;\n"   /* M16: do_2d_mapping (0=FLAT,1=CUBE,2=TUBE,3=SPHERE) */
	"uniform vec2 u_tex_size;\n"   /* M11: mtex->size.xy (translate/scale карты) */
	"uniform vec2 u_tex_ofs;\n"    /* M11: mtex->ofs.xy */
	"uniform float u_alpha;\n"     /* прозрачность материала (1.0 = непрозрачный) */
	/* ★ M10: альфа-карта (MAP_ALPHA) — попиксельная прозрачность */
	"uniform sampler2D u_atex;\n"
	"uniform float u_has_atex;\n"
	"uniform float u_a_fac;\n"     /* mtex->alphafac */
	"uniform float u_a_def;\n"     /* mtex->def_var */
	"uniform int u_a_blend;\n"     /* mtex->blendtype */
	"uniform float u_a_use_a;\n"   /* 1 => tin = альфа картинки, 0 => яркость */
	"uniform float u_a_orco;\n"    /* M11: 1 => альфа-карта по ORCO, 0 => UV */
	"uniform float u_a_glob;\n"    /* M27: 1 => альфа-карта по GLOB (мировая позиция) */
	"uniform float u_a_map;\n"     /* M16: do_2d_mapping альфа-карты */
	"uniform vec2 u_a_size;\n"
	"uniform vec2 u_a_ofs;\n"
	"uniform sampler2D u_nrm;\n"   /* M8: нормальная карта (TANGENT-space) */
	"uniform float u_has_nrm;\n"
	"uniform float u_nrm_is_height;\n" /* M26: 1 => height-bump (нет TEX_NORMALMAP), 0 => тангенциальная */
	"uniform float u_nrm_norfac;\n"    /* M26: mtex->norfac слота карты нормалей */
	"uniform float u_bump_scale;\n"    /* M30: общий множитель высотного bump (подбор) */
	"uniform float u_nrm_orco;\n"      /* M28: 1 => карта нормалей по ORCO */
	"uniform float u_nrm_glob;\n"      /* M28: 1 => карта нормалей по GLOB (мировая позиция) */
	"uniform float u_nrm_map;\n"       /* M28: do_2d_mapping карты нормалей */
	"uniform vec2 u_nrm_size;\n"
	"uniform vec2 u_nrm_ofs;\n"
	/* ★ M5: блик материала. Униформы, а не атрибуты вершин: меш — это «один
	   материал одного инстанса», поэтому внутри меша блик одинаков. */
	"uniform vec3 u_spec_col;\n"
	"uniform float u_hard;\n"      /* hardness/511 */
	/* ★ M12: карты блика/жёсткости (MAP_SPEC/MAP_HAR) — на том же слоте, что и
	   диффузная карта, поэтому переиспользуют её сэмпл (tc) и tin. */
	"uniform float u_has_spec;\n"
	"uniform int u_spec_blend;\n"
	"uniform float u_spec_fac;\n"
	"uniform float u_spec_def;\n"
	"uniform float u_has_har;\n"
	"uniform int u_har_blend;\n"
	"uniform float u_har_fac;\n"
	"uniform float u_har_def;\n"
	"uniform float u_sv_use_a;\n"   /* tin = альфа картинки (Use Alpha), иначе яркость */
	/* ★ M13: карта цвета блика (MAP_COLSPEC) — переиспользует сэмпл tc. */
	"uniform float u_has_colspec;\n"
	"uniform int u_colspec_blend;\n"
	"uniform float u_colspec_fac;\n"
	/* ★ M17: MAP_REFL — сила отражения (5-я цель MRT) */
	"uniform float u_refl_base;\n"   /* ma->ref (0..1) */
	"uniform float u_has_refl;\n"
	"uniform int u_refl_blend;\n"
	"uniform float u_refl_fac;\n"    /* mtex->difffac */
	"uniform float u_refl_def;\n"    /* mtex->def_var */
	/* ★ M9: смешивание карты с базовым цветом — точный порт texture_rgb_blend
	   (render_texture.c). BI при MIX/colfac=1 ЗАМЕНЯЕТ базовый цвет картой;
	   прежнее base*=tex затемняло текстуры и делало чёрные материалы чёрными. */
	"vec3 gpu_texblend(vec3 outc, vec3 tex, float fact, float facg, int bt){\n"
	"  float facm;\n"
	"  if (bt == 0) { fact *= facg; facm = 1.0 - fact; return fact*tex + facm*outc; }\n"
	"  if (bt == 1) { fact *= facg; facm = 1.0 - fact; return (facm + fact*tex) * outc; }\n"
	"  if (bt == 2) { fact *= facg; return fact*tex + outc; }\n"
	"  if (bt == 3) { fact = -fact; fact *= facg; return fact*tex + outc; }\n"
	"  if (bt == 4) { fact *= facg; facm = 1.0 - fact;\n"
	"    vec3 r = outc;\n"
	"    if (tex.r != 0.0) r.r = facm*outc.r + fact*outc.r/tex.r;\n"
	"    if (tex.g != 0.0) r.g = facm*outc.g + fact*outc.g/tex.g;\n"
	"    if (tex.b != 0.0) r.b = facm*outc.b + fact*outc.b/tex.b;\n"
	"    return r; }\n"
	"  if (bt == 5) { fact *= facg; facm = 1.0 - fact; return min(outc, tex)*fact + outc*facm; }\n"
	"  if (bt == 6) { fact *= facg; facm = 1.0 - fact; return facm*outc + fact*abs(tex - outc); }\n"
	"  if (bt == 7) { fact *= facg; return max(fact*tex, outc); }\n"
	"  if (bt == 8) { fact *= facg; facm = 1.0 - fact;\n"
	"    return vec3(1.0) - (facm + fact*(vec3(1.0)-tex)) * (vec3(1.0)-outc); }\n"
	"  if (bt == 9) { fact *= facg; facm = 1.0 - fact;\n"
	"    vec3 lo = outc * (facm + 2.0*fact*tex);\n"
	"    vec3 hi = vec3(1.0) - (facm + 2.0*fact*(vec3(1.0)-tex)) * (vec3(1.0)-outc);\n"
	"    return vec3(outc.r < 0.5 ? lo.r : hi.r, outc.g < 0.5 ? lo.g : hi.g,\n"
	"                outc.b < 0.5 ? lo.b : hi.b); }\n"
	"  fact *= facg; facm = 1.0 - fact; return fact*tex + facm*outc;\n"  /* 10..15 -> MIX */
	"}\n"
	/* ★ M10: скалярный порт `texture_value_blend` (render_texture.c:1574) — для
	   альфа-карты. Сигнатура как в C: (tex, out, fact, facg, blendtype);
	   вызывается как texture_value_blend(def_var, alpha, tin, alphafac, bt).
	   У MTEX_MUL/SCREEN/OVERLAY facm пересчитывается от facg (не от fact). */
	"float gpu_valblend(float tex, float outv, float fact, float facg, int bt){\n"
	"  float facm, col, scf;\n"
	"  int flip = (facg < 0.0) ? 1 : 0;\n"
	"  facg = abs(facg);\n"
	"  fact *= facg;\n"
	"  facm = 1.0 - fact;\n"
	"  if (flip == 1) { float t = fact; fact = facm; facm = t; }\n"
	"  if (bt == 0) return fact*tex + facm*outv;\n"                                   /* MIX */
	"  if (bt == 1) { facm = 1.0 - facg; return (facm + fact*tex)*outv; }\n"          /* MUL */
	"  if (bt == 8) { facm = 1.0 - facg;\n"
	"    return 1.0 - (facm + fact*(1.0-tex))*(1.0-outv); }\n"                        /* SCREEN */
	"  if (bt == 9) { facm = 1.0 - facg;\n"
	"    if (outv < 0.5) return outv * (facm + 2.0*fact*tex);\n"
	"    return 1.0 - (facm + 2.0*fact*(1.0-tex))*(1.0-outv); }\n"                    /* OVERLAY */
	"  if (bt == 3) { fact = -fact; return fact*tex + outv; }\n"                      /* SUB */
	"  if (bt == 2) return fact*tex + outv;\n"                                        /* ADD */
	"  if (bt == 4) { if (tex != 0.0) return facm*outv + fact*outv/tex; return 0.0; }\n" /* DIV */
	"  if (bt == 6) return facm*outv + fact*abs(tex-outv);\n"                         /* DIFF */
	"  if (bt == 5) return min(outv, tex)*fact + outv*facm;\n"                        /* DARK */
	"  if (bt == 7) { col = fact*tex; return (col > outv) ? col : outv; }\n"          /* LIGHT */
	"  if (bt == 15) { scf = 1.0 - (1.0-tex)*(1.0-outv);\n"
	"    return facm*outv + fact*((1.0-outv)*tex*outv) + (outv*scf); }\n"             /* SOFT */
	"  if (bt == 16) { if (tex > 0.5) return outv + fact*(2.0*(tex-0.5));\n"
	"    return outv + fact*(2.0*tex - 1.0); }\n"                                     /* LIN */
	"  return fact*tex + facm*outv;\n"
	"}\n"
	"/* ★★ раунд 83-fix3: фактор смешивания текстуры (BI называет его `tin`).\n"
	"   BI: texture_rgb_blend(&shi->r, tcol, &shi->r, texres.tin, colfac, blendtype)\n"
	"   (render_texture.c:2554), а сам tin задан в render_texture.c:2440:\n"
	"       if ((rgbnor & TEX_RGB) && (mtex->texflag & MTEX_RGBTOINT))\n"
	"           texres.tin = IMB_colormanagement_get_luminance(&texres.tr);\n"
	"   То есть для ОБРАЗНОЙ карты tin == 1.0 (карта при colfac=1 заменяет или\n"
	"   умножает цвет целиком), и только при включённом RGB-to-Intensity — яркость\n"
	"   карты. Процедурная текстура отдаёт в tc.a собственную интенсивность,\n"
	"   поэтому для неё берём именно tc.a, как и было. Раньше для образных тоже\n"
	"   бралась tc.a, то есть АЛЬФА КАРТИНКИ: у карт с частичной альфой фактор\n"
	"   выходил в разы меньше 1, и модуляция цвета была втрое слабее scanline\n"
	"   (замер mapFull: SCAN x1.729 при отвязке карт, GPU лишь x1.238). */\n"
	"float gpu_tex_tin(vec4 tc, float rgbtoint, float is_proc){\n"
	"  if (is_proc > 0.5) return tc.a;\n"
	"  return (rgbtoint > 0.5) ? dot(tc.rgb, vec3(0.2126, 0.7152, 0.0722)) : 1.0;\n"
	"}\n"
	"float gpu_hash(vec2 p){\n"
	"  return fract(sin(dot(p, vec2(12.9898, 78.233))) * 43758.5453);\n"
	"}\n"
	"/* ★ M15: упорядоченный дизеринг (Bayer 4x4) вместо случайного хэша. Рекурсивная\n"
	"   конструкция B4 = 4*B2(sub) + B2(quadrant) (B2 = [[0,2],[3,1]]). Для screen-door\n"
	"   даёт регулярный паттерн (меньше «телевизионного шума»), детерминированный по\n"
	"   пикселю — полупрозрачные поверхности выглядят чище, чем при white-noise. */\n"
	"float gpu_bayer2(vec2 p){\n"
	"  float x = mod(p.x, 2.0);\n"
	"  float y = mod(p.y, 2.0);\n"
	"  if (x < 0.5) return (y < 0.5) ? 0.0 : 3.0;\n"
	"  else         return (y < 0.5) ? 2.0 : 1.0;\n"
	"}\n"
	"float gpu_bayer4(vec2 p){\n"
	"  return 4.0 * gpu_bayer2(p) + gpu_bayer2(floor(p / 2.0));\n"
	"}\n"
	"float gpu_bayer8(vec2 p){\n"
	"  return 4.0 * gpu_bayer4(p) + gpu_bayer2(floor(p / 4.0));\n"
	"}\n"
	"vec3 gpu_pack_depth(float v){\n"
	"  vec3 enc = fract(vec3(1.0, 255.0, 65025.0) * v);\n"
	"  enc -= enc.yzz * vec3(1.0/255.0, 1.0/255.0, 0.0);\n"
	"  return enc;\n"
	"}\n"
	/* ★ M11/M16: координаты для сэмпла карты. BI (render_texture.c): при texco==ORCO
	   co=shi->lo (у нас v_orco, уже в [-1,1]), затем do_2d_mapping: FLAT
	   t=(orco.xy+1)/2; CUBE — проекция на грань куба по доминирующей оси нормали;
	   TUBE/SPHERE — цилиндрическая/сферическая. Затем texco_mapping:
	   t=size*(t-0.5)+ofs+0.5. Для UV (BI масштабирует uv в [-1,1] и FLAT
	   тождественно) просто возвращаем uv. Неподдержанные источники сведены к UV. */
	"vec2 gpu_texuv(vec2 uv, vec3 orco, float is_orco, vec2 size, vec2 ofs, float map_type, vec3 n){\n"
	"  if (is_orco > 0.5) {\n"
	"    vec2 t;\n"
	"    if (map_type < 0.5) {\n"        /* FLAT */
	"      t = (orco.xy + 1.0) * 0.5;\n"
	"    } else if (map_type < 1.5) {\n" /* CUBE */
	"      float ax = abs(n.x), ay = abs(n.y), az = abs(n.z);\n"
	"      if (az >= ax && az >= ay) t = (orco.xy + 1.0) * 0.5;\n"
	"      else if (ay >= ax && ay >= az) t = vec2((orco.x + 1.0) * 0.5, (orco.z + 1.0) * 0.5);\n"
	"      else t = vec2((orco.y + 1.0) * 0.5, (orco.z + 1.0) * 0.5);\n"
	"    } else if (map_type < 2.5) {\n" /* TUBE */
	"      float len = length(orco.xy);\n"
	"      if (len > 0.0) t = vec2((1.0 - atan(orco.x / len, orco.y / len) / 3.14159) * 0.5, (orco.z + 1.0) * 0.5);\n"
	"      else t = vec2(0.0);\n"
	"    } else {\n"                     /* SPHERE */
	"      float len = length(orco);\n"
	"      if (len > 0.0) {\n"
	"        float u = (abs(orco.x) < 1e-6 && abs(orco.y) < 1e-6) ? 0.0 : (1.0 - atan(orco.x, orco.y) / 3.14159) * 0.5;\n"
	"        float v = 1.0 - acos(orco.z / len) / 3.14159;\n"
	"        t = vec2(u, v);\n"
	"      } else t = vec2(0.0);\n"
	"    }\n"
	"    return size * (t - 0.5) + ofs + 0.5;\n"
	"  }\n"
	"  return uv;\n"
	"}\n"
	/* ★ M19: процедурная карта цвета (CLOUDS/BLEND). Таблицы orgBlenderNoise
	   (noise.c: hash[512], hashvectf[768]) приходят ТЕКСТУРАМИ, выборка
	   индексная -> фильтр GL_NEAREST. */
	"uniform sampler2D u_proc_hash;\n"    /* 512x1: hash[i]/255 */
	"uniform sampler2D u_proc_hashv;\n"   /* 256x1: (grad+1)/2 */
	"uniform sampler2D u_proc_ramp;\n"    /* 256x1: запечённый колорбэнд */
	"uniform sampler2D u_proc_hashp;\n"   /* 256x1 RGB8: Voronoi feature points (hashpntf) */
	"uniform int u_proc_type;\n"          /* 0 нет, 1 CLOUDS, 5 BLEND */
	"uniform float u_proc_col;\n"         /* M20: 1 = текстура даёт MAP_COL */
	"uniform int u_proc_stype;\n"
	"uniform int u_proc_flip;\n"
	"uniform float u_proc_noisesize;\n"
	"uniform int u_proc_noisedepth;\n"
	"uniform int u_proc_basis;\n"
	"uniform int u_proc_wf;\n"           /* ★ M31: waveform WOOD: 0=SIN,1=SAW,2=TRI */
	"uniform float u_proc_dist;\n"       /* ★ M31: DISTORTED_NOISE tex->dist_amount */
	"uniform int u_proc_hard;\n"
	"uniform float u_proc_bright;\n"
	"uniform float u_proc_contrast;\n"
	"uniform int u_proc_noclamp;\n"
	"uniform int u_proc_has_ramp;\n"
	"uniform int u_proc_orco;\n"
	"uniform vec2 u_proc_size;\n"
	"uniform vec2 u_proc_ofs;\n"
	"float gpu_p_hash(int i){ return texture2D(u_proc_hash, vec2((float(i)+0.5)/512.0, 0.5)).r*255.0; }\n"
	"vec3 gpu_p_hashv(int i){ return texture2D(u_proc_hashv, vec2((float(i)+0.5)/256.0, 0.5)).rgb*2.0-1.0; }\n"
	"float gpu_p_grad(int i, float x, float y, float z){\n"
	"  vec3 g = gpu_p_hashv(i);\n"
	"  return g.x*x + g.y*y + g.z*z;\n"
	"}\n"
	/* GLSL 1.10 (шейдеры без #version) НЕ имеет битовых операций, поэтому
	   `v & 255` из noise.c выражено через floor — результат совпадает
	   (для отрицательных v это неотрицательный остаток по модулю 256). */
	"int gpu_p_mod256(int v){ return v - 256*int(floor(float(v)/256.0)); }\n"
	/* M20: Voronoi (noise.c voronoi + HASHPNT). hashpntf -> u_proc_hashp (256x1 RGB8),
	   hash[] -> u_proc_hash. Индекс: hash[hash[hash[z&255]+y)&255]+x)&255]. */
	"int gpu_p_hashpnt_idx(int x, int y, int z){\n"
	"  int h0 = int(gpu_p_hash(gpu_p_mod256(z)) + 0.5);\n"
	"  int h1 = int(gpu_p_hash(gpu_p_mod256(h0 + y)) + 0.5);\n"
	"  int h2 = int(gpu_p_hash(gpu_p_mod256(h1 + x)) + 0.5);\n"
	"  return h2;\n"
	"}\n"
	"vec3 gpu_p_hashp(int i){ return texture2D(u_proc_hashp, vec2((float(i)+0.5)/256.0, 0.5)).rgb; }\n"
	/* Возвращает (F1, F2) — два ближайших расстояния Voronoi (dist_Real, dtype=0). */
	"vec4 gpu_p_voronoi4(float x, float y, float z){\n"
	"  int xi=int(floor(x)), yi=int(floor(y)), zi=int(floor(z));\n"
	"  float d0=1e10, d1=1e10, d2=1e10, d3=1e10;\n"
	"  for (int ox=-1; ox<=1; ox++) {\n"
	"    for (int oy=-1; oy<=1; oy++) {\n"
	"      for (int oz=-1; oz<=1; oz++) {\n"
	"        vec3 p = gpu_p_hashp(gpu_p_hashpnt_idx(xi+ox, yi+oy, zi+oz));\n"
	"        float xd = x - (p.x + float(xi+ox));\n"
	"        float yd = y - (p.y + float(yi+oy));\n"
	"        float zd = z - (p.z + float(zi+oz));\n"
	"        float d = sqrt(xd*xd + yd*yd + zd*zd);\n"
	"        if (d < d0) { d3 = d2; d2 = d1; d1 = d0; d0 = d; }\n"
	"        else if (d < d1) { d3 = d2; d2 = d1; d1 = d; }\n"
	"        else if (d < d2) { d3 = d2; d2 = d; }\n"
	"        else if (d < d3) { d3 = d; }\n"
	"      }\n"
	"    }\n"
	"  }\n"
	"  return vec4(d0, d1, d2, d3);\n"
	"}\n"
	/* orgBlenderNoise (noise.c:315) — 1:1; hash[]/hashvectf[] берутся из текстур. */
	"float gpu_p_orgnoise(float x, float y, float z){\n"
	"  float fx=floor(x), fy=floor(y), fz=floor(z);\n"
	"  float ox=x-fx, oy=y-fy, oz=z-fz;\n"
	"  float jx=ox-1.0, jy=oy-1.0, jz=oz-1.0;\n"
	"  float cn1=ox*ox, cn2=oy*oy, cn3=oz*oz, cn4=jx*jx, cn5=jy*jy, cn6=jz*jz;\n"
	"  cn1 = 1.0 - 3.0*cn1 + 2.0*cn1*ox;\n"
	"  cn2 = 1.0 - 3.0*cn2 + 2.0*cn2*oy;\n"
	"  cn3 = 1.0 - 3.0*cn3 + 2.0*cn3*oz;\n"
	"  cn4 = 1.0 - 3.0*cn4 - 2.0*cn4*jx;\n"
	"  cn5 = 1.0 - 3.0*cn5 - 2.0*cn5*jy;\n"
	"  cn6 = 1.0 - 3.0*cn6 - 2.0*cn6*jz;\n"
	"  int ix=gpu_p_mod256(int(fx)), iy=gpu_p_mod256(int(fy)), iz=gpu_p_mod256(int(fz));\n"
	"  int ix1=gpu_p_mod256(int(fx)+1), iy1=gpu_p_mod256(int(fy)+1), iz1=gpu_p_mod256(int(fz)+1);\n"
	"  int b00 = int(gpu_p_hash(int(gpu_p_hash(ix) + 0.5) + iy) + 0.5);\n"
	"  int b10 = int(gpu_p_hash(int(gpu_p_hash(ix1) + 0.5) + iy) + 0.5);\n"
	"  int b01 = int(gpu_p_hash(int(gpu_p_hash(ix) + 0.5) + iy1) + 0.5);\n"
	"  int b11 = int(gpu_p_hash(int(gpu_p_hash(ix1) + 0.5) + iy1) + 0.5);\n"
	"  int b20 = iz;\n"
	"  int b21 = iz1;\n"
	"  float n = 0.5;\n"
	"  n += (cn1*cn2*cn3) * gpu_p_grad(int(gpu_p_hash(b20+b00)+0.5), ox, oy, oz);\n"
	"  n += (cn1*cn2*cn6) * gpu_p_grad(int(gpu_p_hash(b21+b00)+0.5), ox, oy, jz);\n"
	"  n += (cn1*cn5*cn3) * gpu_p_grad(int(gpu_p_hash(b20+b01)+0.5), ox, jy, oz);\n"
	"  n += (cn1*cn5*cn6) * gpu_p_grad(int(gpu_p_hash(b21+b01)+0.5), ox, jy, jz);\n"
	"  n += (cn4*cn2*cn3) * gpu_p_grad(int(gpu_p_hash(b20+b10)+0.5), jx, oy, oz);\n"
	"  n += (cn4*cn2*cn6) * gpu_p_grad(int(gpu_p_hash(b21+b10)+0.5), jx, oy, jz);\n"
	"  n += (cn4*cn5*cn3) * gpu_p_grad(int(gpu_p_hash(b20+b11)+0.5), jx, jy, oz);\n"
	"  n += (cn4*cn5*cn6) * gpu_p_grad(int(gpu_p_hash(b21+b11)+0.5), jx, jy, jz);\n"
	"  return clamp(n, 0.0, 1.0);\n"
	"}\n"
	/* BLI_gTurbulence (noise.c:1489). Реализован только basis 0 (BLENDER_ORIGINAL
	   = orgBlenderNoise) — он и встречается в сценах (disk). */
	"float gpu_p_turb(float nsize, float x, float y, float z, int oct, int hard, int basis){\n"
	"  if (basis == 0) { x += 1.0; y += 1.0; z += 1.0; }\n"
	"  if (nsize != 0.0) { float s = 1.0/nsize; x *= s; y *= s; z *= s; }\n"
	"  float sum = 0.0, amp = 1.0, fs = 1.0;\n"
	"  for (int i = 0; i < 8; i++) {\n"
	"    if (i > oct) break;\n"
	"    float t;\n"
	"    vec4 vv = vec4(0.0);\n"
	"    if (basis == 8 || basis == 3 || basis == 4 || basis == 5 || basis == 6 || basis == 7)\n"
	"      vv = gpu_p_voronoi4(fs*x, fs*y, fs*z);\n"
	"    if (basis == 8) t = clamp(10.0*(vv.y - vv.x), 0.0, 1.0);\n"
	"    else if (basis == 3) t = vv.x;\n"
	"    else if (basis == 4) t = vv.y;\n"
	"    else if (basis == 5) t = vv.z;\n"
	"    else if (basis == 6) t = vv.w;\n"
	"    else if (basis == 7) t = vv.y - vv.x;\n"
	"    else t = gpu_p_orgnoise(fs*x, fs*y, fs*z);\n"
	"    if (hard == 1) t = abs(2.0*t - 1.0);\n"
	"    sum += t*amp;\n"
	"    amp *= 0.5; fs *= 2.0;\n"
	"  }\n"
	"  sum *= pow(2.0, float(oct)) / (pow(2.0, float(oct+1)) - 1.0);\n"
	"  return sum;\n"
	"}\n"
	/* blend() (render_texture.c:184): stype 0=LIN,1=QUAD,2=EASE,3=DIAG,6=RAD,4/5=SPHERE/HALO */
	"float gpu_p_blend(vec3 tv, int stype, int flip){\n"
	"  float x, y, t, s;\n"
	"  if (flip == 1) { x = tv.y; y = tv.x; } else { x = tv.x; y = tv.y; }\n"
	"  if (stype == 0) return (1.0+x)/2.0;\n"
	"  if (stype == 1) { t = (1.0+x)/2.0; if (t < 0.0) t = 0.0; return t*t; }\n"
	"  if (stype == 2) { t = (1.0+x)/2.0;\n"
	"    if (t <= 0.0) return 0.0;\n"
	"    if (t >= 1.0) return 1.0;\n"
	"    s = t*t; return 3.0*s - 2.0*s*t; }\n"
	"  if (stype == 3) return (2.0+x+y)/4.0;\n"
	"  if (stype == 6) return atan(y, x)/6.28318530718 + 0.5;\n"
	"  t = 1.0 - sqrt(x*x + y*y + tv.z*tv.z);\n"
	"  if (t < 0.0) t = 0.0;\n"
	"  if (stype == 5) t *= t;\n"
	"  return t;\n"
	"}\n"
	/* Весь слот: texco -> texfunc -> BRICONT -> колорбэнд -> (rgb=tcol, a=tin). */
	"/* ★ M31: signed noise bases for mg_VLNoise (noise.c:1290 voronoi_F*S). */\n"
	"float gpu_p_ns(float x, float y, float z, int b){\n"
	"  if (b == 3) return 2.0*gpu_p_voronoi4(x,y,z).x - 1.0;\n"
	"  if (b == 4) return 2.0*gpu_p_voronoi4(x,y,z).y - 1.0;\n"
	"  if (b == 5) return 2.0*gpu_p_voronoi4(x,y,z).z - 1.0;\n"
	"  if (b == 6) return 2.0*gpu_p_voronoi4(x,y,z).w - 1.0;\n"
	"  if (b == 7) { vec4 v = gpu_p_voronoi4(x,y,z); return 2.0*(v.y - v.x) - 1.0; }\n"
	"  if (b == 8) { vec4 v = gpu_p_voronoi4(x,y,z); float t = min(10.0*(v.y - v.x), 1.0); return 2.0*t - 1.0; }\n"
	"  return 2.0*gpu_p_orgnoise(x, y, z) - 1.0;\n"
	"}\n"
	"/* ★ M31: DISTORTED_NOISE (noise.c:1926 mg_VLNoise / render_texture.c:610 mg_distNoiseTex).\n"
	"   mapFull: dist=1.0, distortion basis = VORONOI_F4 (tex->noisebasis=6), base = BLENDER_ORIGINAL (noisebasis2=0). */\n"
	"float gpu_p_distnoise(vec3 tv, float nsize, float dist, int b1, int b2){\n"
	"  if (nsize != 0.0) { float s = 1.0/nsize; tv *= s; }\n"   /* BI: tmpvec = texvec/noisesize */
	"  float r0 = gpu_p_ns(tv.x + 13.5, tv.y + 13.5, tv.z + 13.5, b1) * dist;\n"
	"  float r1 = gpu_p_ns(tv.x, tv.y, tv.z, b1) * dist;\n"
	"  float r2 = gpu_p_ns(tv.x - 13.5, tv.y - 13.5, tv.z - 13.5, b1) * dist;\n"
	"  return gpu_p_ns(tv.x + r0, tv.y + r1, tv.z + r2, b2);\n"
	"}\n"
	"/* ★ M31: wood (render_texture.c wood_int): для BANDS (stype=0) wi = waveform[wf]((x+y+z)*10).\n"
	"   wf = tex->noisebasis2: 0=SIN (0.5+0.5*sin), 1=SAW, 2=TRI. mapFull wood — BANDS+SIN. */\n"
	"float gpu_p_wood(vec3 tv, int wf){\n"
	"  float a = (tv.x + tv.y + tv.z) * 10.0;\n"
	"  const float b = 6.28318530718;\n"
	"  if (wf == 1) { int n = int(a/b); a -= n*b; if (a < 0.0) a += b; return a/b; }\n"
	"  if (wf == 2) { return 1.0 - 2.0*abs(floor(a/b + 0.5) - a/b); }\n"
	"  return 0.5 + 0.5*sin(a);\n"
	"}\n"
	"vec4 gpu_p_tex_cfg(vec2 uv, vec3 orco, vec3 wpos, vec3 vpos, int porco, vec2 psize, vec2 pofs, int ptype,\n"
	"                   int pstype, int pflip, float pnsize, int pdepth, int phard, int pbasis, int pwf, float pdist,\n"
	"                   float pcontrast, float pbright, int pnoclamp, int phasramp, sampler2D pramp){\n"
	"  vec3 co = (porco == 1) ? orco : ((porco == 2) ? wpos : ((porco == 3) ? vpos : vec3(uv, 0.0)));\n"
	"  vec3 tv = vec3(psize.x*(co.x + pofs.x), psize.y*(co.y + pofs.y), co.z);\n"
	"  float tin;\n"
	"  if (ptype == 5) tin = gpu_p_blend(tv, pstype, pflip);\n"
	"  else if (ptype == 2) tin = gpu_p_wood(tv, pwf);\n"
	"  else if (ptype == 13) tin = gpu_p_distnoise(tv, pnsize, pdist, pbasis, pwf);\n"
	"  else tin = gpu_p_turb(pnsize, tv.x, tv.y, tv.z, pdepth, phard, pbasis);\n"
	"  tin = (tin - 0.5)*pcontrast + pbright - 0.5;\n"   /* BRICONT */
	"  if (pnoclamp == 0) tin = clamp(tin, 0.0, 1.0);\n"
	"  vec3 col = vec3(tin);\n"
	"  if (phasramp == 1)\n"
	"    col = texture2D(pramp, vec2((clamp(tin, 0.0, 1.0)*255.0 + 0.5)/256.0, 0.5)).rgb;\n"
	"  return vec4(col, clamp(tin, 0.0, 1.0));\n"
	"}\n"
	"vec4 gpu_p_tex(vec2 uv, vec3 orco, vec3 wpos, vec3 vpos){\n"
	"  return gpu_p_tex_cfg(uv, orco, wpos, vpos, u_proc_orco, u_proc_size, u_proc_ofs, u_proc_type,\n"
	"                       u_proc_stype, u_proc_flip, u_proc_noisesize, u_proc_noisedepth,\n"
	"                       u_proc_hard, u_proc_basis, u_proc_wf, u_proc_dist, u_proc_contrast, u_proc_bright,\n"
	"                       u_proc_noclamp, u_proc_has_ramp, u_proc_ramp);\n"
	"}\n"
	/* ★ M22: вторая процедурная текстура (слот != 0) — источник MAP_SPEC/MAP_HAR.
	   Таблицы шума (u_proc_hash/hashv/hashp) общие с первой, свой только колорбэнд.
	   Объявления стоят здесь, у точки использования — GLSL это разрешает. */
	"uniform sampler2D u_proc2_ramp;\n"
	"uniform int u_proc2_type;\n"
	"uniform int u_proc2_stype;\n"
	"uniform int u_proc2_flip;\n"
	"uniform float u_proc2_noisesize;\n"
	"uniform int u_proc2_noisedepth;\n"
	"uniform int u_proc2_basis;\n"
	"uniform int u_proc2_wf;\n"
	"uniform float u_proc2_dist;\n"
	"uniform int u_proc2_hard;\n"
	"uniform float u_proc2_bright;\n"
	"uniform float u_proc2_contrast;\n"
	"uniform int u_proc2_noclamp;\n"
	"uniform int u_proc2_has_ramp;\n"
	"uniform int u_proc2_orco;\n"
	"uniform vec2 u_proc2_size;\n"
	"uniform vec2 u_proc2_ofs;\n"
	"uniform float u_has_proc2;\n"
	"vec4 gpu_p_tex2(vec2 uv, vec3 orco, vec3 wpos, vec3 vpos){\n"
	"  return gpu_p_tex_cfg(uv, orco, wpos, vpos, u_proc2_orco, u_proc2_size, u_proc2_ofs, u_proc2_type,\n"
	"                       u_proc2_stype, u_proc2_flip, u_proc2_noisesize, u_proc2_noisedepth,\n"
	"                       u_proc2_hard, u_proc2_basis, u_proc2_wf, u_proc2_dist, u_proc2_contrast, u_proc2_bright,\n"
	"                       u_proc2_noclamp, u_proc2_has_ramp, u_proc2_ramp);\n"
	"}\n"
	/* ★ M23: MAP_NORM от ПРОЦЕДУРНОЙ карты (bump).
	   BI считает его конечными разностями по ортобазису вокруг нормали с
	   bf = -0.04*Tnor*mtex->norfac (compatible_bump_compute, render_texture.c).
	   Здесь тот же смысл и тот же коэффициент, но градиент берётся через
	   экранные производные (surface gradient, Mikkelsen) — он не требует ни
	   UV-параметризации, ни imat_ren и потому корректен для ORCO.
	   Разложение BI: nvec = N + ud*nu + vd*nv, ud = 0.04*norfac*dh/dnu, т.е.
	   n' = normalize(N + 0.04*norfac*surfGrad(h)). Знак задаётся norfac. */
	"uniform float u_has_proc_nrm;\n"   /* 1 = процедурная карта даёт MAP_NORM */
	"uniform int u_proc_nrm_src;\n"     /* 0 = слот 0 (u_proc_*), 1 = вторая (u_proc2_*) */
	"uniform float u_proc_norfac;\n"    /* mtex->norfac (-2..2) */
	"vec3 gpu_bump(vec3 N, vec3 P, float h, float norfac){\n"
	"  vec3 dpdx = dFdx(P);\n"
	"  vec3 dpdy = dFdy(P);\n"
	"  vec3 r1 = cross(dpdy, N);\n"
	"  vec3 r2 = cross(N, dpdx);\n"
	"  float det = dot(dpdx, r1);\n"
	"  if (abs(det) < 1e-9) return N;\n"
	"  vec3 sg = (r1 * dFdx(h) + r2 * dFdy(h)) / det;\n"
	"  return normalize(N + (0.04 * norfac) * sg);\n"
	"}\n"
	"void main(){\n"
	"  /* Screen-door прозрачность. В деферред-пайплайне G-буфер пишется без\n"
	"     блендинга, а канал albedo.a занят под интенсивность блика, поэтому\n"
	"     прозрачность делаем отбрасыванием доли фрагментов (1-alpha).\n"
	"     Семя включает v_linz, иначе два слоя на разной глубине выбрасывали бы\n"
	"     одни и те же пиксели и вместо просвета получались бы дыры.\n"
	"     Сама alpha считается ниже (материал + альфа-карта, M10). */\n"
	"  vec3 base = v_rgba.rgb;\n"
	"  vec4 tc = vec4(1.0);\n"
	"  if (u_has_tex > 0.5) {\n"
	"    if (u_proc_type != 0)\n"
	"      tc = gpu_p_tex(v_uv_tex, v_orco, v_wpos, v_vpos);\n"
	"    else\n"
	"      tc = texture2D(u_tex, gpu_texuv(v_uv_tex, (u_tex_glob > 0.5) ? v_wpos : v_orco, u_tex_orco, u_tex_size, u_tex_ofs, u_tex_map, v_w_nrm));\n"
	"    if (u_proc_col > 0.5)\n"
	"      base = gpu_texblend(base, tc.rgb,\n"
	"                          gpu_tex_tin(tc, u_tex_rgbtoint, (u_proc_type != 0) ? 1.0 : 0.0),\n"
	"                          u_colfac, u_tex_blend);\n"
	"    /* M14: цепочка дополнительных диффузных слотов (как в BI: подряд texture_rgb_blend) */\n"
	"    if (u_n_tex >= 2) {\n"
	"      vec4 t1 = texture2D(u_tex1, gpu_texuv(v_uv_tex, (u_tex1_glob > 0.5) ? v_wpos : v_orco, u_tex1_orco, u_tex1_size, u_tex1_ofs, u_tex1_map, v_w_nrm));\n"
	"      base = gpu_texblend(base, t1.rgb, gpu_tex_tin(t1, u_tex1_rgbtoint, 0.0), u_tex1_fac, u_tex1_blend);\n"
	"      if (u_n_tex >= 3) {\n"
	"        vec4 t2 = texture2D(u_tex2, gpu_texuv(v_uv_tex, (u_tex2_glob > 0.5) ? v_wpos : v_orco, u_tex2_orco, u_tex2_size, u_tex2_ofs, u_tex2_map, v_w_nrm));\n"
	"        base = gpu_texblend(base, t2.rgb, gpu_tex_tin(t2, u_tex2_rgbtoint, 0.0), u_tex2_fac, u_tex2_blend);\n"
	"        if (u_n_tex >= 4) {\n"
	"          vec4 t3 = texture2D(u_tex3, gpu_texuv(v_uv_tex, (u_tex3_glob > 0.5) ? v_wpos : v_orco, u_tex3_orco, u_tex3_size, u_tex3_ofs, u_tex3_map, v_w_nrm));\n"
	"          base = gpu_texblend(base, t3.rgb, gpu_tex_tin(t3, u_tex3_rgbtoint, 0.0), u_tex3_fac, u_tex3_blend);\n"
	"        }\n"
	"      }\n"
	"    }\n"
	"  }\n"
	"  /* ★ M12: MAP_SPEC/MAP_HAR. tin — как для альфы: альфа картинки (Use Alpha)\n"
	"     либо её яркость. spec интенсивность (albedo.a) и hardness (4-я цель)\n"
	"     модулируются texture_value_blend ровно как BI (render_texture.c:2722,2741). */\n"
	"  float sv_tin = (u_sv_use_a > 0.5) ? tc.a\n"
	"              : dot(tc.rgb, vec3(0.2126, 0.7152, 0.0722));\n"
	/* ★ M22: вторая процедурная текстура (слот != 0). Её значение — как tin
	   для texture_value_blend в BI (процедурная: tin = значение, не альфа). */
	"  float sv_tin2 = sv_tin;\n"
	"  if (u_has_proc2 > 0.5) {\n"
	"    vec4 tc2 = gpu_p_tex2(v_uv_tex, v_orco, v_wpos, v_vpos);\n"
	"    sv_tin2 = tc2.a;\n"
	"  }\n"
	"  float spec_int = v_rgba.a;\n"
	"  if (u_has_spec > 0.5) {\n"
	"    spec_int = max(gpu_valblend(u_spec_def, spec_int, sv_tin2, u_spec_fac, u_spec_blend), 0.0);\n"
	"  }\n"
	"  /* ★ M10: эффективная прозрачность. u_alpha — alpha материала (ma->alpha);\n"
	"     при наличии альфа-карты он модулируется ею ровно как в BI:\n"
	"     shi->alpha = texture_value_blend(def_var, alpha, tin, alphafac, bt).\n"
	"     Это спасает материалы с ma->alpha=0: их прозрачность берётся из карты. */\n"
	"  float gpu_alpha = u_alpha;\n"
	"  if (u_has_atex > 0.5) {\n"
	"    vec4 ac = texture2D(u_atex, gpu_texuv(v_uv_tex, (u_a_glob > 0.5) ? v_wpos : v_orco, u_a_orco, u_a_size, u_a_ofs, u_a_map, v_w_nrm));\n"
	"    float tin = (u_a_use_a > 0.5) ? ac.a\n"
	"              : dot(ac.rgb, vec3(0.2126, 0.7152, 0.0722));\n"
	"    gpu_alpha = gpu_valblend(u_a_def, u_alpha, tin, u_a_fac, u_a_blend);\n"
	"  }\n"
	"  gpu_alpha = clamp(gpu_alpha, 0.0, 1.0);\n"
	"  if (gpu_alpha < 0.999) {\n"
	"    float h = gpu_bayer8(gl_FragCoord.xy) / 64.0;\n"
	"    if (h >= gpu_alpha) discard;\n"
	"  }\n"
	"  /* M8: возмущение нормали картой нормалей (TANGENT-space). T ортонормируем\n"
	"     против геометрической нормали N; B = cross(N,T)*w. Если T вырождена\n"
	"     (нет UV) — откат на касательную, перпендикулярную N. */\n"
	"  vec3 n = v_nrm;\n"
	"  if (u_has_nrm > 0.5) {\n"
	"    if (u_nrm_is_height > 0.5) {\n"
	"      /* M26: BI (render_texture.c:1783) для образа БЕЗ TEX_NORMALMAP считает\n"
	"         height-bump по яркости карты (compatible_bump_compute), а не\n"
	"         тангенциальную нормаль. Карты mapFull *_nor_gl_*.exr как раз без\n"
	"         этого флага — раньше GPU брал их как tangent и пересвечивал/затемнял. */\n"
	"      vec2 ntc = gpu_texuv(v_uv_tex, (u_nrm_glob > 0.5) ? v_wpos : v_orco, u_nrm_orco, u_nrm_size, u_nrm_ofs, u_nrm_map, v_w_nrm);\n"
	"      vec3 h3 = texture2D(u_nrm, ntc).rgb;\n"
	"      /* M30: BI берёт высоту как (tr+tg+tb)/3 (render_texture.c:1886), НЕ яркость;\n"
	"         и нормирует градиент texco-шагом du/dv (idu=bf/du). У нас же gpu_bump\n"
	"         даёт ФИЗИЧЕСКИЙ градиент (1/м) — поэтому домножаем на dphys/dtexco\n"
	"         (pp/fp), иначе bump слабеет ~во столько раз, во сколько тексель крупнее\n"
	"         пикселя (дальние стены/пол почти без рельефа). */\n"
	"      float hb = (h3.r + h3.g + h3.b) * 0.3333333;\n"
	"      n = gpu_bump(n, v_vpos, hb, u_nrm_norfac * u_bump_scale);\n"
	"    } else {\n"
	"      vec3 nt = texture2D(u_nrm, gpu_texuv(v_uv_tex, (u_nrm_glob > 0.5) ? v_wpos : v_orco, u_nrm_orco, u_nrm_size, u_nrm_ofs, u_nrm_map, v_w_nrm)).xyz * 2.0 - 1.0;\n"
	"      vec3 N = normalize(n);\n"
	"      vec3 T = v_tangent - dot(v_tangent, N) * N;\n"
	"      float tl = length(T);\n"
	"      if (tl < 1e-6) {\n"
	"        vec3 up = (abs(N.y) < 0.99) ? vec3(0.0, 1.0, 0.0) : vec3(1.0, 0.0, 0.0);\n"
	"        T = normalize(cross(N, up));\n"
	"      } else T /= tl;\n"
	"      vec3 B = cross(N, T) * v_tang_sign;\n"
	"      n = normalize(T * nt.x + B * nt.y + N * nt.z);\n"
	"    }\n"
	"  }\n"
	"  /* ★ M23: bump от процедурной карты (MAP_NORM). Применяется после образной\n"
	"   карты нормалей, чтобы модифицировать уже возмущённую нормаль.\n"
	"   Условие — uniform, поэтому производные берутся в однородном потоке. */\n"
	"  if (u_has_proc_nrm > 0.5) {\n"
	"    float hb = (u_proc_nrm_src == 1) ? gpu_p_tex2(v_uv_tex, v_orco, v_wpos, v_vpos).a\n"
	"                                    : gpu_p_tex(v_uv_tex, v_orco, v_wpos, v_vpos).a;\n"
	"    n = gpu_bump(n, v_vpos, hb, u_proc_norfac * u_bump_scale);\n"
	"  }\n"
	"  /* ★ M25 (раунд 82): множитель shi->refl = ma->ref применяем ЗДЕСЬ, к albedo,\n"
	"     ровно как BI применяет его к shi->rcol перед светом. Тогда и диффуз\n"
	"     (alb.rgb*diff), и ambient (alb.rgb*amb) получают его автоматически —\n"
	"     как в BI (shadeoutput.c:1455 и :1099). Альфа (интенсивность блика) НЕ\n"
	"     масштабируется: блик в BI от refl не зависит. */\n"
	"  gl_FragData[0] = vec4(base * u_refl, spec_int);\n"
	"  gl_FragData[1] = vec4(n * 0.5 + 0.5, v_emit);\n"   /* a = эмиссия (M5) */
	"  float d = clamp(v_linz * u_pack, 0.0, 0.99999);\n"
	"  gl_FragData[2] = vec4(gpu_pack_depth(d), 1.0);\n"   /* a=1 -> маска геометрии */
	/* ★ M5: цвет блика (rgb) и hardness (a) — 4-я цель MRT. Пишется только если
	   карта её поддерживает (GL_MAX_COLOR_ATTACHMENTS_EXT >= 4); иначе GB_FS
	   компилируется без этого блока, а u_spec_col/u_hard оптимизируются вон. */
	"#ifdef GPU_SPEC_TARGET\n"
	"  float hard511 = u_hard;\n"
	"  if (u_has_har > 0.5) {\n"
	"    float har128 = hard511 * 511.0 / 128.0;\n"   /* BI: har/128 */
	"    har128 = gpu_valblend(u_har_def, har128, sv_tin2, u_har_fac, u_har_blend);\n"
	"    hard511 = floor(clamp(har128 * 128.0, 1.0, 511.0)) / 511.0;\n"   /* BI: har=(int)har */
	"  }\n"
	"  /* M13: MAP_COLSPEC — цвет блика модулируется текстурой (texture_rgb_blend,\n"
	"     tin = tc.a = texres.ta). */\n"
	"  vec3 spec_col = u_spec_col;\n"
	"  if (u_has_colspec > 0.5) {\n"
	"    spec_col = gpu_texblend(spec_col, tc.rgb, tc.a, u_colspec_fac, u_colspec_blend);\n"
	"  }\n"
	"  gl_FragData[3] = vec4(spec_col, hard511);\n"
	"#endif\n"
	/* ★ M17: MAP_REFL — сила отражения (R8, 5-я цель MRT). BI: shi->refl стартует
	   из ma->ref и модулируется texture_value_blend (render_texture.c:2719). */
	"#ifdef GPU_REFL_TARGET\n"
	"  float refl_val = u_refl_base;\n"
	"  if (u_has_refl > 0.5) {\n"
	"    refl_val = gpu_valblend(u_refl_def, refl_val, sv_tin, u_refl_fac, u_refl_blend);\n"
	"    refl_val = max(refl_val, 0.0);\n"
	"  }\n"
	"  gl_FragData[4] = vec4(refl_val);\n"
	"#endif\n"
	"}\n";

/* --- shadow map pass: глубина из perspective света (M3) --- */
static const char *SHADOW_VS =
	"attribute vec3 a_co;\n"
	"uniform mat4 u_lightvp;\n"   /* light_proj * light_view */
	"uniform mat4 u_obmat;\n"     /* object->world */
	"void main(){\n"
	"  gl_Position = u_lightvp * u_obmat * vec4(a_co, 1.0);\n"
	"}\n";
/* Пишем НЕ в depth-буфер, а в color-аттачмент (RGBA8): rgb = упакованная window-глубина,
 * a = 1 (есть окклюдер). Depth-текстура на MRT-FBO не принимала запись глубины,
 * поэтому и здесь используем переносимую схему с упаковкой. */
static const char *SHADOW_FS =
	"vec3 gpu_pack_depth(float v){\n"
	"  vec3 enc = fract(vec3(1.0, 255.0, 65025.0) * v);\n"
	"  enc -= enc.yzz * vec3(1.0/255.0, 1.0/255.0, 0.0);\n"
	"  return enc;\n"
	"}\n"
	"void main(){\n"
	"  gl_FragData[0] = vec4(gpu_pack_depth(clamp(gl_FragCoord.z, 0.0, 0.99999)), 1.0);\n"
	"}\n";

/* --- полноэкранный quad vertex shader (для деферред-освещения) --- */
static const char *LIGHT_VS =
	"attribute vec2 a_pos;\n"
	"attribute vec2 a_uv;\n"
	"varying vec2 v_uv;\n"
	"varying vec2 v_ndc;\n"
	"void main(){\n"
	"  v_uv = a_uv;\n"
	"  v_ndc = a_pos;\n"
	"  gl_Position = vec4(a_pos, 0.0, 1.0);\n"
	"}\n";

/* --- деферред-освещение: ambient + направленный diffuse + тень (M2+M3) --- */
static const char *LIGHT_FS =
	"uniform sampler2D u_albedo;\n"
	"uniform sampler2D u_nrm;\n"
	"uniform sampler2D u_depth;\n"
	"uniform sampler2D u_shadow[10];\n"  /* теневая карта каждой лампы (юнит 6+i) */
	"uniform samplerCube u_shadow_cube[10];\n" /* POINT: кубическая карта лампы */
	"uniform vec3 u_lwpos[10];\n"       /* POINT: мировая позиция света (центр куба) */
	"uniform vec2 u_lpz[10];\n"         /* POINT: (A,B) пересчёта window-глубины */
	"uniform sampler2D u_spec;\n"      /* M5: rgb = цвет блика, a = hardness/511 */
	"uniform sampler2D u_refl;\n"      /* M17: сила отражения (r), 5-я цель MRT */
	"uniform vec2 u_shadow_texel;\n"   /* 1/размер shadow map — шаг выборок PCF */
	"uniform float u_shadow_bias;\n"   /* раунд 83: множитель мирового bias (UBF_GPU_SHADOW_BIAS) */
	"uniform float u_pcf;\n"          /* 1 = PCF 3x3, 0 = одиночная выборка */
	"uniform float u_exposure;\n"
	"/* ★ Лампы. Собираются в gpu_gather_lights; для каждой — те же величины, что\n"
	"   BI кладёт в LampRen, чтобы затухание считалось как в lamp_get_visibility(). */\n"
	"uniform int u_num_lights;\n"
	"/* ★★ Раунд 16: данные ламп — float-текстурой, а не per-light юниформами.\n"
	"   GPU_LIGHT_DATA_ROWS текселей RGBA32F на лампу:\n"
	"     0: pos.xyz, type   1: dir.xyz, dist   2: col.rgb, falloff\n"
	"     3: sphere, spotsi, spotbl, shadows      4: soft\n"
	"   Так потолок «10 ламп» (лимит uniform-компонентов) снят: число ламп задаёт\n"
	"   ширина текстуры, а цикл по лампам — динамический. */\n"
	"uniform sampler2D u_light_data;\n"
	"uniform vec2 u_light_texel;\n"   /* (1/ширина, 1/GPU_LIGHT_DATA_ROWS) */
	"vec4 gpu_ltap(int li, int row){\n"
	"  return texture2D(u_light_data, vec2((float(li) + 0.5) * u_light_texel.x,\n"
	"                                      (float(row) + 0.5) * u_light_texel.y));\n"
	"}\n"
	"/* ★ Мир сцены (re->wrld). Фон — как в BI pixelshading.c:508-545; ambient —\n"
	"   BI берёт wrld.ambient_color*ma->amb (shadeoutput.c:2000), но у наблюдаемых\n"
	"   сцен он нулевой, поэтому дополнительно остаётся встроенный fill. */\n"
	"uniform vec3 u_world_hor;\n"    /* horizon_color */
	"uniform vec3 u_world_zen;\n"    /* zenith_color */
	"uniform vec3 u_world_amb;\n"    /* ambient_color */
	"uniform float u_world_blend;\n" /* WO_SKYBLEND: фон = mix(hor, zen, blend) */
	"uniform float u_amb_fill;\n"    /* множитель встроенного fill (UBF_GPU_AMB_FILL) */
	"uniform float u_env_light;\n"   /* M26: 1 — Environment Lighting мира включён */
	"uniform float u_env_energy;\n"  /* M26: wrld->ao_env_energy */
	"uniform float u_linfac;\n"      /* world Exposure/Range — BI shadeoutput.c:1745 */
	"uniform float u_logfac;\n"
	"uniform float u_expo_on;\n"     /* 1, если exp!=0 || range!=1 (иначе BI не трогает) */
	"uniform mat4 u_inv_proj;\n"     /* clip->view (inverse winmat) */
	"uniform float u_ortho;\n"       /* ★ раунд 83-fix: 1 = ортокамера (winmat[2][3]==0).
	                                    У орто лучи параллельны, поэтому dir — это уже
	                                    готовая view-позиция по XY, а не направление луча. */
	"uniform mat4 u_inv_view;\n"     /* view->world (inverse viewmat) */
	"uniform mat4 u_proj;\n"         /* view->clip (winmat) — для проекции шагов SSR */
	"uniform mat4 u_lvp[10];\n"       /* world->light-clip каждой лампы (для её тени) */
	"/* ★ раунд 16: u_lsoft[10] убран — soft едет в строке 4 u_light_data */\n"
	"uniform int u_debug;\n"         /* 0=свет, 1=albedo, 2=нормаль, 3=упакованная глубина */
	"uniform float u_pack;\n"        /* 1/clipend — та же нормировка, что в G-буфере */
	"uniform vec3 u_up;\n"           /* мировой up (0,0,1) в view-space — для полусфер. ambient */
	"uniform float u_ssr;\n"         /* 1 = включить screen-space reflections */
	"uniform vec2 u_texel;\n"        /* (1/width, 1/height) — размер пикселя для SSAO */
	"uniform float u_ao;\n"          /* 1 = включить screen-space ambient occlusion */
	"varying vec2 v_uv;\n"
	"varying vec2 v_ndc;\n"
	"/* ★★ Раунд 82: кадр растеризатора живёт в ДИСПЛЕЙНОМ пространстве, а весь\n"
	"   расчёт света — линейный. Раньше преобразования на выходе НЕ было вовсе, и\n"
	"   кадр уходил в FBO линейным. Замер (мир ровно 0.5, пустая сцена): scanline\n"
	"   даёт PNG 188/255 (sRGB-кодированное 0.5), GPU давал 127/255 (сырое линейное\n"
	"   0.5) — то есть и вьюпорт, и F12 показывали линейный кадр, он выглядел\n"
	"   темнее и «пережареннее» по насыщенности. Теперь кодируем в sRGB на выходе:\n"
	"   вьюпорт рисует кадр в окно как есть (без color management) — значит он и\n"
	"   должен быть дисплейным, а F12 декодирует его обратно в линейный re->result\n"
	"   (RE_gpu_renderer_render_to_result), и Blender штатно применяет view transform\n"
	"   при записи. Оба пути сходятся с scanline. */\n"
	"vec3 gpu_to_srgb(vec3 c){\n"
	"  c = clamp(c, 0.0, 1.0);\n"
	"  vec3 lo = c * 12.92;\n"
	"  vec3 hi = 1.055 * pow(c, vec3(1.0 / 2.4)) - 0.055;\n"
	"  return mix(lo, hi, step(vec3(0.0031308), c));\n"
	"}\n"
	"float gpu_unpack_depth(vec3 c){\n"
	"  return dot(c, vec3(1.0, 1.0/255.0, 1.0/65025.0));\n"
	"}\n"
	"/* SSR: маршируем отражённый луч R по экрану, сравнивая глубину сцены. */\n"
	"vec3 gpu_ssr(vec2 uv, vec3 vpos, vec3 n, vec3 V) {\n"
	"  vec3 R = reflect(-V, n);\n"
	"  /* луч уходит сильно в сторону камеры (за неё) — сцены там нет, отражение не нужно */\n"
	"  if (dot(R, V) > 0.5) return vec3(0.0);\n"
	"  float dist = max(-vpos.z, 0.1);\n"
	"  float t = dist * 0.005;\n"
	"  float step = dist * 0.02;\n"
	"  float tol = max(0.02, dist * 0.01);\n"
	"  for (int i = 0; i < 48; i++) {\n"
	"    vec3 p = vpos + R * t;\n"
	"    vec4 clip = u_proj * vec4(p, 1.0);\n"
	"    if (clip.w <= 0.0) break;\n"
	"    vec2 suv = clip.xy / clip.w * 0.5 + 0.5;\n"
	"    if (suv.x < 0.0 || suv.x > 1.0 || suv.y < 0.0 || suv.y > 1.0) break;\n"
	"    vec4 sd = texture2D(u_depth, suv);\n"
	"    if (sd.a < 0.5) { t += step; continue; }\n"
	"    float scene_linz = gpu_unpack_depth(sd.rgb) / u_pack;\n"
	"    float ray_linz = -p.z;\n"
	"    if (scene_linz < ray_linz - tol && scene_linz > ray_linz - tol * 8.0) {\n"
	"      return texture2D(u_albedo, suv).rgb;\n"
	"    }\n"
	"    t += step;\n"
	"  }\n"
	"  return vec3(0.0);\n"
	"}\n"
	"/* SSAO: 8 образцов вокруг пикселя; образец ближе к камере -> окклюзия. */\n"
	"float gpu_ssao(vec2 uv, vec3 vpos, float linz, vec3 n) {\n"
	"  float occ = 0.0;\n"
	"  for (int i = 0; i < 8; i++) {\n"
	"    float ang = float(i) * 0.785398;\n"
	"    vec2 off = vec2(cos(ang), sin(ang)) * (float(i % 2) * 2.0 + 1.0) * u_texel * 2.0;\n"
	"    vec2 suv = uv + off;\n"
	"    if (suv.x < 0.0 || suv.x > 1.0 || suv.y < 0.0 || suv.y > 1.0) continue;\n"
	"    vec4 sd = texture2D(u_depth, suv);\n"
	"    if (sd.a < 0.5) continue;\n"
	"    float s_linz = gpu_unpack_depth(sd.rgb) / u_pack;\n"
	"    if (s_linz >= linz - 0.02) continue;\n"
	"    vec4 cp = u_inv_proj * vec4(suv * 2.0 - 1.0, -1.0, 1.0);\n"
	"    vec3 dir = cp.xyz / cp.w;\n"
	"    vec3 sp = (u_ortho > 0.5) ? vec3(dir.xy, -s_linz) : dir * (s_linz / max(-dir.z, 1e-6));\n"
	"    vec3 dvec = sp - vpos;\n"
	"    float dist = length(dvec);\n"
	"    if (dist < 0.25 && dist > 1e-4) {\n"
	"      float nd = dot(n, dvec / dist);\n"
	"      if (nd > 0.0) occ += (1.0 - dist / 0.25) * nd;\n"
	"    }\n"
	"  }\n"
	"  return clamp(1.0 - occ * 0.6, 0.0, 1.0);\n"
	"}\n"
	"/* Вклад ОДНОЙ лампы: диффуз + блик, помноженные на затухание.\n"
	"   Затухание повторяет lamp_get_visibility() из BI (shadeoutput.c) для типов\n"
	"   CONSTANT/INVLINEAR/INVSQUARE; у LA_AREA затухания нет — ровно как в BI,\n"
	"   где switch пропускается. shadow применяется только к солнцу (теневая карта\n"
	"   строится по нему одному). */\n"
	"/* ★ spec() из BI ДОСЛОВНО (shadeoutput.c:541) — форма блика.\n"
	"   Это НЕ просто pow(inp,hard): внутри два стража — b1 (это inp^2) снизу\n"
	"   ограничен 0.01, а слишком малые промежуточные степени обнуляются. Именно\n"
	"   они дают блику BI его хвост на скользящих углах при малой hardness:\n"
	"   проверено, что без стражей ошибка доходит до 0.01 (verify_spec_math.py),\n"
	"   а замкнутой формулы со стражами нет (для нечётных hardness выходит\n"
	"   inp * max(inp,0.1)^(hard-1)). Поэтому переносим функцию как есть.\n"
	"   Битовых операций в GLSL 1.20 нет, но есть остаток (%) — им и проверяем. */\n"
	"float gpu_spec(float inp, int hard)\n"
	"{\n"
	"  float b1;\n"
	"  if (inp >= 1.0) return 1.0;\n"
	"  if (inp <= 0.0) return 0.0;\n"
	"  b1 = inp * inp;\n"
	"  if (b1 < 0.01) b1 = 0.01;\n"
	"  if ((hard % 2) == 0) inp = 1.0;\n"
	"  if ((hard % 4) >= 2)     inp *= b1;\n"
	"  b1 *= b1;\n"
	"  if ((hard % 8) >= 4)     inp *= b1;\n"
	"  b1 *= b1;\n"
	"  if ((hard % 16) >= 8)    inp *= b1;\n"
	"  b1 *= b1;\n"
	"  if ((hard % 32) >= 16)   inp *= b1;\n"
	"  b1 *= b1;\n"
	"  if (b1 < 0.001) b1 = 0.0;\n"
	"  if ((hard % 64) >= 32)   inp *= b1;\n"
	"  b1 *= b1;\n"
	"  if ((hard % 128) >= 64)  inp *= b1;\n"
	"  b1 *= b1;\n"
	"  if ((hard % 256) >= 128) inp *= b1;\n"
	"  if (b1 < 0.001) b1 = 0.0;\n"
	"  /* hard & 256: hardness у нас 1..511, поэтому хватает сравнения */\n"
	"  if (hard >= 256) { b1 *= b1; inp *= b1; }\n"
	"  return inp;\n"
	"}\n"
	"/* ★ Тень от ОДНОЙ лампы: сэмплим её теневую карту (smap) в проекции lvp.\n"
	"   wp — мировая позиция фрагмента, L — направление НА свет. Вне [0,1]^2 или\n"
	"   вне z-диапазона карты света нет — возвращаем 1.0 (не в тени), иначе тень\n"
	"   легла бы на ВСЮ сцену. Смещение — slope-scaled: без него поверхность\n"
	"   затеняет саму себя (acne). PCF 3x3 — по 9 текселям NEAREST, потому что\n"
	"   карта хранит УПАКОВАННУЮ глубину, и билинейное смешивание байтов даёт мусор. */\n"
	"/* ★★ Раунд 83: закрывает ли окклюдер из карты этот фрагмент.\n"
	"   lpz.y != 0 (спот): карта ПЕРСПЕКТИВНАЯ, window-глубина нелинейна\n"
	"     (z_ndc = A + B/d, A=lpz.x, B=lpz.y), поэтому сравниваем РАССТОЯНИЯ\n"
	"     вдоль оси света. Иначе (солнце, ортопроекция): window-глубина линейна,\n"
	"     сравниваем её напрямую. */\n"
	"bool gpu_occludes(float fz, vec3 packed, float fd, vec2 lpz, float wbias, float zbias)\n"
	"{\n"
	"  float occ = gpu_unpack_depth(packed);\n"
	"  if (lpz.y != 0.0) {\n"
	"    float den = (2.0 * occ - 1.0) - lpz.x;\n"
	"    if (den >= -1e-6) return false;\n"
	"    float docc = lpz.y / den;\n"
	"    return (docc > 0.0 && fd - wbias > docc);\n"
	"  }\n"
	"  return (fz - zbias > occ);\n"
	"}\n"
	"float gpu_shadow(sampler2D smap, mat4 lvp, vec3 wp, vec3 n, vec3 L, float soft, vec2 lpz)\n"
	"{\n"
	"  vec4 lp = lvp * vec4(wp, 1.0);\n"
	"  vec3 sc = (lp.xyz / lp.w) * 0.5 + 0.5;\n"
	"  if (sc.x < 0.0 || sc.x > 1.0 || sc.y < 0.0 || sc.y > 1.0 ||\n"
	"      sc.z < 0.0 || sc.z > 1.0) return 1.0;\n"
	"  /* ★★ Раунд 83: у ПЕРСПЕКТИВНОЙ карты спота window-глубина нелинейна\n"
	"     (dz/dd = 2fn/((f-n)d^2)), поэтому константный bias в window-единицах\n"
	"     превращался в МИРОВОЙ порядка 0.0125*d^2 — на d=10 это 1.25 юнита, на\n"
	"     d=20 уже 5. Свет проходил сквозь тонкие стены (в mapFull 20 спотов,\n"
	"     n=clipsta, f=clipend): «чёрная» зона scanline светилась ~10/255.\n"
	"     Теперь сравниваем МИРОВЫЕ расстояния, а мировой размер текселя растёт\n"
	"     как ~0.0043*d, поэтому bias пропорционален d. У солнца (ортопроекция)\n"
	"     lpz=(0,0), для него остаётся прежняя ветка по window-глубине. */\n"
	"  float slope = 1.0 - dot(n, L);\n"
	"  float wbias = max(0.012 * slope, 0.005) * max(lp.w, 1e-4) * u_shadow_bias;\n"
	"  float zbias = max(0.0025 * slope, 0.0008);\n"
	"  float sh = 0.0;\n"
	"  if (u_pcf < 0.5) {\n"
	"    vec4 s = texture2D(smap, sc.xy);\n"
	"    if (s.a >= 0.5 && gpu_occludes(sc.z, s.rgb, lp.w, lpz, wbias, zbias)) sh = 1.0;\n"
	"    return 1.0 - sh;\n"
	"  } else {\n"
	"    /* ★ Мягкая тень: радиус выборок PCF масштабируется lar->soft (в BI это\n"
	"       fac в shadbuf.c). soft=3 (по умолчанию) -> r≈1.1 текселя, т.е. почти\n"
	"       как жёсткая; soft=100 -> r=5 текселей (шире, но при 3x3 грубее). */\n"
	"    float r = 1.0 + soft * 0.04;\n"
	"    for (int dy = -1; dy <= 1; dy++)\n"
	"      for (int dx = -1; dx <= 1; dx++) {\n"
	"        vec2 uv = sc.xy + vec2(float(dx), float(dy)) * u_shadow_texel * r;\n"
	"        vec4 s = texture2D(smap, uv);\n"
	"        if (s.a >= 0.5 && gpu_occludes(sc.z, s.rgb, lp.w, lpz, wbias, zbias)) sh += 1.0;\n"
	"      }\n"
	"    return 1.0 - sh * (1.0 / 9.0);\n"
	"  }\n"
	"}\n"
	"/* ★ Тень от POINT-лампы: кубическая карта. dir — направление ОТ света к\n"
	"   фрагменту (6 граней куба в textureCube выбираются сами), d — ОСЕВОЕ\n"
	"   расстояние фрагмента (см. вызов: max(|dx|,|dy|,|dz|)). Карта хранит\n"
	"   window-глубину той же проекции; у кубмапа far/near огромен, поэтому\n"
	"   window-единицы неравномерны и СРАВНИВАТЬ надо в расстояниях: из scz\n"
	"   восстанавливаем осевое расстояние окклюдера по (A,B) (обратная функция\n"
	"   к scz=(A + B/d)*0.5+0.5) — тогда bias в мировых единицах. */\n"
	"float gpu_shadow_point(samplerCube smap, vec3 dir, float d, vec2 lpz, vec3 n, vec3 L, float soft)\n"
	"{\n"
	"  /* мировой сдвиг против acne: растёт с расстоянием (неравномерность глубины) */\n"
	"  float bias = max(0.02 * (1.0 - dot(n, L)), 0.005) * d;\n"
	"  float sh = 0.0;\n"
	"  if (u_pcf < 0.5) {\n"
	"    vec4 s = textureCube(smap, dir);\n"
	"    if (s.a >= 0.5) {\n"
	"      float den = (2.0 * gpu_unpack_depth(s.rgb) - 1.0) - lpz.x;\n"
	"      if (den < -1e-6 && d - bias > lpz.y / den) sh = 1.0;\n"
	"    }\n"
	"    return 1.0 - sh;\n"
	"  } else {\n"
	"    /* ★ Мягкая тень: 3x3 решётка выборок (изотропнее, чем «крест» 5-точечный).\n"
	"       Соседняя грань куба хранит глубину для своего направления, поэтому сдвиг\n"
	"       через ребро корректен (textureCube выберет нужную грань). Радиус\n"
	"       масштабируется lar->soft: e = 2*texel*(1+soft*0.04); для мягкой тени\n"
	"       углы квадрата дают радиус e*sqrt(2) — как в 2D-ветке (r*sqrt(2)). */\n"
	"    vec3 up = (abs(dir.y) < 0.99) ? vec3(0.0, 1.0, 0.0) : vec3(1.0, 0.0, 0.0);\n"
	"    vec3 t1 = normalize(cross(dir, up));\n"
	"    vec3 t2 = cross(dir, t1);\n"
	"    float e = u_shadow_texel.x * 2.0 * (1.0 + soft * 0.04);\n"
	"    float sh = 0.0;\n"
	"    for (int dy = -1; dy <= 1; dy++)\n"
	"      for (int dx = -1; dx <= 1; dx++) {\n"
	"        vec3 sd = dir + t1 * (float(dx) * e) + t2 * (float(dy) * e);\n"
	"        vec4 s = textureCube(smap, sd);\n"
	"        float dd = (s.a >= 0.5) ? lpz.y / ((2.0 * gpu_unpack_depth(s.rgb) - 1.0) - lpz.x) : -1.0;\n"
	"        if (dd > 0.0 && d - bias > dd) sh += 1.0;\n"
	"      }\n"
	"    return 1.0 - sh * (1.0 / 9.0);\n"
	"  }\n"
	"}\n"
	"vec3 gpu_light(vec3 P, vec3 N, vec3 V, vec4 alb, vec4 sspec, vec4 posT, vec4 dirD,\n"
	"               vec4 colF, vec4 fl, vec3 wp, sampler2D smap, mat4 lvp,\n"
	"               samplerCube scube, vec3 lwpos, vec2 lpz, float soft)\n"
	"{\n"
	"  float type = posT.w;\n"
	"  vec3 lv;\n"
	"  float visifac = 1.0;\n"
	"  if (type < 0.5) {\n"
	"    lv = dirD.xyz;\n"                    /* направление, куда светит солнце */
	"  } else {\n"
	"    lv = P - posT.xyz;\n"
	"    float dist = length(lv);\n"
	"    lv = (dist > 1e-6) ? lv / dist : vec3(0.0, 0.0, 1.0);\n"
	"    if (type < 2.5) {\n"                 /* не area: у неё затухания нет */
	"      float ld = dirD.w;\n"              /* lar->dist */
	"      if (colF.a < 0.5)      visifac = 1.0;\n"
	"      else if (colF.a < 1.5) visifac = ld / (ld + dist);\n"
	"      else                   visifac = ld / (ld + dist * dist);\n"
	"      if (fl.x > 0.5) {\n"               /* LA_SPHERE: за пределами dist света нет */
	"        float t = ld - dist;\n"
	"        visifac = (t <= 0.0) ? 0.0 : visifac * (t / ld);\n"
	"      }\n"
	"      if (visifac > 0.0 && type > 1.5) {\n"   /* spot: конус */
	"        float inpr = dot(lv, dirD.xyz);\n"
	"        float t = fl.y;\n"                    /* spotsi = cos(spotsize/2) */
	"        if (inpr <= t) visifac = 0.0;\n"
	"        else {\n"
	"          t = inpr - t;\n"
	"          if (t < fl.z && fl.z != 0.0) {\n"    /* мягкий край конуса */
	"            float i = t / fl.z;\n"
	"            t = i * i;\n"
	"            inpr *= (3.0 * t - 2.0 * t * i);\n"
	"          }\n"
	"          visifac *= inpr;\n"
	"        }\n"
	"      }\n"
	"      if (visifac <= 0.001) visifac = 0.0;\n"
	"    }\n"
	"  }\n"
	"  if (visifac <= 0.0) return vec3(0.0);\n"
	"  vec3 Ld = -lv;\n"                            /* от поверхности к лампе */
	"  /* ★ Тень: только у sun (type<0.5) и spot (1.5<type<2.5), и только если\n"
	"     лампа реально даёт тень (fl.w=1). Ld — направление НА свет, оно же для\n"
	"     slope-scaled bias. Вне карты gpu_shadow вернёт 1.0 (света нет). */\n"
	"  float sh = 1.0;\n"
	"  if (fl.w > 0.5) {\n"
	"    if (type < 0.5 || (type > 1.5 && type < 2.5))\n"
	"      sh = gpu_shadow(smap, lvp, wp, N, Ld, soft, lpz);\n"
	"    else if (type < 1.5 || (type > 2.5 && type < 3.5)) {\n" /* POINT(1)/AREA(3): куб */
	"      vec3 pd = wp - lwpos;\n"
	"      float plen = length(pd);\n"
	"      /* ★ Глубина кубмапа хранится по ОСИ грани: в перспективе lp.w = -view_z\n"
	"         равен осевому (не евклидову) расстоянию, и gl_FragCoord.z зависит\n"
	"         именно от него. Поэтому и восстанавливать/сравнивать надо осевое\n"
	"         расстояние pax = max(|dx|,|dy|,|dz|) — вдоль луча оно пропорционально\n"
	"         истинному, так что сравнение корректно. */\n"
	"      float pax = max(max(abs(pd.x), abs(pd.y)), abs(pd.z));\n"
	"      if (plen > 1e-6) sh = gpu_shadow_point(scube, pd / plen, pax, lpz, N, Ld, soft);\n"
	"    }\n"
	"  }\n"
	"  /* ★★ Раунд 83-fix2: ОТКАТ «квадрата Ламберта». Проверка по исходнику:\n"
	"     add_to_diffuse(shadeoutput.c:958) принимает `is` пятым параметром, но в\n"
	"     ОБЫЧНОЙ ветке (1005-1009, без color-ramp) делает просто\n"
	"         diff[0] += rgb[0] * shi->r;\n"
	"     где rgb = tcol = i*lacol, а i = is*phongcorr*visifac*shi->refl (1452/1455).\n"
	"     То есть `is` входит РОВНО ОДИН раз, а параметр `is` нужен только ветке\n"
	"     color-ramp (981: fac = is для MA_RAMP_IN_SHADER). Диффуз BI =\n"
	"     shi->r * lacol * is * phongcorr * visifac * refl. Экспонента 1.\n"
	"     Квадрат давал лучшее СРЕДНЕЕ по mapFull (отношение кадров 1.350 -> 0.973),\n"
	"     но это была компенсация другой ошибки: полосы оставались немонотонными\n"
	"     (2.43x в тусклых, 0.81x в 32-64), а тени-выключенные тесты и точечные лампы\n"
	"     становились на 7-16% темнее scanline. Держать неверную формулу нельзя. */\n"
	"  float diff = max(dot(N, Ld), 0.0);\n"
	"  vec3 Hs = Ld + V;\n"                         /* полувектор Blinn-Phong */
	"  float hl = length(Hs);\n"
	"  vec3 H = (hl > 1e-6) ? Hs / hl : N;\n"       /* защита от normalize(0) = NaN */
	"  /* ★ M5: блик как CookTorr_Spec (shadeoutput.c:602) — шейдер блика по\n"
	"     умолчанию в BI. nh = N·H (полувектор), при nh<0 блика нет (это уже\n"
	"     внутри gpu_spec); nv = max(N·V,0); блик = spec(nh,hard)/(0.1+nv).\n"
	"     hardness лежит в G-буфере как hard/511, поэтому возвращаем её\n"
	"     округлением до целого — в BI это int-битовая маска. */\n"
	"  float nh = dot(N, H);\n"
	"  float specfac = 0.0;\n"
	"  float nv = 0.0;\n"
	"  if (nh > 0.0) {\n"
	"    nv = max(dot(N, V), 0.0);\n"
	"    specfac = gpu_spec(nh, int(sspec.a * 511.0 + 0.5)) / (0.1 + nv);\n"
	"  }\n"
	"  /* BI: t = shadfac[3]*shi->spec*visifac*specfac, затем\n"
	"     spec += t*lacol*shi->specr/g/b. Интенсивность (shi->spec) лежит в\n"
	"     alb.a, цвет блика — в sspec.rgb. Раньше здесь была зашита hardness 48\n"
	"     и множитель 0.8 — ни того, ни другого в BI нет. */\n"
	"  vec3 spec = sspec.rgb * (alb.a * specfac);\n"
	"  if (u_debug == 6) return vec3(specfac, nh, nv);\n"
	"  return (alb.rgb * diff + spec) * colF.rgb * visifac * sh;\n"
	"}\n"
	"void main(){\n"
	"  /* rgb = упакованная линейная view-глубина, a = маска геометрии */\n"
	"  vec4 vd = texture2D(u_depth, v_uv);\n"
	"  if (u_debug == 3) { gl_FragColor = vec4(vd.rgb, 1.0); return; }\n"
	"\n"
	"  /* луч взгляда нужен и фону (градиент мира), поэтому считаем его ДО проверки */\n"
	"  vec4 cp = u_inv_proj * vec4(v_ndc, -1.0, 1.0);\n"
	"  vec3 dir = cp.xyz / cp.w;\n"
	"\n"
	"  /* ★ Фон: геометрии в этом пикселе нет. BI берёт horizon_color, а при\n"
	"     WO_SKYBLEND смешивает к zenith_color по blend = |0.5 + view.y|\n"
	"     (pixelshading.c:505-545). Раньше здесь был плоский серый 0.19 — фон\n"
	"     вообще не был связан с настройками World сцены. */\n"
	"  if (vd.a < 0.5) {\n"
	"    if (u_debug != 0) { gl_FragColor = vec4(0.0, 0.0, 0.0, 1.0); return; }\n"
	"    vec3 vdir = normalize(dir);\n"
	"    float bl = min(abs(0.5 + vdir.y), 1.0);\n"
	"    vec3 bg = (u_world_blend > 0.5) ? mix(u_world_hor, u_world_zen, bl) : u_world_hor;\n"
	"    gl_FragColor = vec4(gpu_to_srgb(bg), 1.0);\n"   /* раунд 82: дисплейное пространство */
	"    return;\n"
	"  }\n"
	"\n"
	"  vec4 albedo = texture2D(u_albedo, v_uv);\n"
	"  if (u_debug == 1) { gl_FragColor = vec4(albedo.rgb, 1.0); return; }\n"
	"  if (u_debug == 5) { float si = albedo.a; gl_FragColor = vec4(si, si, si, 1.0); return; }\n"
	"  /* ★ M5: параметры блика материала (rgb = цвет, a = hardness/511).\n"
	"     Если 4-й цели MRT нет, сюда подставлена 1x1 текстура с дефолтами BI. */\n"
	"  vec4 sspec = texture2D(u_spec, v_uv);\n"
	"\n"
	"  vec4 nrm_tex = texture2D(u_nrm, v_uv);\n"
	"  float emit = nrm_tex.a * 2.0;\n"    /* эмиссия материала (M5): в alpha нормали лежит emit/2 */
	"  vec3 n = normalize(nrm_tex.xyz * 2.0 - 1.0);\n"
	"  if (u_debug == 2) { gl_FragColor = vec4(n * 0.5 + 0.5, 1.0); return; }\n"
	"\n"
	"  /* распаковать линейную глубина -> view-позиция по лучу взгляда (dir выше) */\n"
	"  float linz = gpu_unpack_depth(vd.rgb) / u_pack;\n"
	"  /* ★★ раунд 83-fix: восстановление view-позиции. Перспективный вариант\n"
	"     масштабирует луч так, чтобы vpos.z == -linz; для ОРТОКАМЕРЫ это\n"
	"     неверно: там лучи параллельны, dir.xy уже равен нужному vpos.xy, а\n"
	"     dir.z == -near (маленькое число!), поэтому деление на max(-dir.z,1e-6)\n"
	"     раздувало vpos.xy в linz/near раз (при near=0.1 и linz=8 — в 80 раз).\n"
	"     Симптом: орто-сцена освещалась только в пятне ~14 px вокруг центра,\n"
	"     т.к. dot(N, Ld) уходил в ноль. Замер на lighttest2: ORTHO mean|diff|\n"
	"     140.5 -> PERSP 0.688 на той же сцене. */\n"
	"  vec3 vpos = (u_ortho > 0.5) ? vec3(dir.xy, -linz) : dir * (linz / max(-dir.z, 1e-6));\n"
	"  /* ★ Двустороннее освещение как в BI. Нормаль, отвёрнутая от камеры, — это\n"
	"     обратная сторона открытого меша (плоскость, стена, ткань). Раньше такая\n"
	"     поверхность была чёрной. Разворачиваем нормаль к камере ДО расчёта\n"
	"     диффуза, блика, ambient и AO — все они должны видеть одну нормаль. */\n"
	"  vec3 V = normalize(-vpos);\n"
	"  if (dot(n, V) < 0.0) n = -n;\n"
	"  vec4 wp = u_inv_view * vec4(vpos, 1.0);\n"
	"  wp.xyz /= wp.w;\n"
	"\n"
	"  /* ★ Тень считается ВНУТРИ gpu_light, отдельно для каждой лампы: у sun и spot\n"
	"     своя карта и своя проекция. Здесь от старой одиночной выборки осталась\n"
	"     только мировая позиция wp, которую и передаём. */\n"
	"\n"
	"  /* Ambient. BI добавляет shi->ambr = ma->amb * wrld.ambient_color\n"
	"     (shadeoutput.c:2000 и render_texture.c:2773), но у наблюдаемых сцен он\n"
	"     РОВНО НОЛЬ — поэтому держим ещё и встроенный полусферический fill\n"
	"     (UBF_GPU_AMB_FILL=0 его убирает). Мир берём по максимуму с fill: тёмный\n"
	"     World не должен гасить сцену, а светлый — обязан светить. */\n"
	"  float upness = dot(n, u_up) * 0.5 + 0.5;\n"
	"  vec3 world_hemi = mix(u_world_hor, u_world_zen, upness);\n"
	"  vec3 fill = mix(vec3(0.07, 0.06, 0.06), vec3(0.28, 0.30, 0.34), upness) * u_amb_fill;\n"
	"  vec3 amb = u_world_amb + ((u_env_light > 0.5) ? world_hemi * u_env_energy : vec3(0.0)) + fill;\n"
	"  float ao = (u_ao > 0.5) ? gpu_ssao(v_uv, vpos, linz, n) : 1.0;\n"
	"  /* ★ Сумма вкладов ламп (раньше — только солнце). Индексы массивов КОНСТАНТНЫ:\n"
	"     GLSL 1.20 не обязан поддерживать динамическую индексацию uniform-массивов,\n"
	"     поэтому вместо цикла десять вызовов. Тень — отдельно для каждой лампы. */\n"
	"  vec3 lit = vec3(0.0);\n"
	"  /* ★★ Раунд 16: цикл по ВСЕМ лампам сцены (раньше — жёстко 10 вызовов).\n"
	"     Данные лампы берём из float-текстуры: gpu_ltap(li,row). Тень умеют только\n"
	"     первые 10 ламп (GPU_MAX_SHADOW_LIGHTS — их число задано парой сэмплеров на\n"
	"     лампу), поэтому для li>=10 берётся fl.w=0 и заглушки-сэмплеры:\n"
	"     ветка тени в gpu_light не исполняется, лампа просто светит. */\n"
	"  for (int li = 0; li < u_num_lights; li++) {\n"
	"    vec4 lfl = gpu_ltap(li, 3);        /* sphere, spotsi, spotbl, shadows */\n"
	"    float lsf = gpu_ltap(li, 4).x;     /* lar->soft — мягкость тени */\n"
	"    if (li == 0)       lit += gpu_light(vpos, n, V, albedo, sspec, gpu_ltap(0,0), gpu_ltap(0,1), gpu_ltap(0,2), lfl, wp, u_shadow[0], u_lvp[0], u_shadow_cube[0], u_lwpos[0], u_lpz[0], lsf);\n"
	"    else if (li == 1)  lit += gpu_light(vpos, n, V, albedo, sspec, gpu_ltap(1,0), gpu_ltap(1,1), gpu_ltap(1,2), lfl, wp, u_shadow[1], u_lvp[1], u_shadow_cube[1], u_lwpos[1], u_lpz[1], lsf);\n"
	"    else if (li == 2)  lit += gpu_light(vpos, n, V, albedo, sspec, gpu_ltap(2,0), gpu_ltap(2,1), gpu_ltap(2,2), lfl, wp, u_shadow[2], u_lvp[2], u_shadow_cube[2], u_lwpos[2], u_lpz[2], lsf);\n"
	"    else if (li == 3)  lit += gpu_light(vpos, n, V, albedo, sspec, gpu_ltap(3,0), gpu_ltap(3,1), gpu_ltap(3,2), lfl, wp, u_shadow[3], u_lvp[3], u_shadow_cube[3], u_lwpos[3], u_lpz[3], lsf);\n"
	"    else if (li == 4)  lit += gpu_light(vpos, n, V, albedo, sspec, gpu_ltap(4,0), gpu_ltap(4,1), gpu_ltap(4,2), lfl, wp, u_shadow[4], u_lvp[4], u_shadow_cube[4], u_lwpos[4], u_lpz[4], lsf);\n"
	"    else if (li == 5)  lit += gpu_light(vpos, n, V, albedo, sspec, gpu_ltap(5,0), gpu_ltap(5,1), gpu_ltap(5,2), lfl, wp, u_shadow[5], u_lvp[5], u_shadow_cube[5], u_lwpos[5], u_lpz[5], lsf);\n"
	"    else if (li == 6)  lit += gpu_light(vpos, n, V, albedo, sspec, gpu_ltap(6,0), gpu_ltap(6,1), gpu_ltap(6,2), lfl, wp, u_shadow[6], u_lvp[6], u_shadow_cube[6], u_lwpos[6], u_lpz[6], lsf);\n"
	"    else if (li == 7)  lit += gpu_light(vpos, n, V, albedo, sspec, gpu_ltap(7,0), gpu_ltap(7,1), gpu_ltap(7,2), lfl, wp, u_shadow[7], u_lvp[7], u_shadow_cube[7], u_lwpos[7], u_lpz[7], lsf);\n"
	"    else if (li == 8)  lit += gpu_light(vpos, n, V, albedo, sspec, gpu_ltap(8,0), gpu_ltap(8,1), gpu_ltap(8,2), lfl, wp, u_shadow[8], u_lvp[8], u_shadow_cube[8], u_lwpos[8], u_lpz[8], lsf);\n"
	"    else if (li == 9)  lit += gpu_light(vpos, n, V, albedo, sspec, gpu_ltap(9,0), gpu_ltap(9,1), gpu_ltap(9,2), lfl, wp, u_shadow[9], u_lvp[9], u_shadow_cube[9], u_lwpos[9], u_lpz[9], lsf);\n"
	"    else               lit += gpu_light(vpos, n, V, albedo, sspec, gpu_ltap(li,0), gpu_ltap(li,1), gpu_ltap(li,2), vec4(lfl.xyz, 0.0), wp, u_shadow[0], u_lvp[0], u_shadow_cube[0], u_lwpos[0], u_lpz[0], lsf);\n"
	"  }\n"
	"  if (u_debug == 6) { gl_FragColor = vec4(lit.r, lit.r, lit.r, 1.0); return; }\n"
	"  /* Экспозиция World: BI применяет wrld_exposure_correct к вкладу ЛАМП и лишь\n"
	"     если exp!=0 или range!=1 (shadeoutput.c:1745, 1960); ambient и эмиссия — мимо.\n"
	"     u_logfac отрицателен, поэтому exp(lit*logfac) < 1 и результат монотонен. */\n"
	"  if (u_expo_on > 0.5) lit = u_linfac * (1.0 - exp(lit * u_logfac));\n"
	"  vec3 col = albedo.rgb * amb * ao + lit;\n"
	"  col *= u_exposure;\n"
	"  col += albedo.rgb * emit;\n"   /* эмиссия (M5): самосвет материала, не зависит от света/тени */
	"  /* SSR (screen-space reflection): подмешиваем отражённый цвет сцены.\n"
	"     Сила — из 5-й цели MRT (MAP_REFL), иначе фиксированный 0.35. */\n"
	"  if (u_ssr > 0.5) {\n"
	"    vec3 refl = gpu_ssr(v_uv, vpos, n, V);\n"
	"    if (refl != vec3(0.0)) {\n"
	"      float rs = clamp(texture2D(u_refl, v_uv).r, 0.0, 1.0);\n"
	"      col = mix(col, refl, rs);\n"
	"    }\n"
	"  }\n"
	"  /* ★★ Раунд 81: выходная альфа — ПОКРЫТИЕ (1.0), а НЕ albedo.a.\n"
	"     albedo.a занят интенсивностью блика (shi->spec) и нужен только для\n"
	"     расчёта блика внутри gpu_light. Раньше он уезжал в gl_FragColor.a:\n"
	"     у матовых материалов (shi->spec≈0) вся геометрия получала alpha≈0,\n"
	"     и кадр выглядел «прозрачным из неоткуда» — при просмотре/композите\n"
	"     поверх светлого фона тёмные текстуры выбеливались (mapFull: 20%\n"
	"     пикселей alpha<255, 11% alpha==0, тогда как scanline даёт alpha=255\n"
	"     на 100% кадра). Для непрозрачной геометрии BI пишет alpha=1.0\n"
	"     (Combined pass), как и ветка фона строкой выше. */\n"
	"  /* ★★ Раунд 82: кадр — ДИСПЛЕЙНЫЙ (см. gpu_to_srgb). _dbgout оставлен сырым:\n"
	"     отладочные каналы (albedo/нормаль/глубина) не должны проходить кривую. */\n"
	"  gl_FragColor = vec4(gpu_to_srgb(col), 1.0);\n"
	"}\n";

/* --- M15: forward-проход для полупрозрачных мешей. Деферред G-буфер не может дать
 * честную прозрачность (канал albedo.a занят бликом, нет блендинга), поэтому
 * полупрозрачные меши рисуются ПОСЛЕ освещения, в последний проход, с alpha-блендингом.
 * Освещение — как в LIGHT_FS (тот же gpu_light), но свойства поверхности считаются
 * per-fragment из материала; затенение непрозрачной сценой — сравнением v_linz с
 * глубиной из vpos-текстуры (дискриминация фрагментов ЗА непрозрачной геометрией).
 * Формат: vec4(цвет, alpha) -> glBlendFunc(SRC_ALPHA, ONE_MINUS_SRC_ALPHA). */
/* ★ M16 (раунд 67): спрайт-хало (материал HALO). BI рисует хало НЕ
 * треугольниками, а пятном вокруг спроецированной вершины (rendercore.c,
 * renderhalo_*), с затуханием из shadeHaloFloat (pixelshading.c:356-376) и
 * смешиванием addalphaAddfacFloat (pixelblending.c:97). Здесь ровно то же
 * самое: биллборд-квад в мире + копия формулы затухания; вместо записи в
 * z-буфер — сравнение глубины центра хало с глубиной непрозрачной сцены из
 * vpos-текстуры G-буфера (как в forward-проходе).
 * Геометрия квада переиспользуется от полноэкранного (r->quad_vbo): его
 * a_pos как раз (-1..-1)..(1,1), а a_uv — (0,0)..(1,1). */
static const char *HALO_VS =
	"attribute vec2 a_corner;\n"      /* угол биллборда, -1..1 */
	"attribute vec2 a_uv;\n"
	"varying vec2 v_uv;\n"
	"varying float v_rad_px;\n"       /* радиус пятна в ПИКСЕЛЯХ (BI har->rad) */
	"uniform mat4 u_proj;\n"
	"uniform mat4 u_view;\n"
	"uniform vec3 u_pos;\n"           /* центр хало, мир */
	"uniform vec3 u_right;\n"         /* правый вектор камеры, мир */
	"uniform vec3 u_up;\n"            /* верхний вектор камеры, мир */
	"uniform float u_rad;\n"
	"uniform vec2 u_resolution;\n"
	"void main(){\n"
	"  vec3 wp = u_pos + (a_corner.x * u_right + a_corner.y * u_up) * u_rad;\n"
	"  gl_Position = u_proj * (u_view * vec4(wp, 1.0));\n"
	"  v_uv = a_uv;\n"
	"  /* BI har->rad = экранный размер смещения hasize по правой оси камеры\n"
	"   * (renderdatabase.c:1360-1364). Считаем его здесь: проекция центра и\n"
	"   * центра+u_right*u_rad, разница в NDC -> в пиксели. */\n"
	"  vec4 c0 = u_proj * (u_view * vec4(u_pos, 1.0));\n"
	"  vec4 c1 = u_proj * (u_view * vec4(u_pos + u_right * u_rad, 1.0));\n"
	"  vec2 n0 = c0.xy / c0.w;\n"
	"  vec2 n1 = c1.xy / c1.w;\n"
	"  v_rad_px = length((n1 - n0) * 0.5 * u_resolution);\n"
	"}\n";

static const char *HALO_FS =
	"varying vec2 v_uv;\n"
	"varying float v_rad_px;\n"
	"uniform vec3 u_col;\n"
	"uniform float u_alfa;\n"
	"uniform float u_hard;\n"          /* har->hard (ma->har): 1..511 */
	"uniform float u_add;\n"           /* har->add: 0..255 */
	"uniform float u_zcenter;\n"       /* -z центра хало в пространстве камеры */
	"uniform sampler2D u_depth;\n"
	"uniform float u_pack;\n"
	"uniform vec2 u_resolution;\n"
	"uniform sampler2D u_hashv;\n"     /* hashvectf[768], по градиенту на тексель */
	"uniform float u_seed;\n"          /* har->seed: смещение в hashvectf */
	"uniform float u_ringc;\n"         /* число колец (har->ringc) */
	"uniform float u_linec;\n"         /* число лучей (har->linec) */
	"uniform float u_starpoints;\n"    /* число лучей «звезды» (har->starpoints) */
	"uniform float u_xalpha;\n"        /* HA_XALPHA: alpha = falloff^2 */
	"uniform vec3 u_spec_col;\n"       /* ma->specr/g/b — цвет лучей */
	"uniform vec3 u_mir_col;\n"        /* ma->mirr/g/b — цвет колец */
	"/* ★★ Раунд 82: кадр дисплейный (см. LIGHT_FS.gpu_to_srgb) — хало тоже. */\n"
	"vec3 gpu_to_srgb(vec3 c){\n"
	"  c = clamp(c, 0.0, 1.0);\n"
	"  vec3 lo = c * 12.92;\n"
	"  vec3 hi = 1.055 * pow(c, vec3(1.0 / 2.4)) - 0.055;\n"
	"  return mix(lo, hi, step(vec3(0.0031308), c));\n"
	"}\n"
	"float gpu_unpack_depth(vec3 c){\n"
	"  return dot(c, vec3(1.0, 1.0/255.0, 1.0/65025.0));\n"
	"}\n"
	"float gpu_hashf(float ofs){\n"    /* hashvectf[ofs % 768] как float */
	"  float m = mod(ofs, 768.0);\n"
	"  float t = floor(m / 3.0);\n"
	"  float c = m - t * 3.0;\n"
	"  vec4 v = texture2D(u_hashv, vec2((t + 0.5) / 256.0, 0.5)) * 2.0 - 1.0;\n"
	"  if (c < 0.5) return v.r;\n"
	"  else if (c < 1.5) return v.g;\n"
	"  return v.b;\n"
	"}\n"
	"void main(){\n"
	"  vec2 d = v_uv * 2.0 - 1.0;\n"
	"  float d2 = dot(d, d);\n"
	"  if (d2 >= 1.0) discard;\n"
	"  /* пиксельные смещения от центра — BI передаёт их в shadeHaloFloat */\n"
	"  float xn = d.x * v_rad_px;\n"
	"  float yn = d.y * v_rad_px;\n"
	"  float radist = length(d) * v_rad_px;   /* BI radist = sqrt(dist) */\n"
	"  /* Копия shadeHaloFloat: dist — нормированный КВАДРАТ расстояния от\n"
	"   * центра (0 в центре, 1 на краю); BI делит на har->radsq — то же. */\n"
	"  float dist = d2;\n"
	"  if (u_hard >= 30.0) {\n"
	"    dist = sqrt(dist);\n"
	"    if (u_hard >= 40.0) {\n"
	"      dist = sin(dist * 1.5707963);\n"
	"      if (u_hard >= 50.0) dist = sqrt(dist);\n"
	"    }\n"
	"  } else if (u_hard < 20.0) {\n"
	"    dist *= dist;\n"
	"  }\n"
	"  float fall = 1.0 - dist;\n"
	"  /* кольца (pixelshading.c:327-345): антиалиас-круг на радиусе rc[0]*rad */\n"
	"  float ringf = 0.0;\n"
	"  if (u_ringc > 0.0) {\n"
	"    for (int a = 0; a < 8; a++) {\n"
	"      if (float(a) >= u_ringc) break;\n"
	"      float ofs = u_seed + float(a) * 2.0;\n"
	"      float r0 = gpu_hashf(ofs);\n"
	"      float r1 = gpu_hashf(ofs + 1.0);\n"
	"      float fac = abs(r1 * (v_rad_px * abs(r0) - radist));\n"
	"      if (fac < 1.0) ringf += 1.0 - fac;\n"
	"    }\n"
	"  }\n"
	"  /* лучи (pixelshading.c:378-397): антиалиас-линии через центр */\n"
	"  float linef = 0.0;\n"
	"  if (u_linec > 0.0) {\n"
	"    for (int a = 0; a < 8; a++) {\n"
	"      if (float(a) >= u_linec) break;\n"
	"      float ofs = u_seed + float(a) * 3.0;\n"
	"      float r0 = gpu_hashf(ofs);\n"
	"      float r1 = gpu_hashf(ofs + 1.0);\n"
	"      float fac = abs(xn * r0 + yn * r1);\n"
	"      if (fac < 1.0) linef += 1.0 - fac;\n"
	"    }\n"
	"  }\n"
	"  linef *= fall;\n"
	"  /* «звезда» (pixelshading.c:399-416): искажает fall по направлению */\n"
	"  if (u_starpoints > 0.0) {\n"
	"    float angle = atan(yn, xn);\n"
	"    angle *= 1.0 + 0.25 * u_starpoints;\n"
	"    float co = cos(angle);\n"
	"    float si = sin(angle);\n"
	"    angle = (co * xn + si * yn) * (co * yn - si * xn);\n"
	"    float ster = abs(angle);\n"
	"    if (ster > 1.0) {\n"
	"      ster = v_rad_px / ster;\n"
	"      if (ster < 1.0) fall *= sqrt(ster);\n"
	"    }\n"
	"  }\n"
	"  if (fall <= 0.00001) discard;\n"
	"  fall *= u_alfa;\n"
	"  ringf *= fall;\n"
	"  linef *= u_alfa;\n"
	"  vec3 col = u_col * fall;\n"
	"  float al = (u_xalpha > 0.5) ? (fall * fall) : fall;\n"
	"  if (linef != 0.0) {\n"
	"    col += linef * u_spec_col;\n"
	"    al += (u_xalpha > 0.5) ? (linef * linef) : linef;\n"
	"  }\n"
	"  if (ringf != 0.0) {\n"
	"    col += ringf * u_mir_col;\n"
	"    al += (u_xalpha > 0.5) ? (ringf * ringf) : ringf;\n"
	"  }\n"
	"  /* затенение непрозрачной геометрией — как в forward-проходе */\n"
	"  vec2 suv = gl_FragCoord.xy / u_resolution;\n"
	"  vec4 sd = texture2D(u_depth, suv);\n"
	"  if (sd.a >= 0.5) {\n"
	"    float scene_linz = gpu_unpack_depth(sd.rgb) / u_pack;\n"
	"    if (scene_linz < u_zcenter - 0.05) discard;\n"
	"  }\n"
	"  if (al <= 0.002) discard;\n"
	"  /* addalphaAddfacFloat: dest = dest*(1 - a*(255-add)/255) + a*col.\n"
	"   * GL_ONE/GL_ONE_MINUS_SRC_ALPHA даёт dest*(1-alpha_out) + src, поэтому\n"
	"   * alpha_out = a*(255-add)/255 — тогда формулы совпадают точно. */\n"
	"  gl_FragColor = vec4(gpu_to_srgb(col), al * (255.0 - u_add) / 255.0);\n"
	"}\n";

static const char *FORWARD_VS =
	"attribute vec3 a_co;\n"
	"attribute vec3 a_n;\n"
	"attribute vec4 a_rgba;\n"
	"attribute float a_emit;\n"
	"attribute vec2 a_uv;\n"
	"attribute vec4 a_tangent;\n"
	"attribute vec3 a_orco;\n"
	"uniform mat4 u_proj;\n"
	"uniform mat4 u_model;\n"      /* object->view */
	"uniform mat4 u_obmat;\n"      /* object->world */
	"uniform mat3 u_normmat;\n"
	"varying vec3 v_nrm;\n"
	"varying vec4 v_rgba;\n"
	"varying float v_emit;\n"
	"varying vec2 v_uv_tex;\n"
	"varying vec3 v_tangent;\n"
	"varying float v_tang_sign;\n"
	"varying float v_linz;\n"
	"varying vec3 v_orco;\n"
	"varying vec3 v_vpos;\n"
	"varying vec3 v_wpos;\n"
	"varying vec3 v_w_nrm;\n"      /* M29: мировая нормаль — для CUBE-маппинга */
	"void main(){\n"
	"  vec3 nn = u_normmat * a_n;\n"
	"  float nl = length(nn);\n"
	"  v_nrm = (nl > 1e-6) ? nn / nl : vec3(0.0, 0.0, 1.0);\n"
	"  v_w_nrm = normalize(mat3(u_obmat) * a_n);\n"   /* M29: в мир (u_obmat) */
	"  v_rgba = a_rgba;\n"
	"  v_emit = a_emit;\n"
	"  v_uv_tex = a_uv;\n"
	"  v_tangent = mat3(u_model) * a_tangent.xyz;\n"
	"  v_tang_sign = a_tangent.w;\n"
	"  v_orco = a_orco;\n"
	"  vec4 vp = u_model * vec4(a_co, 1.0);\n"
	"  v_vpos = vp.xyz;\n"
	"  v_wpos = (u_obmat * vec4(a_co, 1.0)).xyz;\n"
	"  v_linz = -vp.z;\n"
	"  gl_Position = u_proj * vp;\n"
	"}\n";
static const char *FORWARD_FS =
	"varying vec3 v_nrm;\n"
	"varying vec4 v_rgba;\n"
	"varying float v_emit;\n"
	"varying vec2 v_uv_tex;\n"
	"varying vec3 v_tangent;\n"
	"varying float v_tang_sign;\n"
	"varying float v_linz;\n"
	"varying vec3 v_orco;\n"
	"varying vec3 v_vpos;\n"
	"varying vec3 v_wpos;\n"
	"varying vec3 v_w_nrm;\n"       /* M29: мировая нормаль (для CUBE-маппинга) */
	"uniform float u_pack;\n"
	"uniform float u_alpha;\n"           /* мат. alpha */
	"uniform float u_exposure;\n"
	"uniform float u_expo_on;\n"
	"uniform float u_linfac;\n"
	"uniform float u_logfac;\n"
	"uniform float u_amb_fill;\n"
	"uniform float u_env_light;\n"   /* M26: 1 — Environment Lighting мира включён */
	"uniform float u_env_energy;\n"  /* M26: wrld->ao_env_energy */
	"uniform vec3 u_up;\n"
	"uniform vec3 u_world_hor;\n"
	"uniform vec3 u_world_zen;\n"
	"uniform vec3 u_world_amb;\n"
	"uniform float u_world_blend;\n"
	"uniform int u_num_lights;\n"
	"/* ★★ Раунд 16: данные ламп — float-текстурой, а не per-light юниформами.\n"
	"   GPU_LIGHT_DATA_ROWS текселей RGBA32F на лампу:\n"
	"     0: pos.xyz, type   1: dir.xyz, dist   2: col.rgb, falloff\n"
	"     3: sphere, spotsi, spotbl, shadows      4: soft\n"
	"   Так потолок «10 ламп» (лимит uniform-компонентов) снят: число ламп задаёт\n"
	"   ширина текстуры, а цикл по лампам — динамический. */\n"
	"uniform sampler2D u_light_data;\n"
	"uniform vec2 u_light_texel;\n"   /* (1/ширина, 1/GPU_LIGHT_DATA_ROWS) */
	"vec4 gpu_ltap(int li, int row){\n"
	"  return texture2D(u_light_data, vec2((float(li) + 0.5) * u_light_texel.x,\n"
	"                                      (float(row) + 0.5) * u_light_texel.y));\n"
	"}\n"
	"uniform sampler2D u_shadow[10];\n"
	"uniform mat4 u_lvp[10];\n"
	"uniform samplerCube u_shadow_cube[10];\n"
	"uniform vec3 u_lwpos[10];\n"
	"uniform vec2 u_lpz[10];\n"
	"/* ★ раунд 16: u_lsoft[10] убран — soft едет в строке 4 u_light_data */\n"
	"uniform float u_pcf;\n"
	"uniform vec2 u_shadow_texel;\n"
	"uniform float u_shadow_bias;\n"   /* раунд 83-fix: в FORWARD_FS это объявление было
	                                      ПРОПУЩЕНО, хотя gpu_shadow() использует его в
	                                      выражении wbias — весь forward-программ не
	                                      компилировался (GLSL C1008: undefined variable),
	                                      и forward/прозрачные объекты выпадали в чёрное. */
	"uniform sampler2D u_depth;\n"        /* vpos-текстура (глубина непрозрачной сцены) */
	"uniform vec2 u_resolution;\n"
	"uniform sampler2D u_tex;\n"
	"uniform float u_has_tex;\n"
	"uniform float u_colfac;\n"
	"uniform float u_tex_rgbtoint;\n"   /* ★ раунд 83-fix3: MTEX_RGBTOINT слота 0 */
	"uniform float u_refl;\n"     /* M25: shi->refl = ma->ref (как в GB_FS) */
	"uniform float u_tex_orco;\n"
	"uniform float u_tex_glob;\n"   /* M27: GLOB -> v_wpos */
	"uniform float u_tex_map;\n"
	"uniform vec2 u_tex_size;\n"
	"uniform vec2 u_tex_ofs;\n"
	"uniform int u_tex_blend;\n"
	"uniform sampler2D u_atex;\n"
	"uniform float u_has_atex;\n"
	"uniform float u_a_fac;\n"
	"uniform float u_a_def;\n"
	"uniform float u_a_orco;\n"
	"uniform float u_a_glob;\n"   /* M27: GLOB -> v_wpos */
	"uniform float u_a_map;\n"
	"uniform float u_a_use_a;\n"
	"uniform vec2 u_a_size;\n"
	"uniform vec2 u_a_ofs;\n"
	"uniform int u_a_blend;\n"
	"uniform sampler2D u_nrm;\n"
	"uniform float u_has_nrm;\n"
	"uniform vec3 u_spec_col;\n"
	"uniform float u_hard;\n"
	"uniform sampler2D u_albedo;\n"      /* G-буфер albedo (для SSR отражения сцены) */
	"uniform float u_ortho;\n"          /* ★ раунд 83-fix: 1 = ортокамера (winmat[2][3]==0) */
	"uniform mat4 u_inv_proj;\n"         /* clip->view (для SSAO) */
	"uniform mat4 u_proj;\n"             /* view->clip (для шагов SSR) */
	"uniform vec2 u_texel;\n"            /* (1/width,1/height) — размер пикселя для SSAO */
	"uniform float u_ssr;\n"             /* 1 = screen-space reflections */
	"uniform float u_ao;\n"              /* 1 = screen-space ambient occlusion */
	"/* ★★ Раунд 82: forward-проход блендится ПОВЕРХ кадра освещения, а тот теперь\n"
	"   дисплейный (см. LIGHT_FS.gpu_to_srgb) — значит и здесь выход дисплейный,\n"
	"   иначе альфа-блендинг смешивал бы разные пространства. */\n"
	"vec3 gpu_to_srgb(vec3 c){\n"
	"  c = clamp(c, 0.0, 1.0);\n"
	"  vec3 lo = c * 12.92;\n"
	"  vec3 hi = 1.055 * pow(c, vec3(1.0 / 2.4)) - 0.055;\n"
	"  return mix(lo, hi, step(vec3(0.0031308), c));\n"
	"}\n"
	"float gpu_unpack_depth(vec3 c){\n"
	"  return dot(c, vec3(1.0, 1.0/255.0, 1.0/65025.0));\n"
	"}\n"
	"/* SSR: маршируем отражённый луч R по экрану, сравнивая глубину сцены. */\n"
	"vec3 gpu_ssr(vec2 uv, vec3 vpos, vec3 n, vec3 V) {\n"
	"  vec3 R = reflect(-V, n);\n"
	"  if (dot(R, V) > 0.5) return vec3(0.0);\n"
	"  float dist = max(-vpos.z, 0.1);\n"
	"  float t = dist * 0.005;\n"
	"  float step = dist * 0.02;\n"
	"  float tol = max(0.02, dist * 0.01);\n"
	"  for (int i = 0; i < 48; i++) {\n"
	"    vec3 p = vpos + R * t;\n"
	"    vec4 clip = u_proj * vec4(p, 1.0);\n"
	"    if (clip.w <= 0.0) break;\n"
	"    vec2 suv = clip.xy / clip.w * 0.5 + 0.5;\n"
	"    if (suv.x < 0.0 || suv.x > 1.0 || suv.y < 0.0 || suv.y > 1.0) break;\n"
	"    vec4 sd = texture2D(u_depth, suv);\n"
	"    if (sd.a < 0.5) { t += step; continue; }\n"
	"    float scene_linz = gpu_unpack_depth(sd.rgb) / u_pack;\n"
	"    float ray_linz = -p.z;\n"
	"    if (scene_linz < ray_linz - tol && scene_linz > ray_linz - tol * 8.0) {\n"
	"      return texture2D(u_albedo, suv).rgb;\n"
	"    }\n"
	"    t += step;\n"
	"  }\n"
	"  return vec3(0.0);\n"
	"}\n"
	"/* SSAO: 8 образцов вокруг пикселя; образец ближе к камере -> окклюзия. */\n"
	"float gpu_ssao(vec2 uv, vec3 vpos, float linz, vec3 n) {\n"
	"  float occ = 0.0;\n"
	"  for (int i = 0; i < 8; i++) {\n"
	"    float ang = float(i) * 0.785398;\n"
	"    vec2 off = vec2(cos(ang), sin(ang)) * (float(i % 2) * 2.0 + 1.0) * u_texel * 2.0;\n"
	"    vec2 suv = uv + off;\n"
	"    if (suv.x < 0.0 || suv.x > 1.0 || suv.y < 0.0 || suv.y > 1.0) continue;\n"
	"    vec4 sd = texture2D(u_depth, suv);\n"
	"    if (sd.a < 0.5) continue;\n"
	"    float s_linz = gpu_unpack_depth(sd.rgb) / u_pack;\n"
	"    if (s_linz >= linz - 0.02) continue;\n"
	"    vec4 cp = u_inv_proj * vec4(suv * 2.0 - 1.0, -1.0, 1.0);\n"
	"    vec3 dir = cp.xyz / cp.w;\n"
	"    vec3 sp = (u_ortho > 0.5) ? vec3(dir.xy, -s_linz) : dir * (s_linz / max(-dir.z, 1e-6));\n"
	"    vec3 dvec = sp - vpos;\n"
	"    float dist = length(dvec);\n"
	"    if (dist < 0.25 && dist > 1e-4) {\n"
	"      float nd = dot(n, dvec / dist);\n"
	"      if (nd > 0.0) occ += (1.0 - dist / 0.25) * nd;\n"
	"    }\n"
	"  }\n"
	"  return clamp(1.0 - occ * 0.6, 0.0, 1.0);\n"
	"}\n"
	"vec2 gpu_texuv(vec2 uv, vec3 orco, float is_orco, vec2 size, vec2 ofs, float map_type, vec3 n){\n"
	"  if (is_orco > 0.5) {\n"
	"    vec2 t;\n"
	"    if (map_type < 0.5) {\n"        /* FLAT */
	"      t = (orco.xy + 1.0) * 0.5;\n"
	"    } else if (map_type < 1.5) {\n" /* CUBE */
	"      float ax = abs(n.x), ay = abs(n.y), az = abs(n.z);\n"
	"      if (az >= ax && az >= ay) t = (orco.xy + 1.0) * 0.5;\n"
	"      else if (ay >= ax && ay >= az) t = vec2((orco.x + 1.0) * 0.5, (orco.z + 1.0) * 0.5);\n"
	"      else t = vec2((orco.y + 1.0) * 0.5, (orco.z + 1.0) * 0.5);\n"
	"    } else if (map_type < 2.5) {\n" /* TUBE */
	"      float len = length(orco.xy);\n"
	"      if (len > 0.0) t = vec2((1.0 - atan(orco.x / len, orco.y / len) / 3.14159) * 0.5, (orco.z + 1.0) * 0.5);\n"
	"      else t = vec2(0.0);\n"
	"    } else {\n"                     /* SPHERE */
	"      float len = length(orco);\n"
	"      if (len > 0.0) {\n"
	"        float u = (abs(orco.x) < 1e-6 && abs(orco.y) < 1e-6) ? 0.0 : (1.0 - atan(orco.x, orco.y) / 3.14159) * 0.5;\n"
	"        float v = 1.0 - acos(orco.z / len) / 3.14159;\n"
	"        t = vec2(u, v);\n"
	"      } else t = vec2(0.0);\n"
	"    }\n"
	"    return size * (t - 0.5) + ofs + 0.5;\n"
	"  }\n"
	"  return uv;\n"
	"}\n"
	"vec3 gpu_texblend(vec3 outc, vec3 tex, float fact, float facg, int bt){\n"
	"  float facm;\n"
	"  if (bt == 0) { fact *= facg; facm = 1.0 - fact; return fact*tex + facm*outc; }\n"
	"  if (bt == 1) { fact *= facg; facm = 1.0 - fact; return (facm + fact*tex) * outc; }\n"
	"  if (bt == 2) { fact *= facg; return fact*tex + outc; }\n"
	"  if (bt == 3) { fact = -fact; fact *= facg; return fact*tex + outc; }\n"
	"  if (bt == 4) { fact *= facg; facm = 1.0 - fact;\n"
	"    vec3 r = outc;\n"
	"    if (tex.r != 0.0) r.r = facm*outc.r + fact*outc.r/tex.r;\n"
	"    if (tex.g != 0.0) r.g = facm*outc.g + fact*outc.g/tex.g;\n"
	"    if (tex.b != 0.0) r.b = facm*outc.b + fact*outc.b/tex.b;\n"
	"    return r; }\n"
	"  if (bt == 5) { fact *= facg; facm = 1.0 - fact; return min(outc, tex)*fact + outc*facm; }\n"
	"  if (bt == 6) { fact *= facg; facm = 1.0 - fact; return facm*outc + fact*abs(tex - outc); }\n"
	"  if (bt == 7) { fact *= facg; return max(fact*tex, outc); }\n"
	"  if (bt == 8) { fact *= facg; facm = 1.0 - fact; return 1.0 - (facm + fact*(1.0-tex))*(1.0-outc); }\n"
	"  if (bt == 9) { fact *= facg; facm = 1.0 - fact;\n"
	"    vec3 r;\n"
	"    if (outc.r < 0.5) r.r = outc.r * (facm + 2.0*fact*tex.r);\n"
	"    else r.r = 1.0 - (facm + 2.0*fact*(1.0-tex.r))*(1.0-outc.r);\n"
	"    if (outc.g < 0.5) r.g = outc.g * (facm + 2.0*fact*tex.g);\n"
	"    else r.g = 1.0 - (facm + 2.0*fact*(1.0-tex.g))*(1.0-outc.g);\n"
	"    if (outc.b < 0.5) r.b = outc.b * (facm + 2.0*fact*tex.b);\n"
	"    else r.b = 1.0 - (facm + 2.0*fact*(1.0-tex.b))*(1.0-outc.b);\n"
	"    return r; }\n"
	"  if (bt == 15) { fact *= facg; facm = 1.0 - fact;\n"
	"    return facm*outc + fact*(outc*tex) + outc*(1.0 - (1.0-tex)*(1.0-outc)); }\n"
	"  if (bt == 16) { fact *= facg;\n"
	"    vec3 r;\n"
	"    r.r = outc.r + fact*(tex.r > 0.5 ? 2.0*(tex.r-0.5) : 2.0*tex.r - 1.0);\n"
	"    r.g = outc.g + fact*(tex.g > 0.5 ? 2.0*(tex.g-0.5) : 2.0*tex.g - 1.0);\n"
	"    r.b = outc.b + fact*(tex.b > 0.5 ? 2.0*(tex.b-0.5) : 2.0*tex.b - 1.0);\n"
	"    return r; }\n"
	"  fact *= facg; facm = 1.0 - fact; return fact*tex + facm*outc;\n"
	"}\n"
	"/* ★★ раунд 83-fix3: фактор смешивания текстуры (`tin`). BI (render_texture.c:2554)\n"
	"   зовёт texture_rgb_blend(..., texres.tin, colfac, blendtype), а tin задан в\n"
	"   render_texture.c:2440: для ОБРАЗНОЙ карты это 1.0, и лишь при MTEX_RGBTOINT —\n"
	"   её яркость. Процедурная текстура отдаёт свою интенсивность в tc.a.\n"
	"   Раньше для образных бралась tc.a (АЛЬФА картинки), что в 2-3 раза ослабляло\n"
	"   модуляцию цвета. */\n"
	"float gpu_tex_tin(vec4 tc, float rgbtoint, float is_proc){\n"
	"  if (is_proc > 0.5) return tc.a;\n"
	"  return (rgbtoint > 0.5) ? dot(tc.rgb, vec3(0.2126, 0.7152, 0.0722)) : 1.0;\n"
	"}\n"
	"float gpu_valblend(float def, float val, float tin, float fac, int bt){\n"
	"  if (bt == 0) return def * (1.0 - fac*tin) + val * fac*tin;\n"
	"  if (bt == 1) return def * (1.0 - fac) + val * fac*tin;\n"
	"  if (bt == 2) return def + val * fac*tin;\n"
	"  if (bt == 3) return def - val * fac*tin;\n"
	"  if (bt == 4) { if (val != 0.0) return def*(1.0-fac) + def*fac*tin/val; else return def; }\n"
	"  if (bt == 5) return min(def, val) * fac*tin + def*(1.0-fac);\n"
	"  if (bt == 6) return def*(1.0-fac) + fac*tin*abs(val-def);\n"
	"  if (bt == 7) return max(def, fac*tin*val);\n"
	"  if (bt == 8) return 1.0 - (1.0-fac + fac*(1.0-val))*(1.0-def);\n"
	"  if (bt == 9) { if (def < 0.5) return def*(1.0-fac + 2.0*fac*val); else return 1.0 - (1.0-fac + 2.0*fac*(1.0-val))*(1.0-def); }\n"
	"  if (bt == 15) return def*(1.0-fac) + fac*(def*val) + def*(1.0 - (1.0-val)*(1.0-def));\n"
	"  if (bt == 16) return def + fac*(val > 0.5 ? 2.0*(val-0.5) : 2.0*val - 1.0);\n"
	"  return def*(1.0 - fac*tin) + val*fac*tin;\n"
	"}\n"
	"float gpu_spec(float inp, int hard)\n"
	"{\n"
	"  float b1;\n"
	"  if (inp >= 1.0) return 1.0;\n"
	"  if (inp <= 0.0) return 0.0;\n"
	"  b1 = inp * inp;\n"
	"  if (b1 < 0.01) b1 = 0.01;\n"
	"  if ((hard % 2) == 0) inp = 1.0;\n"
	"  if ((hard % 4) >= 2)     inp *= b1;\n"
	"  b1 *= b1;\n"
	"  if ((hard % 8) >= 4)     inp *= b1;\n"
	"  b1 *= b1;\n"
	"  if ((hard % 16) >= 8)    inp *= b1;\n"
	"  b1 *= b1;\n"
	"  if ((hard % 32) >= 16)   inp *= b1;\n"
	"  b1 *= b1;\n"
	"  if (b1 < 0.001) b1 = 0.0;\n"
	"  if ((hard % 64) >= 32)   inp *= b1;\n"
	"  b1 *= b1;\n"
	"  if ((hard % 128) >= 64)  inp *= b1;\n"
	"  b1 *= b1;\n"
	"  if ((hard % 256) >= 128) inp *= b1;\n"
	"  if (b1 < 0.001) b1 = 0.0;\n"
	"  if (hard >= 256) { b1 *= b1; inp *= b1; }\n"
	"  return inp;\n"
	"}\n"
	"/* ★★ Раунд 83: закрывает ли окклюдер из карты этот фрагмент.\n"
	"   lpz.y != 0 (спот): карта ПЕРСПЕКТИВНАЯ, window-глубина нелинейна\n"
	"     (z_ndc = A + B/d, A=lpz.x, B=lpz.y), поэтому сравниваем РАССТОЯНИЯ\n"
	"     вдоль оси света. Иначе (солнце, ортопроекция): window-глубина линейна,\n"
	"     сравниваем её напрямую. */\n"
	"bool gpu_occludes(float fz, vec3 packed, float fd, vec2 lpz, float wbias, float zbias)\n"
	"{\n"
	"  float occ = gpu_unpack_depth(packed);\n"
	"  if (lpz.y != 0.0) {\n"
	"    float den = (2.0 * occ - 1.0) - lpz.x;\n"
	"    if (den >= -1e-6) return false;\n"
	"    float docc = lpz.y / den;\n"
	"    return (docc > 0.0 && fd - wbias > docc);\n"
	"  }\n"
	"  return (fz - zbias > occ);\n"
	"}\n"
	"float gpu_shadow(sampler2D smap, mat4 lvp, vec3 wp, vec3 n, vec3 L, float soft, vec2 lpz)\n"
	"{\n"
	"  vec4 lp = lvp * vec4(wp, 1.0);\n"
	"  vec3 sc = (lp.xyz / lp.w) * 0.5 + 0.5;\n"
	"  if (sc.x < 0.0 || sc.x > 1.0 || sc.y < 0.0 || sc.y > 1.0 ||\n"
	"      sc.z < 0.0 || sc.z > 1.0) return 1.0;\n"
	"  /* ★★ Раунд 83: у ПЕРСПЕКТИВНОЙ карты спота window-глубина нелинейна\n"
	"     (dz/dd = 2fn/((f-n)d^2)), поэтому константный bias в window-единицах\n"
	"     превращался в МИРОВОЙ порядка 0.0125*d^2 — на d=10 это 1.25 юнита, на\n"
	"     d=20 уже 5. Свет проходил сквозь тонкие стены (в mapFull 20 спотов,\n"
	"     n=clipsta, f=clipend): «чёрная» зона scanline светилась ~10/255.\n"
	"     Теперь сравниваем МИРОВЫЕ расстояния, а мировой размер текселя растёт\n"
	"     как ~0.0043*d, поэтому bias пропорционален d. У солнца (ортопроекция)\n"
	"     lpz=(0,0), для него остаётся прежняя ветка по window-глубине. */\n"
	"  float slope = 1.0 - dot(n, L);\n"
	"  float wbias = max(0.012 * slope, 0.005) * max(lp.w, 1e-4) * u_shadow_bias;\n"
	"  float zbias = max(0.0025 * slope, 0.0008);\n"
	"  float sh = 0.0;\n"
	"  if (u_pcf < 0.5) {\n"
	"    vec4 s = texture2D(smap, sc.xy);\n"
	"    if (s.a >= 0.5 && gpu_occludes(sc.z, s.rgb, lp.w, lpz, wbias, zbias)) sh = 1.0;\n"
	"    return 1.0 - sh;\n"
	"  } else {\n"
	"    float r = 1.0 + soft * 0.04;\n"
	"    for (int dy = -1; dy <= 1; dy++)\n"
	"      for (int dx = -1; dx <= 1; dx++) {\n"
	"        vec2 uv = sc.xy + vec2(float(dx), float(dy)) * u_shadow_texel * r;\n"
	"        vec4 s = texture2D(smap, uv);\n"
	"        if (s.a >= 0.5 && gpu_occludes(sc.z, s.rgb, lp.w, lpz, wbias, zbias)) sh += 1.0;\n"
	"      }\n"
	"    return 1.0 - sh * (1.0 / 9.0);\n"
	"  }\n"
	"}\n"
	"float gpu_shadow_point(samplerCube smap, vec3 dir, float d, vec2 lpz, vec3 n, vec3 L, float soft)\n"
	"{\n"
	"  float bias = max(0.02 * (1.0 - dot(n, L)), 0.005) * d;\n"
	"  float sh = 0.0;\n"
	"  if (u_pcf < 0.5) {\n"
	"    vec4 s = textureCube(smap, dir);\n"
	"    if (s.a >= 0.5) {\n"
	"      float den = (2.0 * gpu_unpack_depth(s.rgb) - 1.0) - lpz.x;\n"
	"      if (den < -1e-6 && d - bias > lpz.y / den) sh = 1.0;\n"
	"    }\n"
	"    return 1.0 - sh;\n"
	"  } else {\n"
	"    vec3 up = (abs(dir.y) < 0.99) ? vec3(0.0, 1.0, 0.0) : vec3(1.0, 0.0, 0.0);\n"
	"    vec3 t1 = normalize(cross(dir, up));\n"
	"    vec3 t2 = cross(dir, t1);\n"
	"    float e = u_shadow_texel.x * 2.0 * (1.0 + soft * 0.04);\n"
	"    float sh = 0.0;\n"
	"    for (int dy = -1; dy <= 1; dy++)\n"
	"      for (int dx = -1; dx <= 1; dx++) {\n"
	"        vec3 sd = dir + t1 * (float(dx) * e) + t2 * (float(dy) * e);\n"
	"        vec4 s = textureCube(smap, sd);\n"
	"        float dd = (s.a >= 0.5) ? lpz.y / ((2.0 * gpu_unpack_depth(s.rgb) - 1.0) - lpz.x) : -1.0;\n"
	"        if (dd > 0.0 && d - bias > dd) sh += 1.0;\n"
	"      }\n"
	"    return 1.0 - sh * (1.0 / 9.0);\n"
	"  }\n"
	"}\n"
	"vec3 gpu_light(vec3 P, vec3 N, vec3 V, vec4 alb, vec4 sspec, vec4 posT, vec4 dirD,\n"
	"               vec4 colF, vec4 fl, vec3 wp, sampler2D smap, mat4 lvp,\n"
	"               samplerCube scube, vec3 lwpos, vec2 lpz, float soft)\n"
	"{\n"
	"  float type = posT.w;\n"
	"  vec3 lv;\n"
	"  float visifac = 1.0;\n"
	"  if (type < 0.5) {\n"
	"    lv = dirD.xyz;\n"
	"  } else {\n"
	"    lv = P - posT.xyz;\n"
	"    float dist = length(lv);\n"
	"    lv = (dist > 1e-6) ? lv / dist : vec3(0.0, 0.0, 1.0);\n"
	"    if (type < 2.5) {\n"
	"      float ld = dirD.w;\n"
	"      if (colF.a < 0.5)      visifac = 1.0;\n"
	"      else if (colF.a < 1.5) visifac = ld / (ld + dist);\n"
	"      else                   visifac = ld / (ld + dist * dist);\n"
	"      if (fl.x > 0.5) {\n"
	"        float t = ld - dist;\n"
	"        visifac = (t <= 0.0) ? 0.0 : visifac * (t / ld);\n"
	"      }\n"
	"      if (visifac > 0.0 && type > 1.5) {\n"
	"        float inpr = dot(lv, dirD.xyz);\n"
	"        float t = fl.y;\n"
	"        if (inpr <= t) visifac = 0.0;\n"
	"        else {\n"
	"          t = inpr - t;\n"
	"          if (t < fl.z && fl.z != 0.0) {\n"
	"            float i = t / fl.z;\n"
	"            t = i * i;\n"
	"            inpr *= (3.0 * t - 2.0 * t * i);\n"
	"          }\n"
	"          visifac *= inpr;\n"
	"        }\n"
	"      }\n"
	"      if (visifac <= 0.001) visifac = 0.0;\n"
	"    }\n"
	"  }\n"
	"  if (visifac <= 0.0) return vec3(0.0);\n"
	"  vec3 Ld = -lv;\n"
	"  float sh = 1.0;\n"
	"  if (fl.w > 0.5) {\n"
	"    if (type < 0.5 || (type > 1.5 && type < 2.5))\n"
	"      sh = gpu_shadow(smap, lvp, wp, N, Ld, soft, lpz);\n"
	"    else if (type < 1.5 || (type > 2.5 && type < 3.5)) {\n"
	"      vec3 pd = wp - lwpos;\n"
	"      float plen = length(pd);\n"
	"      float pax = max(max(abs(pd.x), abs(pd.y)), abs(pd.z));\n"
	"      if (plen > 1e-6) sh = gpu_shadow_point(scube, pd / plen, pax, lpz, N, Ld, soft);\n"
	"    }\n"
	"  }\n"
	"  /* ★★ Раунд 83-fix2: ОТКАТ «квадрата Ламберта». Проверка по исходнику:\n"
	"     add_to_diffuse(shadeoutput.c:958) принимает `is` пятым параметром, но в\n"
	"     ОБЫЧНОЙ ветке (1005-1009, без color-ramp) делает просто\n"
	"         diff[0] += rgb[0] * shi->r;\n"
	"     где rgb = tcol = i*lacol, а i = is*phongcorr*visifac*shi->refl (1452/1455).\n"
	"     То есть `is` входит РОВНО ОДИН раз, а параметр `is` нужен только ветке\n"
	"     color-ramp (981: fac = is для MA_RAMP_IN_SHADER). Диффуз BI =\n"
	"     shi->r * lacol * is * phongcorr * visifac * refl. Экспонента 1.\n"
	"     Квадрат давал лучшее СРЕДНЕЕ по mapFull (отношение кадров 1.350 -> 0.973),\n"
	"     но это была компенсация другой ошибки: полосы оставались немонотонными\n"
	"     (2.43x в тусклых, 0.81x в 32-64), а тени-выключенные тесты и точечные лампы\n"
	"     становились на 7-16% темнее scanline. Держать неверную формулу нельзя. */\n"
	"  float diff = max(dot(N, Ld), 0.0);\n"
	"  vec3 Hs = Ld + V;\n"
	"  float hl = length(Hs);\n"
	"  vec3 H = (hl > 1e-6) ? Hs / hl : N;\n"
	"  float nh = dot(N, H);\n"
	"  float specfac = 0.0;\n"
	"  if (nh > 0.0) {\n"
	"    float nv = max(dot(N, V), 0.0);\n"
	"    specfac = gpu_spec(nh, int(sspec.a * 511.0 + 0.5)) / (0.1 + nv);\n"
	"  }\n"
	"  vec3 spec = sspec.rgb * (alb.a * specfac);\n"
	"  return (alb.rgb * diff + spec) * colF.rgb * visifac * sh;\n"
	"}\n"
	/* ★ M19: процедурная карта цвета (CLOUDS/BLEND) — тот же блок, что в GB_FS
	   (шейдеры GLSL 1.10 без #include, поэтому текст продублирован, как и
	   gpu_texuv/gpu_texblend; юниты задаёт C-сторона). */
	"uniform sampler2D u_proc_hash;\n"    /* 512x1: hash[i]/255 */
	"uniform sampler2D u_proc_hashv;\n"   /* 256x1: (grad+1)/2 */
	"uniform sampler2D u_proc_ramp;\n"    /* 256x1: запечённый колорбэнд */
	"uniform sampler2D u_proc_hashp;\n"   /* 256x1 RGB8: Voronoi feature points (hashpntf) */
	"uniform int u_proc_type;\n"          /* 0 нет, 1 CLOUDS, 5 BLEND */
	"uniform float u_proc_col;\n"         /* M20: 1 = текстура даёт MAP_COL */
	"uniform int u_proc_stype;\n"
	"uniform int u_proc_flip;\n"
	"uniform float u_proc_noisesize;\n"
	"uniform int u_proc_noisedepth;\n"
	"uniform int u_proc_basis;\n"
	"uniform int u_proc_wf;\n"           /* ★ M31: waveform WOOD: 0=SIN,1=SAW,2=TRI */
	"uniform float u_proc_dist;\n"       /* ★ M31: DISTORTED_NOISE tex->dist_amount */
	"uniform int u_proc_hard;\n"
	"uniform float u_proc_bright;\n"
	"uniform float u_proc_contrast;\n"
	"uniform int u_proc_noclamp;\n"
	"uniform int u_proc_has_ramp;\n"
	"uniform int u_proc_orco;\n"
	"uniform vec2 u_proc_size;\n"
	"uniform vec2 u_proc_ofs;\n"
	"float gpu_p_hash(int i){ return texture2D(u_proc_hash, vec2((float(i)+0.5)/512.0, 0.5)).r*255.0; }\n"
	"vec3 gpu_p_hashv(int i){ return texture2D(u_proc_hashv, vec2((float(i)+0.5)/256.0, 0.5)).rgb*2.0-1.0; }\n"
	"float gpu_p_grad(int i, float x, float y, float z){\n"
	"  vec3 g = gpu_p_hashv(i);\n"
	"  return g.x*x + g.y*y + g.z*z;\n"
	"}\n"
	"int gpu_p_mod256(int v){ return v - 256*int(floor(float(v)/256.0)); }\n"
	/* M20: Voronoi (noise.c voronoi + HASHPNT). hashpntf -> u_proc_hashp (256x1 RGB8),
	   hash[] -> u_proc_hash. Индекс: hash[hash[hash[z&255]+y)&255]+x)&255]. */
	"int gpu_p_hashpnt_idx(int x, int y, int z){\n"
	"  int h0 = int(gpu_p_hash(gpu_p_mod256(z)) + 0.5);\n"
	"  int h1 = int(gpu_p_hash(gpu_p_mod256(h0 + y)) + 0.5);\n"
	"  int h2 = int(gpu_p_hash(gpu_p_mod256(h1 + x)) + 0.5);\n"
	"  return h2;\n"
	"}\n"
	"vec3 gpu_p_hashp(int i){ return texture2D(u_proc_hashp, vec2((float(i)+0.5)/256.0, 0.5)).rgb; }\n"
	/* Возвращает (F1, F2) — два ближайших расстояния Voronoi (dist_Real, dtype=0). */
	"vec4 gpu_p_voronoi4(float x, float y, float z){\n"
	"  int xi=int(floor(x)), yi=int(floor(y)), zi=int(floor(z));\n"
	"  float d0=1e10, d1=1e10, d2=1e10, d3=1e10;\n"
	"  for (int ox=-1; ox<=1; ox++) {\n"
	"    for (int oy=-1; oy<=1; oy++) {\n"
	"      for (int oz=-1; oz<=1; oz++) {\n"
	"        vec3 p = gpu_p_hashp(gpu_p_hashpnt_idx(xi+ox, yi+oy, zi+oz));\n"
	"        float xd = x - (p.x + float(xi+ox));\n"
	"        float yd = y - (p.y + float(yi+oy));\n"
	"        float zd = z - (p.z + float(zi+oz));\n"
	"        float d = sqrt(xd*xd + yd*yd + zd*zd);\n"
	"        if (d < d0) { d3 = d2; d2 = d1; d1 = d0; d0 = d; }\n"
	"        else if (d < d1) { d3 = d2; d2 = d1; d1 = d; }\n"
	"        else if (d < d2) { d3 = d2; d2 = d; }\n"
	"        else if (d < d3) { d3 = d; }\n"
	"      }\n"
	"    }\n"
	"  }\n"
	"  return vec4(d0, d1, d2, d3);\n"
	"}\n"
	"float gpu_p_orgnoise(float x, float y, float z){\n"
	"  float fx=floor(x), fy=floor(y), fz=floor(z);\n"
	"  float ox=x-fx, oy=y-fy, oz=z-fz;\n"
	"  float jx=ox-1.0, jy=oy-1.0, jz=oz-1.0;\n"
	"  float cn1=ox*ox, cn2=oy*oy, cn3=oz*oz, cn4=jx*jx, cn5=jy*jy, cn6=jz*jz;\n"
	"  cn1 = 1.0 - 3.0*cn1 + 2.0*cn1*ox;\n"
	"  cn2 = 1.0 - 3.0*cn2 + 2.0*cn2*oy;\n"
	"  cn3 = 1.0 - 3.0*cn3 + 2.0*cn3*oz;\n"
	"  cn4 = 1.0 - 3.0*cn4 - 2.0*cn4*jx;\n"
	"  cn5 = 1.0 - 3.0*cn5 - 2.0*cn5*jy;\n"
	"  cn6 = 1.0 - 3.0*cn6 - 2.0*cn6*jz;\n"
	"  int ix=gpu_p_mod256(int(fx)), iy=gpu_p_mod256(int(fy)), iz=gpu_p_mod256(int(fz));\n"
	"  int ix1=gpu_p_mod256(int(fx)+1), iy1=gpu_p_mod256(int(fy)+1), iz1=gpu_p_mod256(int(fz)+1);\n"
	"  int b00 = int(gpu_p_hash(int(gpu_p_hash(ix) + 0.5) + iy) + 0.5);\n"
	"  int b10 = int(gpu_p_hash(int(gpu_p_hash(ix1) + 0.5) + iy) + 0.5);\n"
	"  int b01 = int(gpu_p_hash(int(gpu_p_hash(ix) + 0.5) + iy1) + 0.5);\n"
	"  int b11 = int(gpu_p_hash(int(gpu_p_hash(ix1) + 0.5) + iy1) + 0.5);\n"
	"  int b20 = iz;\n"
	"  int b21 = iz1;\n"
	"  float n = 0.5;\n"
	"  n += (cn1*cn2*cn3) * gpu_p_grad(int(gpu_p_hash(b20+b00)+0.5), ox, oy, oz);\n"
	"  n += (cn1*cn2*cn6) * gpu_p_grad(int(gpu_p_hash(b21+b00)+0.5), ox, oy, jz);\n"
	"  n += (cn1*cn5*cn3) * gpu_p_grad(int(gpu_p_hash(b20+b01)+0.5), ox, jy, oz);\n"
	"  n += (cn1*cn5*cn6) * gpu_p_grad(int(gpu_p_hash(b21+b01)+0.5), ox, jy, jz);\n"
	"  n += (cn4*cn2*cn3) * gpu_p_grad(int(gpu_p_hash(b20+b10)+0.5), jx, oy, oz);\n"
	"  n += (cn4*cn2*cn6) * gpu_p_grad(int(gpu_p_hash(b21+b10)+0.5), jx, oy, jz);\n"
	"  n += (cn4*cn5*cn3) * gpu_p_grad(int(gpu_p_hash(b20+b11)+0.5), jx, jy, oz);\n"
	"  n += (cn4*cn5*cn6) * gpu_p_grad(int(gpu_p_hash(b21+b11)+0.5), jx, jy, jz);\n"
	"  return clamp(n, 0.0, 1.0);\n"
	"}\n"
	"float gpu_p_turb(float nsize, float x, float y, float z, int oct, int hard, int basis){\n"
	"  if (basis == 0) { x += 1.0; y += 1.0; z += 1.0; }\n"
	"  if (nsize != 0.0) { float s = 1.0/nsize; x *= s; y *= s; z *= s; }\n"
	"  float sum = 0.0, amp = 1.0, fs = 1.0;\n"
	"  for (int i = 0; i < 8; i++) {\n"
	"    if (i > oct) break;\n"
	"    float t;\n"
	"    vec4 vv = vec4(0.0);\n"
	"    if (basis == 8 || basis == 3 || basis == 4 || basis == 5 || basis == 6 || basis == 7)\n"
	"      vv = gpu_p_voronoi4(fs*x, fs*y, fs*z);\n"
	"    if (basis == 8) t = clamp(10.0*(vv.y - vv.x), 0.0, 1.0);\n"
	"    else if (basis == 3) t = vv.x;\n"
	"    else if (basis == 4) t = vv.y;\n"
	"    else if (basis == 5) t = vv.z;\n"
	"    else if (basis == 6) t = vv.w;\n"
	"    else if (basis == 7) t = vv.y - vv.x;\n"
	"    else t = gpu_p_orgnoise(fs*x, fs*y, fs*z);\n"
	"    if (hard == 1) t = abs(2.0*t - 1.0);\n"
	"    sum += t*amp;\n"
	"    amp *= 0.5; fs *= 2.0;\n"
	"  }\n"
	"  sum *= pow(2.0, float(oct)) / (pow(2.0, float(oct+1)) - 1.0);\n"
	"  return sum;\n"
	"}\n"
	"float gpu_p_blend(vec3 tv, int stype, int flip){\n"
	"  float x, y, t, s;\n"
	"  if (flip == 1) { x = tv.y; y = tv.x; } else { x = tv.x; y = tv.y; }\n"
	"  if (stype == 0) return (1.0+x)/2.0;\n"
	"  if (stype == 1) { t = (1.0+x)/2.0; if (t < 0.0) t = 0.0; return t*t; }\n"
	"  if (stype == 2) { t = (1.0+x)/2.0;\n"
	"    if (t <= 0.0) return 0.0;\n"
	"    if (t >= 1.0) return 1.0;\n"
	"    s = t*t; return 3.0*s - 2.0*s*t; }\n"
	"  if (stype == 3) return (2.0+x+y)/4.0;\n"
	"  if (stype == 6) return atan(y, x)/6.28318530718 + 0.5;\n"
	"  t = 1.0 - sqrt(x*x + y*y + tv.z*tv.z);\n"
	"  if (t < 0.0) t = 0.0;\n"
	"  if (stype == 5) t *= t;\n"
	"  return t;\n"
	"}\n"
	"/* ★ M31: signed noise bases for mg_VLNoise (noise.c:1290 voronoi_F*S). */\n"
	"float gpu_p_ns(float x, float y, float z, int b){\n"
	"  if (b == 3) return 2.0*gpu_p_voronoi4(x,y,z).x - 1.0;\n"
	"  if (b == 4) return 2.0*gpu_p_voronoi4(x,y,z).y - 1.0;\n"
	"  if (b == 5) return 2.0*gpu_p_voronoi4(x,y,z).z - 1.0;\n"
	"  if (b == 6) return 2.0*gpu_p_voronoi4(x,y,z).w - 1.0;\n"
	"  if (b == 7) { vec4 v = gpu_p_voronoi4(x,y,z); return 2.0*(v.y - v.x) - 1.0; }\n"
	"  if (b == 8) { vec4 v = gpu_p_voronoi4(x,y,z); float t = min(10.0*(v.y - v.x), 1.0); return 2.0*t - 1.0; }\n"
	"  return 2.0*gpu_p_orgnoise(x, y, z) - 1.0;\n"
	"}\n"
	"/* ★ M31: DISTORTED_NOISE (noise.c:1926 mg_VLNoise / render_texture.c:610 mg_distNoiseTex).\n"
	"   mapFull: dist=1.0, distortion basis = VORONOI_F4 (tex->noisebasis=6), base = BLENDER_ORIGINAL (noisebasis2=0). */\n"
	"float gpu_p_distnoise(vec3 tv, float nsize, float dist, int b1, int b2){\n"
	"  if (nsize != 0.0) { float s = 1.0/nsize; tv *= s; }\n"   /* BI: tmpvec = texvec/noisesize */
	"  float r0 = gpu_p_ns(tv.x + 13.5, tv.y + 13.5, tv.z + 13.5, b1) * dist;\n"
	"  float r1 = gpu_p_ns(tv.x, tv.y, tv.z, b1) * dist;\n"
	"  float r2 = gpu_p_ns(tv.x - 13.5, tv.y - 13.5, tv.z - 13.5, b1) * dist;\n"
	"  return gpu_p_ns(tv.x + r0, tv.y + r1, tv.z + r2, b2);\n"
	"}\n"
	"/* ★ M31: wood (render_texture.c wood_int): для BANDS (stype=0) wi = waveform[wf]((x+y+z)*10).\n"
	"   wf = tex->noisebasis2: 0=SIN (0.5+0.5*sin), 1=SAW, 2=TRI. mapFull wood — BANDS+SIN. */\n"
	"float gpu_p_wood(vec3 tv, int wf){\n"
	"  float a = (tv.x + tv.y + tv.z) * 10.0;\n"
	"  const float b = 6.28318530718;\n"
	"  if (wf == 1) { int n = int(a/b); a -= n*b; if (a < 0.0) a += b; return a/b; }\n"
	"  if (wf == 2) { return 1.0 - 2.0*abs(floor(a/b + 0.5) - a/b); }\n"
	"  return 0.5 + 0.5*sin(a);\n"
	"}\n"
	"vec4 gpu_p_tex_cfg(vec2 uv, vec3 orco, vec3 wpos, vec3 vpos, int porco, vec2 psize, vec2 pofs, int ptype,\n"
	"                   int pstype, int pflip, float pnsize, int pdepth, int phard, int pbasis, int pwf, float pdist,\n"
	"                   float pcontrast, float pbright, int pnoclamp, int phasramp, sampler2D pramp){\n"
	"  vec3 co = (porco == 1) ? orco : ((porco == 2) ? wpos : ((porco == 3) ? vpos : vec3(uv, 0.0)));\n"
	"  vec3 tv = vec3(psize.x*(co.x + pofs.x), psize.y*(co.y + pofs.y), co.z);\n"
	"  float tin;\n"
	"  if (ptype == 5) tin = gpu_p_blend(tv, pstype, pflip);\n"
	"  else if (ptype == 2) tin = gpu_p_wood(tv, pwf);\n"
	"  else if (ptype == 13) tin = gpu_p_distnoise(tv, pnsize, pdist, pbasis, pwf);\n"
	"  else tin = gpu_p_turb(pnsize, tv.x, tv.y, tv.z, pdepth, phard, pbasis);\n"
	"  tin = (tin - 0.5)*pcontrast + pbright - 0.5;\n"
	"  if (pnoclamp == 0) tin = clamp(tin, 0.0, 1.0);\n"
	"  vec3 col = vec3(tin);\n"
	"  if (phasramp == 1)\n"
	"    col = texture2D(pramp, vec2((clamp(tin, 0.0, 1.0)*255.0 + 0.5)/256.0, 0.5)).rgb;\n"
	"  return vec4(col, clamp(tin, 0.0, 1.0));\n"
	"}\n"
	"vec4 gpu_p_tex(vec2 uv, vec3 orco, vec3 wpos, vec3 vpos){\n"
	"  return gpu_p_tex_cfg(uv, orco, wpos, vpos, u_proc_orco, u_proc_size, u_proc_ofs, u_proc_type,\n"
	"                       u_proc_stype, u_proc_flip, u_proc_noisesize, u_proc_noisedepth,\n"
	"                       u_proc_hard, u_proc_basis, u_proc_wf, u_proc_dist, u_proc_contrast, u_proc_bright,\n"
	"                       u_proc_noclamp, u_proc_has_ramp, u_proc_ramp);\n"
	"}\n"
	"void main(){\n"
	"  /* Материал: базовый цвет + диффузная карта (слот 0). */\n"
	"  vec3 base = v_rgba.rgb;\n"
	"  vec4 tc = vec4(1.0);\n"
	"  if (u_has_tex > 0.5) {\n"
	"    if (u_proc_type != 0)\n"
	"      tc = gpu_p_tex(v_uv_tex, v_orco, v_wpos, v_vpos);\n"
	"    else\n"
	"      tc = texture2D(u_tex, gpu_texuv(v_uv_tex, (u_tex_glob > 0.5) ? v_wpos : v_orco, u_tex_orco, u_tex_size, u_tex_ofs, u_tex_map, v_w_nrm));\n"
	"    if (u_proc_col > 0.5)\n"
	"      base = gpu_texblend(base, tc.rgb,\n"
	"                          gpu_tex_tin(tc, u_tex_rgbtoint, (u_proc_type != 0) ? 1.0 : 0.0),\n"
	"                          u_colfac, u_tex_blend);\n"
	"  }\n"
	"  /* Прозрачность: alpha материала, модулированная альфа-картой. */\n"
	"  float gpu_alpha = u_alpha;\n"
	"  if (u_has_atex > 0.5) {\n"
	"    vec4 ac = texture2D(u_atex, gpu_texuv(v_uv_tex, (u_a_glob > 0.5) ? v_wpos : v_orco, u_a_orco, u_a_size, u_a_ofs, u_a_map, v_w_nrm));\n"
	"    float tin = (u_a_use_a > 0.5) ? ac.a\n"
	"              : dot(ac.rgb, vec3(0.2126, 0.7152, 0.0722));\n"
	"    gpu_alpha = gpu_valblend(u_a_def, u_alpha, tin, u_a_fac, u_a_blend);\n"
	"  }\n"
	"  gpu_alpha = clamp(gpu_alpha, 0.0, 1.0);\n"
	"  if (gpu_alpha < 0.001) discard;\n"
	"  /* Затенение непрозрачной сценой: если этот фрагмент ЗА непрозрачной\n"
	"     геометрией (v_linz больше её глубины) — он невидим, отбрасываем. */\n"
	"  vec4 vd = texture2D(u_depth, gl_FragCoord.xy / u_resolution);\n"
	"  if (vd.a >= 0.5) {\n"
	"    float opaque_linz = gpu_unpack_depth(vd.rgb) / u_pack;\n"
	"    if (v_linz > opaque_linz + 0.02) discard;\n"
	"  }\n"
	"  /* Нормаль (опционально возмущённая картой нормалей). */\n"
	"  vec3 n = v_nrm;\n"
	"  if (u_has_nrm > 0.5) {\n"
	"    vec3 nt = texture2D(u_nrm, v_uv_tex).xyz * 2.0 - 1.0;\n"
	"    vec3 N = normalize(n);\n"
	"    vec3 T = v_tangent - dot(v_tangent, N) * N;\n"
	"    float tl = length(T);\n"
	"    if (tl < 1e-6) {\n"
	"      vec3 up = (abs(N.y) < 0.99) ? vec3(0.0, 1.0, 0.0) : vec3(1.0, 0.0, 0.0);\n"
	"      T = normalize(cross(N, up));\n"
	"    } else T /= tl;\n"
	"    vec3 B = cross(N, T) * v_tang_sign;\n"
	"    n = normalize(T * nt.x + B * nt.y + N * nt.z);\n"
	"  }\n"
	"  vec3 V = normalize(-v_vpos);\n"
	"  if (dot(n, V) < 0.0) n = -n;\n"
	"  vec3 wp = v_wpos;\n"
	"  float upness = dot(n, u_up) * 0.5 + 0.5;\n"
	"  vec3 world_hemi = mix(u_world_hor, u_world_zen, upness);\n"
	"  vec3 fill = mix(vec3(0.07, 0.06, 0.06), vec3(0.28, 0.30, 0.34), upness) * u_amb_fill;\n"
	"  vec3 amb = u_world_amb + ((u_env_light > 0.5) ? world_hemi * u_env_energy : vec3(0.0)) + fill;\n"
	"  /* ★ M25 (раунд 82): тот же множитель shi->refl = ma->ref, что и в непрозрачном\n"
	"     проходе (см. GB_FS) — иначе полупрозрачные материалы останутся на 1/0.8\n"
	"     ярче непрозрачных с тем же цветом. Альфа материала не трогается. */\n"
	"  vec4 albedo = vec4(base * u_refl, v_rgba.a);\n"
	"  vec4 sspec = vec4(u_spec_col, u_hard);\n"
	"  /* SSAO: та же окклюзия, что у непрозрачных поверхностей (по глубине сцены). */\n"
	"  float ao = (u_ao > 0.5) ? gpu_ssao(gl_FragCoord.xy / u_resolution, v_vpos, v_linz, n) : 1.0;\n"
	"  vec3 lit = vec3(0.0);\n"
	"  /* ★★ Раунд 16: цикл по ВСЕМ лампам сцены (раньше — жёстко 10 вызовов).\n"
	"     Данные лампы берём из float-текстуры: gpu_ltap(li,row). Тень умеют только\n"
	"     первые 10 ламп (GPU_MAX_SHADOW_LIGHTS — их число задано парой сэмплеров на\n"
	"     лампу), поэтому для li>=10 берётся fl.w=0 и заглушки-сэмплеры:\n"
	"     ветка тени в gpu_light не исполняется, лампа просто светит. */\n"
	"  for (int li = 0; li < u_num_lights; li++) {\n"
	"    vec4 lfl = gpu_ltap(li, 3);        /* sphere, spotsi, spotbl, shadows */\n"
	"    float lsf = gpu_ltap(li, 4).x;     /* lar->soft — мягкость тени */\n"
	"    if (li == 0)       lit += gpu_light(v_vpos, n, V, albedo, sspec, gpu_ltap(0,0), gpu_ltap(0,1), gpu_ltap(0,2), lfl, wp, u_shadow[0], u_lvp[0], u_shadow_cube[0], u_lwpos[0], u_lpz[0], lsf);\n"
	"    else if (li == 1)  lit += gpu_light(v_vpos, n, V, albedo, sspec, gpu_ltap(1,0), gpu_ltap(1,1), gpu_ltap(1,2), lfl, wp, u_shadow[1], u_lvp[1], u_shadow_cube[1], u_lwpos[1], u_lpz[1], lsf);\n"
	"    else if (li == 2)  lit += gpu_light(v_vpos, n, V, albedo, sspec, gpu_ltap(2,0), gpu_ltap(2,1), gpu_ltap(2,2), lfl, wp, u_shadow[2], u_lvp[2], u_shadow_cube[2], u_lwpos[2], u_lpz[2], lsf);\n"
	"    else if (li == 3)  lit += gpu_light(v_vpos, n, V, albedo, sspec, gpu_ltap(3,0), gpu_ltap(3,1), gpu_ltap(3,2), lfl, wp, u_shadow[3], u_lvp[3], u_shadow_cube[3], u_lwpos[3], u_lpz[3], lsf);\n"
	"    else if (li == 4)  lit += gpu_light(v_vpos, n, V, albedo, sspec, gpu_ltap(4,0), gpu_ltap(4,1), gpu_ltap(4,2), lfl, wp, u_shadow[4], u_lvp[4], u_shadow_cube[4], u_lwpos[4], u_lpz[4], lsf);\n"
	"    else if (li == 5)  lit += gpu_light(v_vpos, n, V, albedo, sspec, gpu_ltap(5,0), gpu_ltap(5,1), gpu_ltap(5,2), lfl, wp, u_shadow[5], u_lvp[5], u_shadow_cube[5], u_lwpos[5], u_lpz[5], lsf);\n"
	"    else if (li == 6)  lit += gpu_light(v_vpos, n, V, albedo, sspec, gpu_ltap(6,0), gpu_ltap(6,1), gpu_ltap(6,2), lfl, wp, u_shadow[6], u_lvp[6], u_shadow_cube[6], u_lwpos[6], u_lpz[6], lsf);\n"
	"    else if (li == 7)  lit += gpu_light(v_vpos, n, V, albedo, sspec, gpu_ltap(7,0), gpu_ltap(7,1), gpu_ltap(7,2), lfl, wp, u_shadow[7], u_lvp[7], u_shadow_cube[7], u_lwpos[7], u_lpz[7], lsf);\n"
	"    else if (li == 8)  lit += gpu_light(v_vpos, n, V, albedo, sspec, gpu_ltap(8,0), gpu_ltap(8,1), gpu_ltap(8,2), lfl, wp, u_shadow[8], u_lvp[8], u_shadow_cube[8], u_lwpos[8], u_lpz[8], lsf);\n"
	"    else if (li == 9)  lit += gpu_light(v_vpos, n, V, albedo, sspec, gpu_ltap(9,0), gpu_ltap(9,1), gpu_ltap(9,2), lfl, wp, u_shadow[9], u_lvp[9], u_shadow_cube[9], u_lwpos[9], u_lpz[9], lsf);\n"
	"    else               lit += gpu_light(v_vpos, n, V, albedo, sspec, gpu_ltap(li,0), gpu_ltap(li,1), gpu_ltap(li,2), vec4(lfl.xyz, 0.0), wp, u_shadow[0], u_lvp[0], u_shadow_cube[0], u_lwpos[0], u_lpz[0], lsf);\n"
	"  }\n"
	"  if (u_expo_on > 0.5) lit = u_linfac * (1.0 - exp(lit * u_logfac));\n"
	"  vec3 col = base * amb * ao + lit;\n"
	"  col *= u_exposure;\n"
	"  col += base * v_emit;\n"
	"  /* SSR: подмешиваем отражённый цвет сцены (как в LIGHT_FS). */\n"
	"  if (u_ssr > 0.5) {\n"
	"    vec3 refl = gpu_ssr(gl_FragCoord.xy / u_resolution, v_vpos, n, V);\n"
	"    if (refl != vec3(0.0)) col = mix(col, refl, 0.35);\n"
	"  }\n"
	"  gl_FragColor = vec4(gpu_to_srgb(col), gpu_alpha);\n"
	"}\n";
static void gpu_res_free(RE_GPUGPURes *r)
{
	if (!r) return;

	if (r->prog_gb)   glDeleteProgram(r->prog_gb);
	if (r->prog_light) glDeleteProgram(r->prog_light);
	if (r->prog_shadow) glDeleteProgram(r->prog_shadow);
	if (r->prog_forward) glDeleteProgram(r->prog_forward);
	if (r->prog_halo) glDeleteProgram(r->prog_halo);   /* M16: спрайты HALO */
	if (r->tex_albedo) glDeleteTextures(1, &r->tex_albedo);
	if (r->tex_nrm)   glDeleteTextures(1, &r->tex_nrm);
	if (r->tex_vpos)  glDeleteTextures(1, &r->tex_vpos);
	if (r->tex_spec)  glDeleteTextures(1, &r->tex_spec);
	if (r->tex_refl)  glDeleteTextures(1, &r->tex_refl);
	if (r->tex_white)  glDeleteTextures(1, &r->tex_white);
	if (r->tex_spec_def) glDeleteTextures(1, &r->tex_spec_def);
	if (r->tex_refl_def) glDeleteTextures(1, &r->tex_refl_def);
	if (r->tex_hash)   glDeleteTextures(1, &r->tex_hash);    /* M19 */
	if (r->tex_hashv)  glDeleteTextures(1, &r->tex_hashv);   /* M19 */
	if (r->tex_ramp)   glDeleteTextures(1, &r->tex_ramp);    /* M19 */
	if (r->tex_ramp2)  glDeleteTextures(1, &r->tex_ramp2);   /* M22 */
	if (r->tex_hashp)  glDeleteTextures(1, &r->tex_hashp);   /* M20 */
	if (r->tex_cube_white) glDeleteTextures(1, &r->tex_cube_white);
	if (r->rb_depth)   glDeleteRenderbuffersEXT(1, &r->rb_depth);
	if (r->fb)         glDeleteFramebuffersEXT(1, &r->fb);
	if (r->quad_vbo)   glDeleteBuffers(1, &r->quad_vbo);
	if (r->quad_ibo)   glDeleteBuffers(1, &r->quad_ibo);
	for (int i = 0; i < GPU_MAX_SHADOW_LIGHTS; i++) {
		if (r->tex_shadow[i])  glDeleteTextures(1, &r->tex_shadow[i]);
		if (r->tex_shadow_cube[i]) glDeleteTextures(1, &r->tex_shadow_cube[i]);
		if (r->rb_shadow[i])   glDeleteRenderbuffersEXT(1, &r->rb_shadow[i]);
		if (r->shadow_fb[i])   glDeleteFramebuffersEXT(1, &r->shadow_fb[i]);
	}
	/* ★ раунд 16: текстура данных ламп и её CPU-буфер выгрузки */
	if (r->tex_lights) glDeleteTextures(1, &r->tex_lights);
	if (r->light_buf)  MEM_freeN(r->light_buf);

	memset(r, 0, sizeof(*r));
}

static void gpu_upload_mesh(RE_GPUMesh *m)
{
	if (m->uploaded || m->num_verts == 0) return;

	glGenBuffers(1, &m->vbo);
	glBindBuffer(GL_ARRAY_BUFFER, m->vbo);
	glBufferData(GL_ARRAY_BUFFER, sizeof(RE_GPUVertex) * m->num_verts,
	             m->verts, GL_STATIC_DRAW);
	glBindBuffer(GL_ARRAY_BUFFER, 0);

	glGenBuffers(1, &m->ibo);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m->ibo);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(unsigned int) * m->num_indices,
	             m->indices, GL_STATIC_DRAW);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

	/* ★ M16 (раунд 67): индексный буфер ЛИНИЙ для материала WIRE. */
	if (m->num_line_indices > 0) {
		glGenBuffers(1, &m->lbo);
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m->lbo);
		glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(unsigned int) * m->num_line_indices,
		             m->line_indices, GL_STATIC_DRAW);
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
	}

	m->uploaded = true;
}

/* M5: загрузить диффузную текстуру меша в GL (главный поток, GL активен).
 * При любой неудаче m->tex остаётся 0 -> шейдер берёт белый цвет, то есть
 * картинка деградирует к прежнему плоскому albedo, а не ломается. */
/* ★ M24 (раунд 81): загрузка ImBuf в GL-текстуру с МИП-цепочкой BI.
 * BI строит мипы лениво (image_mipmap_test -> IMB_makemipmap, imagetexture.c:883)
 * и сэмплит трилинейно; здесь переиспользуется ровно та же цепочка ibuf->mipmap[],
 * чтобы сильно минифицированные текстуры (глаза персонажей) совпадали со scanline.
 * Раньше грузился только уровень 0 с GL_LINEAR — мипы не работали вовсе.
 * Возвращает GL-текстуру либо 0 (тогда вызывающий оставляет m->tex = 0). */
static unsigned int gpu_upload_ibuf_tex(ImBuf *ibuf, int mip, int gauss)
{
	const int w = ibuf->x, h = ibuf->y;
	/* ★ M30: у образных карт в формате EXR заполнен только rect_float, а
	 * ibuf->rect == NULL — раньше такие текстуры молча не грузились (tex=0).
	 * Для ЦВЕТОВЫХ карт корректна именно байтовая конверсия с colorspace-трансформом. */
	if (w > 0 && h > 0 && !ibuf->rect && ibuf->rect_float) {
		BLI_thread_lock(LOCK_IMAGE);
		IMB_rect_from_float(ibuf);
		BLI_thread_unlock(LOCK_IMAGE);
	}
	unsigned char *rect = (unsigned char *)ibuf->rect;
	GLuint t = 0;
	int lev, maxlev = 0;

	if (w <= 0 || h <= 0 || !rect) return 0;
	/* BI пропускает генерацию мипов для полей (image_mipmap_test) */
	if (mip && (ibuf->flags & IB_fields)) mip = 0;

	if (mip) {
		if (ibuf->mipmap[0] == NULL) {
			BLI_thread_lock(LOCK_IMAGE);
			if (ibuf->mipmap[0] == NULL)
				IMB_makemipmap(ibuf, gauss);
			BLI_thread_unlock(LOCK_IMAGE);
		}
		/* мипы построить не удалось — BI в этом случае снимает TEX_MIPMAP */
		if (ibuf->mipmap[0] == NULL) mip = 0;
	}

	glGenTextures(1, &t);
	glBindTexture(GL_TEXTURE_2D, t);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
	glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
	/* ★★ M26 (раунд 84): sRGB->linear у образных карт. BI при сэмплинге делает
	 * IMB_colormanagement_colorspace_to_scene_linear_v3(tcol, ibuf->rect_colorspace)
	 * (render_texture.c:1274-1278/2542-2546) — т.е. декодирует sRGB в линейное.
	 * Раньше мы грузили ibuf->rect (сырые sRGB-байты) как GL_RGBA без декода, из-за
	 * чего альбедо засвечивалось (например, светлая текстура sRGB 0.8 давала 0.8
	 * вместо линейных 0.6 — пересвет до 1.3x на сцене mapFull). Загружаем как
	 * GL_SRGB8_ALPHA8, когда буфер в sRGB, — GL декодирует при сэмплинге. */
	int is_srgb = 0;
	{
		const char *cs = IMB_colormanagement_get_rect_colorspace(ibuf);
		is_srgb = (cs && strcmp(cs, "sRGB") == 0) ? 1 : 0;
	}
	GLenum ifmt = is_srgb ? GL_SRGB8_ALPHA8 : GL_RGBA;
	glTexImage2D(GL_TEXTURE_2D, 0, ifmt, w, h, 0, GL_RGBA,
	             GL_UNSIGNED_BYTE, rect);
	if (mip) {
		for (lev = 0; lev < IMB_MIPMAP_LEVELS; lev++) {
			ImBuf *mb = ibuf->mipmap[lev];
			if (!mb || !mb->rect || mb->x <= 0 || mb->y <= 0) break;
			glTexImage2D(GL_TEXTURE_2D, lev + 1, ifmt, mb->x, mb->y, 0,
			             GL_RGBA, GL_UNSIGNED_BYTE, mb->rect);
			maxlev = lev + 1;
		}
	}
	/* цепочка должна быть полной от 0 до MAX_LEVEL, иначе текстура неполная
	 * и GL отдаёт чёрный — поэтому MAX_LEVEL ставим только после загрузки */
	if (mip && maxlev > 0) {
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_BASE_LEVEL, 0);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAX_LEVEL, maxlev);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
	}
	else {
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	}
	glBindTexture(GL_TEXTURE_2D, 0);
	return t;
}

static void gpu_upload_mesh_tex(RE_GPUMesh *m)
{
	if (m->tex_tried) return;
	m->tex_tried = 1;
	if (m->ima) {
		Image *ima = (Image *)m->ima;
		void *lock = NULL;
		ImBuf *ibuf = BKE_image_acquire_ibuf(ima, NULL, &lock);
		if (ibuf) {
			/* ★ M24: текстура с мип-цепочкой BI, если у слота включён TEX_MIPMAP */
			m->tex = gpu_upload_ibuf_tex(ibuf, m->tex_mipmap, m->tex_gauss);
			if (getenv("UBF_GPU_DEBUG"))
				printf("[GPU-MIP] ma='%s' ima='%s' mip=%d gauss=%d nlev=%d tex=%u\n",
				       m->ma ? m->ma->id.name + 2 : "<none>", ima->id.name + 2,
				       m->tex_mipmap, m->tex_gauss,
				       (m->tex_mipmap && ibuf->mipmap[0]) ? 1 : 0, m->tex);
			BKE_image_release_ibuf(ima, ibuf, lock);
		}
	}
	/* M14: дополнительные диффузные слоты 1..3 — отдельные текстуры. */
	if (m->n_tex > 1) {
		for (int k = 0; k < GPU_MAX_TEX_SLOTS - 1; k++) {
			if (!m->tex_ima2[k]) continue;
			Image *ima = (Image *)m->tex_ima2[k];
			void *lock = NULL;
			ImBuf *ibuf = BKE_image_acquire_ibuf(ima, NULL, &lock);
			if (!ibuf) continue;
			/* ★ M24: доп. диффузные слоты — та же мип-цепочка BI */
			m->tex2[k] = gpu_upload_ibuf_tex(ibuf, m->tex_mipmap, m->tex_gauss);
			BKE_image_release_ibuf(ima, ibuf, lock);
		}
	}
}

/* M10: загрузка альфа-карты (MAP_ALPHA). Если это ТА ЖЕ картинка, что и
 * диффузная (обычный случай в сценах), отдельная GL-текстура не нужна — в
 * шейдер биндится уже загруженная m->tex. Иначе грузим свою (GL_RGBA, чтобы
 * был альфа-канал). */
static void gpu_upload_mesh_atex(RE_GPUMesh *m)
{
	if (m->a_tex_tried) return;
	m->a_tex_tried = 1;
	if (!m->aima || !m->a_has) return;
	if (m->aima == m->ima) return;   /* та же картинка — переиспользуем m->tex */

	Image *ima = (Image *)m->aima;
	void *lock = NULL;
	ImBuf *ibuf = BKE_image_acquire_ibuf(ima, NULL, &lock);
	if (!ibuf) return;

	const int w = ibuf->x, h = ibuf->y;
	unsigned char *rect = (unsigned char *)ibuf->rect;
	if (w > 0 && h > 0 && rect) {
		GLuint t = 0;
		glGenTextures(1, &t);
		glBindTexture(GL_TEXTURE_2D, t);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
		glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, w, h, 0, GL_RGBA,
		             GL_UNSIGNED_BYTE, rect);
		glBindTexture(GL_TEXTURE_2D, 0);
		m->atex = t;
	}
	BKE_image_release_ibuf(ima, ibuf, lock);
}

/* M8: загрузка нормальной карты (TANGENT-space) в GL-текстуру. Аналог диффузной,
 * но для m->nima; при неудаче m->ntex=0 и шейдер сэмплит белую заглушку, а
 * u_has_nrm остаётся 1 — белая карта (0,0,1) даёт нулевое возмущение. */
static void gpu_upload_mesh_ntex(RE_GPUMesh *m)
{
	if (m->n_tex_tried) return;
	m->n_tex_tried = 1;
	if (!m->nima || !m->n_has) return;

	Image *ima = (Image *)m->nima;
	void *lock = NULL;
	ImBuf *ibuf = BKE_image_acquire_ibuf(ima, NULL, &lock);
	if (!ibuf) return;

	const int w = ibuf->x, h = ibuf->y;
	unsigned char *rect = (unsigned char *)ibuf->rect;
	GLuint t = 0;
	if (w > 0 && h > 0 && rect) {
		glGenTextures(1, &t);
		glBindTexture(GL_TEXTURE_2D, t);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
		glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, w, h, 0, GL_RGBA,
		             GL_UNSIGNED_BYTE, rect);
		glBindTexture(GL_TEXTURE_2D, 0);
	}
	else if (!getenv("UBF_GPU_NO_NRM_FLOAT") && w > 0 && h > 0 && ibuf->rect_float) {
		/* ★ M30: карты нормалей mapFull — EXR (*_nor_gl_1k.exr). У EXR заполнен
		 * только rect_float, а ibuf->rect == NULL, поэтому раньше текстура ВООБЩЕ
		 * не загружалась: m->ntex=0 -> шейдер сэмплил белую заглушку (0,0,1),
		 * dFdx(h)=0 -> bump отсутствовал на всех стенах/полах. Грузим float
		 * напрямую (GL_RGB(A)16F): colorspace-конверсия байтового пути для карт
		 * нормалей недопустима — значения должны попадать как есть. */
		const int nc = ibuf->channels ? ibuf->channels : 4;
		const GLenum fmt = (nc == 3) ? GL_RGB : GL_RGBA;
		const GLint ifmt = (nc == 3) ? GL_RGB16F : GL_RGBA16F;
		glGenTextures(1, &t);
		glBindTexture(GL_TEXTURE_2D, t);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
		glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
		glTexImage2D(GL_TEXTURE_2D, 0, ifmt, w, h, 0, fmt, GL_FLOAT,
		             ibuf->rect_float);
		/* ★ M30: мип-цепочка критична для height-bump: без неё dFdx(h) ловит
		 * алиасинг полного разрешения и bump взрывается (ratio падал до 0.55). */
		if (GLEW_VERSION_3_0 || GLEW_ARB_framebuffer_object)
			glGenerateMipmap(GL_TEXTURE_2D);
		else if (GLEW_EXT_framebuffer_object)
			glGenerateMipmapEXT(GL_TEXTURE_2D);
		glBindTexture(GL_TEXTURE_2D, 0);
		if (getenv("UBF_GPU_DEBUG")) {
			printf("[GPU-NTEX-F] ma='%s' ima='%s' %dx%d ch=%d float->tex=%u\n",
			       m->ma ? m->ma->id.name + 2 : "<none>", ima->id.name + 2,
			       w, h, nc, t);
		}
	}
	if (t) m->ntex = t;
	BKE_image_release_ibuf(ima, ibuf, lock);
}

int RE_gpu_renderer_gl_init(RE_GPURenderer *gr, int width, int height)
{
	if (!gr || !gr->scene) return -1;

	/* уже инициализировано с тем же размером: только дозагружаем новые меши.
	 * Если загрузился хотя бы один НОВЫЙ меш — сцена пересобрана, теневые карты
	 * (зависят от геометрии) надо пересчитать. Иначе кэш теней остаётся валидным. */
	if (gr->gpu_res) {
		RE_GPUGPURes *res = (RE_GPUGPURes *)gr->gpu_res;
		if (res->inited && res->width == width && res->height == height) {
			int any_new = 0;
			for (RE_GPUMesh *m = gr->scene->meshes; m; m = m->next)
				if (!m->uploaded) { gpu_upload_mesh(m); any_new = 1; }
			if (any_new) res->shadow_dirty = true;
			return 0;
		}
	}

	if (gr->gpu_res) {
		gpu_res_free(gr->gpu_res);
		MEM_freeN(gr->gpu_res);
	}
	gr->gpu_res = MEM_callocN(sizeof(RE_GPUGPURes), "RE_GPUGPURes");
	RE_GPUGPURes *r = gr->gpu_res;

	r->width = width;
	r->height = height;
	/* новый GL-ресурс — теневые карты ещё не считались, помечаем кэш грязным */
	r->shadow_dirty = true;

	gl_print_caps();

	/* раскладка VBO задана смещениями-литералами (co=0, n=3, uv=6, rgba=8, emit=12,
	 * tangent=13, orco=17 float); если структура перестанет быть плотной (20 float),
	 * смещения разъедутся */
	if (sizeof(RE_GPUVertex) != 20 * sizeof(float))
		printf("[GPU] !! sizeof(RE_GPUVertex)=%d, ожидалось %d (20 float) — "
		       "смещения атрибутов co=0 n=3 rgba=8 emit=12 orco=17 будут неверны\n",
		       (int)sizeof(RE_GPUVertex), (int)(20 * sizeof(float)));

	/* ★ M5: 4-я цель MRT (цвет блика + hardness) требует 4 color attachment.
	 * Спрашиваем драйвер ДО создания текстур: от ответа зависит и число
	 * аттачментов у FBO, и вариант GB_FS (с записью в gl_FragData[3] или без). */
	{
		GLint maxct = 0, maxdb = 0;
		glGetIntegerv(GL_MAX_COLOR_ATTACHMENTS_EXT, &maxct);
		glGetIntegerv(GL_MAX_DRAW_BUFFERS, &maxdb);
		/* нужны ОБА: 4 аттачмента бессмысленны, если draw buffer'ов меньше */
		r->spec_target = (maxct >= 4 && maxdb >= 4) ? true : false;
		if (!r->spec_target) {
			printf("[GPU] !! 4-й цели G-буфера не будет: "
			       "max_color_attachments=%d max_draw_buffers=%d (нужно >= 4 оба) — "
			       "блик берётся из значений BI по умолчанию (белый, hardness 50)\n",
			       maxct, maxdb);
		}
		/* M17: 5-я цель для MAP_REFL (пер-пиксельная сила отражения). Если нет —
		 * SSR использует фиксированный микс 0.35. */
		r->refl_target = (maxct >= 5 && maxdb >= 5) ? true : false;
		if (!r->refl_target) {
			printf("[GPU] !! 5-й цели G-буфера не будет: "
			       "max_color_attachments=%d max_draw_buffers=%d (нужно >= 5 оба) — "
			       "сила SSR фиксированная (0.35)\n", maxct, maxdb);
		}
	}

	/* --- G-буфер текстуры --- */
	glGenTextures(1, &r->tex_albedo);
	glBindTexture(GL_TEXTURE_2D, r->tex_albedo);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, width, height, 0, GL_RGBA,
	             GL_UNSIGNED_BYTE, NULL);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

	/* view-space нормаль в RGB (кодирована *0.5+0.5) */
	glGenTextures(1, &r->tex_nrm);
	glBindTexture(GL_TEXTURE_2D, r->tex_nrm);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, width, height, 0, GL_RGBA,
	             GL_UNSIGNED_BYTE, NULL);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

	/* view-глубина: rgb = упакованная линейная глубина, a = маска геометрии (255 = есть).
	 * Специально НЕ depth-текстура: сэмплинг depth-текстуры в GLSL 1.20 ненадёжен,
	 * а packed-глубина в color-аттачменте работает везде (GL 2.1, без расширений). */
	glGenTextures(1, &r->tex_vpos);
	glBindTexture(GL_TEXTURE_2D, r->tex_vpos);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, width, height, 0, GL_RGBA,
	             GL_UNSIGNED_BYTE, NULL);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

	/* ★ M5: 4-я цель — цвет блика (rgb) и hardness (a). Создаётся только если
	 * карта умеет 4 color attachment (проверка выше, r->spec_target). */
	if (r->spec_target) {
		glGenTextures(1, &r->tex_spec);
		glBindTexture(GL_TEXTURE_2D, r->tex_spec);
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, width, height, 0, GL_RGBA,
		             GL_UNSIGNED_BYTE, NULL);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
	}
	/* M17: 5-я цель — сила отражения (MAP_REFL). R8. Если карта не умеет 5
	 * color attachment (r->refl_target=false), не создаём и используем tex_refl_def. */
	if (r->refl_target) {
		glGenTextures(1, &r->tex_refl);
		glBindTexture(GL_TEXTURE_2D, r->tex_refl);
		glTexImage2D(GL_TEXTURE_2D, 0, GL_R8, width, height, 0, GL_RED,
		             GL_UNSIGNED_BYTE, NULL);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
	}
	glBindTexture(GL_TEXTURE_2D, 0);

	/* --- ★ Теневые карты: по одной на каждую лампу (все GPU_MAX_SHADOW_LIGHTS, чтобы не
	 * зависеть от того, какие лампы дают тень в ЭТОЙ сцене). Каждая — упакованная
	 * window-глубина в color + depth renderbuffer, как раньше, только в массиве. --- */
	const int shw = GPU_SHADOW_SIZE, shh = GPU_SHADOW_SIZE;
	for (int si = 0; si < GPU_MAX_SHADOW_LIGHTS; si++) {
		glGenTextures(1, &r->tex_shadow[si]);
		glBindTexture(GL_TEXTURE_2D, r->tex_shadow[si]);
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, shw, shh, 0, GL_RGBA,
		             GL_UNSIGNED_BYTE, NULL);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
		glBindTexture(GL_TEXTURE_2D, 0);
		glGenFramebuffersEXT(1, &r->shadow_fb[si]);
		glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, r->shadow_fb[si]);
		glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT,
		                          GL_TEXTURE_2D, r->tex_shadow[si], 0);
		glGenRenderbuffersEXT(1, &r->rb_shadow[si]);
		glBindRenderbufferEXT(GL_RENDERBUFFER_EXT, r->rb_shadow[si]);
		glRenderbufferStorageEXT(GL_RENDERBUFFER_EXT, GL_DEPTH_COMPONENT24, shw, shh);
		glFramebufferRenderbufferEXT(GL_FRAMEBUFFER_EXT, GL_DEPTH_ATTACHMENT_EXT,
		                             GL_RENDERBUFFER_EXT, r->rb_shadow[si]);
		glBindRenderbufferEXT(GL_RENDERBUFFER_EXT, 0);
		GLenum satt = GL_COLOR_ATTACHMENT0_EXT;
		glDrawBuffers(1, &satt);
		glReadBuffer(GL_COLOR_ATTACHMENT0_EXT);
		if (glCheckFramebufferStatusEXT(GL_FRAMEBUFFER_EXT) != GL_FRAMEBUFFER_COMPLETE_EXT) {
			GLenum st = glCheckFramebufferStatusEXT(GL_FRAMEBUFFER_EXT);
			printf("[GPU] shadow FBO[%d] not complete: 0x%04X (%s)\n",
			       si, st, gl_fbo_status_str(st));
			glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, 0);
			return -2;
		}
		glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, 0);
	}

	/* --- ★ POINT: кубические карты (6 граней в одной текстуре). FBO/глубинный
	 * renderbuffer переиспользуются из цикла выше: перед рендером грани k меняем
	 * color-attachment на GL_TEXTURE_CUBE_MAP_*_k. Заглушка tex_cube_white —
	 * samplerCube в шейдере обязан быть привязан (даже если не сэмплится). --- */
	for (int si = 0; si < GPU_MAX_SHADOW_LIGHTS; si++) {
		glGenTextures(1, &r->tex_shadow_cube[si]);
		glBindTexture(GL_TEXTURE_CUBE_MAP, r->tex_shadow_cube[si]);
		for (int face = 0; face < 6; face++) {
			glTexImage2D(GL_TEXTURE_CUBE_MAP_POSITIVE_X + face, 0, GL_RGBA8,
			             shw, shh, 0, GL_RGBA, GL_UNSIGNED_BYTE, NULL);
		}
		glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
		glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
		glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
		glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
		glBindTexture(GL_TEXTURE_CUBE_MAP, 0);
	}
	glGenTextures(1, &r->tex_cube_white);
	glBindTexture(GL_TEXTURE_CUBE_MAP, r->tex_cube_white);
	for (int face = 0; face < 6; face++) {
		const unsigned char px[4] = {0, 0, 0, 0};   /* a=0: тени нет */
		glTexImage2D(GL_TEXTURE_CUBE_MAP_POSITIVE_X + face, 0, GL_RGBA8, 1, 1, 0,
		             GL_RGBA, GL_UNSIGNED_BYTE, px);
	}
	glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
	glBindTexture(GL_TEXTURE_CUBE_MAP, 0);

	/* ★ Проверка: FBO с ПРИВЯЗАННОЙ ГРАНЬЮ КУБА должен быть полным — иначе
	 * point-тени молча не нарисуются (glClear/Draw уйдут в неполный FBO). Потом
	 * возвращаем 2D-привязку, чтобы обычный путь не сломался. */
	glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, r->shadow_fb[0]);
	glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT,
	                          GL_TEXTURE_CUBE_MAP_POSITIVE_X, r->tex_shadow_cube[0], 0);
	{
		GLenum st = glCheckFramebufferStatusEXT(GL_FRAMEBUFFER_EXT);
		if (st != GL_FRAMEBUFFER_COMPLETE_EXT) {
			printf("[GPU] point-shadow cubemap FBO not complete: 0x%04X (%s)\n",
			       st, gl_fbo_status_str(st));
			glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, 0);
			return -3;
		}
	}
	glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT,
	                          GL_TEXTURE_2D, r->tex_shadow[0], 0);
	glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, 0);

	/* --- FBO с MRT: albedo(0), normal(1), view-глубина(2) + depth renderbuffer --- */
	glGenFramebuffersEXT(1, &r->fb);
	glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, r->fb);
	glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT,
	                          GL_TEXTURE_2D, r->tex_albedo, 0);
	glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT1_EXT,
	                          GL_TEXTURE_2D, r->tex_nrm, 0);
	glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT2_EXT,
	                          GL_TEXTURE_2D, r->tex_vpos, 0);
	if (r->spec_target) {
		glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT3_EXT,
		                          GL_TEXTURE_2D, r->tex_spec, 0);
	}
	if (r->refl_target) {
		glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT4_EXT,
		                          GL_TEXTURE_2D, r->tex_refl, 0);
	}

	/* depth для самого depth-теста — renderbuffer: он поддерживается везде и,
	 * в отличие от depth-текстуры, гарантированно принимает запись глубины. */
	glGenRenderbuffersEXT(1, &r->rb_depth);
	glBindRenderbufferEXT(GL_RENDERBUFFER_EXT, r->rb_depth);
	glRenderbufferStorageEXT(GL_RENDERBUFFER_EXT, GL_DEPTH_COMPONENT24, width, height);
	glFramebufferRenderbufferEXT(GL_FRAMEBUFFER_EXT, GL_DEPTH_ATTACHMENT_EXT,
	                             GL_RENDERBUFFER_EXT, r->rb_depth);
	glBindRenderbufferEXT(GL_RENDERBUFFER_EXT, 0);

	/* MRT: albedo + normal + view-глубина (+ блик материала, M5; + отражение, M17) */
	GLenum bufs[5] = {
		GL_COLOR_ATTACHMENT0_EXT,
		GL_COLOR_ATTACHMENT1_EXT,
		GL_COLOR_ATTACHMENT2_EXT,
		GL_COLOR_ATTACHMENT3_EXT,
		GL_COLOR_ATTACHMENT4_EXT,
	};
	glDrawBuffers((r->spec_target ? 4 : 3) + (r->refl_target ? 1 : 0), bufs);
	glReadBuffer(GL_COLOR_ATTACHMENT0_EXT);

	if (glCheckFramebufferStatusEXT(GL_FRAMEBUFFER_EXT) != GL_FRAMEBUFFER_COMPLETE_EXT) {
		GLenum st = glCheckFramebufferStatusEXT(GL_FRAMEBUFFER_EXT);
		printf("[GPU] G-buffer FBO not complete: 0x%04X (%s)\n", st, gl_fbo_status_str(st));
		glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, 0);
		return -2;
	}
	glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, 0);

	/* --- шейдеры --- */
	/* ★ M5: вариант GB_FS зависит от наличия 4-й цели: без неё писать в
	 * gl_FragData[3] нельзя, а униформы блика всё равно оптимизируются вон.
	 * Один текст шейдера + преамбула с #define — вместо двух копий. */
	{
		const char *pre = r->spec_target ? "#define GPU_SPEC_TARGET 1\n" : "";
		const char *pre_refl = r->refl_target ? "#define GPU_REFL_TARGET 1\n" : "";
		size_t nsrc = strlen(pre) + strlen(pre_refl) + strlen(GB_FS) + 1;
		char *fsrc = (char *)MEM_mallocN(nsrc, "gpu GB_FS src");
		strcpy(fsrc, pre);
		strcat(fsrc, pre_refl);
		strcat(fsrc, GB_FS);
		r->prog_gb = gl_link(gl_compile(GL_VERTEX_SHADER, GB_VS),
		                     gl_compile(GL_FRAGMENT_SHADER, fsrc));
		MEM_freeN(fsrc);
	}
	r->gb_u_proj = glGetUniformLocation(r->prog_gb, "u_proj");
	r->gb_u_model = glGetUniformLocation(r->prog_gb, "u_model");
	r->gb_u_normmat = glGetUniformLocation(r->prog_gb, "u_normmat");
	r->gb_u_pack = glGetUniformLocation(r->prog_gb, "u_pack");
	r->gb_a_co = glGetAttribLocation(r->prog_gb, "a_co");
	r->gb_a_n = glGetAttribLocation(r->prog_gb, "a_n");
	r->gb_a_rgba = glGetAttribLocation(r->prog_gb, "a_rgba");
	r->gb_a_emit = glGetAttribLocation(r->prog_gb, "a_emit");
	r->gb_a_uv = glGetAttribLocation(r->prog_gb, "a_uv");
	r->gb_a_tangent = glGetAttribLocation(r->prog_gb, "a_tangent");
	r->gb_a_orco = glGetAttribLocation(r->prog_gb, "a_orco");
	r->gb_u_tex = glGetUniformLocation(r->prog_gb, "u_tex");
	r->gb_u_has_tex = glGetUniformLocation(r->prog_gb, "u_has_tex");
	r->gb_u_colfac = glGetUniformLocation(r->prog_gb, "u_colfac");
	r->gb_u_refl = glGetUniformLocation(r->prog_gb, "u_refl");
	r->gb_u_tex_blend = glGetUniformLocation(r->prog_gb, "u_tex_blend");
	r->gb_u_tex_orco = glGetUniformLocation(r->prog_gb, "u_tex_orco");
	r->gb_u_tex_glob = glGetUniformLocation(r->prog_gb, "u_tex_glob");
	r->gb_u_tex_map = glGetUniformLocation(r->prog_gb, "u_tex_map");
	/* ★ M19: процедурная карта цвета (CLOUDS/BLEND) */
	r->gb_u_proc_type = glGetUniformLocation(r->prog_gb, "u_proc_type");
	r->gb_u_proc_col = glGetUniformLocation(r->prog_gb, "u_proc_col");   /* M20 */
	r->gb_u_proc_stype = glGetUniformLocation(r->prog_gb, "u_proc_stype");
	r->gb_u_proc_flip = glGetUniformLocation(r->prog_gb, "u_proc_flip");
	r->gb_u_proc_noisesize = glGetUniformLocation(r->prog_gb, "u_proc_noisesize");
	r->gb_u_proc_noisedepth = glGetUniformLocation(r->prog_gb, "u_proc_noisedepth");
	r->gb_u_proc_basis = glGetUniformLocation(r->prog_gb, "u_proc_basis");
	r->gb_u_proc_wf = glGetUniformLocation(r->prog_gb, "u_proc_wf");
	r->gb_u_proc_dist = glGetUniformLocation(r->prog_gb, "u_proc_dist");
	r->gb_u_proc_hard = glGetUniformLocation(r->prog_gb, "u_proc_hard");
	r->gb_u_proc_bright = glGetUniformLocation(r->prog_gb, "u_proc_bright");
	r->gb_u_proc_contrast = glGetUniformLocation(r->prog_gb, "u_proc_contrast");
	r->gb_u_proc_noclamp = glGetUniformLocation(r->prog_gb, "u_proc_noclamp");
	r->gb_u_proc_has_ramp = glGetUniformLocation(r->prog_gb, "u_proc_has_ramp");
	r->gb_u_proc_ramp = glGetUniformLocation(r->prog_gb, "u_proc_ramp");
	r->gb_u_proc_hash = glGetUniformLocation(r->prog_gb, "u_proc_hash");
	r->gb_u_proc_hashv = glGetUniformLocation(r->prog_gb, "u_proc_hashv");
	r->gb_u_proc_hashp = glGetUniformLocation(r->prog_gb, "u_proc_hashp");   /* M20 */
	r->gb_u_proc_orco = glGetUniformLocation(r->prog_gb, "u_proc_orco");
	r->gb_u_proc_size = glGetUniformLocation(r->prog_gb, "u_proc_size");
	r->gb_u_proc_ofs = glGetUniformLocation(r->prog_gb, "u_proc_ofs");
	/* ★ M22: вторая процедурная текстура */
	r->gb_u_has_proc2 = glGetUniformLocation(r->prog_gb, "u_has_proc2");
	r->gb_u_proc2_type = glGetUniformLocation(r->prog_gb, "u_proc2_type");
	r->gb_u_proc2_stype = glGetUniformLocation(r->prog_gb, "u_proc2_stype");
	r->gb_u_proc2_flip = glGetUniformLocation(r->prog_gb, "u_proc2_flip");
	r->gb_u_proc2_noisesize = glGetUniformLocation(r->prog_gb, "u_proc2_noisesize");
	r->gb_u_proc2_noisedepth = glGetUniformLocation(r->prog_gb, "u_proc2_noisedepth");
	r->gb_u_proc2_basis = glGetUniformLocation(r->prog_gb, "u_proc2_basis");
	r->gb_u_proc2_wf = glGetUniformLocation(r->prog_gb, "u_proc2_wf");
	r->gb_u_proc2_dist = glGetUniformLocation(r->prog_gb, "u_proc2_dist");
	r->gb_u_proc2_hard = glGetUniformLocation(r->prog_gb, "u_proc2_hard");
	r->gb_u_proc2_bright = glGetUniformLocation(r->prog_gb, "u_proc2_bright");
	r->gb_u_proc2_contrast = glGetUniformLocation(r->prog_gb, "u_proc2_contrast");
	r->gb_u_proc2_noclamp = glGetUniformLocation(r->prog_gb, "u_proc2_noclamp");
	r->gb_u_proc2_has_ramp = glGetUniformLocation(r->prog_gb, "u_proc2_has_ramp");
	r->gb_u_proc2_ramp = glGetUniformLocation(r->prog_gb, "u_proc2_ramp");
	r->gb_u_proc2_orco = glGetUniformLocation(r->prog_gb, "u_proc2_orco");
	r->gb_u_proc2_size = glGetUniformLocation(r->prog_gb, "u_proc2_size");
	r->gb_u_proc2_ofs = glGetUniformLocation(r->prog_gb, "u_proc2_ofs");
	/* ★ M23: bump от процедурной карты */
	r->gb_u_has_proc_nrm = glGetUniformLocation(r->prog_gb, "u_has_proc_nrm");
	r->gb_u_proc_nrm_src = glGetUniformLocation(r->prog_gb, "u_proc_nrm_src");
	r->gb_u_proc_norfac = glGetUniformLocation(r->prog_gb, "u_proc_norfac");
	r->gb_u_tex_size = glGetUniformLocation(r->prog_gb, "u_tex_size");
	r->gb_u_tex_ofs = glGetUniformLocation(r->prog_gb, "u_tex_ofs");
	r->gb_u_tex1 = glGetUniformLocation(r->prog_gb, "u_tex1");
	r->gb_u_tex1_blend = glGetUniformLocation(r->prog_gb, "u_tex1_blend");
	r->gb_u_tex1_fac = glGetUniformLocation(r->prog_gb, "u_tex1_fac");
	r->gb_u_tex1_orco = glGetUniformLocation(r->prog_gb, "u_tex1_orco");
	r->gb_u_tex1_glob = glGetUniformLocation(r->prog_gb, "u_tex1_glob");
	r->gb_u_tex1_map = glGetUniformLocation(r->prog_gb, "u_tex1_map");
	r->gb_u_tex1_size = glGetUniformLocation(r->prog_gb, "u_tex1_size");
	r->gb_u_tex1_ofs = glGetUniformLocation(r->prog_gb, "u_tex1_ofs");
	r->gb_u_tex2 = glGetUniformLocation(r->prog_gb, "u_tex2");
	r->gb_u_tex2_blend = glGetUniformLocation(r->prog_gb, "u_tex2_blend");
	r->gb_u_tex2_fac = glGetUniformLocation(r->prog_gb, "u_tex2_fac");
	r->gb_u_tex2_orco = glGetUniformLocation(r->prog_gb, "u_tex2_orco");
	r->gb_u_tex2_glob = glGetUniformLocation(r->prog_gb, "u_tex2_glob");
	r->gb_u_tex2_map = glGetUniformLocation(r->prog_gb, "u_tex2_map");
	r->gb_u_tex2_size = glGetUniformLocation(r->prog_gb, "u_tex2_size");
	r->gb_u_tex2_ofs = glGetUniformLocation(r->prog_gb, "u_tex2_ofs");
	r->gb_u_tex3 = glGetUniformLocation(r->prog_gb, "u_tex3");
	r->gb_u_tex3_blend = glGetUniformLocation(r->prog_gb, "u_tex3_blend");
	r->gb_u_tex3_fac = glGetUniformLocation(r->prog_gb, "u_tex3_fac");
	r->gb_u_tex3_orco = glGetUniformLocation(r->prog_gb, "u_tex3_orco");
	r->gb_u_tex3_glob = glGetUniformLocation(r->prog_gb, "u_tex3_glob");
	r->gb_u_tex3_map = glGetUniformLocation(r->prog_gb, "u_tex3_map");
	r->gb_u_tex3_size = glGetUniformLocation(r->prog_gb, "u_tex3_size");
	r->gb_u_tex3_ofs = glGetUniformLocation(r->prog_gb, "u_tex3_ofs");
	r->gb_u_n_tex = glGetUniformLocation(r->prog_gb, "u_n_tex");
	r->gb_u_tex_rgbtoint = glGetUniformLocation(r->prog_gb, "u_tex_rgbtoint");
	r->gb_u_tex1_rgbtoint = glGetUniformLocation(r->prog_gb, "u_tex1_rgbtoint");
	r->gb_u_tex2_rgbtoint = glGetUniformLocation(r->prog_gb, "u_tex2_rgbtoint");
	r->gb_u_tex3_rgbtoint = glGetUniformLocation(r->prog_gb, "u_tex3_rgbtoint");
	r->gb_u_atex = glGetUniformLocation(r->prog_gb, "u_atex");
	r->gb_u_has_atex = glGetUniformLocation(r->prog_gb, "u_has_atex");
	r->gb_u_a_fac = glGetUniformLocation(r->prog_gb, "u_a_fac");
	r->gb_u_a_def = glGetUniformLocation(r->prog_gb, "u_a_def");
	r->gb_u_a_blend = glGetUniformLocation(r->prog_gb, "u_a_blend");
	r->gb_u_a_use_a = glGetUniformLocation(r->prog_gb, "u_a_use_a");
	r->gb_u_a_orco = glGetUniformLocation(r->prog_gb, "u_a_orco");
	r->gb_u_a_glob = glGetUniformLocation(r->prog_gb, "u_a_glob");
	r->gb_u_a_map = glGetUniformLocation(r->prog_gb, "u_a_map");
	r->gb_u_a_size = glGetUniformLocation(r->prog_gb, "u_a_size");
	r->gb_u_a_ofs = glGetUniformLocation(r->prog_gb, "u_a_ofs");
	r->gb_u_has_spec = glGetUniformLocation(r->prog_gb, "u_has_spec");
	r->gb_u_spec_blend = glGetUniformLocation(r->prog_gb, "u_spec_blend");
	r->gb_u_spec_fac = glGetUniformLocation(r->prog_gb, "u_spec_fac");
	r->gb_u_spec_def = glGetUniformLocation(r->prog_gb, "u_spec_def");
	r->gb_u_has_har = glGetUniformLocation(r->prog_gb, "u_has_har");
	r->gb_u_har_blend = glGetUniformLocation(r->prog_gb, "u_har_blend");
	r->gb_u_har_fac = glGetUniformLocation(r->prog_gb, "u_har_fac");
	r->gb_u_har_def = glGetUniformLocation(r->prog_gb, "u_har_def");
	r->gb_u_sv_use_a = glGetUniformLocation(r->prog_gb, "u_sv_use_a");
	r->gb_u_has_colspec = glGetUniformLocation(r->prog_gb, "u_has_colspec");
	r->gb_u_colspec_blend = glGetUniformLocation(r->prog_gb, "u_colspec_blend");
	r->gb_u_colspec_fac = glGetUniformLocation(r->prog_gb, "u_colspec_fac");
	/* M17: MAP_REFL */
	r->gb_u_refl_base = glGetUniformLocation(r->prog_gb, "u_refl_base");
	r->gb_u_has_refl = glGetUniformLocation(r->prog_gb, "u_has_refl");
	r->gb_u_refl_blend = glGetUniformLocation(r->prog_gb, "u_refl_blend");
	r->gb_u_refl_fac = glGetUniformLocation(r->prog_gb, "u_refl_fac");
	r->gb_u_refl_def = glGetUniformLocation(r->prog_gb, "u_refl_def");
	r->gb_u_alpha = glGetUniformLocation(r->prog_gb, "u_alpha");
	r->gb_u_nrm = glGetUniformLocation(r->prog_gb, "u_nrm");
	r->gb_u_has_nrm = glGetUniformLocation(r->prog_gb, "u_has_nrm");
	r->gb_u_nrm_is_height = glGetUniformLocation(r->prog_gb, "u_nrm_is_height");
	r->gb_u_nrm_orco = glGetUniformLocation(r->prog_gb, "u_nrm_orco");
	r->gb_u_nrm_glob = glGetUniformLocation(r->prog_gb, "u_nrm_glob");
	r->gb_u_nrm_map = glGetUniformLocation(r->prog_gb, "u_nrm_map");
	r->gb_u_nrm_size = glGetUniformLocation(r->prog_gb, "u_nrm_size");
	r->gb_u_nrm_ofs = glGetUniformLocation(r->prog_gb, "u_nrm_ofs");
	r->gb_u_bump_scale = glGetUniformLocation(r->prog_gb, "u_bump_scale");
	r->gb_u_nrm_norfac = glGetUniformLocation(r->prog_gb, "u_nrm_norfac");
	/* M5: блик материала. При отсутствии 4-й цели эти униформы в шейдере не
	 * используются, и glGetUniformLocation честно вернёт -1 — вызовы с -1
	 * безвредны, но мы их всё равно пропускаем по r->spec_target. */
	r->gb_u_spec_col = glGetUniformLocation(r->prog_gb, "u_spec_col");
	r->gb_u_hard = glGetUniformLocation(r->prog_gb, "u_hard");

	/* 1x1 белая текстура: подставляется для материалов без карты.
	 * БЕЗ неё сэмплер u_tex читал бы непроинициализированный юнит (чёрный). */
	{
		GLubyte white[4] = { 255, 255, 255, 255 };
		glGenTextures(1, &r->tex_white);
		glBindTexture(GL_TEXTURE_2D, r->tex_white);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, 1, 1, 0, GL_RGBA,
		             GL_UNSIGNED_BYTE, white);
		glBindTexture(GL_TEXTURE_2D, 0);
	}

	/* 1x1 «значения BI по умолчанию» для блика: белый цвет (specr/g/b = 1) и
	 * hardness 50 (material.c:113,119). Подставляется в u_spec, когда 4-й цели
	 * MRT нет, — так сэмплер всегда читает осмысленную текстуру, а не мусор. */
	{
		GLubyte spec1[4];
		spec1[0] = spec1[1] = spec1[2] = 255;
		spec1[3] = (GLubyte)(50.0f / 511.0f * 255.0f + 0.5f);
		glGenTextures(1, &r->tex_spec_def);
		glBindTexture(GL_TEXTURE_2D, r->tex_spec_def);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, 1, 1, 0, GL_RGBA,
		             GL_UNSIGNED_BYTE, spec1);
		glBindTexture(GL_TEXTURE_2D, 0);
	}

	/* 1x1 заглушка для MAP_REFL, когда 5-й цели нет: 0.35 (текущий фиксированный
	 * микс SSR). GL_R8, значение R=0.35*255. */
	{
		GLubyte refl1[1];
		refl1[0] = (GLubyte)(0.35f * 255.0f + 0.5f);
		glGenTextures(1, &r->tex_refl_def);
		glBindTexture(GL_TEXTURE_2D, r->tex_refl_def);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
		glTexImage2D(GL_TEXTURE_2D, 0, GL_R8, 1, 1, 0, GL_RED,
		             GL_UNSIGNED_BYTE, refl1);
		glBindTexture(GL_TEXTURE_2D, 0);
	}

	/* ★ M19: таблицы orgBlenderNoise из noise.c (hash[512], hashvectf[768])
	 * грузим ТЕКСТУРАМИ, а не uniform-массивами: 1280 float не влезли бы в лимит
	 * uniform-компонентов GL 2.1, а выборка по индексу всё равно нужна точечная. */
	{
		extern const unsigned char hash[512];
		extern const float hashvectf[768];
		extern const float hashpntf[768];   /* M20: Voronoi feature points */
		GLubyte hb[512];
		GLubyte hv[256 * 4];
		GLubyte ramp0[256 * 4];
		GLubyte hp[256 * 3];
		int i;
		for (i = 0; i < 512; i++) hb[i] = (GLubyte)hash[i];
		for (i = 0; i < 256; i++) {
			hv[i * 4 + 0] = (GLubyte)(CLAMPIS((hashvectf[i * 3 + 0] + 1.0f) * 0.5f, 0.0f, 1.0f) * 255.0f + 0.5f);
			hv[i * 4 + 1] = (GLubyte)(CLAMPIS((hashvectf[i * 3 + 1] + 1.0f) * 0.5f, 0.0f, 1.0f) * 255.0f + 0.5f);
			hv[i * 4 + 2] = (GLubyte)(CLAMPIS((hashvectf[i * 3 + 2] + 1.0f) * 0.5f, 0.0f, 1.0f) * 255.0f + 0.5f);
			hv[i * 4 + 3] = 255;
			/* заглушка колорбэнда: серый = tin (BI без колорбэнда даёт (tin,tin,tin)) */
			ramp0[i * 4 + 0] = (GLubyte)i;
			ramp0[i * 4 + 1] = (GLubyte)i;
			ramp0[i * 4 + 2] = (GLubyte)i;
			ramp0[i * 4 + 3] = 255;
		}
		glPixelStorei(GL_UNPACK_ALIGNMENT, 1);

		glGenTextures(1, &r->tex_hash);
		glBindTexture(GL_TEXTURE_2D, r->tex_hash);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
		glTexImage2D(GL_TEXTURE_2D, 0, GL_LUMINANCE8, 512, 1, 0, GL_LUMINANCE,
		             GL_UNSIGNED_BYTE, hb);

		glGenTextures(1, &r->tex_hashv);
		glBindTexture(GL_TEXTURE_2D, r->tex_hashv);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, 256, 1, 0, GL_RGBA,
		             GL_UNSIGNED_BYTE, hv);

		glGenTextures(1, &r->tex_ramp);
		glBindTexture(GL_TEXTURE_2D, r->tex_ramp);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, 256, 1, 0, GL_RGBA,
		             GL_UNSIGNED_BYTE, ramp0);

		/* M22: свой колорбэнд для ВТОРОЙ процедурной текстуры материала.
		 * Формат/фильтр как у tex_ramp; содержимое — та же единичная рампа,
		 * перезаливается glTexSubImage2D под материал. */
		glGenTextures(1, &r->tex_ramp2);
		glBindTexture(GL_TEXTURE_2D, r->tex_ramp2);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, 256, 1, 0, GL_RGBA,
		             GL_UNSIGNED_BYTE, ramp0);

		/* M20: hashpntf[] — 256 Voronoi feature points (x,y,z в 0..1).
		 * 256x1 RGB8: texel i = (p[i*3], p[i*3+1], p[i*3+2]). */
		for (i = 0; i < 256; i++) {
			hp[i * 3 + 0] = (GLubyte)(CLAMPIS(hashpntf[i * 3 + 0], 0.0f, 1.0f) * 255.0f + 0.5f);
			hp[i * 3 + 1] = (GLubyte)(CLAMPIS(hashpntf[i * 3 + 1], 0.0f, 1.0f) * 255.0f + 0.5f);
			hp[i * 3 + 2] = (GLubyte)(CLAMPIS(hashpntf[i * 3 + 2], 0.0f, 1.0f) * 255.0f + 0.5f);
		}
		glGenTextures(1, &r->tex_hashp);
		glBindTexture(GL_TEXTURE_2D, r->tex_hashp);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB8, 256, 1, 0, GL_RGB,
		             GL_UNSIGNED_BYTE, hp);

		glBindTexture(GL_TEXTURE_2D, 0);
		if (getenv("UBF_GPU_DEBUG")) {
			printf("[GPU-PROCTEX] hash=%u hashv=%u ramp=%u hashp=%u (512x1 L8, 256x1 RGBA8, 256x1 RGBA8, 256x1 RGB8)\n",
			       (unsigned)r->tex_hash, (unsigned)r->tex_hashv, (unsigned)r->tex_ramp,
			       (unsigned)r->tex_hashp);
		}
	}

	r->prog_light = gl_link(gl_compile(GL_VERTEX_SHADER, LIGHT_VS),
	                        gl_compile(GL_FRAGMENT_SHADER, LIGHT_FS));
	r->light_a_pos = glGetAttribLocation(r->prog_light, "a_pos");
	r->light_a_uv = glGetAttribLocation(r->prog_light, "a_uv");
	r->light_u_albedo = glGetUniformLocation(r->prog_light, "u_albedo");
	r->light_u_nrm = glGetUniformLocation(r->prog_light, "u_nrm");
	r->light_u_num_lights = glGetUniformLocation(r->prog_light, "u_num_lights");
	r->light_u_exposure = glGetUniformLocation(r->prog_light, "u_exposure");
	r->light_u_world_hor = glGetUniformLocation(r->prog_light, "u_world_hor");
	r->light_u_world_zen = glGetUniformLocation(r->prog_light, "u_world_zen");
	r->light_u_world_amb = glGetUniformLocation(r->prog_light, "u_world_amb");
	r->light_u_world_blend = glGetUniformLocation(r->prog_light, "u_world_blend");
	r->light_u_amb_fill = glGetUniformLocation(r->prog_light, "u_amb_fill");
	r->light_u_env_light = glGetUniformLocation(r->prog_light, "u_env_light");
	r->light_u_env_energy = glGetUniformLocation(r->prog_light, "u_env_energy");
	r->light_u_linfac = glGetUniformLocation(r->prog_light, "u_linfac");
	r->light_u_logfac = glGetUniformLocation(r->prog_light, "u_logfac");
	r->light_u_expo_on = glGetUniformLocation(r->prog_light, "u_expo_on");
	/* Если имена не нашлись, glUniform* молча ничего не сделает и кадр выйдет
	 * «только ambient» — поэтому кричим в консоль вьюпорта, а не молчим. */
	if (r->light_u_num_lights < 0 || r->light_u_exposure < 0) {
		printf("[GPU] !! u_num_lights=%d u_exposure=%d — лампы в шейдер не попадут!\n",
		       (int)r->light_u_num_lights, (int)r->light_u_exposure);
	}
	/* ★★ Раунд 16: per-light юниформы u_lpos/u_ldir/u_lcol/u_lfl/u_lsoft УБРАНЫ.
	 * Данные ВСЕХ ламп идут float-текстурой u_light_data (см. LIGHT_FS), поэтому
	 * запрашиваем только её локации — это и есть снятие потолка «10 ламп». */
	r->light_u_lightdata = glGetUniformLocation(r->prog_light, "u_light_data");
	r->light_u_lighttexel = glGetUniformLocation(r->prog_light, "u_light_texel");
	if (r->light_u_lightdata < 0 || r->light_u_lighttexel < 0) {
		printf("[GPU] !! u_light_data=%d u_light_texel=%d — данные ламп НЕ попадут "
		       "в шейдер (останется только ambient)!\n",
		       (int)r->light_u_lightdata, (int)r->light_u_lighttexel);
	}
	r->light_u_depth = glGetUniformLocation(r->prog_light, "u_depth");
	r->light_u_spec = glGetUniformLocation(r->prog_light, "u_spec");
	r->light_u_refl = glGetUniformLocation(r->prog_light, "u_refl");
	/* ★ Теневые карты и матрицы — по имени элемента массива, как и лампы. */
	for (int i = 0; i < GPU_MAX_SHADOW_LIGHTS; i++) {
		char nm[32];
		sprintf(nm, "u_shadow[%d]", i);
		r->light_u_shadow[i] = glGetUniformLocation(r->prog_light, nm);
		sprintf(nm, "u_lvp[%d]", i);
		r->light_u_lvp[i] = glGetUniformLocation(r->prog_light, nm);
		sprintf(nm, "u_shadow_cube[%d]", i);
		r->light_u_shadow_cube[i] = glGetUniformLocation(r->prog_light, nm);
		sprintf(nm, "u_lwpos[%d]", i);
		r->light_u_lwpos[i] = glGetUniformLocation(r->prog_light, nm);
		sprintf(nm, "u_lpz[%d]", i);
		r->light_u_lpz[i] = glGetUniformLocation(r->prog_light, nm);
		if (r->light_u_shadow[i] < 0 || r->light_u_lvp[i] < 0 ||
		    r->light_u_shadow_cube[i] < 0 || r->light_u_lwpos[i] < 0 ||
		    r->light_u_lpz[i] < 0) {
			printf("[GPU] !! uniform тени лампы[%d] не найден (shadow=%d lvp=%d "
			       "cube=%d wpos=%d pz=%d)\n", i,
			       (int)r->light_u_shadow[i], (int)r->light_u_lvp[i],
			       (int)r->light_u_shadow_cube[i], (int)r->light_u_lwpos[i],
			       (int)r->light_u_lpz[i]);
		}
	}
	r->light_u_invproj = glGetUniformLocation(r->prog_light, "u_inv_proj");
	r->light_u_invview = glGetUniformLocation(r->prog_light, "u_inv_view");
	r->light_u_debug = glGetUniformLocation(r->prog_light, "u_debug");
	r->light_u_pack = glGetUniformLocation(r->prog_light, "u_pack");
	r->light_u_ortho = glGetUniformLocation(r->prog_light, "u_ortho");
	r->light_u_up = glGetUniformLocation(r->prog_light, "u_up");
	r->light_u_proj = glGetUniformLocation(r->prog_light, "u_proj");
	r->light_u_ssr = glGetUniformLocation(r->prog_light, "u_ssr");
	r->light_u_texel = glGetUniformLocation(r->prog_light, "u_texel");
	r->light_u_ao = glGetUniformLocation(r->prog_light, "u_ao");
	r->light_u_shadow_texel = glGetUniformLocation(r->prog_light, "u_shadow_texel");
	r->light_u_pcf = glGetUniformLocation(r->prog_light, "u_pcf");
	r->light_u_shadow_bias = glGetUniformLocation(r->prog_light, "u_shadow_bias");

	r->prog_shadow = gl_link(gl_compile(GL_VERTEX_SHADER, SHADOW_VS),
	                         gl_compile(GL_FRAGMENT_SHADER, SHADOW_FS));
	r->shadow_a_co = glGetAttribLocation(r->prog_shadow, "a_co");
	r->shadow_u_lightvp = glGetUniformLocation(r->prog_shadow, "u_lightvp");
	r->shadow_u_obmat = glGetUniformLocation(r->prog_shadow, "u_obmat");

	/* M15: forward-проход полупрозрачных мешей (ОПЦИОНАЛЬНЫЙ). Если не слинкуется —
	 * полупрозрачные меши остаются на screen-door, деферред-путь работает. */
	r->prog_forward = gl_link(gl_compile(GL_VERTEX_SHADER, FORWARD_VS),
	                          gl_compile(GL_FRAGMENT_SHADER, FORWARD_FS));
	if (r->prog_forward) {
		r->fw_a_co = glGetAttribLocation(r->prog_forward, "a_co");
		r->fw_a_n = glGetAttribLocation(r->prog_forward, "a_n");
		r->fw_a_rgba = glGetAttribLocation(r->prog_forward, "a_rgba");
		r->fw_a_emit = glGetAttribLocation(r->prog_forward, "a_emit");
		r->fw_a_uv = glGetAttribLocation(r->prog_forward, "a_uv");
		r->fw_a_tangent = glGetAttribLocation(r->prog_forward, "a_tangent");
		r->fw_a_orco = glGetAttribLocation(r->prog_forward, "a_orco");
		r->fw_u_proj = glGetUniformLocation(r->prog_forward, "u_proj");
		r->fw_u_model = glGetUniformLocation(r->prog_forward, "u_model");
		r->fw_u_obmat = glGetUniformLocation(r->prog_forward, "u_obmat");
		r->fw_u_normmat = glGetUniformLocation(r->prog_forward, "u_normmat");
		r->fw_u_alpha = glGetUniformLocation(r->prog_forward, "u_alpha");
		r->fw_u_exposure = glGetUniformLocation(r->prog_forward, "u_exposure");
		r->fw_u_expo_on = glGetUniformLocation(r->prog_forward, "u_expo_on");
		r->fw_u_linfac = glGetUniformLocation(r->prog_forward, "u_linfac");
		r->fw_u_logfac = glGetUniformLocation(r->prog_forward, "u_logfac");
		r->fw_u_amb_fill = glGetUniformLocation(r->prog_forward, "u_amb_fill");
		r->fw_u_env_light = glGetUniformLocation(r->prog_forward, "u_env_light");
		r->fw_u_env_energy = glGetUniformLocation(r->prog_forward, "u_env_energy");
		r->fw_u_up = glGetUniformLocation(r->prog_forward, "u_up");
		r->fw_u_world_hor = glGetUniformLocation(r->prog_forward, "u_world_hor");
		r->fw_u_world_zen = glGetUniformLocation(r->prog_forward, "u_world_zen");
		r->fw_u_world_amb = glGetUniformLocation(r->prog_forward, "u_world_amb");
		r->fw_u_world_blend = glGetUniformLocation(r->prog_forward, "u_world_blend");
		r->fw_u_num_lights = glGetUniformLocation(r->prog_forward, "u_num_lights");
		/* ★ раунд 16: в forward те же данные ламп float-текстурой */
		r->fw_u_lightdata = glGetUniformLocation(r->prog_forward, "u_light_data");
		r->fw_u_lighttexel = glGetUniformLocation(r->prog_forward, "u_light_texel");
		for (int li = 0; li < GPU_MAX_SHADOW_LIGHTS; li++) {
			char nm[32];
			sprintf(nm, "u_shadow[%d]", li);
			r->fw_u_shadow[li] = glGetUniformLocation(r->prog_forward, nm);
			sprintf(nm, "u_lvp[%d]", li);
			r->fw_u_lvp[li] = glGetUniformLocation(r->prog_forward, nm);
			sprintf(nm, "u_shadow_cube[%d]", li);
			r->fw_u_shadow_cube[li] = glGetUniformLocation(r->prog_forward, nm);
			sprintf(nm, "u_lwpos[%d]", li);
			r->fw_u_lwpos[li] = glGetUniformLocation(r->prog_forward, nm);
			sprintf(nm, "u_lpz[%d]", li);
			r->fw_u_lpz[li] = glGetUniformLocation(r->prog_forward, nm);
		}
		r->fw_u_pcf = glGetUniformLocation(r->prog_forward, "u_pcf");
		r->fw_u_shadow_texel = glGetUniformLocation(r->prog_forward, "u_shadow_texel");
		r->fw_u_shadow_bias = glGetUniformLocation(r->prog_forward, "u_shadow_bias");
		r->fw_u_depth = glGetUniformLocation(r->prog_forward, "u_depth");
		r->fw_u_resolution = glGetUniformLocation(r->prog_forward, "u_resolution");
		r->fw_u_pack = glGetUniformLocation(r->prog_forward, "u_pack");
	r->fw_u_ortho = glGetUniformLocation(r->prog_forward, "u_ortho");
		r->fw_u_albedo = glGetUniformLocation(r->prog_forward, "u_albedo");
		r->fw_u_inv_proj = glGetUniformLocation(r->prog_forward, "u_inv_proj");
		r->fw_u_texel = glGetUniformLocation(r->prog_forward, "u_texel");
		r->fw_u_ssr = glGetUniformLocation(r->prog_forward, "u_ssr");
		r->fw_u_ao = glGetUniformLocation(r->prog_forward, "u_ao");
		r->fw_u_tex = glGetUniformLocation(r->prog_forward, "u_tex");
		r->fw_u_has_tex = glGetUniformLocation(r->prog_forward, "u_has_tex");
		r->fw_u_colfac = glGetUniformLocation(r->prog_forward, "u_colfac");
		r->fw_u_refl = glGetUniformLocation(r->prog_forward, "u_refl");
		r->fw_u_tex_blend = glGetUniformLocation(r->prog_forward, "u_tex_blend");
		r->fw_u_tex_orco = glGetUniformLocation(r->prog_forward, "u_tex_orco");
	r->fw_u_tex_glob = glGetUniformLocation(r->prog_forward, "u_tex_glob");
		r->fw_u_tex_map = glGetUniformLocation(r->prog_forward, "u_tex_map");
	/* ★ M19: процедурная карта цвета в forward-проходе */
	r->fw_u_proc_type = glGetUniformLocation(r->prog_forward, "u_proc_type");
	r->fw_u_proc_col = glGetUniformLocation(r->prog_forward, "u_proc_col");   /* M20 */
	r->fw_u_tex_rgbtoint = glGetUniformLocation(r->prog_forward, "u_tex_rgbtoint");
	r->fw_u_proc_stype = glGetUniformLocation(r->prog_forward, "u_proc_stype");
	r->fw_u_proc_flip = glGetUniformLocation(r->prog_forward, "u_proc_flip");
	r->fw_u_proc_noisesize = glGetUniformLocation(r->prog_forward, "u_proc_noisesize");
	r->fw_u_proc_noisedepth = glGetUniformLocation(r->prog_forward, "u_proc_noisedepth");
	r->fw_u_proc_basis = glGetUniformLocation(r->prog_forward, "u_proc_basis");
	r->fw_u_proc_wf = glGetUniformLocation(r->prog_forward, "u_proc_wf");
	r->fw_u_proc_dist = glGetUniformLocation(r->prog_forward, "u_proc_dist");
	r->fw_u_proc_hard = glGetUniformLocation(r->prog_forward, "u_proc_hard");
	r->fw_u_proc_bright = glGetUniformLocation(r->prog_forward, "u_proc_bright");
	r->fw_u_proc_contrast = glGetUniformLocation(r->prog_forward, "u_proc_contrast");
	r->fw_u_proc_noclamp = glGetUniformLocation(r->prog_forward, "u_proc_noclamp");
	r->fw_u_proc_has_ramp = glGetUniformLocation(r->prog_forward, "u_proc_has_ramp");
	r->fw_u_proc_ramp = glGetUniformLocation(r->prog_forward, "u_proc_ramp");
	r->fw_u_proc_hash = glGetUniformLocation(r->prog_forward, "u_proc_hash");
	r->fw_u_proc_hashv = glGetUniformLocation(r->prog_forward, "u_proc_hashv");
	r->fw_u_proc_hashp = glGetUniformLocation(r->prog_forward, "u_proc_hashp");   /* M20 */
	r->fw_u_proc_orco = glGetUniformLocation(r->prog_forward, "u_proc_orco");
	r->fw_u_proc_size = glGetUniformLocation(r->prog_forward, "u_proc_size");
	r->fw_u_proc_ofs = glGetUniformLocation(r->prog_forward, "u_proc_ofs");
		r->fw_u_tex_size = glGetUniformLocation(r->prog_forward, "u_tex_size");
		r->fw_u_tex_ofs = glGetUniformLocation(r->prog_forward, "u_tex_ofs");
		r->fw_u_atex = glGetUniformLocation(r->prog_forward, "u_atex");
		r->fw_u_has_atex = glGetUniformLocation(r->prog_forward, "u_has_atex");
		r->fw_u_a_fac = glGetUniformLocation(r->prog_forward, "u_a_fac");
		r->fw_u_a_def = glGetUniformLocation(r->prog_forward, "u_a_def");
		r->fw_u_a_blend = glGetUniformLocation(r->prog_forward, "u_a_blend");
		r->fw_u_a_use_a = glGetUniformLocation(r->prog_forward, "u_a_use_a");
		r->fw_u_a_orco = glGetUniformLocation(r->prog_forward, "u_a_orco");
	r->fw_u_a_glob = glGetUniformLocation(r->prog_forward, "u_a_glob");
		r->fw_u_a_map = glGetUniformLocation(r->prog_forward, "u_a_map");
		r->fw_u_a_size = glGetUniformLocation(r->prog_forward, "u_a_size");
		r->fw_u_a_ofs = glGetUniformLocation(r->prog_forward, "u_a_ofs");
		r->fw_u_nrm = glGetUniformLocation(r->prog_forward, "u_nrm");
		r->fw_u_has_nrm = glGetUniformLocation(r->prog_forward, "u_has_nrm");
		r->fw_u_spec_col = glGetUniformLocation(r->prog_forward, "u_spec_col");
		r->fw_u_hard = glGetUniformLocation(r->prog_forward, "u_hard");
		printf("[GPU-INIT] prog_forward=%u (M15 forward pass)\n", r->prog_forward);
	} else {
		printf("[GPU-INIT] prog_forward link FAILED -> screen-door fallback\n");
	}

	/* ★ M16: спрайт-хало (материалы типа HALO). Тоже опциональный: без него
	 * хало просто не рисуются, деферред-путь продолжает работать. */
	r->prog_halo = gl_link(gl_compile(GL_VERTEX_SHADER, HALO_VS),
	                       gl_compile(GL_FRAGMENT_SHADER, HALO_FS));
	if (r->prog_halo) {
		r->halo_a_corner = glGetAttribLocation(r->prog_halo, "a_corner");
		r->halo_a_uv = glGetAttribLocation(r->prog_halo, "a_uv");
		r->halo_u_proj = glGetUniformLocation(r->prog_halo, "u_proj");
		r->halo_u_view = glGetUniformLocation(r->prog_halo, "u_view");
		r->halo_u_pos = glGetUniformLocation(r->prog_halo, "u_pos");
		r->halo_u_right = glGetUniformLocation(r->prog_halo, "u_right");
		r->halo_u_up = glGetUniformLocation(r->prog_halo, "u_up");
		r->halo_u_rad = glGetUniformLocation(r->prog_halo, "u_rad");
		r->halo_u_col = glGetUniformLocation(r->prog_halo, "u_col");
		r->halo_u_alfa = glGetUniformLocation(r->prog_halo, "u_alfa");
		r->halo_u_hard = glGetUniformLocation(r->prog_halo, "u_hard");
		r->halo_u_add = glGetUniformLocation(r->prog_halo, "u_add");
		r->halo_u_zcenter = glGetUniformLocation(r->prog_halo, "u_zcenter");
		r->halo_u_depth = glGetUniformLocation(r->prog_halo, "u_depth");
		r->halo_u_pack = glGetUniformLocation(r->prog_halo, "u_pack");
		r->halo_u_resolution = glGetUniformLocation(r->prog_halo, "u_resolution");
	r->halo_u_hashv = glGetUniformLocation(r->prog_halo, "u_hashv");
	r->halo_u_seed = glGetUniformLocation(r->prog_halo, "u_seed");
	r->halo_u_ringc = glGetUniformLocation(r->prog_halo, "u_ringc");
	r->halo_u_linec = glGetUniformLocation(r->prog_halo, "u_linec");
	r->halo_u_starpoints = glGetUniformLocation(r->prog_halo, "u_starpoints");
	r->halo_u_xalpha = glGetUniformLocation(r->prog_halo, "u_xalpha");
	r->halo_u_spec_col = glGetUniformLocation(r->prog_halo, "u_spec_col");
	r->halo_u_mir_col = glGetUniformLocation(r->prog_halo, "u_mir_col");
		printf("[GPU-INIT] prog_halo=%u (M16 halo sprites)\n", r->prog_halo);
	} else {
		printf("[GPU-INIT] prog_halo link FAILED -> halo materials skipped\n");
	}

	/* если хоть одна программа не слинковалась — не рисуем (иначе порча UI) */
	if (!r->prog_gb || !r->prog_light || !r->prog_shadow) {
		printf("[GPU] shader link failed, disabling GPU deferred draw\n");
		gpu_res_free(r);
		MEM_freeN(gr->gpu_res);
		gr->gpu_res = NULL;
		return -3;
	}

	/* --- полноэкранный quad --- */
	const float quad[16] = {
		-1, -1,  0, 0,
		 1, -1,  1, 0,
		 1,  1,  1, 1,
		-1,  1,  0, 1,
	};
	glGenBuffers(1, &r->quad_vbo);
	glBindBuffer(GL_ARRAY_BUFFER, r->quad_vbo);
	glBufferData(GL_ARRAY_BUFFER, sizeof(quad), quad, GL_STATIC_DRAW);
	const unsigned int quad_idx[6] = { 0, 1, 2, 0, 2, 3 };
	glGenBuffers(1, &r->quad_ibo);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, r->quad_ibo);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(quad_idx), quad_idx, GL_STATIC_DRAW);
	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

	/* --- загрузка мешей в VBO/IBO --- */
	for (RE_GPUMesh *m = gr->scene->meshes; m; m = m->next)
		gpu_upload_mesh(m);

	r->inited = true;

	printf("[GPU-INIT] %dx%d prog_gb=%u prog_light=%u prog_shadow=%u "
	       "loc(proj=%d model=%d normmat=%d pack=%d) attr(co=%d n=%d rgba=%d emit=%d) "
	       "tex(albedo=%u nrm=%u vpos=%u) rb(depth=%u)\n",
	       width, height, r->prog_gb, r->prog_light, r->prog_shadow,
	       r->gb_u_proj, r->gb_u_model, r->gb_u_normmat, r->gb_u_pack,
	       r->gb_a_co, r->gb_a_n, r->gb_a_rgba, r->gb_a_emit,
	       r->tex_albedo, r->tex_nrm, r->tex_vpos,
	       r->rb_depth);
	/* ★ По одной строке на лампу — раньше было жёстко 4, и при большем лимите
	 * диагностика молчала о теневых картах остальных ламп. */
	for (int si = 0; si < GPU_MAX_SHADOW_LIGHTS; si++)
		printf("[GPU-INIT] shadow[%d]=%u/%u/%u\n", si,
		       r->tex_shadow[si], r->shadow_fb[si], r->rb_shadow[si]);
	{
		int nmesh = 0, total_v = 0, total_i = 0;
		for (RE_GPUMesh *m = gr->scene->meshes; m; m = m->next) {
			nmesh++; total_v += m->num_verts; total_i += m->num_indices;
			if (nmesh <= 3)
				printf("[GPU-INIT] mesh %d: uploaded=%d vbo=%u ibo=%u verts=%d idx=%d\n",
				       nmesh, (int)m->uploaded, m->vbo, m->ibo, m->num_verts, m->num_indices);
		}
		printf("[GPU-INIT] meshes=%d total_verts=%d total_idx=%d\n", nmesh, total_v, total_i);
	}
	return 0;
}

void RE_gpu_renderer_draw(RE_GPURenderer *gr)
{
	if (!gr || !gr->gpu_res || !gr->scene) return;
	RE_GPUGPURes *r = gr->gpu_res;
	if (!r->inited) return;

	Render *re = gr->re;

	glDisable(GL_DEPTH_TEST);
	/* ★ (0,0,0,0): альфа color-аттачментов служит маской «есть геометрия»,
	 * поэтому фон обязан очищаться в 0 — при любом другом clear-цвете
	 * маска выставилась бы и фон считался бы геометрией. */
	glClearColor(0.0f, 0.0f, 0.0f, 0.0f);

	/* сохранить текущий (вьюпортовский) framebuffer и viewport: результат рисуем в них */
	GLint saved_fb = 0;
	GLint saved_vp[4] = { 0, 0, 0, 0 };
	glGetIntegerv(GL_FRAMEBUFFER_BINDING_EXT, &saved_fb);
	glGetIntegerv(GL_VIEWPORT, saved_vp);

	/* ★ СКАССОР. При отрисовке окна GL_SCISSOR_TEST включён (wm_window.c:631), а
	 * прямоугольник выставлен по РЕГИОНУ (wm_subwindow.c:310, координаты окна).
	 * Промежуточные проходы рисуют в СВОИ FBO размера r->width x r->height, но
	 * glViewport(0,0,...) их от скассора не спасает: скассор задан в координатах
	 * ОКНА, поэтому в FBO попадала только подобласть
	 * x in [saved_vp[0], r->width), y in [saved_vp[1], r->height) — G-буфер и карта
	 * теней оказывались обрезаны и сдвинуты, а финальный проход показывал этот
	 * полупустой буфер. Держим скассор выключенным на время проходов в FBO. */
	GLboolean scissor_was_on = glIsEnabled(GL_SCISSOR_TEST);
	GLint saved_scissor[4] = { 0, 0, 0, 0 };
	glGetIntegerv(GL_SCISSOR_BOX, saved_scissor);
	glDisable(GL_SCISSOR_TEST);

	/* нормировка линейной глубины в [0,1): общая для G-буфера и освещения */
	const float pack_scale = 1.0f / ((re->clipend > 0.001f) ? re->clipend : 1000.0f);

	/* диагностика первых кадров: видно, есть ли геометрия и валидны ли матрицы */
	{
		static int dbg_frames = 0;
		if (dbg_frames < 3) {
			GLint vp[4];
			glGetIntegerv(GL_VIEWPORT, vp);
			RE_GPUMesh *m0 = gr->scene->meshes;
			dbg_frames++;
			/* M15: подсчёт полупрозрачных мешей (для будущего forward-прохода) */
			int ntrans = 0;
			{
				RE_GPUMesh *mm = gr->scene->meshes;
				while (mm) { if (mm->transparent) ntrans++; mm = mm->next; }
			}
			printf("[GPU-DRAW] #%d buf=%dx%d saved_fb=%d vp=(%d,%d,%d,%d) meshes=%d "
			       "transparent=%d "
			       "clipend=%.3f pack=%.6f m0(uploaded=%d vbo=%u nidx=%d) "
			       "winmat0=(%.3f %.3f %.3f %.3f) viewmat0=(%.3f %.3f %.3f %.3f)\n",
			       dbg_frames, r->width, r->height, (int)saved_fb,
			       vp[0], vp[1], vp[2], vp[3], gr->scene->num_meshes, ntrans,
			       re->clipend, pack_scale,
			       m0 ? (int)m0->uploaded : -1, m0 ? m0->vbo : 0u,
			       m0 ? m0->num_indices : -1,
			       re->winmat[0][0], re->winmat[0][1], re->winmat[0][2], re->winmat[0][3],
			       re->viewmat[0][0], re->viewmat[0][1], re->viewmat[0][2], re->viewmat[0][3]);
		}
	}

	const int stride = sizeof(RE_GPUVertex);

	/* --- проход 0: ★ теневые карты. По одной на каждую лампу, которая даёт тень.
	 * SUN/SPOT — 2D-карта (орто/перспектива), POINT — кубическая (6 граней).
	 * Рисуем ВСЮ сцену в каждую карту, но с отсечением по фрустуму грани/лампы.
	 * Кэш: тени зависят только от геометрии и ламп (не от камеры), поэтому
	 * пересчитываются при пересборке сцены, а при движении камеры берутся из кэша
	 * (иначе кубические тени point-ламп пересчитывались бы 6 раз каждый кадр).
	 * UBF_GPU_SHADOW_ALWAYS=1 принудительно пересчитывает каждый кадр (без кэша). */
	const char *sh_always_e = getenv("UBF_GPU_SHADOW_ALWAYS");
	const int sh_always = sh_always_e ? (atoi(sh_always_e) != 0) : 0;
	const int sh_do = (sh_always || r->shadow_dirty) ? 1 : 0;
	/* диагностика первых кадров: RENDER — пересчитали, CACHED — взяли из кэша */
	{
		static int sh_frames = 0;
		if (sh_frames < 4) {
			sh_frames++;
			printf("[GPU-SHADOW] frame %d: %s (dirty=%d always=%d)\n",
			       sh_frames, sh_do ? "RENDER" : "CACHED",
			       (int)r->shadow_dirty, sh_always);
		}
	}
	/* ★ Отсечение по фрустуму вкл/выкл (страховка): UBF_GPU_CULL=0 отключает.
	 * Используется и в проходе теней, и в G-буфере, поэтому объявлен здесь. */
	const char *cull_e = getenv("UBF_GPU_CULL");
	const int cull_on = cull_e ? (atoi(cull_e) != 0) : 1;
	if (sh_do) {
	glUseProgram(r->prog_shadow);
	glDisable(GL_CULL_FACE);
	glDisable(GL_BLEND);
	glDepthMask(GL_TRUE);
	for (int si = 0; si < gr->num_lights; si++) {
		RE_GPULight *L = &gr->lights[si];
		if (!L->shadows) continue;

		const int is_cube = (L->type == GPU_LIGHT_POINT || L->type == GPU_LIGHT_AREA);
		const int nface = is_cube ? 6 : 1;
		for (int face = 0; face < nface; face++) {
			float vp[4][4];
			glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, r->shadow_fb[si]);
			if (is_cube) {
				/* грань куба: proj * face_view[face]; color-attachment — эта грань */
				mul_m4_m4m4(vp, L->proj, L->face_view[face]);
				glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT,
				                          GL_TEXTURE_CUBE_MAP_POSITIVE_X + face,
				                          r->tex_shadow_cube[si], 0);
			} else {
				/* 2D: attachment мог остаться на грани куба от point-лампы — вернуть */
				copy_m4_m4(vp, L->vp);
				glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT,
				                          GL_TEXTURE_2D, r->tex_shadow[si], 0);
			}
			glViewport(0, 0, GPU_SHADOW_SIZE, GPU_SHADOW_SIZE);
			glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
			glEnable(GL_DEPTH_TEST);
			glUniformMatrix4fv(r->shadow_u_lightvp, 1, GL_FALSE, (const GLfloat *)vp);

			/* ★ Отсечение по фрустуму СВЕТА/ГРАНИ: меши вне конуса/орта не могут
			 * отбросить тень — их можно не рисовать. */
			float spl[6][4];
			gpu_frustum_planes(vp, spl);

			/* ★ Смещение глубины при растеризации карты теней — штатное средство
			 * против acne (самотенения получателя): записанная глубина отодвигается
			 * от источника света на slope-scaled величину. Без него квантование
			 * глубины по текселю (~0.03 юнита при текселе 0.038) превышает
			 * константный bias в gpu_shadow и даёт муар по всей поверхности.
			 * Шейдеры не меняются, поэтому проверки gpu_shadow остаются валидны. */
			glEnable(GL_POLYGON_OFFSET_FILL);
			glPolygonOffset(2.0f, 4.0f);

			for (RE_GPUMesh *m = gr->scene->meshes; m; m = m->next) {
				if (!m->uploaded || m->num_indices == 0) continue;
				if (cull_on && gpu_aabb_outside(spl, m->bb_min, m->bb_max)) continue;
				glUniformMatrix4fv(r->shadow_u_obmat, 1, GL_FALSE, (const GLfloat *)m->obmat);
				glBindBuffer(GL_ARRAY_BUFFER, m->vbo);
				glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m->ibo);
				glEnableVertexAttribArray(r->shadow_a_co);
				glVertexAttribPointer(r->shadow_a_co, 3, GL_FLOAT, GL_FALSE, stride, (void *)0);
				glDrawElements(GL_TRIANGLES, m->num_indices, GL_UNSIGNED_INT, NULL);
			}
			glDisable(GL_POLYGON_OFFSET_FILL);
		}
	}
	r->shadow_dirty = false;   /* кэш теперь валиден до следующей пересборки */
	}  /* if (sh_do) — конец прохода теней */
	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

	/* --- проход 1: геометрия -> G-буфер --- */
	glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, r->fb);
	glViewport(0, 0, r->width, r->height);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	GLenum bufs[5] = {
		GL_COLOR_ATTACHMENT0_EXT,
		GL_COLOR_ATTACHMENT1_EXT,
		GL_COLOR_ATTACHMENT2_EXT,
		GL_COLOR_ATTACHMENT3_EXT,
		GL_COLOR_ATTACHMENT4_EXT,
	};
	glDrawBuffers((r->spec_target ? 4 : 3) + (r->refl_target ? 1 : 0), bufs);

	glEnable(GL_DEPTH_TEST);
	glDepthMask(GL_TRUE);
	glDisable(GL_BLEND);
	glDisable(GL_CULL_FACE);

	glUseProgram(r->prog_gb);
	glUniformMatrix4fv(r->gb_u_proj, 1, GL_FALSE, (const GLfloat *)re->winmat);
	glUniform1f(r->gb_u_pack, pack_scale);
	/* M5: диффузная карта материала — юнит 4 (0..3 заняты проходом освещения).
	 * M8: нормальная карта — юнит 5 (в G-буфер шейдере других текстур нет). */
	glActiveTexture(GL_TEXTURE4);
	glUniform1i(r->gb_u_tex, 4);
	/* M14: слоты 1..3 — юниты 7..9 */
	glActiveTexture(GL_TEXTURE7);
	glUniform1i(r->gb_u_tex1, 7);
	glActiveTexture(GL_TEXTURE8);
	glUniform1i(r->gb_u_tex2, 8);
	glActiveTexture(GL_TEXTURE9);
	glUniform1i(r->gb_u_tex3, 9);
	glActiveTexture(GL_TEXTURE5);
	glUniform1i(r->gb_u_nrm, 5);
	/* M10: альфа-карта — юнит 6 (в G-буфере юниты 6..9 не заняты) */
	glActiveTexture(GL_TEXTURE6);
	glUniform1i(r->gb_u_atex, 6);

	/* screen-door прозрачность включена по умолчанию; UBF_GPU_ALPHA=0 выключает */
	float bump_scale = 1.0f;
	const char *alpha_e = getenv("UBF_GPU_ALPHA");
	const int alpha_on = alpha_e ? (atoi(alpha_e) != 0) : 1;

	/* ★ M30: множитель высотного bump (карты нормалей). BI нормирует градиент
	 * texco-шагом (bf/du), у нас же Mikkelsen-градиент в физических единицах —
	 * поэтому нужен подборочный коэффициент. UBF_GPU_BUMPSCALE переопределяет. */
	{
		const char *bs_e = getenv("UBF_GPU_BUMPSCALE");
		bump_scale = bs_e ? (float)atof(bs_e) : 1.0f;
		if (getenv("UBF_GPU_DEBUG"))
			printf("[GPU-BUMPSCALE] env=%s scale=%.4f\n",
			       bs_e ? bs_e : "<unset>", bump_scale);
	}
	/* ★ раунд 17: UBF_GPU_NOSPEC=1 отключает карты блика (диагностика крапчатости) */
	int no_spec = 0;
	{
		const char *ns_e = getenv("UBF_GPU_NOSPEC");
		no_spec = ns_e ? (atoi(ns_e) != 0) : 0;
	}

	/* M15: forward-проход полупрозрачных мешей. Включается, если программа
	 * слинковалась и UBF_GPU_FWD != 0. При включённом forward полупрозрачные меши
	 * НЕ рисуются в G-буфер (там screen-door) — они уходят в последний проход. */
	const char *fwd_e = getenv("UBF_GPU_FWD");
	const int fwd_on = (r->prog_forward != 0) && (fwd_e ? (atoi(fwd_e) != 0) : 1);

	/* ★ Отсечение по фрустуму камеры (world->clip = winmat * viewmat). Меши вне
	 * кадра не рисуем — экономит draw-вызовы на больших сценах. */
	float cull_m[4][4], cull_pl[6][4];
	mul_m4_m4m4(cull_m, re->winmat, re->viewmat);
	gpu_frustum_planes(cull_m, cull_pl);

	for (RE_GPUMesh *m = gr->scene->meshes; m; m = m->next) {
		/* ★ M16: у меша материала WIRE треугольников нет вообще — только линии,
		 * поэтому одного num_indices == 0 для пропуска уже недостаточно. */
		if (!m->uploaded || (m->num_indices == 0 && m->num_line_indices == 0)) continue;
		if (fwd_on && m->transparent) continue;   /* M15: полупрозрачные — в forward-проход */
		if (cull_on && gpu_aabb_outside(cull_pl, m->bb_min, m->bb_max)) continue;

		/* ★ obj->view и нормальная матрица считаются здесь, каждый кадр, из
		 * ТЕКУЩЕГО re->viewmat и obmat объекта. Именно это позволяет не
		 * пересобирать сцену при движении камеры (см. build_render). */
		/* ★★ ИСПРАВЛЕНО (r59): порядок умножения был ОБРАТНЫМ.
		 * У Blender соглашение СТРОКА-ВЕКТОР: v' = v * A * B, и функция
		 * mul_m4_m4m4(R, A, B) даёт R = A*B, т.е. СНАЧАЛА применяется A,
		 * потом B. Значит «объект -> мир -> вид» это obmat * viewmat.
		 * Было mul_m4_m4m4(model, re->viewmat, m->obmat) — вид применялся
		 * РАНЬШЕ объекта: у любого объекта с поворотом/переносом и позиция,
		 * и ориентация получались неверными («сцена во вьюпорте сильно
		 * повёрнута», объекты разъезжаются). Для одного куба в начале
		 * координат (obmat = единица) ошибка незаметна — поэтому и
		 * проявлялась только на реальных сценах. */
		float model[4][4], nm3[3][3], nm3i[3][3], nrm[3][3];
		mul_m4_m4m4(model, m->obmat, re->viewmat);
		glUniformMatrix4fv(r->gb_u_model, 1, GL_FALSE, (const GLfloat *)model);

		/* ★ Диагностика: что именно лежит в вершинах, которые мы рисуем.
		 * Если тут мусор — виновата сборка/заливка меша, а не матрицы. */
		{
			static int vtx_dbg = 0;
			if (vtx_dbg < 2) {
				vtx_dbg++;
				printf("[GPU-VTX] mesh#%d verts=%d idx=%d "
				       "v0.co=(%.3f %.3f %.3f) v1.co=(%.3f %.3f %.3f) "
				       "bb=(%.2f %.2f %.2f)..(%.2f %.2f %.2f)\n",
				       vtx_dbg, m->num_verts, m->num_indices,
				       m->verts[0].co[0], m->verts[0].co[1], m->verts[0].co[2],
				       (m->num_verts > 1) ? m->verts[1].co[0] : 0.0f,
				       (m->num_verts > 1) ? m->verts[1].co[1] : 0.0f,
				       (m->num_verts > 1) ? m->verts[1].co[2] : 0.0f,
				       m->bb_min[0], m->bb_min[1], m->bb_min[2],
				       m->bb_max[0], m->bb_max[1], m->bb_max[2]);
			}
		}

		/* ★ Диагностика: куда попадает НАЧАЛО МИРА (0,0,0,1) после
		 * u_proj*u_model. Ожидаем NDC около (0,0) — тогда объект в начале
		 * координат рисуется в центре кадра. Если y сильно уходит — виновата
		 * проекция (re->winmat / viewplane), а не вид. */
		{
			static int origin_dbg = 0;
			if (origin_dbg < 3) {
				origin_dbg++;
				/* ★ ВАЖНО: у Blender соглашение СТРОКА-ВЕКТОР: v' = v * M, и
				 * перенос лежит в СТРОКЕ 3 (mat[3][0..2]), а не в столбце.
				 * Поэтому считаем именно так, как это делает mul_m4_v4. */
				float mv[4], c[4];
				int i;
				mv[0] = model[3][0]; mv[1] = model[3][1];
				mv[2] = model[3][2]; mv[3] = model[3][3];
				for (i = 0; i < 4; i++)
					c[i] = mv[0] * re->winmat[0][i] + mv[1] * re->winmat[1][i] +
					       mv[2] * re->winmat[2][i] + mv[3] * re->winmat[3][i];
				printf("[GPU-ORIGIN] мир(0,0,0): view=(%.3f %.3f %.3f %.3f) "
				       "clip=(%.3f %.3f %.3f %.3f) ndc=(%.4f %.4f) win=(%.4f %.4f)\n"
				       "            obmat  строка3=(%.3f %.3f %.3f %.3f)\n"
				       "            viewmat строка3=(%.3f %.3f %.3f %.3f)\n"
				       "            winmat строка3=(%.4f %.4f %.4f %.4f) строка2=(%.4f %.4f %.4f %.4f)\n",
				       mv[0], mv[1], mv[2], mv[3],
				       c[0], c[1], c[2], c[3],
				       (c[3] != 0.0f) ? c[0] / c[3] : 0.0f,
				       (c[3] != 0.0f) ? c[1] / c[3] : 0.0f,
				       (c[3] != 0.0f) ? (c[0] / c[3]) * 0.5f + 0.5f : 0.0f,
				       (c[3] != 0.0f) ? (c[1] / c[3]) * 0.5f + 0.5f : 0.0f,
				       m->obmat[3][0], m->obmat[3][1], m->obmat[3][2], m->obmat[3][3],
				       re->viewmat[3][0], re->viewmat[3][1], re->viewmat[3][2], re->viewmat[3][3],
				       re->winmat[3][0], re->winmat[3][1], re->winmat[3][2], re->winmat[3][3],
				       re->winmat[2][0], re->winmat[2][1], re->winmat[2][2], re->winmat[2][3]);
			}
		}

		copy_m3_m4(nm3, model);
		invert_m3_m3(nm3i, nm3);
		transpose_m3_m3(nrm, nm3i);
		glUniformMatrix3fv(r->gb_u_normmat, 1, GL_FALSE, (const GLfloat *)nrm);

		glBindBuffer(GL_ARRAY_BUFFER, m->vbo);
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m->ibo);

		glEnableVertexAttribArray(r->gb_a_co);
		glVertexAttribPointer(r->gb_a_co, 3, GL_FLOAT, GL_FALSE, stride, (void *)0);
		glEnableVertexAttribArray(r->gb_a_n);
		glVertexAttribPointer(r->gb_a_n, 3, GL_FLOAT, GL_FALSE, stride,
		                      (void *)(3 * sizeof(float)));
		glEnableVertexAttribArray(r->gb_a_rgba);
		glVertexAttribPointer(r->gb_a_rgba, 4, GL_FLOAT, GL_FALSE, stride,
		                      (void *)(8 * sizeof(float)));
		glEnableVertexAttribArray(r->gb_a_emit);
		glVertexAttribPointer(r->gb_a_emit, 1, GL_FLOAT, GL_FALSE, stride,
		                      (void *)(12 * sizeof(float)));
		glEnableVertexAttribArray(r->gb_a_uv);
		glVertexAttribPointer(r->gb_a_uv, 2, GL_FLOAT, GL_FALSE, stride,
		                      (void *)(6 * sizeof(float)));
		/* M8: касательная (T.xyz + знак в w) */
		glEnableVertexAttribArray(r->gb_a_tangent);
		glVertexAttribPointer(r->gb_a_tangent, 4, GL_FLOAT, GL_FALSE, stride,
		                      (void *)(13 * sizeof(float)));
		/* M11: ORCO вершины (для карт с texco==ORCO) */
		glEnableVertexAttribArray(r->gb_a_orco);
		glVertexAttribPointer(r->gb_a_orco, 3, GL_FLOAT, GL_FALSE, stride,
		                      (void *)(17 * sizeof(float)));

		/* M5: карта материала (белая заглушка, если карты нет) */
		gpu_upload_mesh_tex(m);
		glBindTexture(GL_TEXTURE_2D, m->tex ? (GLuint)m->tex : r->tex_white);
		glUniform1f(r->gb_u_has_tex, m->tex ? 1.0f : 0.0f);
		/* M9: как смешивать карту с базовым цветом (BI texture_rgb_blend) */
		glUniform1f(r->gb_u_colfac, m->colfac);
		glUniform1f(r->gb_u_refl, m->refl);   /* M25: shi->refl = ma->ref */
		glUniform1i(r->gb_u_tex_blend, m->tex_blend);
		/* M11: источник координат диффузной карты (ORCO или UV) */
		glUniform1f(r->gb_u_tex_orco, m->tex_orco ? 1.0f : 0.0f);
	glUniform1f(r->gb_u_tex_glob, m->tex_glob ? 1.0f : 0.0f);
		glUniform1f(r->gb_u_tex_map, (float)m->tex_map);
		glUniform2f(r->gb_u_tex_size, m->tex_size[0], m->tex_size[1]);
		glUniform2f(r->gb_u_tex_ofs, m->tex_ofs[0], m->tex_ofs[1]);
		/* ★ M19: процедурная карта цвета (CLOUDS/BLEND). Таблицы orgBlenderNoise —
		 * юниты 10/11, колорбэнд материала — юнит 12 (в G-буферном проходе они
		 * свободны: forward занимает 10..13, но это другой проход и другой
		 * шейдер). Колорбэнд заливаем glTexSubImage2D на каждый меш — 1 КБ. */
		glUniform1i(r->gb_u_proc_type, m->proc_type);
		/* M20: base модулируется, если это образная карта (proc_type==0) либо
		 * процедурная с MAP_COL. Процедурная только с MAP_SPEC/HAR base не трогает. */
		glUniform1f(r->gb_u_proc_col, (m->proc_type != 0 && !m->proc_col) ? 0.0f : 1.0f);
		glUniform1i(r->gb_u_proc_stype, m->proc_stype);
		glUniform1i(r->gb_u_proc_flip, m->proc_flip);
		glUniform1f(r->gb_u_proc_noisesize, m->proc_noisesize);
		glUniform1i(r->gb_u_proc_noisedepth, m->proc_noisedepth);
		glUniform1i(r->gb_u_proc_basis, m->proc_basis);
	glUniform1i(r->gb_u_proc_wf, m->proc_wf);
	glUniform1f(r->gb_u_proc_dist, m->proc_dist);
		glUniform1i(r->gb_u_proc_hard, m->proc_hard);
		glUniform1f(r->gb_u_proc_bright, m->proc_bright);
		glUniform1f(r->gb_u_proc_contrast, m->proc_contrast);
		glUniform1i(r->gb_u_proc_noclamp, m->proc_noclamp);
		glUniform1i(r->gb_u_proc_has_ramp, m->proc_has_ramp);
		glUniform1i(r->gb_u_proc_orco, m->proc_orco);   /* M21/M27: 0=UV, 1=ORCO, 2=мир */
		glUniform2f(r->gb_u_proc_size, m->proc_size[0], m->proc_size[1]);
		glUniform2f(r->gb_u_proc_ofs, m->proc_ofs[0], m->proc_ofs[1]);
		if (m->proc_type != 0) {
			glUniform1f(r->gb_u_has_tex, 1.0f);   /* процедурная карта вместо образной */
			glActiveTexture(GL_TEXTURE10);
			glBindTexture(GL_TEXTURE_2D, r->tex_hash);
			glUniform1i(r->gb_u_proc_hash, 10);
			glActiveTexture(GL_TEXTURE11);
			glBindTexture(GL_TEXTURE_2D, r->tex_hashv);
			glUniform1i(r->gb_u_proc_hashv, 11);
			glActiveTexture(GL_TEXTURE12);
			glBindTexture(GL_TEXTURE_2D, r->tex_ramp);
			glUniform1i(r->gb_u_proc_ramp, 12);
			glActiveTexture(GL_TEXTURE13);
			glBindTexture(GL_TEXTURE_2D, r->tex_hashp);
			glUniform1i(r->gb_u_proc_hashp, 13);
			if (m->proc_has_ramp) {
				glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, 256, 1, GL_RGBA,
				                GL_UNSIGNED_BYTE, m->proc_ramp);
			}
			if (getenv("UBF_GPU_DEBUG")) {
				static int proc_draws = 0;
				if (proc_draws < 8) {
					proc_draws++;
					printf("[GPU-PROCDRAW] ma='%s' type=%d stype=%d ramp=%d basis=%d depth=%d scale=%.3f orco=%d colfac=%.2f bt=%d\n",
					       m->ma ? m->ma->id.name + 2 : "<none>", m->proc_type, m->proc_stype,
					       m->proc_has_ramp, m->proc_basis, m->proc_noisedepth,
					       m->proc_noisesize, m->proc_orco, m->colfac, m->tex_blend);
				}
			}
			glActiveTexture(GL_TEXTURE4);   /* вернуть юнит диффузной карты */
		}
		/* ★ M22: вторая процедурная текстура материала (слот != 0) — источник
		 * MAP_SPEC/MAP_HAR. Таблицы шума общие с первой (юниты 10/11/13), свой
		 * только колорбэнд — юнит 3 (в G-буферном проходе 0..3 свободны). */
		glUniform1f(r->gb_u_has_proc2, (m->has_proc2 && m->proc2_type != 0) ? 1.0f : 0.0f);
		if (m->has_proc2 && m->proc2_type != 0) {
			glUniform1i(r->gb_u_proc2_type, m->proc2_type);
			glUniform1i(r->gb_u_proc2_stype, m->proc2_stype);
			glUniform1i(r->gb_u_proc2_flip, m->proc2_flip);
			glUniform1f(r->gb_u_proc2_noisesize, m->proc2_noisesize);
			glUniform1i(r->gb_u_proc2_noisedepth, m->proc2_noisedepth);
			glUniform1i(r->gb_u_proc2_basis, m->proc2_basis);
			glUniform1i(r->gb_u_proc2_wf, m->proc2_wf);
			glUniform1f(r->gb_u_proc2_dist, m->proc2_dist);
			glUniform1i(r->gb_u_proc2_hard, m->proc2_hard);
			glUniform1f(r->gb_u_proc2_bright, m->proc2_bright);
			glUniform1f(r->gb_u_proc2_contrast, m->proc2_contrast);
			glUniform1i(r->gb_u_proc2_noclamp, m->proc2_noclamp);
			glUniform1i(r->gb_u_proc2_has_ramp, m->proc2_has_ramp);
			glUniform1i(r->gb_u_proc2_orco, m->proc2_orco);   /* M27: 0=UV, 1=ORCO, 2=мир */
			glUniform2f(r->gb_u_proc2_size, m->proc2_size[0], m->proc2_size[1]);
			glUniform2f(r->gb_u_proc2_ofs, m->proc2_ofs[0], m->proc2_ofs[1]);
			glActiveTexture(GL_TEXTURE3);
			glBindTexture(GL_TEXTURE_2D, r->tex_ramp2);
			glUniform1i(r->gb_u_proc2_ramp, 3);
			if (m->proc2_has_ramp) {
				glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, 256, 1, GL_RGBA,
				                GL_UNSIGNED_BYTE, m->proc2_ramp);
			}
			if (getenv("UBF_GPU_DEBUG")) {
				static int proc2_draws = 0;
				if (proc2_draws < 8) {
					proc2_draws++;
					printf("[GPU-PROC2DRAW] ma='%s' type=%d stype=%d ramp=%d basis=%d depth=%d scale=%.3f orco=%d | spec fac=%.2f bt=%d def=%.2f | har fac=%.2f bt=%d def=%.2f\n",
					       m->ma ? m->ma->id.name + 2 : "<none>", m->proc2_type, m->proc2_stype,
					       m->proc2_has_ramp, m->proc2_basis, m->proc2_noisedepth,
					       m->proc2_noisesize, m->proc2_orco,
					       m->proc2_spec_fac, m->proc2_spec_blend, m->proc2_spec_def,
					       m->proc2_har_fac, m->proc2_har_blend, m->proc2_har_def);
				}
			}
			glActiveTexture(GL_TEXTURE4);   /* вернуть юнит диффузной карты */
		}
		/* ★ M23: bump (MAP_NORM) от процедурной карты — слот 0 или вторая. */
		glUniform1f(r->gb_u_has_proc_nrm, (m->bump_src != 0) ? 1.0f : 0.0f);
		glUniform1i(r->gb_u_proc_nrm_src, (m->bump_src == 2) ? 1 : 0);
		glUniform1f(r->gb_u_proc_norfac, m->bump_norfac);
		/* M14: дополнительные диффузные слоты 1..3. Слот k -> юнит 7+k. */
		glUniform1i(r->gb_u_n_tex, m->n_tex);
	glUniform1f(r->gb_u_tex_rgbtoint, (float)m->tex_rgbtoint);
	glUniform1f(r->gb_u_tex1_rgbtoint, (float)m->tex_rgbtoint2[0]);
	glUniform1f(r->gb_u_tex2_rgbtoint, (float)m->tex_rgbtoint2[1]);
	glUniform1f(r->gb_u_tex3_rgbtoint, (float)m->tex_rgbtoint2[2]);
		if (m->n_tex >= 2) {
			glActiveTexture(GL_TEXTURE7);
			glBindTexture(GL_TEXTURE_2D, m->tex2[0] ? (GLuint)m->tex2[0] : r->tex_white);
			glUniform1i(r->gb_u_tex1_blend, m->tex_blend2[0]);
			glUniform1f(r->gb_u_tex1_fac, m->colfac2[0]);
			glUniform1f(r->gb_u_tex1_orco, m->tex_orco2[0] ? 1.0f : 0.0f);
	glUniform1f(r->gb_u_tex1_glob, m->tex_glob2[0] ? 1.0f : 0.0f);
			glUniform1f(r->gb_u_tex1_map, (float)m->tex_map2[0]);
			glUniform2f(r->gb_u_tex1_size, m->tex_size2[0][0], m->tex_size2[0][1]);
			glUniform2f(r->gb_u_tex1_ofs, m->tex_ofs2[0][0], m->tex_ofs2[0][1]);
		}
		if (m->n_tex >= 3) {
			glActiveTexture(GL_TEXTURE8);
			glBindTexture(GL_TEXTURE_2D, m->tex2[1] ? (GLuint)m->tex2[1] : r->tex_white);
			glUniform1i(r->gb_u_tex2_blend, m->tex_blend2[1]);
			glUniform1f(r->gb_u_tex2_fac, m->colfac2[1]);
			glUniform1f(r->gb_u_tex2_orco, m->tex_orco2[1] ? 1.0f : 0.0f);
	glUniform1f(r->gb_u_tex2_glob, m->tex_glob2[1] ? 1.0f : 0.0f);
			glUniform1f(r->gb_u_tex2_map, (float)m->tex_map2[1]);
			glUniform2f(r->gb_u_tex2_size, m->tex_size2[1][0], m->tex_size2[1][1]);
			glUniform2f(r->gb_u_tex2_ofs, m->tex_ofs2[1][0], m->tex_ofs2[1][1]);
		}
		if (m->n_tex >= 4) {
			glActiveTexture(GL_TEXTURE9);
			glBindTexture(GL_TEXTURE_2D, m->tex2[2] ? (GLuint)m->tex2[2] : r->tex_white);
			glUniform1i(r->gb_u_tex3_blend, m->tex_blend2[2]);
			glUniform1f(r->gb_u_tex3_fac, m->colfac2[2]);
			glUniform1f(r->gb_u_tex3_orco, m->tex_orco2[2] ? 1.0f : 0.0f);
	glUniform1f(r->gb_u_tex3_glob, m->tex_glob2[2] ? 1.0f : 0.0f);
			glUniform1f(r->gb_u_tex3_map, (float)m->tex_map2[2]);
			glUniform2f(r->gb_u_tex3_size, m->tex_size2[2][0], m->tex_size2[2][1]);
			glUniform2f(r->gb_u_tex3_ofs, m->tex_ofs2[2][0], m->tex_ofs2[2][1]);
		}
		glActiveTexture(GL_TEXTURE4);
		/* M12: карты блика/жёсткости (переиспользуют сэмпл диффузной карты) */
		glUniform1f(r->gb_u_has_spec, (m->spec_has && !no_spec) ? 1.0f : 0.0f);
		glUniform1i(r->gb_u_spec_blend, m->spec_blend);
		glUniform1f(r->gb_u_spec_fac, m->spec_fac);
		glUniform1f(r->gb_u_spec_def, m->spec_def);
		glUniform1f(r->gb_u_has_har, m->har_has ? 1.0f : 0.0f);
		glUniform1i(r->gb_u_har_blend, m->har_blend);
		glUniform1f(r->gb_u_har_fac, m->har_fac);
		glUniform1f(r->gb_u_har_def, m->har_def);
		glUniform1f(r->gb_u_sv_use_a, (float)m->sv_use_a);
		/* M13: карта цвета блика (переиспользует сэмпл диффузной карты) */
		glUniform1f(r->gb_u_has_colspec, (m->colspec_has && !no_spec) ? 1.0f : 0.0f);
		glUniform1i(r->gb_u_colspec_blend, m->colspec_blend);
		glUniform1f(r->gb_u_colspec_fac, m->colspec_fac);
		/* M17: MAP_REFL (сила отражения, 5-я цель) */
		glUniform1f(r->gb_u_refl_base, m->refl_base);
		glUniform1f(r->gb_u_has_refl, m->refl_has ? 1.0f : 0.0f);
		glUniform1i(r->gb_u_refl_blend, m->refl_blend);
		glUniform1f(r->gb_u_refl_fac, m->refl_fac);
		glUniform1f(r->gb_u_refl_def, m->refl_def);
		/* M10: альфа-карта (попиксельная прозрачность). Если карта — та же
		 * картинка, что и цвет (обычный случай), переиспользуем m->tex. */
		gpu_upload_mesh_atex(m);
		{
			unsigned int at = 0;
			if (m->a_has && m->a_tex_tried)
				at = (m->aima == m->ima) ? m->tex : m->atex;
			glActiveTexture(GL_TEXTURE6);
			glBindTexture(GL_TEXTURE_2D, at ? (GLuint)at : r->tex_white);
			glUniform1f(r->gb_u_has_atex, at ? 1.0f : 0.0f);
			glUniform1f(r->gb_u_a_fac, m->a_fac);
			glUniform1f(r->gb_u_a_def, m->a_def);
			glUniform1i(r->gb_u_a_blend, m->a_blend);
			glUniform1f(r->gb_u_a_use_a, (float)m->a_use_a);
			glUniform1f(r->gb_u_a_orco, m->a_orco ? 1.0f : 0.0f);
	glUniform1f(r->gb_u_a_glob, m->a_glob ? 1.0f : 0.0f);
			glUniform1f(r->gb_u_a_map, (float)m->a_map);
			glUniform2f(r->gb_u_a_size, m->a_size[0], m->a_size[1]);
			glUniform2f(r->gb_u_a_ofs, m->a_ofs[0], m->a_ofs[1]);
			glActiveTexture(GL_TEXTURE4);
		}
		/* M8: нормальная карта (белая заглушка, если нет) — юнит 5.
		 * ★ M30: юнит 5 делаем активным ДО загрузки — gpu_upload_mesh_ntex
		 * создаёт/биндит текстуру на ТЕКУЩЕМ юните и в конце отвязывает его
		 * (glBindTexture(0)). Раньше активным был юнит 4 (диффузная карта),
		 * поэтому у материалов с загружаемой картой нормалей диффузная текстура
		 * терялась -> альбедо чёрное -> вся сцена темнела (ratio 1.04 -> 0.57). */
		glActiveTexture(GL_TEXTURE5);
		gpu_upload_mesh_ntex(m);
		glBindTexture(GL_TEXTURE_2D, m->ntex ? (GLuint)m->ntex : r->tex_white);
		glUniform1f(r->gb_u_has_nrm, m->n_has ? 1.0f : 0.0f);
	glUniform1f(r->gb_u_nrm_is_height, m->n_has ? (float)m->n_is_height : 0.0f);
	glUniform1f(r->gb_u_nrm_orco, m->n_has ? (float)m->n_orco : 0.0f);
	glUniform1f(r->gb_u_nrm_glob, m->n_has ? (float)m->n_glob : 0.0f);
	glUniform1f(r->gb_u_nrm_map, m->n_has ? (float)m->n_map : 0.0f);
	glUniform2fv(r->gb_u_nrm_size, 1, m->n_size);
	glUniform2fv(r->gb_u_nrm_ofs, 1, m->n_ofs);
	glUniform1f(r->gb_u_nrm_norfac, m->n_has ? m->n_norfac : 0.0f);
	glUniform1f(r->gb_u_bump_scale, bump_scale);   /* ★ раунд 17: общий для всех ламп/карт множитель bump */
		glActiveTexture(GL_TEXTURE4);
		/* screen-door прозрачность (UBF_GPU_ALPHA=0 выключает) */
		glUniform1f(r->gb_u_alpha, alpha_on ? m->alpha : 1.0f);

		/* ★ M5: блик материала в 4-ю цель. hardness едет нормированной в
		 * [0,1] (в шейдер попадает 8 бит), обратно умножается на 511. */
		if (r->spec_target) {
			glUniform3fv(r->gb_u_spec_col, 1, (const GLfloat *)m->spec_col);
			glUniform1f(r->gb_u_hard, m->hard / 511.0f);
		}

		glDrawElements(GL_TRIANGLES, m->num_indices, GL_UNSIGNED_INT, NULL);

		/* ★ M16 (раунд 67): проволока (материал WIRE). Рисуем ЛИНИЯМИ в тот же
		 * G-буфер — поэтому проволока получает обычное деферред-освещение, тени
		 * и пр. без отдельного шейдера. Ширина 1 px: BI тоже рисует проволоку в
		 * один пиксель (convertblender.c создаёт вырожденные VlakRen, и
		 * сканирующий растеризатор чертит по ним линию шириной 1). */
		if (m->num_line_indices > 0 && m->lbo) {
			glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m->lbo);
			glDrawElements(GL_LINES, m->num_line_indices, GL_UNSIGNED_INT, NULL);
			glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m->ibo);
		}
	}

	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

	/* ★ диагностика: читаем COLOR2 (упакованная view-глубина, a = маска геометрии).
	 * Если geom_px=0 — треугольники не растеризовались (матрицы/VBO/FBO),
	 * и «тёмная картинка» это пустой G-буфер, а не освещение. */
	{
		static int dbg_depth = 0;
		if (dbg_depth < 3 && r->width > 0 && r->height > 0) {
			static unsigned char cbuf[128 * 128 * 4];
			int bw = 128, bh = 128;
			int bx = (r->width - bw) / 2, by = (r->height - bh) / 2;
			if (bx < 0) { bx = 0; bw = r->width; }
			if (by < 0) { by = 0; bh = r->height; }
			if (bw > 128) bw = 128;
			if (bh > 128) bh = 128;
			dbg_depth++;
			glReadBuffer(GL_COLOR_ATTACHMENT2_EXT);
			glReadPixels(bx, by, bw, bh, GL_RGBA, GL_UNSIGNED_BYTE, cbuf);
			int geom = 0, n = bw * bh;
			float dmin = 1e9f, dmax = -1e9f;
			for (int i = 0; i < n; i++) {
				if (cbuf[i * 4 + 3] > 0) {          /* маска геометрии */
					float d = (cbuf[i * 4 + 0] / 255.0f)
					        + (cbuf[i * 4 + 1] / 255.0f) / 255.0f
					        + (cbuf[i * 4 + 2] / 255.0f) / 65025.0f;
					if (d < dmin) dmin = d;
					if (d > dmax) dmax = d;
					geom++;
				}
			}
			if (geom == 0) { dmin = -1.0f; dmax = -1.0f; }
			printf("[GPU-DEPTH] center %dx%d at (%d,%d): linz_norm=[%.6f..%.6f] "
			       "(world=[%.3f..%.3f]) geom_px=%d/%d\n",
			       bw, bh, bx, by, dmin, dmax,
			       (dmin < 0) ? -1.0f : dmin / pack_scale,
			       (dmax < 0) ? -1.0f : dmax / pack_scale, geom, n);

			/* состояние depth-теста, тип depth-аттачмента и центр экрана */
			{
				GLboolean dt = glIsEnabled(GL_DEPTH_TEST);
				GLint dw = -1, df = -1, attached = -1, objname = -1;
				glGetIntegerv(GL_DEPTH_WRITEMASK, &dw);
				glGetIntegerv(GL_DEPTH_FUNC, &df);
				glGetFramebufferAttachmentParameterivEXT(
					GL_FRAMEBUFFER_EXT, GL_DEPTH_ATTACHMENT_EXT,
					GL_FRAMEBUFFER_ATTACHMENT_OBJECT_TYPE_EXT, &attached);
				if (attached != GL_NONE) {
					glGetFramebufferAttachmentParameterivEXT(
						GL_FRAMEBUFFER_EXT, GL_DEPTH_ATTACHMENT_EXT,
						GL_FRAMEBUFFER_ATTACHMENT_OBJECT_NAME_EXT, &objname);
				}
				unsigned char pix[4] = { 0, 0, 0, 0 };
				int px = bx + bw / 2, py = by + bh / 2;
				glReadBuffer(GL_COLOR_ATTACHMENT0_EXT);
				glReadPixels(px, py, 1, 1, GL_RGBA, GL_UNSIGNED_BYTE, pix);
				printf("[GPU-DEPTHST] test=%d mask=%d func=%d depth_attach=%d(5890=TEX,36161=RB) "
				       "name=%d rb_depth=%u | center(%d,%d) albedo=(%d %d %d %d) glerr=%d\n",
				       (int)dt, dw, df, attached, objname, r->rb_depth,
				       px, py, pix[0], pix[1], pix[2], pix[3], (int)glGetError());
			}
		}
	}

	/* ★ ОТЛАДКА: слить весь G-буфер в PPM (UBF_GPU_DUMP=префикс). Нужно, чтобы
	 * ГЛАЗАМИ увидеть, где в G-буфере оказалась геометрия, и сравнить это с
	 * позицией, которую даёт матрица (см. [GPU-ORIGIN]). */
	{
		const char *dump = getenv("UBF_GPU_DUMP");
		static int dump_done = 0;
		if (dump && !dump_done && r->width > 0 && r->height > 0) {
			unsigned char *buf = MEM_mallocN((size_t)r->width * r->height * 4, "gpu gb dump");
			int att;
			dump_done = 1;
			for (att = 0; att < 3; att++) {
				char path[512];
				FILE *fp;
				int y;
				glReadBuffer(GL_COLOR_ATTACHMENT0_EXT + att);
				glReadPixels(0, 0, r->width, r->height, GL_RGBA, GL_UNSIGNED_BYTE, buf);
				sprintf(path, "%s_att%d.ppm", dump, att);
				fp = fopen(path, "wb");
				if (fp) {
					fprintf(fp, "P6\n%d %d\n255\n", r->width, r->height);
					for (y = r->height - 1; y >= 0; y--)
						fwrite(buf + (size_t)y * r->width * 4, 1, (size_t)r->width * 3, fp);
					fclose(fp);
				}
			}
			glReadBuffer(GL_COLOR_ATTACHMENT0_EXT);
			MEM_freeN(buf);
			printf("[GPU-DUMP] G-буфер слит: %s_att0..2.ppm (%dx%d)\n",
			       dump, r->width, r->height);
		}
	}

	/* --- проход 2: деферред-освещение (+ тень), вывод во вьюпорт --- */
	glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, (GLuint)saved_fb);
	/* ★ ВАЖНО: это единственный проход, который пишет в ЧУЖОЙ framebuffer —
	 * в тот, что был привязан до нас (обычно 0, окно). Регион вьюпорта
	 * устанавливает свой viewport через wmSubWindowScissorSet()
	 * (`glViewport(winrct.xmin, winrct.ymin, ...)`, wm_subwindow.c:297).
	 * Если здесь жёстко поставить (0,0), изображение уезжает в левый-нижний
	 * угол ОКНА на величину начала региона: сцена смещена и не заполняет
	 * вьюпорт. Промежуточные проходы (G-буфер, карта теней) рисуют в СВОИ
	 * FBO размера r->width x r->height, там (0,0) как раз верно. */
	glViewport(saved_vp[0], saved_vp[1], r->width, r->height);
	/* ★ финальный проход пишет во вьюпорт — возвращаем скассор как был, чтобы
	 * результат не вылез за пределы региона и состояние GL осталось прежним. */
	if (scissor_was_on) {
		glScissor(saved_scissor[0], saved_scissor[1], saved_scissor[2], saved_scissor[3]);
		glEnable(GL_SCISSOR_TEST);
	}
	{
		/* ★ печатаем не «первые N кадров», а при каждом ИЗМЕНЕНИИ параметров:
		 * так в логе видно, что происходит при панораме/ресайзе. */
		static int last_vp[4] = { -1, -1, -1, -1 };
		static int last_wh[2] = { -1, -1 };
		const char *lg = getenv("UBF_GPU_LOG");
		if (last_vp[0] != saved_vp[0] || last_vp[1] != saved_vp[1] ||
		    last_wh[0] != r->width || last_wh[1] != r->height)
		{
			FILE *fp = NULL;
			last_vp[0] = saved_vp[0]; last_vp[1] = saved_vp[1];
			last_vp[2] = saved_vp[2]; last_vp[3] = saved_vp[3];
			last_wh[0] = r->width; last_wh[1] = r->height;
			if (lg) fp = fopen(lg, "a");
			printf("[GPU-FINAL] регион viewport=(%d,%d %dx%d) буфер вывода=%dx%d "
			       "scissor=%d box=(%d,%d %dx%d)\n"
			       "            winmat[0]=(%.4f %.4f %.4f %.4f) winmat[1]=(%.4f %.4f %.4f %.4f)\n"
			       "            viewmat[0]=(%.4f %.4f %.4f %.4f) viewmat[1]=(%.4f %.4f %.4f %.4f)\n"
			       "            clip=%.3f..%.3f rectx=%d recty=%d\n",
			       saved_vp[0], saved_vp[1], saved_vp[2], saved_vp[3],
			       r->width, r->height,
			       (int)scissor_was_on,
			       saved_scissor[0], saved_scissor[1], saved_scissor[2], saved_scissor[3],
			       re->winmat[0][0], re->winmat[0][1], re->winmat[0][2], re->winmat[0][3],
			       re->winmat[1][0], re->winmat[1][1], re->winmat[1][2], re->winmat[1][3],
			       re->viewmat[0][0], re->viewmat[0][1], re->viewmat[0][2], re->viewmat[0][3],
			       re->viewmat[1][0], re->viewmat[1][1], re->viewmat[1][2], re->viewmat[1][3],
			       re->clipsta, re->clipend, re->rectx, re->recty);
			if (fp) {
				fprintf(fp, "[GPU-FINAL] vp=(%d,%d %dx%d) buf=%dx%d scissor=%d box=(%d,%d %dx%d) "
				            "winmat0=(%.4f %.4f %.4f %.4f) winmat1=(%.4f %.4f %.4f %.4f) "
				            "viewmat0=(%.4f %.4f %.4f %.4f) viewmat1=(%.4f %.4f %.4f %.4f) "
				            "clip=%.3f..%.3f rectx=%d recty=%d\n",
				        saved_vp[0], saved_vp[1], saved_vp[2], saved_vp[3],
				        r->width, r->height, (int)scissor_was_on,
				        saved_scissor[0], saved_scissor[1], saved_scissor[2], saved_scissor[3],
				        re->winmat[0][0], re->winmat[0][1], re->winmat[0][2], re->winmat[0][3],
				        re->winmat[1][0], re->winmat[1][1], re->winmat[1][2], re->winmat[1][3],
				        re->viewmat[0][0], re->viewmat[0][1], re->viewmat[0][2], re->viewmat[0][3],
				        re->viewmat[1][0], re->viewmat[1][1], re->viewmat[1][2], re->viewmat[1][3],
				        re->clipsta, re->clipend, re->rectx, re->recty);
				fclose(fp);
			}
		}
	}
	glDisable(GL_DEPTH_TEST);

	float inv_proj[4][4];
	invert_m4_m4(inv_proj, re->winmat);

	glUseProgram(r->prog_light);
	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, r->tex_albedo);
	glUniform1i(r->light_u_albedo, 0);
	glActiveTexture(GL_TEXTURE1);
	glBindTexture(GL_TEXTURE_2D, r->tex_nrm);
	glUniform1i(r->light_u_nrm, 1);
	glActiveTexture(GL_TEXTURE2);
	glBindTexture(GL_TEXTURE_2D, r->tex_vpos);
	glUniform1i(r->light_u_depth, 2);
	/* ★ M5: блик материала — юнит 5 (0..3 заняты, 4 — карта материала). Если
	 * 4-й цели нет, читаем заглушку с дефолтами BI. Теневые карты привязываются
	 * ниже в цикле ламп на юниты 6..9. */
	glActiveTexture(GL_TEXTURE5);
	glBindTexture(GL_TEXTURE_2D, r->spec_target ? r->tex_spec : r->tex_spec_def);
	glUniform1i(r->light_u_spec, 5);
	/* ★ M17: сила отражения — юнит 4. Если 5-й цели нет, читаем заглушку 0.35. */
	glActiveTexture(GL_TEXTURE4);
	glBindTexture(GL_TEXTURE_2D, r->refl_target ? r->tex_refl : r->tex_refl_def);
	glUniform1i(r->light_u_refl, 4);

	/* ★★ Раунд 16: данные ВСЕХ ламп сцены — одной float-текстурой u_light_data
	 * (RGBA32F, ширина = число ламп, высота GPU_LIGHT_DATA_ROWS=5). Раньше это
	 * были четыре per-light uniform-массива (u_lpos/u_ldir/u_lcol/u_lfl), а они
	 * упирались в лимит uniform-компонентов — отсюда и брался потолок «10 ламп».
	 * Строки: 0 pos/type, 1 dir/dist, 2 col/falloff,
	 *         3 sphere/spotsi/spotbl/shadows, 4 soft.
	 * Юнит 3 (0,1,2,4,5 заняты). */
	{
		int n = gr->num_lights;   /* ★ искусственного потолка больше нет */
		glUniform1i(r->light_u_num_lights, n);

		{
			int w = (n > 0) ? n : 1;
			if (r->tex_lights == 0) glGenTextures(1, &r->tex_lights);
			glActiveTexture(GL_TEXTURE3);
			glBindTexture(GL_TEXTURE_2D, r->tex_lights);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
			if (r->light_buf_cap < w * GPU_LIGHT_DATA_ROWS * 4) {
				if (r->light_buf) MEM_freeN(r->light_buf);
				r->light_buf_cap = w * GPU_LIGHT_DATA_ROWS * 4;
				r->light_buf = MEM_callocN(sizeof(float) * r->light_buf_cap,
				                           "gpu light tex");
			}
			float *b = r->light_buf;
			memset(b, 0, sizeof(float) * w * GPU_LIGHT_DATA_ROWS * 4);
			for (int i = 0; i < n; i++) {
				const RE_GPULight *L = &gr->lights[i];
				float lv_pos[3], lv_dir[3];
				float *r0 = b + (0 * w + i) * 4;
				float *r1 = b + (1 * w + i) * 4;
				float *r2 = b + (2 * w + i) * 4;
				float *r3 = b + (3 * w + i) * 4;
				float *r4 = b + (4 * w + i) * 4;
				gpu_light_view(L, gr->re->viewmat, lv_pos, lv_dir);
				/* солнце: в pos кладём НАПРАВЛЕНИЕ (как делал прежний u_lpos) */
				r0[0] = (L->type == GPU_LIGHT_SUN) ? lv_dir[0] : lv_pos[0];
				r0[1] = (L->type == GPU_LIGHT_SUN) ? lv_dir[1] : lv_pos[1];
				r0[2] = (L->type == GPU_LIGHT_SUN) ? lv_dir[2] : lv_pos[2];
				r0[3] = (float)L->type;
				r1[0] = lv_dir[0]; r1[1] = lv_dir[1]; r1[2] = lv_dir[2];
				r1[3] = L->dist;
				r2[0] = L->color[0]; r2[1] = L->color[1]; r2[2] = L->color[2];
				r2[3] = (float)L->falloff;
				r3[0] = (float)L->sphere; r3[1] = L->spotsi; r3[2] = L->spotbl;
				/* w = даёт ли лампа тень. UBF_GPU_NOSHADOW=1 — диагностика:
				 * полностью выключить тени ламп (проверка, не «затеняет» ли
				 * теневая карта всю сцену). */
				r3[3] = (L->shadows && !getenv("UBF_GPU_NOSHADOW")) ? 1.0f : 0.0f;
				r4[0] = L->soft;
			}
			glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
			if (r->tex_lights_w != w) {
				glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA32F, w, GPU_LIGHT_DATA_ROWS, 0,
				             GL_RGBA, GL_FLOAT, b);
				r->tex_lights_w = w;
			}
			else {
				glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, w, GPU_LIGHT_DATA_ROWS,
				                GL_RGBA, GL_FLOAT, b);
			}
			glUniform1i(r->light_u_lightdata, 3);
			glUniform2f(r->light_u_lighttexel, 1.0f / (float)w,
			            1.0f / (float)GPU_LIGHT_DATA_ROWS);
			/* ★ Диагностика (UBF_GPU_LIGHTDBG): мировая позиция обязана быть
			 * НЕИЗМЕННОЙ, свежая камерная — меняться вместе с видом. */
			if (getenv("UBF_GPU_LIGHTDBG") && n > 0) {
				static float dbg_vm0 = 999.0f;
				if (re->viewmat[0][0] != dbg_vm0) {
					float dp[3], dd[3];
					dbg_vm0 = re->viewmat[0][0];
					gpu_light_view(&gr->lights[0], re->viewmat, dp, dd);
					printf("[GPU-LIGHTVIEW] L0 world=(%.2f %.2f %.2f) view_now=(%.2f %.2f %.2f) "
					       "stale=(%.2f %.2f %.2f) viewmat0=(%.3f %.3f %.3f)\n",
					       gr->lights[0].wpos[0], gr->lights[0].wpos[1], gr->lights[0].wpos[2],
					       dp[0], dp[1], dp[2],
					       gr->lights[0].pos[0], gr->lights[0].pos[1], gr->lights[0].pos[2],
					       re->viewmat[0][0], re->viewmat[0][1], re->viewmat[0][2]);
				}
			}
		}

		/* ★ Теневая карта лампы: юнит 6+i для 2D (SUN/SPOT) и
		 * 6+GPU_MAX_SHADOW_LIGHTS+i для кубической (POINT/AREA); у не-теневых ламп
		 * заглушка (шейдер её не сэмплит: r3.w=0). ★ Диапазоны НЕ должны
		 * пересекаться: один юнит не может держать sampler2D и samplerCube — это
		 * GL_INVALID_OPERATION на draw, и драйвер отменяет весь проход (кадр
		 * чёрный). Тень дают только первые GPU_MAX_SHADOW_LIGHTS ламп; остальные
		 * (индекс >= него) СВЕТЯТ, но без тени. */
		int ns = n;
		if (ns > GPU_MAX_SHADOW_LIGHTS) ns = GPU_MAX_SHADOW_LIGHTS;
		for (int i = 0; i < ns; i++) {
			const RE_GPULight *L = &gr->lights[i];
			const int is_cube = (L->type == GPU_LIGHT_POINT || L->type == GPU_LIGHT_AREA);
			glActiveTexture(GL_TEXTURE6 + i);
			glBindTexture(GL_TEXTURE_2D,
			              (!is_cube && L->shadows) ? r->tex_shadow[i] : r->tex_white);
			glActiveTexture(GL_TEXTURE6 + GPU_MAX_SHADOW_LIGHTS + i);
			glBindTexture(GL_TEXTURE_CUBE_MAP,
			              (is_cube && L->shadows) ? r->tex_shadow_cube[i] : r->tex_cube_white);
			glUniform1i(r->light_u_shadow[i], 6 + i);
			glUniform1i(r->light_u_shadow_cube[i], 6 + GPU_MAX_SHADOW_LIGHTS + i);
			/* point/area: мировая позиция света (центр куба) и (A,B) пересчёта глубины */
			glUniform3fv(r->light_u_lwpos[i], 1, (const GLfloat *)L->wpos);
			glUniform2fv(r->light_u_lpz[i], 1, (const GLfloat *)L->pz);
			if (L->shadows && !is_cube) {
				glUniformMatrix4fv(r->light_u_lvp[i], 1, GL_FALSE,
				                   (const GLfloat *)L->vp);
			}
		}
		/* вернуть активный юнит: цикл оставил его на 6+GPU_MAX_SHADOW_LIGHTS+i */
		glActiveTexture(GL_TEXTURE0);
		/* UBF_GPU_EXPOSURE — ручная яркость: BI-верное затухание делает сцены с
		 * маленьким energy заметно темнее, это аварийная ручка. */
		const char *expo_e = getenv("UBF_GPU_EXPOSURE");
		glUniform1f(r->light_u_exposure, expo_e ? (float)atof(expo_e) : 1.0f);
	}

	/* ★ Мир сцены: фон и ambient. */
	glUniform3fv(r->light_u_world_hor, 1, (const GLfloat *)gr->world_hor);
	glUniform3fv(r->light_u_world_zen, 1, (const GLfloat *)gr->world_zen);
	glUniform3fv(r->light_u_world_amb, 1, (const GLfloat *)gr->world_amb);
	glUniform1f(r->light_u_world_blend, (float)gr->world_blend);
	{
		/* ★★ Раунд 82: встроенный полусферический fill по умолчанию ВЫКЛЮЧЕН (0).
		 * Он появился как компенсация «слишком тёмной» картинки — но настоящая
		 * причина была в отсутствии дисплейного преобразования на выходе (см.
		 * gpu_to_srgb) и в непереданном shi->refl. Замер на контрольной сцене
		 * (плоскость albedo=0.5, sun energy=1, мир чёрный): с fill=0 GPU даёт
		 * линейные 0.4969 против 0.5029 у scanline (1.2%), то есть модель света
		 * СОВПАДАЕТ; с fill=1 было 0.6491 (лишние +30%). BI же ambient берёт как
		 * ma->amb * wrld.ambient_color (shadeoutput.c:2000) — у mapFull
		 * ma->amb=1.0, а ambient_color=(0,0,0), т.е. ровно ноль.
		 * UBF_GPU_AMB_FILL=<x> по-прежнему включает fill вручную. */
		const char *fill_e = getenv("UBF_GPU_AMB_FILL");
		glUniform1f(r->light_u_amb_fill, fill_e ? (float)atof(fill_e) : 0.0f);
	}
	/* ★ M26: horizon/zenith становятся ambient-светом ТОЛЬКО при Environment
	 * Lighting (BI shadeoutput.c:1127). Иначе ambient = ma->amb * ambient_color. */
	glUniform1f(r->light_u_env_light, (float)gr->world_env);
	glUniform1f(r->light_u_env_energy, gr->world_env_energy);
	glUniform1f(r->light_u_linfac, gr->linfac);
	glUniform1f(r->light_u_logfac, gr->logfac);
	glUniform1f(r->light_u_expo_on, (float)gr->expo_on);
	glUniformMatrix4fv(r->light_u_invproj, 1, GL_FALSE, (const GLfloat *)inv_proj);
	glUniformMatrix4fv(r->light_u_invview, 1, GL_FALSE, (const GLfloat *)re->viewinv);
	glUniform1f(r->light_u_pack, pack_scale);
	/* ★ раунд 83-fix: у перспективной winmat[2][3] == -1, у ортографической == 0.
	 * Световой проход обязан знать это, иначе восстанавливает view-позицию как
	 * перспективный луч и орто-сцены освещаются только в пятне у центра. */
	glUniform1f(r->light_u_ortho, (re->winmat[2][3] == 0.0f) ? 1.0f : 0.0f);
	glUniformMatrix4fv(r->light_u_proj, 1, GL_FALSE, (const GLfloat *)re->winmat);
	/* SSR по умолчанию включён; UBF_GPU_SSR=0 выключает */
	{
		const char *ssr_e = getenv("UBF_GPU_SSR");
		glUniform1f(r->light_u_ssr, ssr_e ? (float)(atoi(ssr_e) != 0) : 1.0f);
	}
	/* SSAO по умолчанию включён; UBF_GPU_AO=0 выключает */
	{
		const char *ao_e = getenv("UBF_GPU_AO");
		glUniform1f(r->light_u_ao, ao_e ? (float)(atoi(ao_e) != 0) : 1.0f);
		glUniform2f(r->light_u_texel, 1.0f / (float)r->width, 1.0f / (float)r->height);
	}
	/* PCF по умолчанию включён; UBF_GPU_PCF=0 возвращает одиночную выборку */
	{
		const char *pcf_e = getenv("UBF_GPU_PCF");
		glUniform1f(r->light_u_pcf, pcf_e ? (float)(atoi(pcf_e) != 0) : 1.0f);
		glUniform2f(r->light_u_shadow_texel,
		            1.0f / (float)GPU_SHADOW_SIZE, 1.0f / (float)GPU_SHADOW_SIZE);
	}
	/* ★ Раунд 83: ручка мирового bias теней (UBF_GPU_SHADOW_BIAS, по умолчанию 1.0).
	 * Нужна для подбора: слишком мал — acne по всей поверхности, слишком велик —
	 * свет проходит сквозь стены (именно это и было причиной утечки в mapFull). */
	{
		const char *sb_e = getenv("UBF_GPU_SHADOW_BIAS");
		glUniform1f(r->light_u_shadow_bias, sb_e ? (float)atof(sb_e) : 1.0f);
	}

	/* мировой up в view-space: для полусферического ambient (небо/земля) */
	{
		float up[3] = { 0.0f, 0.0f, 1.0f };
		mul_mat3_m4_v3(re->viewmat, up);
		glUniform3fv(r->light_u_up, 1, (const GLfloat *)up);
	}

	/* диагностика G-буфера: UBF_GPU_DEBUG_GB=1 albedo, 2 нормаль, 3 глубина */
	{
		const char *dbg_e = getenv("UBF_GPU_DEBUG_GB");
		glUniform1i(r->light_u_debug, dbg_e ? atoi(dbg_e) : 0);
	}

	glBindBuffer(GL_ARRAY_BUFFER, r->quad_vbo);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, r->quad_ibo);
	const int qstride = 4 * sizeof(float);
	glEnableVertexAttribArray(r->light_a_pos);
	glVertexAttribPointer(r->light_a_pos, 2, GL_FLOAT, GL_FALSE, qstride, (void *)0);
	glEnableVertexAttribArray(r->light_a_uv);
	glVertexAttribPointer(r->light_a_uv, 2, GL_FLOAT, GL_FALSE, qstride,
	                      (void *)(2 * sizeof(float)));
	glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, NULL);
	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

	/* --- проход 3 (M15): forward-освещение полупрозрачных мешей. Рисуем ПОСЛЕ
	 * деферред-освещения, в тот же вьюпорт-FBO, с alpha-блендингом. Затенение
	 * непрозрачной сценой — в самом шейдере (сравнение с vpos-глубиной), поэтому
	 * GL depth-test не нужен. Полупрозрачные меши пропущены в проходах теней и
	 * G-буфера (см. fwd_on / m->transparent). */
	if (fwd_on) {
		glUseProgram(r->prog_forward);
		glEnable(GL_BLEND);
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
		glDepthMask(GL_FALSE);
		/* проекция (для gl_Position в VS и шагов SSR в FS) — не меняется по мешам */
		glUniformMatrix4fv(r->fw_u_proj, 1, GL_FALSE, (const GLfloat *)re->winmat);

		/* глубина непрозрачной сцены (для затенения) и разрешение экрана */
		glActiveTexture(GL_TEXTURE0);
		glBindTexture(GL_TEXTURE_2D, r->tex_vpos);
		glUniform1i(r->fw_u_depth, 0);
		glUniform2f(r->fw_u_resolution, (float)r->width, (float)r->height);
		glUniform1f(r->fw_u_pack, pack_scale);
		glUniform1f(r->fw_u_ortho, (re->winmat[2][3] == 0.0f) ? 1.0f : 0.0f);
		/* SSAO/SSR: читаем albedo и нормаль непрозрачной сцены из G-буфера. */
		glActiveTexture(GL_TEXTURE1);
		glBindTexture(GL_TEXTURE_2D, r->tex_albedo);
		glUniform1i(r->fw_u_albedo, 1);
		glUniformMatrix4fv(r->fw_u_inv_proj, 1, GL_FALSE, (const GLfloat *)inv_proj);
		glUniform2f(r->fw_u_texel, 1.0f / (float)r->width, 1.0f / (float)r->height);
		{
			const char *ssr_e = getenv("UBF_GPU_SSR");
			glUniform1f(r->fw_u_ssr, ssr_e ? (float)(atoi(ssr_e) != 0) : 1.0f);
			const char *ao_e = getenv("UBF_GPU_AO");
			glUniform1f(r->fw_u_ao, ao_e ? (float)(atoi(ao_e) != 0) : 1.0f);
		}

		/* ★ Лампы (те же значения, что и в проходе освещения), теневые карты на
		 * юнитах 10..13 (материальные карты занимают 4,5,6). */
		{
			int n = gr->num_lights;   /* ★ раунд 16: без потолка */
			glUniform1i(r->fw_u_num_lights, n);
			/* ★ те же данные ламп float-текстурой. Юнит 9 — единственный
			 * свободный в forward: 0=глубина, 1=albedo, 2/3/7/8=таблицы шума,
			 * 4=диффуз, 5=нормали, 6=альфа, 10..19=2D-тени и 20..29=куб-тени.
			 * ★ НЕ юнит 2: его занимает u_proc_hash (процедурные карты), и
			 * данные ламп были бы перезаписаны на каждом меше с процедуркой. */
			glActiveTexture(GL_TEXTURE9);
			glBindTexture(GL_TEXTURE_2D, r->tex_lights);
			glUniform1i(r->fw_u_lightdata, 9);
			glUniform2f(r->fw_u_lighttexel,
			            1.0f / (float)(r->tex_lights_w > 0 ? r->tex_lights_w : 1),
			            1.0f / (float)GPU_LIGHT_DATA_ROWS);
			/* Тень — только первые GPU_MAX_SHADOW_LIGHTS ламп (пара сэмплеров на
			 * лампу); остальные светят без тени, см. LIGHT_FS. */
			int fw_ns = n;
			if (fw_ns > GPU_MAX_SHADOW_LIGHTS) fw_ns = GPU_MAX_SHADOW_LIGHTS;
			for (int i = 0; i < fw_ns; i++) {
				const RE_GPULight *L = &gr->lights[i];
				/* ★ 2D-карта — юнит 10+i, кубическая — 10+GPU_MAX_SHADOW_LIGHTS+i:
				 * разного типа сэмплеры не должны делить один юнит (иначе
				 * GL_INVALID_OPERATION и отмена прохода — см. проход освещения). */
				const int is_cube = (L->type == GPU_LIGHT_POINT || L->type == GPU_LIGHT_AREA);
				glActiveTexture(GL_TEXTURE10 + i);
				glBindTexture(GL_TEXTURE_2D,
				              (!is_cube && L->shadows) ? r->tex_shadow[i] : r->tex_white);
				glActiveTexture(GL_TEXTURE10 + GPU_MAX_SHADOW_LIGHTS + i);
				glBindTexture(GL_TEXTURE_CUBE_MAP,
				              (is_cube && L->shadows) ? r->tex_shadow_cube[i] : r->tex_cube_white);
				glUniform1i(r->fw_u_shadow[i], 10 + i);
				glUniform1i(r->fw_u_shadow_cube[i], 10 + GPU_MAX_SHADOW_LIGHTS + i);
				glUniform3fv(r->fw_u_lwpos[i], 1, (const GLfloat *)L->wpos);
				glUniform2fv(r->fw_u_lpz[i], 1, (const GLfloat *)L->pz);
				if (L->shadows && !is_cube) {
					glUniformMatrix4fv(r->fw_u_lvp[i], 1, GL_FALSE,
					                   (const GLfloat *)L->vp);
				}
			}
		}
		/* вернуть активный юнит: цикл оставил его на 10+GPU_MAX_SHADOW_LIGHTS+i */
		glActiveTexture(GL_TEXTURE0);
		{
			const char *expo_e = getenv("UBF_GPU_EXPOSURE");
			glUniform1f(r->fw_u_exposure, expo_e ? (float)atof(expo_e) : 1.0f);
			const char *pcf_e = getenv("UBF_GPU_PCF");
			glUniform1f(r->fw_u_pcf, pcf_e ? (float)(atoi(pcf_e) != 0) : 1.0f);
			glUniform2f(r->fw_u_shadow_texel,
			            1.0f / (float)GPU_SHADOW_SIZE, 1.0f / (float)GPU_SHADOW_SIZE);
			/* ★ Раунд 83: тот же множитель мирового bias, что и в проходе освещения. */
			const char *sb_e = getenv("UBF_GPU_SHADOW_BIAS");
			glUniform1f(r->fw_u_shadow_bias, sb_e ? (float)atof(sb_e) : 1.0f);
		}

		/* Мир сцены: фон и ambient. */
		glUniform3fv(r->fw_u_world_hor, 1, (const GLfloat *)gr->world_hor);
		glUniform3fv(r->fw_u_world_zen, 1, (const GLfloat *)gr->world_zen);
		glUniform3fv(r->fw_u_world_amb, 1, (const GLfloat *)gr->world_amb);
		glUniform1f(r->fw_u_world_blend, (float)gr->world_blend);
		{
			/* ★★ Раунд 82: как и в проходе освещения — fill по умолчанию выключен
			 * (см. подробный комментарий у light_u_amb_fill). */
			const char *fill_e = getenv("UBF_GPU_AMB_FILL");
			glUniform1f(r->fw_u_amb_fill, fill_e ? (float)atof(fill_e) : 0.0f);
		}
		/* ★ M26: то же, что и в проходе освещения. */
		glUniform1f(r->fw_u_env_light, (float)gr->world_env);
		glUniform1f(r->fw_u_env_energy, gr->world_env_energy);
		{
			float up[3] = { 0.0f, 0.0f, 1.0f };
			mul_mat3_m4_v3(re->viewmat, up);
			glUniform3fv(r->fw_u_up, 1, (const GLfloat *)up);
		}
		glUniform1f(r->fw_u_expo_on, 1.0f);
		glUniform1f(r->fw_u_linfac, 1.0f);
		glUniform1f(r->fw_u_logfac, -1.0f);

		/* ★ Рисуем полупрозрачные меши. Отсечение по фрустуму — как в G-буфере. */
		for (RE_GPUMesh *m = gr->scene->meshes; m; m = m->next) {
			/* ★ M16: как и в G-буфере, прозрачной может быть и ПРОВОЛОКА
			 * (num_indices == 0, рёбра в num_line_indices). */
			if (!m->transparent || !m->uploaded ||
			    (m->num_indices == 0 && m->num_line_indices == 0)) continue;
			if (cull_on && gpu_aabb_outside(cull_pl, m->bb_min, m->bb_max)) continue;

			float model[4][4], nm3[3][3], nm3i[3][3], nrm[3][3];
			mul_m4_m4m4(model, m->obmat, re->viewmat);   /* r59: тот же порядок, что и в G-буфере */
			glUniformMatrix4fv(r->fw_u_model, 1, GL_FALSE, (const GLfloat *)model);
			copy_m3_m4(nm3, model);
			invert_m3_m3(nm3i, nm3);
			transpose_m3_m3(nrm, nm3i);
			glUniformMatrix3fv(r->fw_u_normmat, 1, GL_FALSE, (const GLfloat *)nrm);
			glUniformMatrix4fv(r->fw_u_obmat, 1, GL_FALSE, (const GLfloat *)m->obmat);

			glBindBuffer(GL_ARRAY_BUFFER, m->vbo);
			glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m->ibo);
			glEnableVertexAttribArray(r->fw_a_co);
			glVertexAttribPointer(r->fw_a_co, 3, GL_FLOAT, GL_FALSE, stride, (void *)0);
			glEnableVertexAttribArray(r->fw_a_n);
			glVertexAttribPointer(r->fw_a_n, 3, GL_FLOAT, GL_FALSE, stride,
			                      (void *)(3 * sizeof(float)));
			glEnableVertexAttribArray(r->fw_a_rgba);
			glVertexAttribPointer(r->fw_a_rgba, 4, GL_FLOAT, GL_FALSE, stride,
			                      (void *)(8 * sizeof(float)));
			glEnableVertexAttribArray(r->fw_a_emit);
			glVertexAttribPointer(r->fw_a_emit, 1, GL_FLOAT, GL_FALSE, stride,
			                      (void *)(12 * sizeof(float)));
			glEnableVertexAttribArray(r->fw_a_uv);
			glVertexAttribPointer(r->fw_a_uv, 2, GL_FLOAT, GL_FALSE, stride,
			                      (void *)(6 * sizeof(float)));
			glEnableVertexAttribArray(r->fw_a_tangent);
			glVertexAttribPointer(r->fw_a_tangent, 4, GL_FLOAT, GL_FALSE, stride,
			                      (void *)(13 * sizeof(float)));
			glEnableVertexAttribArray(r->fw_a_orco);
			glVertexAttribPointer(r->fw_a_orco, 3, GL_FLOAT, GL_FALSE, stride,
			                      (void *)(17 * sizeof(float)));

			/* материал: диффузная карта (юнит 4) */
			gpu_upload_mesh_tex(m);
			glActiveTexture(GL_TEXTURE4);
			glBindTexture(GL_TEXTURE_2D, m->tex ? (GLuint)m->tex : r->tex_white);
			glUniform1i(r->fw_u_tex, 4);   /* M21: было НЕ выставлено -> u_tex=0 (текстура глубины!) */
			glUniform1f(r->fw_u_has_tex, m->tex ? 1.0f : 0.0f);
			glUniform1f(r->fw_u_colfac, m->colfac);
			glUniform1f(r->fw_u_refl, m->refl);   /* M25: shi->refl = ma->ref */
			glUniform1i(r->fw_u_tex_blend, m->tex_blend);
			glUniform1f(r->fw_u_tex_orco, m->tex_orco ? 1.0f : 0.0f);
	glUniform1f(r->fw_u_tex_glob, m->tex_glob ? 1.0f : 0.0f);
			glUniform1f(r->fw_u_tex_map, (float)m->tex_map);
			glUniform2f(r->fw_u_tex_size, m->tex_size[0], m->tex_size[1]);
			glUniform2f(r->fw_u_tex_ofs, m->tex_ofs[0], m->tex_ofs[1]);
			/* ★ M19: процедурная карта цвета. В forward-проходе юниты 2/3/7
			 * свободны (0=глубина, 1=albedo, 4=диффуз, 5=нормали, 6=альфа,
			 * 10..13=тени). */
			glUniform1i(r->fw_u_proc_type, m->proc_type);
			glUniform1f(r->fw_u_proc_col, (m->proc_type != 0 && !m->proc_col) ? 0.0f : 1.0f);   /* M20 */
	glUniform1f(r->fw_u_tex_rgbtoint, (float)m->tex_rgbtoint);
			glUniform1i(r->fw_u_proc_stype, m->proc_stype);
			glUniform1i(r->fw_u_proc_flip, m->proc_flip);
			glUniform1f(r->fw_u_proc_noisesize, m->proc_noisesize);
			glUniform1i(r->fw_u_proc_noisedepth, m->proc_noisedepth);
			glUniform1i(r->fw_u_proc_basis, m->proc_basis);
	glUniform1i(r->fw_u_proc_wf, m->proc_wf);
	glUniform1f(r->fw_u_proc_dist, m->proc_dist);
			glUniform1i(r->fw_u_proc_hard, m->proc_hard);
			glUniform1f(r->fw_u_proc_bright, m->proc_bright);
			glUniform1f(r->fw_u_proc_contrast, m->proc_contrast);
			glUniform1i(r->fw_u_proc_noclamp, m->proc_noclamp);
			glUniform1i(r->fw_u_proc_has_ramp, m->proc_has_ramp);
			glUniform1i(r->fw_u_proc_orco, m->proc_orco);   /* M21/M27: 0=UV, 1=ORCO, 2=мир */
			glUniform2f(r->fw_u_proc_size, m->proc_size[0], m->proc_size[1]);
			glUniform2f(r->fw_u_proc_ofs, m->proc_ofs[0], m->proc_ofs[1]);
			if (m->proc_type != 0) {
				glUniform1f(r->fw_u_has_tex, 1.0f);
				glActiveTexture(GL_TEXTURE2);
				glBindTexture(GL_TEXTURE_2D, r->tex_hash);
				glUniform1i(r->fw_u_proc_hash, 2);
				glActiveTexture(GL_TEXTURE3);
				glBindTexture(GL_TEXTURE_2D, r->tex_hashv);
				glUniform1i(r->fw_u_proc_hashv, 3);
				glActiveTexture(GL_TEXTURE7);
				glBindTexture(GL_TEXTURE_2D, r->tex_ramp);
				glUniform1i(r->fw_u_proc_ramp, 7);
				glActiveTexture(GL_TEXTURE8);
				glBindTexture(GL_TEXTURE_2D, r->tex_hashp);
				glUniform1i(r->fw_u_proc_hashp, 8);
				if (m->proc_has_ramp) {
					glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, 256, 1, GL_RGBA,
					                GL_UNSIGNED_BYTE, m->proc_ramp);
				}
				glActiveTexture(GL_TEXTURE4);   /* вернуть юнит диффузной карты */
			}
			/* альфа-карта (юнит 6) */
			gpu_upload_mesh_atex(m);
			{
				unsigned int at = 0;
				if (m->a_has && m->a_tex_tried)
					at = (m->aima == m->ima) ? m->tex : m->atex;
				glActiveTexture(GL_TEXTURE6);
				glBindTexture(GL_TEXTURE_2D, at ? (GLuint)at : r->tex_white);
				glUniform1i(r->fw_u_atex, 6);   /* M21: тоже не выставлялось */
				glUniform1f(r->fw_u_has_atex, at ? 1.0f : 0.0f);
				glUniform1f(r->fw_u_a_fac, m->a_fac);
				glUniform1f(r->fw_u_a_def, m->a_def);
				glUniform1i(r->fw_u_a_blend, m->a_blend);
				glUniform1f(r->fw_u_a_use_a, (float)m->a_use_a);
				glUniform1f(r->fw_u_a_orco, m->a_orco ? 1.0f : 0.0f);
	glUniform1f(r->fw_u_a_glob, m->a_glob ? 1.0f : 0.0f);
				glUniform1f(r->fw_u_a_map, (float)m->a_map);
				glUniform2f(r->fw_u_a_size, m->a_size[0], m->a_size[1]);
				glUniform2f(r->fw_u_a_ofs, m->a_ofs[0], m->a_ofs[1]);
			}
			glUniform1f(r->fw_u_alpha, alpha_on ? m->alpha : 1.0f);
			/* нормальная карта (юнит 5). M30: активируем юнит ДО загрузки,
			 * иначе gpu_upload_mesh_ntex отвяжет диффузную карту с юнита 4. */
			glActiveTexture(GL_TEXTURE5);
			gpu_upload_mesh_ntex(m);
			glBindTexture(GL_TEXTURE_2D, m->ntex ? (GLuint)m->ntex : r->tex_white);
			glUniform1i(r->fw_u_nrm, 5);   /* M21: тоже не выставлялось */
			glUniform1f(r->fw_u_has_nrm, m->n_has ? 1.0f : 0.0f);
			glActiveTexture(GL_TEXTURE4);
			/* блик материала */
			glUniform3fv(r->fw_u_spec_col, 1, (const GLfloat *)m->spec_col);
			glUniform1f(r->fw_u_hard, m->hard / 511.0f);

			glDrawElements(GL_TRIANGLES, m->num_indices, GL_UNSIGNED_INT, NULL);

			/* ★ M16: прозрачная проволока (материал WIRE) — линиями. */
			if (m->num_line_indices > 0 && m->lbo) {
				glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m->lbo);
				glDrawElements(GL_LINES, m->num_line_indices, GL_UNSIGNED_INT, NULL);
				glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m->ibo);
			}
		}
		glBindBuffer(GL_ARRAY_BUFFER, 0);
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
		glDisable(GL_BLEND);
		glDepthMask(GL_TRUE);
	}

	/* --- очистка GL-состояния: иначе ломается отрисовка остального UI --- */
	glDisableVertexAttribArray(r->light_a_pos);
	glDisableVertexAttribArray(r->light_a_uv);
	glDisableVertexAttribArray(r->gb_a_co);
	glDisableVertexAttribArray(r->gb_a_n);
	glDisableVertexAttribArray(r->gb_a_rgba);
	glDisableVertexAttribArray(r->gb_a_emit);
	glDisableVertexAttribArray(r->gb_a_uv);
	glDisableVertexAttribArray(r->gb_a_tangent);
	glDisableVertexAttribArray(r->gb_a_orco);
	glDisableVertexAttribArray(r->shadow_a_co);
	if (fwd_on) {
		glDisableVertexAttribArray(r->fw_a_co);
		glDisableVertexAttribArray(r->fw_a_n);
		glDisableVertexAttribArray(r->fw_a_rgba);
		glDisableVertexAttribArray(r->fw_a_emit);
		glDisableVertexAttribArray(r->fw_a_uv);
		glDisableVertexAttribArray(r->fw_a_tangent);
		glDisableVertexAttribArray(r->fw_a_orco);
	}
	/* --- проход 4 (M16): спрайты-хало (материалы типа HALO). Рисуем ПОСЛЕ
	 * forward-прохода, в тот же FBO: у BI хало — светящееся пятно, которое
	 * подмешивается поверх уже отрисованной сцены (rendercore.c). Каждое хало
	 * это один биллборд-квад, повёрнутый к камере; затенение непрозрачной
	 * геометрией — в шейдере, сравнением с глубиной G-буфера. */
	const char *halo_e = getenv("UBF_GPU_HALO");
	const int halos_on = halo_e ? (atoi(halo_e) != 0) : 1;

	if (halos_on && r->prog_halo && gr->scene && gr->scene->num_halos > 0) {
		RE_GPUScene *sc = gr->scene;
		float right[3], up[3];
		int hi;

		printf("[GPU-HALO] %d спрайтов\n", sc->num_halos);

		glUseProgram(r->prog_halo);
		glEnable(GL_BLEND);
		/* BI addalphaAddfacFloat: dest*(1 - a*(255-add)/255) + a*col.
		 * В шейдере alpha_out = a*(255-add)/255, поэтому ONE/ONE_MINUS_SRC_ALPHA
		 * даёт ровно dest*(1-alpha_out) + src. */
		glBlendFunc(GL_ONE, GL_ONE_MINUS_SRC_ALPHA);
		glDepthMask(GL_FALSE);
		glDisable(GL_DEPTH_TEST);

		glUniformMatrix4fv(r->halo_u_proj, 1, GL_FALSE, (const GLfloat *)re->winmat);
		glUniformMatrix4fv(r->halo_u_view, 1, GL_FALSE, (const GLfloat *)re->viewmat);
		/* ★ Оси камеры в МИРЕ. У Blender соглашение СТРОКА-ВЕКТОР:
		 * v_камеры = v_мира * viewmat, т.е. v_камеры[j] = Σ_i v_мира[i]*viewmat[i][j].
		 * Значит вектор мира, попадающий в камеру как (1,0,0) — это СТОЛБЕЦ 0
		 * матрицы вида (и столбец 1 для (0,1,0)). Строки здесь НЕ годятся: у
		 * ортонормированной матрицы строки тоже единичные, но указывают в другие
		 * стороны, и биллборд получался перекошенным (эллипсы вместо кругов).
		 * Это ровно то же mul_m4_v3-поведение: res[0] = x*m[0][0]+y*m[1][0]+z*m[2][0]. */
		right[0] = re->viewmat[0][0]; right[1] = re->viewmat[1][0]; right[2] = re->viewmat[2][0];
		up[0]    = re->viewmat[0][1]; up[1]    = re->viewmat[1][1]; up[2]    = re->viewmat[2][1];
		glUniform3fv(r->halo_u_right, 1, (const GLfloat *)right);
		glUniform3fv(r->halo_u_up, 1, (const GLfloat *)up);
		glUniform2f(r->halo_u_resolution, (float)r->width, (float)r->height);
		glUniform1f(r->halo_u_pack, pack_scale);

		glActiveTexture(GL_TEXTURE0);
		glBindTexture(GL_TEXTURE_2D, r->tex_vpos);
		glUniform1i(r->halo_u_depth, 0);

		/* hashvectf[768] уже лежит текстурой 256x1 (та же, что питает
		 * процедурный шум) — кольца и лучи берут из неё «случайные» радиусы
		 * и направления ровно как BI (pixelshading.c:337/388). */
		glActiveTexture(GL_TEXTURE1);
		glBindTexture(GL_TEXTURE_2D, r->tex_hashv);
		glUniform1i(r->halo_u_hashv, 1);
		glActiveTexture(GL_TEXTURE0);

		glBindBuffer(GL_ARRAY_BUFFER, r->quad_vbo);
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, r->quad_ibo);
		const int hstride = 4 * sizeof(float);
		glEnableVertexAttribArray(r->halo_a_corner);
		glVertexAttribPointer(r->halo_a_corner, 2, GL_FLOAT, GL_FALSE, hstride, (void *)0);
		glEnableVertexAttribArray(r->halo_a_uv);
		glVertexAttribPointer(r->halo_a_uv, 2, GL_FLOAT, GL_FALSE, hstride,
		                      (void *)(2 * sizeof(float)));

		for (hi = 0; hi < sc->num_halos; hi++) {
			RE_GPUHalo *h = &sc->halos[hi];
			float vpos[3];

			copy_v3_v3(vpos, h->pos);
			mul_m4_v3(re->viewmat, vpos);   /* мир -> камера */

			glUniform3fv(r->halo_u_pos, 1, (const GLfloat *)h->pos);
			glUniform1f(r->halo_u_rad, h->rad);
			glUniform1f(r->halo_u_zcenter, -vpos[2]);   /* положительная дистанция */
			glUniform3fv(r->halo_u_col, 1, (const GLfloat *)h->col);
			glUniform1f(r->halo_u_alfa, h->alfa);
			glUniform1f(r->halo_u_hard, h->hard);
			glUniform1f(r->halo_u_add, h->add);
			/* M24: кольца / лучи / «звезда» / XALPHA — как в shadeHaloFloat.
			 * Кольца и лучи ограничены 8 штуками (потолок GLSL-цикла). */
			glUniform1f(r->halo_u_seed, (float)h->seed);
			glUniform1f(r->halo_u_ringc, (float)((h->ringc > 8) ? 8 : h->ringc));
			glUniform1f(r->halo_u_linec, (float)((h->linec > 8) ? 8 : h->linec));
			glUniform1f(r->halo_u_starpoints, (float)h->starpoints);
			glUniform1f(r->halo_u_xalpha, (float)h->xalpha);
			glUniform3fv(r->halo_u_spec_col, 1, (const GLfloat *)h->spec_col);
			glUniform3fv(r->halo_u_mir_col, 1, (const GLfloat *)h->mir_col);
			glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, NULL);
		}

		glDisableVertexAttribArray(r->halo_a_corner);
		glDisableVertexAttribArray(r->halo_a_uv);
		glBindBuffer(GL_ARRAY_BUFFER, 0);
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
		glDisable(GL_BLEND);
	}

	/* отвязать все юниты, которые использовали как источники (иначе feedback loop) */
	glActiveTexture(GL_TEXTURE13); glBindTexture(GL_TEXTURE_2D, 0);
	glActiveTexture(GL_TEXTURE12); glBindTexture(GL_TEXTURE_2D, 0);
	glActiveTexture(GL_TEXTURE11); glBindTexture(GL_TEXTURE_2D, 0);
	glActiveTexture(GL_TEXTURE10); glBindTexture(GL_TEXTURE_2D, 0);
	glActiveTexture(GL_TEXTURE9); glBindTexture(GL_TEXTURE_2D, 0);
	glActiveTexture(GL_TEXTURE8); glBindTexture(GL_TEXTURE_2D, 0);
	glActiveTexture(GL_TEXTURE7); glBindTexture(GL_TEXTURE_2D, 0);
	glActiveTexture(GL_TEXTURE6); glBindTexture(GL_TEXTURE_2D, 0);
	glActiveTexture(GL_TEXTURE5); glBindTexture(GL_TEXTURE_2D, 0);
	glActiveTexture(GL_TEXTURE4); glBindTexture(GL_TEXTURE_2D, 0);
	glActiveTexture(GL_TEXTURE3); glBindTexture(GL_TEXTURE_2D, 0);
	glActiveTexture(GL_TEXTURE2); glBindTexture(GL_TEXTURE_2D, 0);
	glActiveTexture(GL_TEXTURE1); glBindTexture(GL_TEXTURE_2D, 0);
	glActiveTexture(GL_TEXTURE0); glBindTexture(GL_TEXTURE_2D, 0);
	glUseProgram(0);
	/* вернуть вьюпорт-FBO и ИСХОДНЫЙ viewport (главный поток продолжает рисовать UI) */
	glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, (GLuint)saved_fb);
	glViewport(saved_vp[0], saved_vp[1], saved_vp[2], saved_vp[3]);
	glDisable(GL_DEPTH_TEST);
	glDepthMask(GL_TRUE);
}

void RE_gpu_renderer_gl_free_scene(RE_GPUScene *scene)
{
	if (!scene) return;
	/* удалить VBO/IBO мешей ИМЕННО этой сцены (главный поток, GL активен) */
	for (RE_GPUMesh *m = scene->meshes; m; m = m->next) {
		if (m->vbo) glDeleteBuffers(1, &m->vbo);
		if (m->ibo) glDeleteBuffers(1, &m->ibo);
		if (m->lbo) glDeleteBuffers(1, &m->lbo);   /* M16: буфер линий WIRE */
		if (m->tex) glDeleteTextures(1, &m->tex);
		if (m->ntex) glDeleteTextures(1, &m->ntex);
		if (m->atex) glDeleteTextures(1, &m->atex);
		for (int k = 0; k < GPU_MAX_TEX_SLOTS - 1; k++)
			if (m->tex2[k]) glDeleteTextures(1, &m->tex2[k]);
		m->vbo = 0;
		m->ibo = 0;
		m->lbo = 0;
		m->tex = 0;
		m->tex_tried = 0;
		m->ntex = 0;
		m->n_tex_tried = 0;
		m->atex = 0;
		m->a_tex_tried = 0;
		m->uploaded = false;
	}
}

void RE_gpu_renderer_gl_free(RE_GPURenderer *gr)
{
	if (!gr || !gr->gpu_res) return;

	RE_gpu_renderer_gl_free_scene(gr->scene);

	gpu_res_free(gr->gpu_res);
	MEM_freeN(gr->gpu_res);
	gr->gpu_res = NULL;
}


