/*
 * RE_Shader.c — свой шейдер (шаг B3, «вариант B»).
 *
 * Своё здесь: модель освещения (Lambert + MA_CUBIC), свой цикл по лампам,
 * своя аккумуляция и свой выход в кадр. Дорогая часть шейдинга BI —
 * shade_material_loop() (то есть цикл по лампам и трассировка отражений) —
 * для поддержанных пикселей НЕ вызывается вообще.
 *
 * Остаётся вызвать BI только на дешёвый setup: shade_samples_fill_with_ps()
 * резолвит obi/facenr -> vlr/material и раскладывает co/vn/uv.
 * ★ ШАГ B3-iii: material-side скаляры читаются из Material напрямую, и
 * shade_input_init_material() больше не вызывается — шейдер не зависит от
 * того, разложил ли BI материал в ShadeInput.
 *
 * Что берётся у BI как утилита (осознанно, на этот шаг):
 *   - lamp_get_visibility() — направление на лампу, затухание, конус спота.
 *     Это геометрия лампы, а не модель освещения; брать её у BI гарантирует,
 *     что расхождение в картинке будет от нашей модели, а не от арифметики
 *     затухания. Полностью свой расчёт направления — следующий шаг.
 *   - get_lights() — список ламп и порядок обхода (тот же, что у BI).
 *   - shader_unsupported() — фильтр отката. Читает ТОЛЬКО поля Material и
 *     настройки сцены/мира; из ShadeInput ему нужен один указатель shi->mat,
 *     который всё равно резолвит setup. Это не значения для расчёта, а
 *     признак «такой материал мы не берём».
 *   - геометрия пикселя (co/vn) приходит из G-буфера шага 3a, НЕ из
 *     shi->co/shi->vn. Это и есть смысл шага 3a: шейдер не зависит от
 *     того, как BI посчитал интерполяцию.
 *
 * Откат: если материал, лампа, пасс или настройка сцены не поддержаны,
 * пиксель целиком отдаётся BI (shade_samples_shade()), причём setup уже
 * сделан, так что он не прогоняется дважды. Откат безопасен на любом
 * шаге: shade_input_do_shade() полностью перезаписывает ShadeResult,
 * поэтому частично посчитанные нами сэмплы не «протекают» в результат BI.
 */

#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "BLI_math.h"
#include "BLI_utildefines.h"

#include "RE_Mirror.h"

#include "DNA_group_types.h"
#include "DNA_lamp_types.h"
#include "DNA_material_types.h"
#include "DNA_object_types.h"
#include "DNA_scene_types.h"
#include "DNA_world_types.h"

#include "BKE_scene.h"

#include "render_types.h"
#include "rendercore.h"
#include "shading.h"
#include "texture.h"      /* do_material_tex — текстуры в быстром пути */
#include "RE_Rasterizer.h"

/* defined in pipeline.c; копия активного Render, как во всех файлах рендера
 * (rendercore.c:70, zbuf.c:79, ...). Нужна, чтобы признак быстрого режима
 * читался из того же места, откуда его читает растеризатор. */
extern struct Render R;

/*
 * Пассы, которые свой шейдер заполняет сам: COMBINED и Z. По умолчанию
 * слой запрашивает именно COMBINED|Z (blenkernel/intern/scene.c), так что
 * без Z свой шейдер не брал бы ни одного пикселя.
 * Z у нас тот же, что у BI (-shi->co[2]): эту величину мы НЕ подменяем.
 * DIFFUSE сюда не входит: у BI shr->diff не включает ambient (ambient
 * добавляется в combined отдельно), поэтому DIFFUSE-пасс мы бы отдали
 * неверно. Всё, что вне этого набора, отдаётся BI целиком — в частности
 * INDEXOB/INDEXMA, MIST, RGBA, NORMAL, UV и пассы трассировки.
 */
#define SHADER_PASS_OK (SCE_PASS_COMBINED | SCE_PASS_Z)

/* ★ БЫСТРЫЙ ПУТЬ (rasterizer_mode = FAST).
 *
 * Отличие от UBF_SHADER (шаг B3/B4) — не в шейдере, а в КОНТРАКТЕ:
 *   - UBF_SHADER обязан быть побитово равен BI, поэтому откатывает на BI всё,
 *     что не умеет (текстуры, тени, AO, прозрачность, ...); на боевой сцене
 *     это 0% покрытия, то есть весь рендер всё равно считает BI;
 *   - FAST откатываться не имеет права: он покрывает 100% пикселей и
 *     заменяет неподдержанное приближением (нет теней, нет AO, нет спекуляра,
 *     прозрачность = непрозрачность). Приёмка у него своя — допуск по картинке
 *     и время, а не max=0 против BI.
 *
 * Старый путь при этом никуда не девается: он включается своим значением
 * rasterizer_mode (OFF — BI, SCANLINE — наш растеризатор + BI-затенение,
 * UBF_SHADER=1 — побитовый свой шейдер).
 *
 * Кэш: признак режима нельзя кэшировать в static на процесс — в GUI режим
 * переключается без перезапуска. Поле R.r.rasterizer_mode читается напрямую
 * (это чтение поля, не getenv); getenv остался только как внешняя ручка для
 * замеров и кэшируется отдельно. */
bool RE_shader_fast(void)
{
	static int env_cached = -1;

	if (R.r.rasterizer_mode == RE_RASTERIZER_FAST)
		return true;

	if (env_cached < 0)
		env_cached = getenv("UBF_FAST") ? 1 : 0;

	return env_cached != 0;
}

bool RE_shader_enabled(void)
{
	static int cached = -1;

	if (cached < 0) cached = getenv("UBF_SHADER") ? 1 : 0;
	return cached != 0 || RE_shader_fast();
}

/* Текстуры и рампы в быстром пути: по умолчанию ВКЛЮЧЕНЫ.
 *
 * Это единственная часть BI, которую быстрый путь берёт как есть
 * (do_material_tex) — она дешёвая (один проход по слотам mtex на пиксель) и
 * без неё сцена с 151 текстурой выглядит плоской, из-за чего сравнение с
 * эталоном теряет смысл. UBF_FAST_TEX=0 выключает для абляционных замеров. */
static bool shader_fast_tex_enabled(void)
{
	static int cached = -1;

	if (cached < 0) {
		const char *e = getenv("UBF_FAST_TEX");

		cached = (e && e[0] == '0') ? 0 : 1;
	}
	return cached != 0;
}

/* Есть ли у материала настоящие текстурные слоты.
 *
 * ★ ШАГ B4: проверять `ma->texco` здесь НЕЛЬЗЯ. init_render_material()
 * (blenkernel/material.c:987) для любого материала с MA_RAYMIRROR (или
 * MA_SHADOW_TRA, или MA_TRANSP|MA_RAYTRANSP) при включённом R_RAYTRACE
 * выставляет ma->texco |= NEED_UV | TEXCO_ORCO | TEXCO_REFL | TEXCO_NORM,
 * потому что рейтрейсеру нужны O-структуры без пересчёта на каждый луч.
 * То есть у зеркального материала без единой текстуры texco заведомо
 * ненулевой, и проверка по нему отвергала бы ровно то, что мы теперь умеем.
 * Считаем слоты mtex — это и есть «есть текстуры». */
static bool material_has_texture(const Material *ma)
{
	int a;

	for (a = 0; a < MAX_MTEX; a++) {
		const MTex *mtex = ma->mtex[a];

		if (ma->septex & (1 << a)) continue;
		if (mtex && mtex->tex) return true;
	}

	return false;
}

/* Материал или сцена используют то, что свой шейдер пока не воспроизводит.
 * Возвращает текстовую причину отказа либо NULL, если всё поддерживается. */
static const char *shader_unsupported(Render *re, ShadeInput *shi)
{
	Material *ma = shi->mat;

	if (!ma) return "no material";
	if (ma->nodetree && ma->use_nodes) return "node material";
	if (ma->material_type == MA_TYPE_VOLUME) return "volume material";
	if (ma->material_type == MA_TYPE_WIRE) return "wire material";
	if (ma->material_type == MA_TYPE_HALO) return "halo material";
	/* ★ ШАГ B4: MA_RAYMIRROR здесь больше НЕ отвергается — отражение считает
	 * shader_mirror() своим лучом и своим шейдером точки попадания. Что именно
	 * в зеркале не поддержано (размытие, fadeout, sky-fallback, неподдержанный
	 * материал попадания), проверяется там и приводит к тому же откату. */
	if (ma->mode & (MA_TRANSP | MA_RAYTRANSP)) return "transparency";
	if (ma->mode & (MA_ONLYSHADOW | MA_ONLYCAST)) return "onlyshadow/onlycast";
	if (material_has_texture(ma)) return "material textures";
	if (ma->mode & (MA_FACETEXTURE | MA_VERTEXCOLP)) return "vertex/fac colors";
	if (ma->ramp_col || ma->ramp_spec) return "color ramps";
	if (ma->sss_flag & MA_DIFF_SSS) return "subsurface scattering";
	if (ma->diff_shader != MA_DIFF_LAMBERT) return "non-lambert diffuse shader";
	if (re->wrld.mode & (WO_AMB_OCC | WO_ENV_LIGHT | WO_INDIRECT_LIGHT))
		return "AO/env/indirect light";
	if (re->wrld.mode & WO_MIST) return "mist";
	if (re->i.curblur) return "motion blur";
	if ((ma->mode & MA_SHADOW) && (re->r.mode & R_SHADOW)) return "shadows";

	return NULL;
}

/*
 * ★ ШАГ B3-iii: свой разбор материала — значения берутся из Material, а не
 * из ShadeInput после shade_input_init_material().
 *
 * shade_input_init_material() (shadeinput.c:80) — это memcpy(&shi->r,
 * &shi->mat->r, 23 * sizeof(float)), то есть просто копия полей Material.
 * Значит чтение тех же полей Material даёт побитово те же числа: никакого
 * пересчёта тут нет и быть не должно. Любые преобразования (например,
 * свёртка ambient с цветом мира) уже сделаны конвертером задолго до этого:
 *
 *   init_render_material()  (blenkernel/material.c:993)
 *       ma->ambr = ma->amb * wrld.ambr   (то же для g и b)
 *
 * ВАЖНО про имена: слот 19 в обеих структурах называется по-разному —
 * в Material это `ref`, в ShadeInput `refl`. Это одно и то же поле, и оно же
 * RNA `diffuse_intensity` (rna_material.c:1047-1048). Так что shi->refl надо
 * брать как ma->ref; соблазн взять ma->ray_mirror или искать ma->refl даёт
 * другое число.
 */
typedef struct RE_MaterialVals {
	float r, g, b;
	float refl;                  /* == ma->ref == diffuse_intensity */
	float ambr, ambg, ambb;      /* уже свёрнуто с цветом мира конвертером */
} RE_MaterialVals;

static void shader_material_values(const Material *ma, RE_MaterialVals *mv)
{
	mv->r = ma->r;
	mv->g = ma->g;
	mv->b = ma->b;
	mv->refl = ma->ref;
	mv->ambr = ma->ambr;
	mv->ambg = ma->ambg;
	mv->ambb = ma->ambb;
}

/*
 * Свой цвет для одного сэмпла: Lambert по нашему vn, лампы в том же
 * порядке, что у BI, ambient из материала.
 *
 * out[3] — цвет (не premultiplied, как shr->combined у BI)
 * co/vn  — из G-буфера части (view space)
 * Возвращает false, если встретилось то, что мы не умеем; тогда пиксель
 * нужно целиком отдать BI.
 */
static bool shader_light(Render *re, ShadeInput *shi, const RE_MaterialVals *mv,
                         const float co[3], const float vn[3],
                         float out[3], const char **why)
{
	Material *ma = shi->mat;
	ListBase *lights;
	GroupObject *go;
	float acc[3];

	/* ambient: материал x мир, свёрнутый конвертером (см. выше) */
	acc[0] = mv->ambr;
	acc[1] = mv->ambg;
	acc[2] = mv->ambb;

	lights = get_lights(shi);

	for (go = lights->first; go; go = go->next) {
		LampRen *lar = go->lampren;
		float lv[3], lampdist, visifac, is, i;
		float lacol[3];

		if (!lar) continue;
		if (lar->energy == 0.0f) continue;
		if (lar->mode & LA_NO_DIFF) continue;

		if (lar->mode & (LA_TEXTURE | LA_SHAD_TEX)) {
			if (why) *why = "lamp texture";
			return false;
		}
		if (lar->type == LA_AREA) {
			if (why) *why = "area lamp";
			return false;
		}

		/* видимость лампы: те же условия и тот же порядок, что в shade_lamp_loop() */
		if (lar->mode & LA_LAYER)
			if ((lar->lay & shi->obi->lay) == 0 && !BKE_object_layer_visible(shi->obi->ob))
				continue;
		if ((lar->lay & shi->lay) == 0 && !BKE_object_layer_visible(lar->ob))
			continue;

		visifac = lamp_get_visibility(lar, co, lv, &lampdist);
		if (visifac == 0.0f) continue;

		lacol[0] = lar->r;
		lacol[1] = lar->g;
		lacol[2] = lar->b;

		/* диффузный шейдер: Lambert (остальные отсеяны выше), у hemi своя кривая */
		is = dot_v3v3(vn, lv);
		if (lar->type == LA_HEMI) is = 0.5f * is + 0.5f;

		if ((ma->shade_flag & MA_CUBIC) && is > 0.0f && is < 1.0f)
			is = 3.0f * is * is - 2.0f * is * is * is;

		/* i = is * phongcorr (без теней phongcorr == 1), затем энергия и рефлективность */
		i = is;
		if (i > 0.0f) i *= visifac * mv->refl;

		/* теней нет, поэтому i_noshad == i; порядок операций как в add_to_diffuse() */
		if (i > 0.0f) {
			acc[0] += i * lacol[0] * mv->r;
			acc[1] += i * lacol[1] * mv->g;
			acc[2] += i * lacol[2] * mv->b;
		}
	}

	out[0] = acc[0];
	out[1] = acc[1];
	out[2] = acc[2];

	return true;
}

/*
 * ★ ШАГ B4: своё отражение mirror-материала.
 *
 * Своё здесь: вектор отражения и обход сцены (RE_mirror_trace, RE_Mirror.c),
 * а также освещение точки попадания — оно считается тем же нашим
 * shader_light(), что и первичный пиксель, своим разбором материала
 * попадания. BI-рейтрейс (ray_trace/trace_reflect/traceray/
 * RE_rayobject_raycast) для поддержанных зеркал не вызывается вообще.
 *
 * Как у BI устроено смешение (ray_trace(), rayshade.c:1562):
 *   i  = ray_mirror * fresnel_fac(view, vn, fresnel_mir_i, fresnel_mir)
 *   mircol = цвет, пришедший из traceray() — либо освещение точки попадания,
 *            либо (при промахе) shr->combined первичного пикселя, если
 *            fadeto_mir == MA_RAYMIR_FADETOMAT (иначе — небо);
 *   combined = (1 - i) * col + i * mirr/g/b * mircol
 * Здесь повторено ровно это, включая порядок умножений (fr = i*mirr,
 * затем mircol*fr), чтобы совпадение было побитовым.
 *
 * Границы шага (всё остальное честно отдаётся BI с причиной):
 *   - размытое зеркало (gloss_mir != 1.0) — у BI это много лучей на пиксель;
 *   - dist_mir > 0 — у BI это fadeout к цвету материала/неба;
 *   - промах при fadeto_mir != FADETOMAT — у BI это цвет неба, которого у нас нет;
 *   - материал точки попадания не поддержан нашим шейдером (в том числе
 *     зеркало/прозрачность) — рекурсии второго отскока у нас пока нет.
 */
static bool shader_mirror(Render *re, ShadeInput *shi,
                          const float co[3], const float vn[3],
                          float col[3], const char **why)
{
	Material *ma = shi->mat;
	RE_MirrorHit hit;
	float dir[3], i, mircol[3];

	/* ★ SSR в BI (UBF_BI_SSR, по умолчанию ВЫКЛ): свой луч зеркала не пускаем.
	 * Отражение соберёт резолв кадрового G-буфера (RE_fast_frame_resolve) по
	 * данным, которые пишет bi_ssr_store_pixel() в rendercore.c. Иначе
	 * отражение считалось бы дважды — здесь лучом и там экранным маршем.
	 * Возврат true означает «пиксель обработан»: col остаётся плоским.
	 *
	 * ★ ГИБРИД (шаг 5): в ремонтном проходе луч как раз и нужен — это пиксели,
	 * где экранный марш промахнулся. */
	if (RE_bi_ssr_enabled() && !RE_bi_ssr_repair_pass())
		return true;

	if (ma->gloss_mir != 1.0f) {
		if (why) *why = "blurry mirror";
		return false;
	}
	if (ma->dist_mir > 0.0f) {
		if (why) *why = "mirror fadeout";
		return false;
	}

	/* i == 0 — у BI do_mir == false, отражения нет вообще */
	i = ma->ray_mirror * fresnel_fac(shi->view, vn, ma->fresnel_mir_i, ma->fresnel_mir);
	if (i == 0.0f)
		return true;

	memset(&hit, 0, sizeof(hit));

	if (!RE_mirror_trace(re, shi, co, vn, &hit, dir)) {
		if (ma->fadeto_mir == MA_RAYMIR_FADETOMAT) {
			/* ray_fadeout_endcolor(): fadeto material => shr->combined */
			copy_v3_v3(mircol, col);
		}
		else {
			if (why) *why = "mirror sky fallback";
			return false;
		}
	}
	else {
		/* Setup точки попадания — ровно то, что делает shade_ray()
		 * (rayshade.c:487) до собственно шейдинга. Освещение здесь наше. */
		ShadeInput hshi;
		RE_MaterialVals hmv;
		const char *bad;

		memset(&hshi, 0, sizeof(ShadeInput));

		copy_v3_v3(hshi.view, dir);
		hshi.co[0] = co[0] + hit.dist * dir[0];
		hshi.co[1] = co[1] + hit.dist * dir[1];
		hshi.co[2] = co[2] + hit.dist * dir[2];
		normalize_v3(hshi.view);

		hshi.obi = hit.obi;
		hshi.obr = hit.obi->obr;
		hshi.vlr = hit.vlr;
		hshi.mat = hit.vlr->mat;
		hshi.depth = shi->depth + 1;
		hshi.thread = shi->thread;
		hshi.xs = shi->xs;
		hshi.ys = shi->ys;
		hshi.lay = shi->lay;
		hshi.mask = shi->mask;
		hshi.passflag = SCE_PASS_COMBINED;
		hshi.combinedflag = 0xFFFFFF;
		hshi.light_override = shi->light_override;
		hshi.mat_override = shi->mat_override;
		copy_v3_v3(hshi.dxco, shi->dxco);
		copy_v3_v3(hshi.dyco, shi->dyco);

		shade_input_set_triangle_i(&hshi, hit.obi, hit.vlr, 0,
		                           (hit.tri == 2) ? 2 : 1,
		                           (hit.tri == 2) ? 3 : 2);
		hshi.u = hit.uv[0];
		hshi.v = hit.uv[1];
		shade_input_set_normals(&hshi);

		bad = shader_unsupported(re, &hshi);
		if (bad) {
			if (why) *why = bad;
			return false;
		}

		shader_material_values(hshi.mat, &hmv);
		if (!shader_light(re, &hshi, &hmv, hshi.co, hshi.vn, mircol, why))
			return false;
	}

	/* ray_trace(): fr = i*mirr; combined = (1-i)*col + mircol*fr
	 * (при spec == 0, а наш шейдер спекуляр не считает) */
	col[0] = (1.0f - i) * col[0] + i * ma->mirr * mircol[0];
	col[1] = (1.0f - i) * col[1] + i * ma->mirg * mircol[1];
	col[2] = (1.0f - i) * col[2] + i * ma->mirb * mircol[2];

	return true;
}

/* ========================================================================= */
/* ★ БЫСТРЫЙ ПУТЬ (rasterizer_mode = FAST): затенение без откатов на BI.      */
/* ========================================================================= */

/* Потолок рекурсии зеркал в быстром пути. У BI это ma->ray_depth (в боевой
 * сцене 2), и это же главная цена: каждый отскок перезатеняет точку
 * попадания полным циклом по лампам. Быстрый путь разрешает один отскок
 * (depth 0 -> 1) и не идёт дальше, если не сказано иначе. */
static int shader_fast_max_depth(void)
{
	static int cached = -1;

	if (cached < 0) {
		const char *e = getenv("UBF_FAST_DEPTH_MAX");

		cached = e ? atoi(e) : 1;
		if (cached < 0) cached = 0;
		if (cached > 8) cached = 8;
	}
	return cached;
}

/* Базовые значения материала для быстрого пути.
 *
 * С текстурами идём через BI: shade_input_set_shade_texco() + 
 * shade_input_init_material() + do_material_tex(). Порядок тот же, что в BI
 * (shade_input_do_shade -> shade_material_loop -> shade_lamp_loop), а
 * set_shade_texco обязателен: только он заполняет lo/gl/orn/winco/tang,
 * которые нужны режимам TEXCO_ORCO/GLOB/NORM/WINDOW/TANGENT. Без текстур
 * читаем те же поля Material напрямую (см. комментарий у RE_MaterialVals).
 *
 * do_material_tex читает и МОЖЕТ менять shi->vn (bump/normal map) — как в BI. */
static void shader_fast_material(Render *re, ShadeInput *shi, RE_MaterialVals *mv)
{
	Material *ma = shi->mat;

	if (ma == NULL) {
		mv->r = mv->g = mv->b = 0.0f;
		mv->refl = 1.0f;
		mv->ambr = mv->ambg = mv->ambb = 0.0f;
		return;
	}

	if (shader_fast_tex_enabled()) {
		shade_input_set_shade_texco(shi);
		shade_input_init_material(shi);
		do_material_tex(shi, re);

		mv->r = shi->r;
		mv->g = shi->g;
		mv->b = shi->b;
		mv->refl = shi->refl;
		mv->ambr = shi->ambr;
		mv->ambg = shi->ambg;
		mv->ambb = shi->ambb;
	}
	else {
		shader_material_values(ma, mv);
	}
}

/* ★ S6: диагностика состава света нашего шейдера (ведётся здесь, печатается
 * из RE_Rasterizer.c в конце кадра). Нужна, чтобы различить две причины
 * слабых теней: «солнца в шейдере мало» (тогда гасить нечего) или «тени
 * срабатывают не там». */
double re_fast_stat_hist[8];
double re_fast_stat_amb;
double re_fast_stat_tot;
double re_fast_stat_sunsum;
long re_fast_stat_npx;

/* Освещение быстрого пути.
 *
 * Это цикл shader_light() БЕЗ фильтров отката: всё, что тот отвергал,
 * здесь приближается, а не отвергается.
 *   - текстуры ламп (LA_TEXTURE/LA_SHAD_TEX) игнорируются;
 *   - площадные лампы считаются точечными (lamp_get_visibility даёт для них
 *     направление и затухание из lar->co — форма пятна теряется);
 *   - ТЕНЕЙ НЕТ. Это осознанная часть приближения: теневой буфер на каждую
 *     из 71 лампы — самая дорогая часть BI-шейдинга после зеркал.
 *   - AO/env/indirect не считаются.
 * Возвращает false только при полной невозможности (нет материала). */
static bool shader_fast_light(Render *re, ShadeInput *shi, const RE_MaterialVals *mv,
                              const float co[3], const float vn[3], float out[3])
{
	Material *ma = shi->mat;
	ListBase *lights;
	GroupObject *go;
	float acc[3];
	/* ★ S1: вклад СОЛНЦА копим отдельно от остальных ламп. Нужно теням: в
	 * затенённом пикселе надо вычесть именно долю солнца, а не погасить весь
	 * цвет вместе с ambient и точечными лампами. */
	float sun[3] = { 0.0f, 0.0f, 0.0f };
	float sundir[3] = { 0.0f, 0.0f, 0.0f };
	int have_sun = 0;

	if (ma == NULL) return false;

	/* shadeless: у BI цвет поверхности отдаётся как есть, без ламп и ambient */
	if (ma->mode & MA_SHLESS) {
		out[0] = mv->r;
		out[1] = mv->g;
		out[2] = mv->b;
		return true;
	}

	/* ambient: материал x мир, свёрнутый конвертером (как в BI) */
	acc[0] = mv->ambr;
	acc[1] = mv->ambg;
	acc[2] = mv->ambb;

	lights = get_lights(shi);

	for (go = lights->first; go; go = go->next) {
		LampRen *lar = go->lampren;
		float lv[3], lampdist, visifac, is, i;
		float lacol[3];

		if (!lar) continue;
		if (lar->energy == 0.0f) continue;
		if (lar->mode & LA_NO_DIFF) continue;

		/* видимость лампы: те же условия и тот же порядок, что в BI */
		if (lar->mode & LA_LAYER)
			if ((lar->lay & shi->obi->lay) == 0 && !BKE_object_layer_visible(shi->obi->ob))
				continue;
		if ((lar->lay & shi->lay) == 0 && !BKE_object_layer_visible(lar->ob))
			continue;

		visifac = lamp_get_visibility(lar, co, lv, &lampdist);
		if (visifac == 0.0f) continue;

		lacol[0] = lar->r;
		lacol[1] = lar->g;
		lacol[2] = lar->b;

		is = dot_v3v3(vn, lv);
		if (lar->type == LA_HEMI) is = 0.5f * is + 0.5f;

		if ((ma->shade_flag & MA_CUBIC) && is > 0.0f && is < 1.0f)
			is = 3.0f * is * is - 2.0f * is * is * is;

		i = is;
		if (i > 0.0f) i *= visifac * mv->refl;

		if (i > 0.0f) {
			acc[0] += i * lacol[0] * mv->r;
			acc[1] += i * lacol[1] * mv->g;
			acc[2] += i * lacol[2] * mv->b;
		}

		/* ★ S1: солнце копим отдельно и запоминаем направление НА него
		 * (оно одно на весь кадр — солнце бесконечно далеко). */
		if (lar->type == LA_SUN) {
			const float c0 = i * lacol[0] * mv->r;
			const float c1 = i * lacol[1] * mv->g;
			const float c2 = i * lacol[2] * mv->b;

			sun[0] += c0;
			sun[1] += c1;
			sun[2] += c2;

			if (c0 + c1 + c2 > 0.0f) {
				normalize_v3_v3(sundir, lv);
				have_sun = 1;
			}
		}
	}

	out[0] = acc[0];
	out[1] = acc[1];
	out[2] = acc[2];

	/* ★ S1: отдаём в кадровый буфер долю солнца (сколько цвета пикселя даёт
	 * солнце) и направление на него. Запись одноразовая: для пикселя первым
	 * приходит первичное затенение, лучи отражений его не перебивают. */
	{
		const float tot = (acc[0] + acc[1] + acc[2]) / 3.0f;
		const float sn  = (sun[0] + sun[1] + sun[2]) / 3.0f;
		const float amb = (mv->ambr + mv->ambg + mv->ambb) / 3.0f;
		float share = (tot > 1e-6f) ? (sn / tot) : 0.0f;

		if (share > 1.0f) share = 1.0f;

		/* ★ S6: диагностика состава света — печатается из RE_Rasterizer.c.
		 * Нужна, чтобы понять, мало ли солнца в шейдере (тогда тени гасить
		 * нечего) или тени срабатывают не там. */
		re_fast_stat_npx++;
		re_fast_stat_tot += tot;
		re_fast_stat_amb += amb;
		re_fast_stat_sunsum += sn;
		{
			int b = (int)(share * 8.0f);

			if (b > 7) b = 7;
			if (b < 0) b = 0;
			re_fast_stat_hist[b] += 1.0;
		}

		if (have_sun)
			RE_fast_frame_store_sun(shi->xs, shi->ys, share, sundir);
	}

	return true;
}

static bool shader_fast_shade_point(Render *re, ShadeInput *shi,
                                    const float co[3], const float vn[3],
                                    float out[3], int depth);

/* Отражение в быстром пути: та же формула смешения, что у BI
 * (ray_trace(): combined = (1-i)*col + i*mirr*mircol), но без отказов:
 * размытие игнорируется (один луч вместо QMC-цикла), fadeout игнорируется,
 * промах даёт цвет самого пикселя (небо не считаем).
 * Точка попадания затеняется тем же быстрым шейдером и может дать ещё один
 * отскок, пока depth < shader_fast_max_depth(). */
static bool shader_fast_mirror(Render *re, ShadeInput *shi,
                               const float co[3], const float vn[3],
                               float col[3], int depth)
{
	Material *ma = shi->mat;
	RE_MirrorHit hit;
	float dir[3], i, mircol[3];

	i = ma->ray_mirror * fresnel_fac(shi->view, vn, ma->fresnel_mir_i, ma->fresnel_mir);
	if (i == 0.0f)
		return true;

	/* ★ Октодерева нет — значит и лучей нет. В режиме FAST оно не строится
	 * (convertblender.c): сборка стоит 4.2 с из 8.3 с кадра, а нужна только
	 * ради этих лучей. Отражение тогда не считается — пиксель остаётся со
	 * своим цветом (зеркало «плоское»). Проверка ОБЯЗАНА стоять здесь, ДО
	 * RE_mirror_trace: у того при пустом дереве есть запасной перебор ВСЕХ
	 * граней сцены (RE_Mirror.c:283) — это 1.97M граней на каждый луч. */
	if (re->raytree == NULL)
		return true;

	memset(&hit, 0, sizeof(hit));

	if (!RE_mirror_trace(re, shi, co, vn, &hit, dir)) {
		/* промах: у BI это цвет неба либо цвет материала (fadeto_mir).
		 * В быстром пути неба нет — берём цвет самого пикселя. */
		copy_v3_v3(mircol, col);
	}
	else {
		ShadeInput hshi;

		memset(&hshi, 0, sizeof(ShadeInput));

		copy_v3_v3(hshi.view, dir);
		hshi.co[0] = co[0] + hit.dist * dir[0];
		hshi.co[1] = co[1] + hit.dist * dir[1];
		hshi.co[2] = co[2] + hit.dist * dir[2];
		normalize_v3(hshi.view);

		hshi.obi = hit.obi;
		hshi.obr = hit.obi->obr;
		hshi.vlr = hit.vlr;
		hshi.mat = hit.vlr->mat;
		hshi.depth = shi->depth + 1;
		hshi.thread = shi->thread;
		hshi.xs = shi->xs;
		hshi.ys = shi->ys;
		hshi.lay = shi->lay;
		hshi.mask = shi->mask;
		hshi.passflag = SCE_PASS_COMBINED;
		hshi.combinedflag = 0xFFFFFF;
		hshi.light_override = shi->light_override;
		hshi.mat_override = shi->mat_override;
		copy_v3_v3(hshi.dxco, shi->dxco);
		copy_v3_v3(hshi.dyco, shi->dyco);

		shade_input_set_triangle_i(&hshi, hit.obi, hit.vlr, 0,
		                           (hit.tri == 2) ? 2 : 1,
		                           (hit.tri == 2) ? 3 : 2);
		hshi.u = hit.uv[0];
		hshi.v = hit.uv[1];

		/* ★ В отличие от побитового shader_mirror(), здесь UV НАДО посчитать:
		 * покрытие 100% означает, что в зеркалах встречаются материалы с
		 * текстурами (TEXCO_UV), а без set_uv они читали бы мусор. */
		shade_input_set_uv(&hshi);
		shade_input_set_normals(&hshi);

		/* ★ depth+1 обязателен именно ЗДЕСЬ: на ребре shade_point -> mirror
		 * глубина должна расти, иначе зеркало, попавшее в зеркало, уходит в
		 * бесконечную рекурсию (на 25% это не проявлялось, на полном кадре
		 * дало EXCEPTION_STACK_OVERFLOW). */
		if (!shader_fast_shade_point(re, &hshi, hshi.co, hshi.vn, mircol, depth + 1))
			return false;

		if ((hshi.mat->mode & MA_RAYMIRROR) && (depth + 1 < shader_fast_max_depth()))
			if (!shader_fast_mirror(re, &hshi, hshi.co, hshi.vn, mircol, depth + 1))
				return false;
	}

	col[0] = (1.0f - i) * col[0] + i * ma->mirr * mircol[0];
	col[1] = (1.0f - i) * col[1] + i * ma->mirg * mircol[1];
	col[2] = (1.0f - i) * col[2] + i * ma->mirb * mircol[2];

	return true;
}

/* Полное затенение одной точки быстрым путём: материал -> лампы -> зеркало.
 * depth — номер отскока (0 для пикселя кадра). */
static bool shader_fast_shade_point(Render *re, ShadeInput *shi,
                                    const float co[3], const float vn[3],
                                    float out[3], int depth)
{
	RE_MaterialVals mv;

	shader_fast_material(re, shi, &mv);

	if (!shader_fast_light(re, shi, &mv, co, vn, out))
		return false;

	/* vn мог быть изменён bump-картой в do_material_tex: отражение должно
	 * считать по той же нормали, что и освещение (у BI порядок тот же). */
	if ((shi->mat->mode & MA_RAYMIRROR) && (depth < shader_fast_max_depth()))
		if (!shader_fast_mirror(re, shi, co, shi->vn, out, depth))
			return false;

	return true;
}

/* Обёртка под контракт RE_shader_shade_samples: setup уже сделан
 * (shade_samples_fill_with_ps), откатов нет по определению режима. */
static int shader_fast_samples(Render *re, ShadeSample *ssamp, const float *co, const float *vn)
{
	ShadeInput *shi = ssamp->shi;
	ShadeResult *shr = ssamp->shr;
	int samp;

	for (samp = 0; samp < ssamp->tot; samp++, shi++, shr++) {
		float col[3];

		memset(shr, 0, sizeof(ShadeResult));

		if (!shader_fast_shade_point(re, shi, co, vn, col, 0))
			return -1;

		/* хвост shade_input_do_shade() без нод/mist/rayhits */
		shr->alpha = 1.0f;      /* прозрачность в быстром пути не считается */
		shi->alpha = 1.0f;
		shr->combined[0] = col[0];
		shr->combined[1] = col[1];
		shr->combined[2] = col[2];
		shr->combined[3] = 1.0f;
		copy_v3_v3(shr->diff, col);
		shr->z = -shi->co[2];

		/* ★ R3: наполняем КАДРОВЫЙ G-буфер. Позиция, нормаль и вид — те же
		 * величины, что уходят в освещение; i считается ТОЙ ЖЕ формулой, что
		 * в shader_fast_mirror (ray_mirror * fresnel), чтобы экранное
		 * отражение смешивалось с плоским цветом так же, как лучевое.
		 * Сам резолв идёт позже, когда все части кадра уже собраны. */
		{
			float mi = 0.0f;
			float mirc[3] = { 0.0f, 0.0f, 0.0f };

			if (shi->mat && (shi->mat->mode & MA_RAYMIRROR)) {
				mi = shi->mat->ray_mirror *
				     fresnel_fac(shi->view, shi->vn, shi->mat->fresnel_mir_i, shi->mat->fresnel_mir);
				mirc[0] = shi->mat->mirr;
				mirc[1] = shi->mat->mirg;
				mirc[2] = shi->mat->mirb;
			}

			RE_fast_frame_store(shi->xs, shi->ys, co, shi->vn, shi->view, mi, mirc);
		}
	}

	return 1;
}

/*
 * Шейдит сэмплы своим шейдером, а что не умеет — отдаёт BI.
 *
 * Семантика как у shade_samples(): возвращает 1, если что-то нарисовано,
 * 0 — если шейдить нечего. Setup (shade_samples_fill_with_ps) делается
 * ровно один раз при любом исходе.
 *
 * co/vn — G-буфер части для этого пикселя; why — необязательная причина
 * отката (можно NULL).
 */
int RE_shader_shade_samples(Render *re, ShadeSample *ssamp, PixStr *ps, int x, int y,
                           const float *co, const float *vn, const char **why)
{
	ShadeInput *shi;
	ShadeResult *shr;
	const char *bad;
	int samp;

	if (why) *why = NULL;

	shade_samples_fill_with_ps(ssamp, ps, x, y);

	if (!ssamp->tot) return 0;

	shi = ssamp->shi;
	shr = ssamp->shr;

	/* пассы: берём пиксель только если запрошен COMBINED и ничего сверх Z */
	if (!(ssamp->shi[0].passflag & SCE_PASS_COMBINED) ||
	    (ssamp->shi[0].passflag & ~SHADER_PASS_OK))
	{
		if (why) *why = "unsupported pass";
		return shade_samples_shade(ssamp);
	}

	/* ★ БЫСТРЫЙ ПУТЬ: setup уже сделан (он один и тот же у обоих путей —
	 * именно он резолвит obi/facenr -> vlr/material и раскладывает co/vn/uv).
	 * Дальше откатов нет по определению режима: неподдержанное приближается,
	 * а не отдаётся BI. Запрос пассов сверх COMBINED|Z обработан выше и
	 * по-прежнему отдаётся BI — иначе мы бы «рисовали» незаполненные пассы. */
	if (RE_shader_fast()) {
		if (shader_fast_samples(re, ssamp, co, vn) >= 0)
			return 1;
		if (why) *why = "fast: shade failed";
		return shade_samples_shade(ssamp);
	}

	bad = shader_unsupported(re, shi);
	if (bad) {
		if (why) *why = bad;
		return shade_samples_shade(ssamp);
	}

	for (samp = 0; samp < ssamp->tot; samp++, shi++, shr++) {
		RE_MaterialVals mv;
		float col[3];

		memset(shr, 0, sizeof(ShadeResult));

		/* ★ ШАГ B3-iii: свой разбор материала вместо
		 * shade_input_init_material(). Читаем те же поля Material напрямую,
		 * поэтому shi->r/g/b, shi->refl, shi->ambr/g/b больше не нужны. */
		shader_material_values(shi->mat, &mv);

		if (!shader_light(re, shi, &mv, co, vn, col, why))
			return shade_samples_shade(ssamp);   /* лампа не наша — пиксель целиком BI */

		/* ★ ШАГ B4: зеркальный материал — отражение своим лучом и своим
		 * шейдером точки попадания (BI-рейтрейс не вызывается). */
		if (shi->mat->mode & MA_RAYMIRROR) {
			if (!shader_mirror(re, shi, co, vn, col, why))
				return shade_samples_shade(ssamp);
		}

		/* хвост shade_input_do_shade() без нод/mist/rayhits */
		shr->alpha = 1.0f;
		shi->alpha = 1.0f;
		shr->combined[0] = col[0];
		shr->combined[1] = col[1];
		shr->combined[2] = col[2];
		shr->combined[3] = 1.0f;
		copy_v3_v3(shr->diff, col);
		shr->z = -shi->co[2];
	}

	return 1;
}

/* Свой цвет для одного пикселя (без прохода через ShadeSample). Оставлено
 * как отдельная точка входа: удобно для точечной сверки с BI. */
bool RE_shader_lambert(Render *re, ShadeInput *shi, const float co[3], const float vn[3],
                       float out[3], const char **why)
{
	const char *bad;
	RE_MaterialVals mv;

	if (why) *why = NULL;
	if (!re || !shi || !co || !vn || !out) return false;

	bad = shader_unsupported(re, shi);
	if (bad) {
		if (why) *why = bad;
		return false;
	}

	shader_material_values(shi->mat, &mv);

	return shader_light(re, shi, &mv, co, vn, out, why);
}
