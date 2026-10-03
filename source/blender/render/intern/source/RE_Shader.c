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
#include "RE_Rasterizer.h"

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

bool RE_shader_enabled(void)
{
	static int cached = -1;

	if (cached < 0) cached = getenv("UBF_SHADER") ? 1 : 0;
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
