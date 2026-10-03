/*
 * RE_Mirror.h — своё отражение mirror-материала (шаг B4, «вариант B»).
 *
 * Своё: вектор отражения, обход сцены и поиск попадания. BI-рейтрейс
 * (ray_trace/trace_reflect/traceray/RE_rayobject_raycast) для поддержанных
 * mirror-материалов не вызывается.
 */

#ifndef __RE_MIRROR_H__
#define __RE_MIRROR_H__

#ifdef __cplusplus
extern "C" {
#endif

struct Render;
struct ShadeInput;
struct ObjectInstanceRen;
struct VlakRen;

/* Включено ли своё отражение (UBF_MIRROR=1). */
int RE_mirror_enabled(void);

/* Попадание своего луча. */
typedef struct RE_MirrorHit {
	struct ObjectInstanceRen *obi;
	struct VlakRen *vlr;
	float dist;        /* расстояние вдоль dir (лямбда), как у BI */
	float uv[2];       /* барицентрика в виде BI (то есть -uv из BLI) */
	int tri;           /* 1 — первый треугольник квада, 2 — второй */
} RE_MirrorHit;

/*
 * Своё пересечение: ближайшее попадание вдоль dir из start.
 *
 * orig_obi/orig_vlr — грань, из которой пущен луч: её саму и её соседей
 * (общая вершина, dist < 0.1) пропускаем по тем же правилам, что BI.
 * maxdist — предел (у BI это dist_mir либо RE_RAYTRACE_MAXDIST).
 * Возвращает 1 при попадании, иначе 0; hit заполняется только при 1.
 */
int RE_mirror_cast(struct Render *re,
                   struct ObjectInstanceRen *orig_obi, struct VlakRen *orig_vlr,
                   const float start[3], const float dir[3], float maxdist,
                   RE_MirrorHit *hit);

/*
 * Своя точка входа: считает вектор отражения (reflection() для гладкой грани,
 * reflection_simple() иначе) и пускает свой луч. dir_out — посчитанный
 * вектор отражения (для сверки с BI). Возвращает 1 при попадании.
 */
int RE_mirror_trace(struct Render *re, struct ShadeInput *shi,
                    const float co[3], const float vn[3],
                    RE_MirrorHit *hit, float dir_out[3]);

#ifdef __cplusplus
}
#endif

#endif /* __RE_MIRROR_H__ */
