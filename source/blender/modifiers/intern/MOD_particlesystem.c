/*
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software  Foundation,
 * Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 *
 * The Original Code is Copyright (C) 2005 by the Blender Foundation.
 * All rights reserved.
 */

/** \file blender/modifiers/intern/MOD_particlesystem.c
 *  \ingroup modifiers
 */


#include <stddef.h>
#include <stdlib.h>

#include "DNA_material_types.h"
#include "DNA_mesh_types.h"
#include "DNA_particle_types.h"
#include "DNA_scene_types.h"

#include "BLI_utildefines.h"


#include "BKE_cdderivedmesh.h"
#include "BKE_global.h"
#include "BKE_modifier.h"
#include "BKE_particle.h"

#include "MOD_util.h"


static void initData(ModifierData *md)
{
	ParticleSystemModifierData *psmd = (ParticleSystemModifierData *) md;
	psmd->psys = NULL;
	psmd->dm_final = NULL;
	psmd->dm_deformed = NULL;
	psmd->totdmvert = psmd->totdmedge = psmd->totdmface = 0;
}
static void freeData(ModifierData *md)
{
	ParticleSystemModifierData *psmd = (ParticleSystemModifierData *) md;

	if (psmd->dm_final) {
		psmd->dm_final->needsFree = true;
		psmd->dm_final->release(psmd->dm_final);
		psmd->dm_final = NULL;
		if (psmd->dm_deformed) {
			psmd->dm_deformed->needsFree = true;
			psmd->dm_deformed->release(psmd->dm_deformed);
			psmd->dm_deformed = NULL;
		}
	}
	psmd->totdmvert = psmd->totdmedge = psmd->totdmface = 0;

	/* ED_object_modifier_remove may have freed this first before calling
	 * modifier_free (which calls this function) */
	if (psmd->psys)
		psmd->psys->flag |= PSYS_DELETE;
}

static void copyData(const ModifierData *md, ModifierData *target)
{
#if 0
	const ParticleSystemModifierData *psmd = (const ParticleSystemModifierData *) md;
#endif
	ParticleSystemModifierData *tpsmd = (ParticleSystemModifierData *) target;

	modifier_copyData_generic(md, target);

	tpsmd->dm_final = NULL;
	tpsmd->dm_deformed = NULL;
	tpsmd->totdmvert = tpsmd->totdmedge = tpsmd->totdmface = 0;
}

static CustomDataMask requiredDataMask(Object *UNUSED(ob), ModifierData *md)
{
	ParticleSystemModifierData *psmd = (ParticleSystemModifierData *) md;
	return psys_emitter_customdata_mask(psmd->psys);
}

/* ★ БЫСТРЫЙ ПУТЬ (rasterizer_mode = FAST): системы частиц не пересчитываются.
 *
 * Замер на боевой сцене: particle_system_update() стоит 1.48 с (Plane.084,
 * 100000 волос) + 0.25 с (Plane.083, 50000 волос) НА КАЖДЫЙ КАДР. При этом в
 * сетку результат не попадает: у Plane.084 после пересчёта всё те же 15 вершин
 * и 12 граней — волосы живут в pathcache и нужны только отрисовке страндов
 * (BI), а быстрый путь странды не рисует вообще (проверено: см. ниже).
 *
 * Контрольный опыт: снять два модификатора PARTICLE_SYSTEM на боевой сцене —
 * кадр 3.530 с -> 1.357 с, картинка отличается на 1 пиксель из 76800
 * (mean=0.0007). То есть быстрый путь платил 1.6 с за волосы, которых у него
 * на картинке нет.
 *
 * Выключатель: UBF_FAST_PSYS=0 (вернуть пересчёт частиц).
 * Битовый путь не затронут: там rasterizer_mode != FAST, поведение прежнее.
 *
 * Цена решения (честно): пока в режиме FAST не рисуются странды, это чистая
 * экономия; когда быстрый путь научится рисовать волосы, этот пропуск надо
 * будет снять вместе с отрисовкой. И ещё: hair dynamics в FAST-рендере не
 * продвигается — на картинку не влияет (волос не видно), но помнить об этом
 * надо, если рендер делить между BI и FAST.
 *
 * ПОЧЕМУ ТОЛЬКО ВОЛОСЫ (PART_HAIR): particle_system_update() двигает ЛЮБУЮ
 * систему частиц, в том числе эмиттерные, а те дают видимую геометрию —
 * инстансы объектов (ParticleSettings.dupli_object) и частицы как меш. Пропуск
 * для них убрал бы геометрию из кадра. Волосы же в сетку не превращаются
 * (замерено: 15 вершин), их результат живёт в pathcache и нужен только
 * отрисовке страндов, которой у быстрого пути нет. */
static bool fast_skip_psys_update(Scene *scene, ParticleSystem *psys)
{
	static int env = -1;

	if (env < 0) {
		const char *e = getenv("UBF_FAST_PSYS");
		env = (e && e[0] == '0') ? 0 : 1;
	}
	if (!env) return false;
	if (!scene || !psys || !psys->part) return false;
	if (psys->part->type != PART_HAIR) return false;

	return (scene->r.rasterizer_mode == RE_RASTERIZER_FAST);
}

/* saves the current emitter state for a particle system and calculates particles */
static void deformVerts(
        ModifierData *md, Object *ob,
        DerivedMesh *derivedData,
        float (*vertexCos)[3],
        int UNUSED(numVerts),
        ModifierApplyFlag flag)
{
	DerivedMesh *dm = derivedData;
	ParticleSystemModifierData *psmd = (ParticleSystemModifierData *) md;
	ParticleSystem *psys = NULL;
	bool needsFree = false;
	/* float cfra = BKE_scene_frame_get(md->scene); */  /* UNUSED */

	if (ob->particlesystem.first)
		psys = psmd->psys;
	else
		return;

	if (!psys_check_enabled(ob, psys, (flag & MOD_APPLY_RENDER) != 0))
		return;

	if (dm == NULL) {
		dm = get_dm(ob, NULL, NULL, vertexCos, false, true);

		if (!dm)
			return;

		needsFree = true;
	}

	/* clear old dm */
	if (psmd->dm_final) {
		psmd->dm_final->needsFree = true;
		psmd->dm_final->release(psmd->dm_final);
		if (psmd->dm_deformed) {
			psmd->dm_deformed->needsFree = 1;
			psmd->dm_deformed->release(psmd->dm_deformed);
			psmd->dm_deformed = NULL;
		}
	}
	else if (psmd->flag & eParticleSystemFlag_file_loaded) {
		/* in file read dm just wasn't saved in file so no need to reset everything */
		psmd->flag &= ~eParticleSystemFlag_file_loaded;
	}
	else {
		/* no dm before, so recalc particles fully */
		psys->recalc |= PSYS_RECALC_RESET;
	}

	/* make new dm */
	psmd->dm_final = CDDM_copy(dm);
	CDDM_apply_vert_coords(psmd->dm_final, vertexCos);
	CDDM_calc_normals(psmd->dm_final);

	if (needsFree) {
		dm->needsFree = true;
		dm->release(dm);
	}

	/* protect dm */
	psmd->dm_final->needsFree = false;

	DM_ensure_tessface(psmd->dm_final);

	if (!psmd->dm_final->deformedOnly) {
		/* XXX Think we can assume here that if current DM is not only-deformed, ob->deformedOnly has been set.
		 *     This is awfully weak though. :| */
		if (ob->derivedDeform) {
			psmd->dm_deformed = CDDM_copy(ob->derivedDeform);
		}
		else {  /* Can happen in some cases, e.g. when rendering from Edit mode... */
			psmd->dm_deformed = CDDM_from_mesh((Mesh *)ob->data);
		}
		DM_ensure_tessface(psmd->dm_deformed);
	}

	/* report change in mesh structure */
	if (psmd->dm_final->getNumVerts(psmd->dm_final) != psmd->totdmvert ||
	    psmd->dm_final->getNumEdges(psmd->dm_final) != psmd->totdmedge ||
	    psmd->dm_final->getNumTessFaces(psmd->dm_final) != psmd->totdmface)
	{
		psys->recalc |= PSYS_RECALC_RESET;

		psmd->totdmvert = psmd->dm_final->getNumVerts(psmd->dm_final);
		psmd->totdmedge = psmd->dm_final->getNumEdges(psmd->dm_final);
		psmd->totdmface = psmd->dm_final->getNumTessFaces(psmd->dm_final);
	}

	if (!(ob->transflag & OB_NO_PSYS_UPDATE)) {
		psmd->flag &= ~eParticleSystemFlag_psys_updated;
		/* ★ БЫСТРЫЙ ПУТЬ: см. fast_skip_psys_update() выше. */
		if (!fast_skip_psys_update(md->scene, psys)) {
			particle_system_update(G.main, md->scene, ob, psys, (flag & MOD_APPLY_RENDER) != 0);
			psmd->flag |= eParticleSystemFlag_psys_updated;
		}
	}
}

/* disabled particles in editmode for now, until support for proper derivedmesh
 * updates is coded */
#if 0
static void deformVertsEM(
        ModifierData *md, Object *ob, EditMesh *editData,
        DerivedMesh *derivedData, float (*vertexCos)[3], int numVerts)
{
	DerivedMesh *dm = derivedData;

	if (!derivedData) dm = CDDM_from_editmesh(editData, ob->data);

	deformVerts(md, ob, dm, vertexCos, numVerts);

	if (!derivedData) dm->release(dm);
}
#endif


ModifierTypeInfo modifierType_ParticleSystem = {
	/* name */              "ParticleSystem",
	/* structName */        "ParticleSystemModifierData",
	/* structSize */        sizeof(ParticleSystemModifierData),
	/* type */              eModifierTypeType_OnlyDeform,
	/* flags */             eModifierTypeFlag_AcceptsMesh |
	                        eModifierTypeFlag_SupportsMapping |
	                        eModifierTypeFlag_UsesPointCache /* |
	                        eModifierTypeFlag_SupportsEditmode |
	                        eModifierTypeFlag_EnableInEditmode */,

	/* copyData */          copyData,
	/* deformVerts */       deformVerts,
	/* deformVertsEM */     NULL,
	/* deformMatrices */    NULL,
	/* deformMatricesEM */  NULL,
	/* applyModifier */     NULL,
	/* applyModifierEM */   NULL,
	/* initData */          initData,
	/* requiredDataMask */  requiredDataMask,
	/* freeData */          freeData,
	/* isDisabled */        NULL,
	/* updateDepgraph */    NULL,
	/* updateDepsgraph */   NULL,
	/* dependsOnTime */     NULL,
	/* dependsOnNormals */	NULL,
	/* foreachObjectLink */ NULL,
	/* foreachIDLink */     NULL,
	/* foreachTexLink */    NULL,
};
