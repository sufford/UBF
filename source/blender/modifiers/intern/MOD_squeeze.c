/*
 * MOD_squeeze.c - Faithful 3ds Max Style Squeeze
 */

#include "MEM_guardedalloc.h"

#include "BLI_math.h"
#include "BLI_utildefines.h"

#include "DNA_mesh_types.h"
#include "DNA_meshdata_types.h"
#include "DNA_modifier_types.h"
#include "DNA_object_types.h"

#include "BKE_cdderivedmesh.h"
#include "BKE_modifier.h"

#include "MOD_modifiertypes.h"

static void deformVerts(
    ModifierData *md, 
    Object *ob,
    DerivedMesh *derivedData,
    float (*vertexCos)[3],
    int numVerts,
    ModifierApplyFlag UNUSED(flag))
{
	SqueezeModifierData *smd = (SqueezeModifierData *)md;
	
	if (smd->amount == 0.0f && smd->radial_curve == 0.0f && 
	    smd->axial_amount == 0.0f && smd->axial_curve == 0.0f) return;

	int axis = smd->axis;

	/* Calculate object height along spine axis */
	float min_val = FLT_MAX;
	float max_val = -FLT_MAX;
	for (int i = 0; i < numVerts; i++) {
		float pos = vertexCos[i][axis];
		if (pos < min_val) min_val = pos;
		if (pos > max_val) max_val = pos;
	}

	float total_height = max_val - min_val;
	if (total_height <= 0.0001f) return;

		for (int i = 0; i < numVerts; i++) {
		float orig_x = vertexCos[i][0];
		float orig_y = vertexCos[i][1];
		float orig_z = vertexCos[i][2];
		
		/* Position along spine axis, normalized 0..1 (0=bottom, 1=top) */
		float distance = vertexCos[i][axis];
		float factor = (distance - min_val) / total_height;
		
		/* Remap to -1..1 where 0 is CENTER of object */
		float centered = (factor * 2.0f) - 1.0f;
		
		/* Base profile waves: peak at the center equator and fade to 0 at the tips */
		float center_profile = cosf(centered * (float)M_PI_2);
		float edge_profile = 1.0f - (centered * centered);

		/* === 1. AXIAL BULGE (Direct Multiplicative Height Scale) === */
		/*
		 * FIX: To eliminate the flat trash-can tops and barrel artifacts permanently,
		 * we use direct multiplication instead of a linear displacement offset!
		 * The scaling factor uses center_profile so the stretch peaks at the equator 
		 * and gracefully tapers down to 1.0 at the poles, keeping the caps fully closed.
		 */
		float axial_scale = 1.0f;
		if (smd->axial_amount != 0.0f) {
			float curve_exp = 1.0f + fabsf(smd->axial_curve);
			float profile = powf(fabsf(center_profile), curve_exp);
			
			/* Convert slider value (e.g. 20.0) into a clean, smooth percentage scalar */
			axial_scale = 1.0f + (smd->axial_amount * 0.01f * profile);
		}
		
		/* === 2. RADIAL SQUEEZE (Uniform Thickness Profile Scale) === */
		/*
		 * To achieve a true organic egg ellipsoid/rugby ball instead of a flat tube,
		 * the horizontal cross-sections must contract proportionally to the height stretch.
		 */
		float radial_scale = 1.0f;
		if (smd->axial_amount > 0.0f) {
			/* As the axis elongates, the sides curve inward smoothly toward the ends */
			radial_scale -= (smd->axial_amount * 0.012f * (centered * centered));
		}
		
		/* Apply the manual Radial Squeeze sliders on top of the base map */
		if (smd->amount != 0.0f) {
			float curve_exp = 1.0f + fabsf(smd->radial_curve);
			float profile = powf(fabsf(center_profile), curve_exp);
			
			radial_scale -= (smd->amount * profile * edge_profile);
		}
		
		/* === 3. VOLUME PRESERVE CHECKBOX === */
		if (smd->flag & MOD_SQUEEZE_VOLUME) {
			float volume_compensation = 1.0f - (fabsf(axial_scale - 1.0f) * 0.3f);
			radial_scale *= volume_compensation;
		}
		
		if (radial_scale < 0.0f) radial_scale = 0.0f;
		
		/* Apply transformations using clean, non-interfering multipliers */
		switch (axis) {
			case 0: /* Spine X */
				vertexCos[i][0] = orig_x * axial_scale;
				vertexCos[i][1] = orig_y * radial_scale;
				vertexCos[i][2] = orig_z * radial_scale;
				break;
			case 1: /* Spine Y */
				vertexCos[i][0] = orig_x * radial_scale;
				vertexCos[i][1] = orig_y * axial_scale;
				vertexCos[i][2] = orig_z * radial_scale;
				break;
			case 2: /* Spine Z */
			default:
				vertexCos[i][0] = orig_x * radial_scale;
				vertexCos[i][1] = orig_y * radial_scale;
				vertexCos[i][2] = orig_z * axial_scale;
				break;
		}
	}

}

static void initData(ModifierData *md) {
	SqueezeModifierData *smd = (SqueezeModifierData *)md;
	smd->amount = 0.0f;
	smd->radial_curve = 0.0f;
	smd->axial_amount = 0.0f;
	smd->axial_curve = 0.0f;
	smd->axis = 2;
	smd->flag = 0;
	smd->_pad = 0;
	smd->_pad2 = 0;
}

ModifierTypeInfo modifierType_Squeeze = {
	/* name */              "Squeeze",
	/* structName */        "SqueezeModifierData",
	/* structSize */        sizeof(SqueezeModifierData),
	/* type */              eModifierTypeType_OnlyDeform,
	/* flags */             eModifierTypeFlag_AcceptsMesh |
	                        eModifierTypeFlag_SupportsEditmode |
	                        eModifierTypeFlag_SupportsMapping,
	
	/* copyData */          NULL,
	/* deformVerts */       deformVerts,
	/* deformMatrices */    NULL,
	/* deformVertsEM */     deformVerts,
	/* deformMatricesEM */  NULL,
	/* applyModifier */     NULL,
	/* applyModifierEM */   NULL,
	/* initData */          initData,
	/* requiredDataMask */  NULL,
	/* freeData */          NULL,
	/* isDisabled */        NULL,
	/* updateDepgraph */    NULL,
	/* updateDepsgraph */   NULL,
	/* dependsOnTime */     NULL,
	/* dependsOnNormals */  NULL,
	/* foreachObjectLink */ NULL,
	/* foreachIDLink */     NULL,
	/* foreachTexLink */    NULL,
};