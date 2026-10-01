/*
 * Edit Poly modifier operator.
 */

#include "MEM_guardedalloc.h"

#include "DNA_modifier_types.h"
#include "DNA_object_types.h"

#include "BKE_context.h"
#include "BKE_DerivedMesh.h"
#include "BKE_depsgraph.h"     /* DAG_id_tag_update */
#include "BKE_mesh.h"
#include "BKE_modifier.h"

#include "ED_object.h"         /* ED_operator_object_active_editable */
#include "ED_screen.h" 

#include "WM_api.h"
#include "WM_types.h"

#include "RNA_access.h"

static int editpoly_reset_exec(bContext *C, wmOperator *UNUSED(op))
{
	Object *ob = CTX_data_active_object(C);
	ModifierData *md;

	if (ob == NULL || ob->type != OB_MESH) return OPERATOR_CANCELLED;

	for (md = ob->modifiers.first; md; md = md->next) {
		if (md->type == eModifierType_EditPoly) {
			EditPolyModifierData *epmd = (EditPolyModifierData *)md;

			if (epmd->edit_mesh) {
				BKE_mesh_free(epmd->edit_mesh);
				MEM_freeN(epmd->edit_mesh);
				epmd->edit_mesh = NULL;
			}

			epmd->flag &= ~EDITPOLY_INITIALIZED;

			DAG_id_tag_update(&ob->id, OB_RECALC_DATA);
			WM_event_add_notifier(C, NC_OBJECT | ND_MODIFIER, ob);

			return OPERATOR_FINISHED;
		}
	}

	return OPERATOR_CANCELLED;
}

void OBJECT_OT_editpoly_reset(wmOperatorType *ot)
{
	ot->name = "Reset Edit Poly";
	ot->description = "Reset Edit Poly cached geometry";
	ot->idname = "OBJECT_OT_editpoly_reset";
	ot->exec = editpoly_reset_exec;
	ot->poll = ED_operator_object_active_editable;
	ot->flag = OPTYPE_REGISTER | OPTYPE_UNDO;
}
