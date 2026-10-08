/*
 * Edit Poly modifier operators: history reset.
 */

#include "MEM_guardedalloc.h"

#include "BLI_string.h"
#include "BLI_utildefines.h"

#include "DNA_mesh_types.h"
#include "DNA_modifier_types.h"
#include "DNA_object_types.h"

#include "BKE_context.h"
#include "BKE_DerivedMesh.h"
#include "BKE_depsgraph.h"     /* DAG_id_tag_update */
#include "BKE_editpoly.h"
#include "BKE_mesh.h"
#include "BKE_modifier.h"
#include "BKE_report.h"

#include "ED_object.h"         /* ED_operator_object_active_editable */
#include "ED_screen.h"

#include "WM_api.h"
#include "WM_types.h"

#include "RNA_access.h"

static int editpoly_reset_exec(bContext *C, wmOperator *op)
{
	Object *ob = CTX_data_active_object(C);
	EditPolyModifierData *epmd;

	if (ob == NULL || ob->type != OB_MESH) {
		return OPERATOR_CANCELLED;
	}

	if (editpoly_object_is_in_editmode(ob)) {
		BKE_report(op->reports, RPT_ERROR, "Leave the Edit Poly edit mode first");
		return OPERATOR_CANCELLED;
	}

	epmd = editpoly_modifier_find_active(ob);

	if (epmd == NULL) {
		BKE_report(op->reports, RPT_ERROR, "Active object has no Edit Poly modifier");
		return OPERATOR_CANCELLED;
	}

	editpoly_ops_free(epmd);
	epmd->flag &= ~EDITPOLY_HAS_OPS;
	epmd->flag &= ~EDITPOLY_INPUT_MISMATCH;
	/* the input becomes the new reference when the history is recorded again */
	epmd->input_vert_count = 0;
	epmd->input_edge_count = 0;
	epmd->input_face_count = 0;

	DAG_id_tag_update(&ob->id, OB_RECALC_DATA);
	WM_event_add_notifier(C, NC_OBJECT | ND_MODIFIER, ob);

	return OPERATOR_FINISHED;
}

/** Interactive use asks first: the history is not recoverable. */
static int editpoly_reset_invoke(bContext *C, wmOperator *op, const wmEvent *UNUSED(event))
{
	Object *ob = CTX_data_active_object(C);
	EditPolyModifierData *epmd;
	char message[256];

	if (ob == NULL || ob->type != OB_MESH) {
		return editpoly_reset_exec(C, op);
	}

	epmd = editpoly_modifier_find_active(ob);

	/* nothing to confirm when there is nothing to clear */
	if (epmd == NULL || editpoly_ops_count(epmd) == 0) {
		return editpoly_reset_exec(C, op);
	}

	BLI_snprintf(message, sizeof(message),
	             "Clear %d operation(s) of the Edit Poly history of '%s'?",
	             editpoly_ops_count(epmd), ob->id.name + 2);

	return WM_operator_confirm_message(C, op, message);
}

void OBJECT_OT_editpoly_reset(wmOperatorType *ot)
{
	ot->name = "Reset Edit Poly";
	ot->description = "Clear the Edit Poly operation history";
	ot->idname = "OBJECT_OT_editpoly_reset";
	ot->exec = editpoly_reset_exec;
	ot->invoke = editpoly_reset_invoke;
	ot->poll = ED_operator_object_active_editable;
	ot->flag = OPTYPE_REGISTER | OPTYPE_UNDO;
}
