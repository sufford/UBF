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
 * along with this program; if not, write to the Free Software Foundation,
 * Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 */

/** \file blender/editors/object/editpoly_mode.c
 *  \ingroup edobj
 *
 *  Entering/leaving the Edit Poly edit mode.
 *
 *  While the mode is active the object data is swapped for a temporary mesh
 *  that contains the modifier input plus the replayed operation history. The
 *  native Blender Edit Mode is then used to edit that mesh.
 */

#include <string.h>

#include "MEM_guardedalloc.h"

#include "BLI_listbase.h"
#include "BLI_utildefines.h"

#include "DNA_mesh_types.h"
#include "DNA_modifier_types.h"
#include "DNA_object_types.h"
#include "DNA_scene_types.h"

#include "BKE_context.h"
#include "BKE_depsgraph.h"
#include "BKE_editmesh.h"
#include "BKE_editpoly.h"
#include "BKE_library.h"
#include "BKE_main.h"
#include "BKE_mesh.h"
#include "BKE_modifier.h"
#include "BKE_object.h"
#include "BKE_report.h"
#include "BKE_scene.h"

#include "ED_object.h"
#include "ED_screen.h"
#include "ED_editpoly.h"

#include "WM_api.h"
#include "WM_types.h"

#include "RNA_access.h"
#include "RNA_define.h"

#include "object_intern.h"

/* -------------------------------------------------------------------- */
/** \name Mode helpers
 * \{ */

/**
 * Put the original object data back and drop the runtime state.
 *
 * Must be called after the native editmode data has been freed (or when there
 * never was any), it frees the temporary edit mesh.
 */
void editpoly_mode_cleanup(Main *bmain, Scene *scene, Object *ob)
{
	EditPolyRuntime *rt = editpoly_runtime_get(ob);
	EditPolyModifierData *epmd;
	Mesh *edit_mesh;

	(void)bmain;
	(void)scene;

	if (rt == NULL) {
		return;
	}

	edit_mesh = rt->edit_mesh;

	/* drop any derived mesh that still points into the temporary mesh */
	BKE_object_free_derived_caches(ob);

	/* the operation history has been synced already, just forget the snapshot */
	editpoly_record_end(ob);

	/* Put the original object data back. The check is a safety net: the object
	 * data must never be left pointing at the temporary mesh, and a mesh that is
	 * still in use must never be freed. */
	if (ob->data == edit_mesh || ob->data == NULL) {
		ob->data = rt->saved_ob_data;
	}

	editpoly_runtime_restore_modes(rt, ob);

	/* the modifier may have been re-created meanwhile, never trust rt->epmd */
	epmd = editpoly_modifier_find_active(ob);

	if (epmd) {
		epmd->flag &= ~EDITPOLY_IN_EDITMODE;
	}

	editpoly_runtime_free(ob);

	if (edit_mesh) {
		BKE_mesh_free(edit_mesh);
		MEM_freeN(edit_mesh);
	}

	DAG_id_tag_update(&ob->id, OB_RECALC_DATA);
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Operators
 * \{ */

/** Report helper: the mode operators are also called internally (see
 *  editpoly_undo_session_step()), where there is no operator to report to. */
static void ep_report(wmOperator *op, ReportType type, const char *message)
{
	BKE_report(op ? op->reports : NULL, type, message);
}

static int editpoly_enter_exec(bContext *C, wmOperator *op)
{
	Main *bmain = CTX_data_main(C);
	Scene *scene = CTX_data_scene(C);
	Object *ob = CTX_data_active_object(C);
	EditPolyModifierData *epmd;
	EditPolyRuntime *rt;
	BMEditMesh *em;
	Mesh *edit_mesh;

	if (ob == NULL) {
		ep_report(op, RPT_ERROR, "No active object");
		return OPERATOR_CANCELLED;
	}

	if (ob->type != OB_MESH) {
		ep_report(op, RPT_ERROR, "Edit Poly can only be used on mesh objects");
		return OPERATOR_CANCELLED;
	}

	epmd = editpoly_modifier_find_active(ob);

	if (epmd == NULL) {
		ep_report(op, RPT_ERROR, "Active object has no Edit Poly modifier");
		return OPERATOR_CANCELLED;
	}

	if (editpoly_object_is_in_editmode(ob)) {
		return OPERATOR_FINISHED;
	}

	if (ob->mode & OB_MODE_EDIT) {
		ep_report(op, RPT_ERROR,
		           "Object is already in Edit Mode, leave it before entering Edit Poly");
		return OPERATOR_CANCELLED;
	}

	if (ob->data == NULL || ((Mesh *)ob->data)->totvert == 0) {
		ep_report(op, RPT_ERROR, "Cannot enter Edit Poly with an empty mesh");
		return OPERATOR_CANCELLED;
	}

	edit_mesh = editpoly_build_mesh(bmain, scene, ob, epmd);

	if (edit_mesh == NULL) {
		ep_report(op, RPT_ERROR, "Could not build the Edit Poly geometry");
		return OPERATOR_CANCELLED;
	}

	/* the history is replayed on the input of the modifier: if the modifiers above
	 * Edit Poly changed the input after the history was recorded, the operations
	 * reference other elements and the result is wrong. Warn instead of showing a
	 * silently corrupted mesh. */
	if (epmd->flag & EDITPOLY_INPUT_MISMATCH) {
		ep_report(op, RPT_WARNING,
		          "Edit Poly history does not match the modifier input "
		          "(the modifiers above it changed). The result will be wrong; "
		          "fix the modifier order or clear the history first.");
	}

	rt = editpoly_runtime_add(ob, epmd, edit_mesh, ob->data);

	/* The modifier stack is intentionally left alone: the modifers that come
	 * before Edit Poly are already baked into the edit mesh and the ones after
	 * it keep working on the geometry being edited (see applyModifier()). */

	ob->data = edit_mesh;
	epmd->flag |= EDITPOLY_IN_EDITMODE;

	if (!ED_object_editmode_enter(C, EM_WAITCURSOR)) {
		/* rollback */
		ob->data = rt->saved_ob_data;
		editpoly_runtime_restore_modes(rt, ob);
		epmd->flag &= ~EDITPOLY_IN_EDITMODE;
		editpoly_runtime_free(ob);

		BKE_mesh_free(edit_mesh);
		MEM_freeN(edit_mesh);

		ep_report(op, RPT_ERROR, "Could not enter Edit Mode");
		return OPERATOR_CANCELLED;
	}

	em = BKE_editmesh_from_object(ob);
	rt->bm = em ? em->bm : NULL;

	/* remember the state every operation of the user is compared against */
	editpoly_record_begin(ob);

	WM_event_add_notifier(C, NC_OBJECT | ND_MODIFIER, ob);
	WM_event_add_notifier(C, NC_SCENE | ND_MODE | NS_EDITMODE_MESH, scene);

	return OPERATOR_FINISHED;
}

/** Leave the Edit Poly mode of one object. The object must be in a session. */
static int editpoly_exit_object(bContext *C, Object *ob, wmOperator *op)
{
	Main *bmain = CTX_data_main(C);
	Scene *scene = CTX_data_scene(C);

	if (ob == NULL || !editpoly_object_is_in_editmode(ob)) {
		ep_report(op, RPT_ERROR, "Not in Edit Poly edit mode");
		return OPERATOR_CANCELLED;
	}

	if (ob->mode & OB_MODE_EDIT) {
		/* this restores the object data through editpoly_mode_cleanup() */
		ED_object_editmode_exit_ex(bmain, scene, ob, EM_FREEDATA);
	}
	else {
		editpoly_mode_cleanup(bmain, scene, ob);
	}

	WM_event_add_notifier(C, NC_OBJECT | ND_MODIFIER, ob);
	WM_event_add_notifier(C, NC_SCENE | ND_MODE | NS_MODE_OBJECT, scene);

	return OPERATOR_FINISHED;
}

static int editpoly_exit_exec(bContext *C, wmOperator *op)
{
	Object *ob = CTX_data_active_object(C);

	if (ob == NULL) {
		ob = CTX_data_edit_object(C);
	}

	return editpoly_exit_object(C, ob, op);
}

/** Leave an active Edit Poly session, whoever started it. Used by the paths that
 *  are not Edit Poly operators, deleting objects for instance. */
bool editpoly_session_exit(bContext *C)
{
	Object *ob = editpoly_session_object();

	if (ob == NULL) {
		return false;
	}

	return editpoly_exit_object(C, ob, NULL) == OPERATOR_FINISHED;
}

/* Toggle, used by the modifier panel button. */
static int editpoly_toggle_exec(bContext *C, wmOperator *op)
{
	Object *ob = CTX_data_active_object(C);

	if (ob == NULL) {
		ep_report(op, RPT_ERROR, "No active object");
		return OPERATOR_CANCELLED;
	}

	if (editpoly_object_is_in_editmode(ob)) {
		return editpoly_exit_exec(C, op);
	}

	return editpoly_enter_exec(C, op);
}

/**
 * Undo and redo inside the Edit Poly edit mode.
 *
 * The native undo system must not run while the mode is active: the object data
 * is the temporary edit mesh, which is not part of the main database, so a
 * memfile step referencing it would restore an object without data. The
 * operation history of the modifier is the undo stack instead: the last
 * operation is dropped and the edited geometry is rebuilt from the modifier
 * input plus the remaining history.
 *
 * Returns true when Edit Poly handled the request, so the native undo steps
 * aside.
 */
bool editpoly_undo_session_step(bContext *C, bool undo)
{
	EditPolyModifierData *epmd;
	EditPolyRuntime *rt;
	Object *ob = CTX_data_edit_object(C);

	if (ob == NULL) {
		ob = CTX_data_active_object(C);
	}

	if (ob == NULL || !editpoly_object_is_in_editmode(ob)) {
		return false;
	}

	rt = editpoly_runtime_get(ob);

	if (rt == NULL) {
		return false;
	}

	epmd = editpoly_modifier_find_active(ob);

	if (epmd == NULL) {
		/* the history is gone, but the native undo must still not run */
		return true;
	}

	/* The changes of the last action may not be recorded yet (the exit path does
	 * it). Flush them first, otherwise the operation that is undone would be the
	 * wrong one, and the pending difference would be recorded afterwards with a
	 * snapshot of a state that no longer exists. */
	editpoly_record_sync(ob, NULL);

	if (!undo) {
		if (!editpoly_op_unskip_first(epmd)) {
			BKE_report(NULL, RPT_INFO, "Edit Poly: nothing to redo in the edit mode");
			return true;
		}
	}
	else {
		if (!editpoly_op_skip_last(epmd)) {
			return true;
		}
	}

	/* Rebuild the edit mesh: leaving the mode restores the object data, entering
	 * it again builds the temporary mesh out of the modifier input plus the
	 * remaining history (the undone operations are kept, but not applied). */
	editpoly_exit_exec(C, NULL);
	editpoly_enter_exec(C, NULL);

	return true;
}

static bool editpoly_enter_poll(bContext *C)
{
	Object *ob = CTX_data_active_object(C);

	if (ob == NULL || ob->type != OB_MESH || ob->data == NULL) {
		return false;
	}

	if (editpoly_object_is_in_editmode(ob)) {
		return false;
	}

	if (ob->mode & OB_MODE_EDIT) {
		return false;
	}

	return editpoly_modifier_find(ob) != NULL;
}

static bool editpoly_exit_poll(bContext *C)
{
	Object *ob = CTX_data_active_object(C);

	return editpoly_object_is_in_editmode(ob);
}

void OBJECT_OT_editpoly_enter(wmOperatorType *ot)
{
	ot->name = "Enter Edit Poly";
	ot->description = "Edit the geometry of this Edit Poly modifier";
	ot->idname = "OBJECT_OT_editpoly_enter";

	ot->exec = editpoly_enter_exec;
	ot->poll = editpoly_enter_poll;

	ot->flag = OPTYPE_REGISTER;
}

void OBJECT_OT_editpoly_exit(wmOperatorType *ot)
{
	ot->name = "Exit Edit Poly";
	ot->description = "Leave the Edit Poly edit mode";
	ot->idname = "OBJECT_OT_editpoly_exit";

	ot->exec = editpoly_exit_exec;
	ot->poll = editpoly_exit_poll;

	ot->flag = OPTYPE_REGISTER;
}

static bool editpoly_toggle_poll(bContext *C)
{
	Object *ob = CTX_data_active_object(C);

	if (ob == NULL || ob->type != OB_MESH || ob->data == NULL) {
		return false;
	}

	return editpoly_object_is_in_editmode(ob) || editpoly_modifier_find(ob) != NULL;
}

void OBJECT_OT_editpoly_toggle(wmOperatorType *ot)
{
	ot->name = "Toggle Edit Poly";
	ot->description = "Enter or leave the Edit Poly edit mode";
	ot->idname = "OBJECT_OT_editpoly_toggle";

	ot->exec = editpoly_toggle_exec;
	ot->poll = editpoly_toggle_poll;

	ot->flag = OPTYPE_REGISTER;
}

/** Select which Edit Poly modifier the edit mode works on.
 *  While the mode is active this is a switch: leave the mode, mark the other
 *  modifier as the active one and enter the mode again. */
static int editpoly_set_active_exec(bContext *C, wmOperator *op)
{
	Object *ob_active = CTX_data_active_object(C);
	Object *ob = ob_active;
	char *ob_name = RNA_string_get_alloc(op->ptr, "object", NULL, 0, NULL);
	int index = RNA_int_get(op->ptr, "index");
	bool in_session = false;
	int count;

	/* the header list works on any selected object, the panel one on the drawn
	 * object, which is the active one */
	if (ob_name != NULL && ob_name[0] != '\0') {
		ob = (Object *)BKE_libblock_find_name(CTX_data_main(C), ID_OB, ob_name);
	}

	if (ob_name != NULL) {
		MEM_freeN(ob_name);
	}

	if (ob == NULL || ob->type != OB_MESH) {
		ep_report(op, RPT_ERROR, "No such mesh object");
		return OPERATOR_CANCELLED;
	}

	count = editpoly_modifier_count(ob);

	if (count == 0) {
		ep_report(op, RPT_ERROR, "Object has no Edit Poly modifier");
		return OPERATOR_CANCELLED;
	}

	if (index < 0 || index >= count) {
		ep_report(op, RPT_ERROR, "No such Edit Poly modifier");
		return OPERATOR_CANCELLED;
	}

	if (ob == ob_active && editpoly_modifier_active_index(ob) == index) {
		/* make sure the flag really points at this modifier ("mesh 1" by default) */
		editpoly_modifier_set_active(ob, index);
		return OPERATOR_FINISHED;
	}

	/* the edit session belongs to one object and one modifier at a time: whenever
	 * the choice changes the mode is left and entered again, because the input
	 * chain of the old choice is already baked into its temporary mesh */
	if (editpoly_object_is_in_editmode(ob_active)) {
		in_session = true;
		editpoly_exit_exec(C, NULL);
	}

	if (ob != ob_active) {
		Base *base = BKE_scene_base_find(CTX_data_scene(C), ob);

		if (base == NULL) {
			ep_report(op, RPT_ERROR, "Object is not in the current scene");
			return OPERATOR_CANCELLED;
		}

		ED_base_object_select(base, BA_SELECT);
		ED_base_object_activate(C, base);
		ob_active = ob;

		WM_event_add_notifier(C, NC_SCENE | ND_OB_SELECT | ND_OB_ACTIVE, CTX_data_scene(C));
	}

	editpoly_modifier_set_active(ob, index);

	if (in_session) {
		if (editpoly_enter_exec(C, NULL) == OPERATOR_CANCELLED) {
			ep_report(op, RPT_ERROR, "Could not switch the active Edit Poly modifier");
			return OPERATOR_CANCELLED;
		}
	}

	DAG_id_tag_update(&ob->id, OB_RECALC_DATA);
	WM_event_add_notifier(C, NC_OBJECT | ND_MODIFIER, ob);
	WM_event_add_notifier(C, NC_SCENE | ND_MODE | NS_EDITMODE_MESH, CTX_data_scene(C));

	return OPERATOR_FINISHED;
}

static bool editpoly_set_active_poll(bContext *C)
{
	Object *ob = CTX_data_active_object(C);

	return (ob != NULL) && (ob->type == OB_MESH) && (editpoly_modifier_count(ob) > 0);
}

void OBJECT_OT_editpoly_set_active(wmOperatorType *ot)
{
	ot->name = "Set Active Edit Poly";
	ot->description = "Choose which Edit Poly modifier the edit mode works on";
	ot->idname = "OBJECT_OT_editpoly_set_active";

	ot->exec = editpoly_set_active_exec;
	ot->poll = editpoly_set_active_poll;

	ot->flag = OPTYPE_REGISTER;

	RNA_def_int(ot->srna, "index", 0, 0, 64, "Index",
	            "Index of the Edit Poly modifier", 0, 64);
	RNA_def_string(ot->srna, "object", NULL, MAX_ID_NAME - 2, "Object",
	               "Object that owns the Edit Poly modifier (empty: the active object)");
}

/** \} */
