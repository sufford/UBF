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
#ifndef __BKE_EDITPOLY_H__
#define __BKE_EDITPOLY_H__

/** \file BKE_editpoly.h
 *  \ingroup bke
 *
 *  Edit Poly modifier: runtime state and history evaluation.
 */

#include "BLI_sys_types.h"

#include "DNA_listBase.h"
#include "DNA_modifier_types.h"

struct BMesh;
struct DerivedMesh;
struct Main;
struct Mesh;
struct Object;
struct Scene;

/**
 * Runtime data of an Edit Poly modifier. Never saved to a .blend file,
 * it only exists while the object is in the Edit Poly edit mode.
 * Stored in a global hash table (see editpoly_runtime.c).
 */
typedef struct EditPolyRuntime {
	struct Object *ob;
	struct EditPolyModifierData *epmd;
	struct Mesh *edit_mesh;         /* geometry being edited, temporary ob->data */
	struct Mesh *saved_ob_data;     /* ob->data before entering the mode */
	struct BMesh *bm;               /* bmesh owned by edit_mesh */
	void *snapshot;                 /* EditPolySnapshot *, internal to editpoly_record.c */
	/** How to free the snapshot (set by editpoly_record.c). The runtime owns it, so
	 *  it must be freed even when the object disappears without leaving the mode. */
	void (*snapshot_free)(void *snapshot);

	ListBase saved_modes;           /* EditPolyModState, original modifier flags */
	int flag;                       /* EDITPOLY_RT_* */
	int next_id;                    /* next free element id */

	/* CustomData layer offsets of "editpoly_id" in bm, -1 when missing. */
	int cd_id_vert;
	int cd_id_edge;
	int cd_id_face;
} EditPolyRuntime;

enum {
	EDITPOLY_RT_EDITMODE = (1 << 0),
};

/* ---------- editpoly_runtime.c ---------- */

void             editpoly_runtime_init(void);
EditPolyRuntime *editpoly_runtime_get(struct Object *ob);
EditPolyRuntime *editpoly_runtime_first(void);

/** The object whose Edit Poly session is active (at most one can exist), or NULL. */
struct Object *editpoly_session_object(void);
/** True while any Edit Poly session is active, i.e. while an object data is the
 *  temporary edit mesh (which is not part of the main database). */
bool editpoly_session_active(void);
EditPolyRuntime *editpoly_runtime_add(struct Object *ob, struct EditPolyModifierData *epmd,
                                      struct Mesh *edit_mesh, struct Mesh *saved_ob_data);
void             editpoly_runtime_free(struct Object *ob);
void             editpoly_runtime_free_all(void);
/** The object is being deleted while its Edit Poly session is open: put the
 *  original object data back, free the temporary edit mesh (it is not owned by
 *  any main database) and drop the runtime. Called from BKE_object_free(), so it
 *  must not call into the editors. */
void             editpoly_runtime_object_free(struct Object *ob);

/* Save (and temporarily disable) the whole modifier stack, restore it later. */
void editpoly_runtime_save_modes(EditPolyRuntime *rt, struct Object *ob);
void editpoly_runtime_restore_modes(EditPolyRuntime *rt, struct Object *ob);

/* ---------- helpers ---------- */

struct EditPolyModifierData *editpoly_modifier_find(struct Object *ob);
/** The modifier the edit mode works on (see EDITPOLY_ACTIVE). When no modifier is
 *  flagged yet the first one is flagged and used, so the default is "mesh 1". */
struct EditPolyModifierData *editpoly_modifier_find_active(struct Object *ob);
struct EditPolyModifierData *editpoly_modifier_find_nth(struct Object *ob, int index);
/** Mark the index-th Edit Poly modifier as the active one, returns it or NULL. */
struct EditPolyModifierData *editpoly_modifier_set_active(struct Object *ob, int index);
/** Number of Edit Poly modifiers on the object. */
int  editpoly_modifier_count(struct Object *ob);
/** Index of the active Edit Poly modifier (0 when nothing is flagged yet). */
int  editpoly_modifier_active_index(struct Object *ob);
bool editpoly_object_is_in_editmode(struct Object *ob);

/* ---------- MOD_editpoly_ops.c ---------- */

/** Element ids, stored in a "editpoly_id" int layer of the bmesh.
 *  They are the only way an operation can name elements, because the order of
 *  bmesh elements is not stable across the operators. */
bool editpoly_ids_offsets(struct BMesh *bm, int *r_vert, int *r_edge, int *r_face);
void editpoly_ids_ensure(struct BMesh *bm);
int  editpoly_ids_max(struct BMesh *bm);

struct Mesh *editpoly_build_mesh(struct Main *bmain, struct Scene *scene, struct Object *ob,
                                 struct EditPolyModifierData *epmd);
/** Build the geometry the history is applied to out of a derived mesh. */
struct Mesh *editpoly_mesh_from_derived(struct Object *ob, struct DerivedMesh *dm);
/** Replay the whole history of \a epmd on \a me (ids are assigned/reused). */
void editpoly_mesh_apply_history(struct Mesh *me, struct EditPolyModifierData *epmd);
/** Drop the element ids, they must not leak into the evaluated geometry. */
void editpoly_mesh_ids_free(struct Mesh *me);
/** Compare the input topology with the one the history was recorded on and update
 *  the EDITPOLY_INPUT_MISMATCH flag; the history is replayed on \a me before it is
 *  called. Returns whether the history no longer matches the input. */
bool editpoly_input_check(struct Mesh *me, struct EditPolyModifierData *epmd);
void editpoly_apply_history(struct BMesh *bm, struct EditPolyModifierData *epmd);
void editpoly_apply_op(struct BMesh *bm, const struct EditPolyOp *op);

/* ---------- MOD_editpoly.c ---------- */

void editpoly_ops_copy(struct EditPolyModifierData *epmd, struct EditPolyModifierData *target);
void editpoly_ops_free(struct EditPolyModifierData *epmd);
int  editpoly_ops_count(struct EditPolyModifierData *epmd);
/** Drop the last operation of the history (undo inside the edit mode). */
void editpoly_op_remove_last(struct EditPolyModifierData *epmd);
/** Undo: mark the last active operation as undone instead of removing it, so that
 *  redo (Ctrl+Shift+Z) can bring it back. Returns false when there is nothing to
 *  undo. */
bool editpoly_op_skip_last(struct EditPolyModifierData *epmd);
/** Redo: clear the skip flag of the first undone operation. The undone operations
 *  are always a block at the end of the history. Returns false when there is
 *  nothing to redo. */
bool editpoly_op_unskip_first(struct EditPolyModifierData *epmd);
/** Forget the undone operations, called when a new operation is recorded. */
void editpoly_ops_drop_skipped(struct EditPolyModifierData *epmd);

#endif  /* __BKE_EDITPOLY_H__ */
