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

/** \file blender/blenkernel/intern/editpoly_runtime.c
 *  \ingroup bke
 *
 *  Runtime state of the Edit Poly modifier. Lives in memory only, while an
 *  object is being edited through an Edit Poly modifier.
 */

#include <string.h>

#include "MEM_guardedalloc.h"

#include "BLI_utildefines.h"
#include "BLI_ghash.h"
#include "BLI_listbase.h"

#include "DNA_mesh_types.h"
#include "DNA_modifier_types.h"
#include "DNA_object_types.h"

#include "BKE_editpoly.h"
#include "BKE_editmesh.h"
#include "BKE_mesh.h"
#include "BKE_object.h"

/* Original mode of one modifier, kept while the Edit Poly edit mode is active. */
typedef struct EditPolyModState {
	struct EditPolyModState *next, *prev;
	struct ModifierData *md;
	int mode;
} EditPolyModState;

static GHash *g_editpoly_runtime = NULL;

void editpoly_runtime_init(void)
{
	if (g_editpoly_runtime == NULL) {
		g_editpoly_runtime = BLI_ghash_ptr_new("EditPolyRuntime gh");
	}
}

EditPolyRuntime *editpoly_runtime_get(Object *ob)
{
	if (ob == NULL || g_editpoly_runtime == NULL) {
		return NULL;
	}

	return BLI_ghash_lookup(g_editpoly_runtime, ob);
}

EditPolyRuntime *editpoly_runtime_first(void)
{
	GHashIterator gh_iter;

	if (g_editpoly_runtime == NULL) {
		return NULL;
	}

	GHASH_ITER (gh_iter, g_editpoly_runtime) {
		return BLI_ghashIterator_getValue(&gh_iter);
	}

	return NULL;
}

/**
 * The object whose Edit Poly session is active.
 *
 * Only one session can exist at a time, so this is just the first runtime.
 * Used by the undo hook, which has no object at hand.
 */
Object *editpoly_session_object(void)
{
	EditPolyRuntime *rt = editpoly_runtime_first();

	return (rt != NULL) ? rt->ob : NULL;
}

bool editpoly_session_active(void)
{
	return editpoly_runtime_first() != NULL;
}

EditPolyRuntime *editpoly_runtime_add(
        Object *ob, EditPolyModifierData *epmd,
        Mesh *edit_mesh, Mesh *saved_ob_data)
{
	EditPolyRuntime *rt;

	BLI_assert(ob != NULL);

	editpoly_runtime_init();

	/* only one runtime per object */
	editpoly_runtime_free(ob);

	rt = MEM_callocN(sizeof(EditPolyRuntime), "EditPolyRuntime");
	rt->ob = ob;
	rt->epmd = epmd;
	rt->edit_mesh = edit_mesh;
	rt->saved_ob_data = saved_ob_data;
	rt->flag = EDITPOLY_RT_EDITMODE;

	BLI_ghash_insert(g_editpoly_runtime, ob, rt);

	return rt;
}

void editpoly_runtime_free(Object *ob)
{
	EditPolyRuntime *rt = editpoly_runtime_get(ob);

	if (rt == NULL) {
		return;
	}

	BLI_ghash_remove(g_editpoly_runtime, ob, NULL, NULL);

	BLI_freelistN(&rt->saved_modes);

	MEM_freeN(rt);

	/* the registry is only needed while something is being edited, free it so
	 * nothing stays allocated after the last Edit Poly mode was left */
	if (BLI_ghash_len(g_editpoly_runtime) == 0) {
		BLI_ghash_free(g_editpoly_runtime, NULL, NULL);
		g_editpoly_runtime = NULL;
	}
}

void editpoly_runtime_object_free(Object *ob)
{
	EditPolyRuntime *rt = editpoly_runtime_get(ob);
	Mesh *edit_mesh;

	if (rt == NULL) {
		return;
	}

	if (rt->snapshot != NULL && rt->snapshot_free != NULL) {
		rt->snapshot_free(rt->snapshot);
	}

	rt->snapshot = NULL;

	edit_mesh = rt->edit_mesh;

	if (edit_mesh != NULL) {
		/* the object must never keep pointing at the temporary mesh */
		if (ob->data == edit_mesh) {
			ob->data = rt->saved_ob_data;
		}

		/* only the edit mode owns these, and the mode may still be active */
		BKE_object_free_derived_caches(ob);

		if (edit_mesh->edit_btmesh != NULL) {
			BKE_editmesh_free(edit_mesh->edit_btmesh);
			MEM_freeN(edit_mesh->edit_btmesh);
			edit_mesh->edit_btmesh = NULL;
		}

		BKE_mesh_free(edit_mesh);
		MEM_freeN(edit_mesh);
		rt->edit_mesh = NULL;
	}

	editpoly_runtime_free(ob);
}

void editpoly_runtime_free_all(void)
{
	EditPolyRuntime *rt;
	GHashIterator gh_iter;

	if (g_editpoly_runtime == NULL) {
		return;
	}

	GHASH_ITER (gh_iter, g_editpoly_runtime) {
		rt = BLI_ghashIterator_getValue(&gh_iter);

		/* the mode was never left: the temporary edit mesh is not owned by any
		 * bmain, it would leak */
		if (rt->edit_mesh) {
			BKE_mesh_free(rt->edit_mesh);
			MEM_freeN(rt->edit_mesh);
		}

		BLI_freelistN(&rt->saved_modes);
		MEM_freeN(rt);
	}

	BLI_ghash_free(g_editpoly_runtime, NULL, NULL);
	g_editpoly_runtime = NULL;
}

/**
 * Remember the state of every modifier and disable the whole stack.
 *
 * The stack must not run while the Edit Poly edit mode is active: the geometry
 * of the edit mesh already contains every modifier that comes before Edit Poly,
 * and (as in 3ds Max) modifiers that come after it are not shown while editing.
 */
void editpoly_runtime_save_modes(EditPolyRuntime *rt, Object *ob)
{
	ModifierData *md;

	if (rt == NULL || ob == NULL) {
		return;
	}

	BLI_freelistN(&rt->saved_modes);
	BLI_listbase_clear(&rt->saved_modes);

	for (md = ob->modifiers.first; md; md = md->next) {
		EditPolyModState *state = MEM_callocN(sizeof(EditPolyModState), "EditPolyModState");
		state->md = md;
		state->mode = md->mode;
		BLI_addtail(&rt->saved_modes, state);

		md->mode |= eModifierMode_DisableTemporary;
	}
}

void editpoly_runtime_restore_modes(EditPolyRuntime *rt, Object *ob)
{
	EditPolyModState *state, *state_next;

	if (rt == NULL) {
		return;
	}

	for (state = rt->saved_modes.first; state; state = state_next) {
		state_next = state->next;

		/* the modifier may have been removed while we were in edit mode */
		if (BLI_findindex(&ob->modifiers, state->md) != -1) {
			state->md->mode = state->mode;
		}

		MEM_freeN(state);
	}

	BLI_listbase_clear(&rt->saved_modes);
}

/* ---------- helpers ---------- */

EditPolyModifierData *editpoly_modifier_find(Object *ob)
{
	ModifierData *md;

	if (ob == NULL) {
		return NULL;
	}

	for (md = ob->modifiers.first; md; md = md->next) {
		if (md->type == eModifierType_EditPoly) {
			return (EditPolyModifierData *)md;
		}
	}

	return NULL;
}

int editpoly_modifier_count(Object *ob)
{
	ModifierData *md;
	int count = 0;

	if (ob == NULL) {
		return 0;
	}

	for (md = ob->modifiers.first; md; md = md->next) {
		if (md->type == eModifierType_EditPoly) {
			count++;
		}
	}

	return count;
}

EditPolyModifierData *editpoly_modifier_find_nth(Object *ob, int index)
{
	ModifierData *md;
	int i = 0;

	if (ob == NULL || index < 0) {
		return NULL;
	}

	for (md = ob->modifiers.first; md; md = md->next) {
		if (md->type != eModifierType_EditPoly) {
			continue;
		}

		if (i == index) {
			return (EditPolyModifierData *)md;
		}

		i++;
	}

	return NULL;
}

EditPolyModifierData *editpoly_modifier_find_active(Object *ob)
{
	ModifierData *md;
	EditPolyModifierData *first = NULL;

	if (ob == NULL) {
		return NULL;
	}

	for (md = ob->modifiers.first; md; md = md->next) {
		EditPolyModifierData *epmd;

		if (md->type != eModifierType_EditPoly) {
			continue;
		}

		epmd = (EditPolyModifierData *)md;

		if (first == NULL) {
			first = epmd;
		}

		if (epmd->flag & EDITPOLY_ACTIVE) {
			return epmd;
		}
	}

	/* nothing was selected yet: the first Edit Poly ("mesh 1") is the default */
	if (first) {
		first->flag |= EDITPOLY_ACTIVE;
	}

	return first;
}

int editpoly_modifier_active_index(Object *ob)
{
	ModifierData *md;
	int i = 0;

	if (ob == NULL) {
		return 0;
	}

	for (md = ob->modifiers.first; md; md = md->next) {
		if (md->type != eModifierType_EditPoly) {
			continue;
		}

		if (((EditPolyModifierData *)md)->flag & EDITPOLY_ACTIVE) {
			return i;
		}

		i++;
	}

	return 0;
}

EditPolyModifierData *editpoly_modifier_set_active(Object *ob, int index)
{
	ModifierData *md;
	EditPolyModifierData *target;

	if (ob == NULL) {
		return NULL;
	}

	target = editpoly_modifier_find_nth(ob, index);

	if (target == NULL) {
		return NULL;
	}

	for (md = ob->modifiers.first; md; md = md->next) {
		if (md->type == eModifierType_EditPoly) {
			((EditPolyModifierData *)md)->flag &= ~EDITPOLY_ACTIVE;
		}
	}

	target->flag |= EDITPOLY_ACTIVE;

	return target;
}

bool editpoly_object_is_in_editmode(Object *ob)
{
	return editpoly_runtime_get(ob) != NULL;
}
