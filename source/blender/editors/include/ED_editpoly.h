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
#ifndef __ED_EDITPOLY_H__
#define __ED_EDITPOLY_H__

/** \file ED_editpoly.h
 *  \ingroup editors
 *
 *  Recording of the Edit Poly operation history while the object is being
 *  edited with the native Edit Mode.
 */

struct bContext;
struct Object;

/* Take the snapshot the next operation is compared against. Called right after
 * the object entered the Edit Poly edit mode. */
void editpoly_record_begin(struct Object *ob);

/* Compare the current geometry of \a ob with the last snapshot and append the
 * difference to the operation history of the modifier. */
void editpoly_record_sync(struct Object *ob, const char *opname);

/* Drop the snapshot, called when leaving the Edit Poly edit mode. */
void editpoly_record_end(struct Object *ob);

/* Drop the snapshot of a session that is still active when Blender shuts down. */
void editpoly_record_free_all(void);

/* Hook for #ED_undo_push: every operator that changes geometry pushes an undo
 * step, which is exactly the moment the Edit Poly history has to be updated.
 * Returns true when an Edit Poly session consumed the step: the native undo
 * step must not be written then, because the object data is the temporary edit
 * mesh which is not part of the main database. */
bool editpoly_record_undo_hook(const char *opname);

/* Undo (or redo) inside the Edit Poly edit mode, driven by the operation
 * history instead of the native undo system. Returns true when the request
 * was handled (and the native undo must not run). */
bool editpoly_undo_session_step(struct bContext *C, bool undo);

/** Leave the Edit Poly edit mode if a session is active (whoever started it) and
 *  return whether one was left. Used by the paths that must not keep an object
 *  with a temporary object data: deleting objects, for instance. */
bool editpoly_session_exit(struct bContext *C);

#endif  /* __ED_EDITPOLY_H__ */
