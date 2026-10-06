/*
 * ***** BEGIN GPL LICENSE BLOCK *****
 *
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
 *
 * ***** END GPL LICENSE BLOCK *****
 */

/** \file B3DCharacter.h
 *  \ingroup physbox3d
 *
 * Box3D character mover: the implementation of PHY_ICharacter for objects using
 * Blender's "Character" physics type.  It is the Box3D counterpart of
 * BlenderBulletCharacterController.
 */

#ifndef __B3DCHARACTER_H__
#define __B3DCHARACTER_H__

#include "PHY_ICharacter.h"

#include "MT_Vector3.h"

#include "box3d/box3d.h"

/* Collision category reserved for character capsules.  The mover queries inside
 * the character mask this bit out so that a character never collides with
 * itself:  Box3D has no "ignore this body" argument for b3World_CollideMover(),
 * only a bit filter.  Everything else (rays, Near and Radar sensors) still sees
 * the character. */
#define B3D_CHARACTER_CATEGORY (((uint64_t)1) << 63)

class B3DPhysicsController;
class B3DPhysicsEnvironment;

/**
 * Kinematic character controller built on Box3D's capsule mover.
 *
 * Box3D has no ready made character controller, but it has exactly the pieces
 * Bullet's btKinematicCharacterController is assembled from:
 *
 *   - b3World_CollideMover() gathers the collision planes around a capsule,
 *   - b3SolvePlanes() turns them into the shortest translation that separates
 *     the capsule from all of them at once,
 *   - b3World_CastMover() sweeps the capsule through the world and slides it
 *     along whatever it hits.
 *
 * Update() runs those three in a loop, which is the same "collide, solve, cast"
 * iteration the Bullet controller performs in updateAction().  The character
 * itself is a kinematic body with a sensor capsule:  it never pushes anything
 * (like Bullet's CF_CHARACTER_OBJECT ghost) and the mover does the collision
 * work against the planes.
 */
class B3DCharacter : public PHY_ICharacter
{
public:
	B3DCharacter(B3DPhysicsEnvironment *env,
	             B3DPhysicsController *ctrl,
	             const b3Capsule& mover,
	             float stepHeight,
	             float jumpSpeed,
	             float fallSpeed,
	             unsigned char maxJumps);
	virtual ~B3DCharacter();

	/** Move the character by one simulation step.  Called right before every
	 *  b3World_Step(), so that the body it moves is in place when the solver
	 *  runs. */
	void Update(float timeStep);

	/* ---- PHY_ICharacter ---- */

	virtual void Jump();
	virtual bool OnGround() { return m_onGround; }

	virtual float GetGravity() { return m_gravity; }
	virtual void SetGravity(float gravity) { m_gravity = gravity; }

	virtual unsigned char GetMaxJumps() { return m_maxJumps; }
	virtual void SetMaxJumps(unsigned char maxJumps) { m_maxJumps = maxJumps; }
	virtual unsigned char GetJumpCount() { return m_jumps; }

	virtual void SetWalkDirection(const MT_Vector3& dir) { m_walkDirection = dir; }
	virtual MT_Vector3 GetWalkDirection() { return m_walkDirection; }

private:
	/** Collect the planes around the mover, see b3PlaneResultFcn. */
	static bool GatherPlanes(b3ShapeId shapeId, const b3PlaneResult* plane, int planeCount, void* context);
	/** Ignore the character's own shape while casting. */
	static bool FilterOwnShape(b3ShapeId shapeId, void* context);

	/** True when the plane is flat enough to be stood on. */
	static bool IsWalkable(const b3Vec3& normal);

	/**
	 * Run "collide, solve, cast" for one translation: gather the planes around
	 * the capsule, solve them into the shortest separating translation and sweep
	 * the capsule there, sliding along whatever is in the way.  m_origin ends up
	 * where the capsule could go.
	 */
	void MoveCapsule(const b3Vec3& target, const b3QueryFilter& filter);

	/** Cast the capsule straight down by the step height and follow the ground.
	 *  \return true when something was found below the capsule. */
	bool SnapToGround(const b3QueryFilter& filter);

	B3DPhysicsEnvironment *m_env;
	B3DPhysicsController *m_ctrl;    /* not owned */

	b3Capsule m_mover;               /* capsule, relative to the body origin */
	b3Pos m_origin;                  /* current mover origin (the body centre) */

	MT_Vector3 m_walkDirection;      /* metres per second, set by the actuators */
	float m_gravity;                 /* negative, along the world Z axis */
	float m_jumpSpeed;
	float m_fallSpeed;
	float m_stepHeight;
	float m_verticalVelocity;

	unsigned char m_maxJumps;
	unsigned char m_jumps;
	bool m_onGround;
	bool m_wantJump;
};

#endif  /* __B3DCHARACTER_H__ */
