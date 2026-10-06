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

/** \file B3DCharacter.cpp
 *  \ingroup physbox3d
 */

#include "B3DCharacter.h"

#include "B3DPhysicsController.h"
#include "B3DPhysicsEnvironment.h"

#include <cfloat>
#include <cmath>

/* Number of "collide, solve, cast" rounds per step.  Bullet's controller loops
 * while it recovers from penetration; four rounds are enough for the shapes a
 * character walks into and keep the cost per step bounded. */
#define B3D_CHARACTER_ITERATIONS 4

/* Planes handed to b3SolvePlanes in one round. */
#define B3D_CHARACTER_MAX_PLANES 16

/* cos of the steepest slope a character can stand on (45 degrees).  Bullet's
 * btKinematicCharacterController uses the same value for its ground test. */
#define B3D_CHARACTER_WALK_COS 0.70710678f

/* A delta shorter than this is settled. */
#define B3D_CHARACTER_EPSILON 1.0e-8f

/** Collected collision planes of one round. */
struct B3DCharacterPlanes
{
	b3CollisionPlane planes[B3D_CHARACTER_MAX_PLANES];
	int count;
	bool walkable;
};

static inline b3Vec3 b3dCharacterMakeVector(float x, float y, float z)
{
	b3Vec3 v;
	v.x = x;
	v.y = y;
	v.z = z;
	return v;
}

B3DCharacter::B3DCharacter(B3DPhysicsEnvironment *env,
                           B3DPhysicsController *ctrl,
                           const b3Capsule& mover,
                           float stepHeight,
                           float jumpSpeed,
                           float fallSpeed,
                           unsigned char maxJumps) :
    m_env(env),
    m_ctrl(ctrl),
    m_mover(mover),
    m_origin(),
    m_walkDirection(0.0, 0.0, 0.0),
    /* Bullet's btKinematicCharacterController starts with 3G of gravity as a
     * positive magnitude and BGE never overrides it, so the default matches. */
    m_gravity(29.4f),
    m_jumpSpeed((jumpSpeed > 0.0f) ? jumpSpeed : 10.0f),
    m_fallSpeed((fallSpeed > 0.0f) ? fallSpeed : 55.0f),
    m_stepHeight(stepHeight),
    m_verticalVelocity(0.0f),
    m_maxJumps(maxJumps),
    m_jumps(0),
    m_onGround(false),
    m_wantJump(false)
{
	if (m_ctrl && b3Body_IsValid(m_ctrl->GetBodyId()))
		m_origin = b3Body_GetPosition(m_ctrl->GetBodyId());
	else
		m_origin = b3Pos_zero;

	/* The character body must never sleep:  it is moved by hand and a sleeping
	 * kinematic body would still be moved, but its contacts would be stale. */
	if (m_ctrl && b3Body_IsValid(m_ctrl->GetBodyId()))
		b3Body_EnableSleep(m_ctrl->GetBodyId(), false);
}

B3DCharacter::~B3DCharacter()
{
}

bool B3DCharacter::IsWalkable(const b3Vec3& normal)
{
	/* Blender's game engine is Z up, so "up" is +Z here. */
	return normal.z >= B3D_CHARACTER_WALK_COS;
}

bool B3DCharacter::GatherPlanes(b3ShapeId shapeId, const b3PlaneResult* plane, int planeCount, void* context)
{
	(void)shapeId;

	B3DCharacterPlanes *buffer = (B3DCharacterPlanes *)context;

	for (int i = 0; i < planeCount && buffer->count < B3D_CHARACTER_MAX_PLANES; i++) {
		b3CollisionPlane& out = buffer->planes[buffer->count++];
		out.plane = plane[i].plane;
		out.pushLimit = FLT_MAX;   /* the character cannot be pushed into anything */
		out.push = 0.0f;
		out.clipVelocity = true;

		if (IsWalkable(plane[i].plane.normal))
			buffer->walkable = true;


	}

	/* Stop gathering once the solver input is full. */
	return buffer->count < B3D_CHARACTER_MAX_PLANES;
}

bool B3DCharacter::FilterOwnShape(b3ShapeId shapeId, void* context)
{
	B3DCharacter *self = (B3DCharacter *)context;
	if (!self->m_ctrl)
		return true;

	const b3BodyId other = b3Shape_GetBody(shapeId);
	const b3BodyId own = self->m_ctrl->GetBodyId();
	return other.index1 != own.index1 || other.generation != own.generation;
}

void B3DCharacter::Update(float timeStep)
{
	if (!m_env || !m_ctrl || !m_env->IsWorldAlive() || timeStep <= 0.0f)
		return;

	b3BodyId body = m_ctrl->GetBodyId();
	if (!b3Body_IsValid(body))
		return;

	b3WorldId world = m_env->GetWorldId();

	/* The body carries the character position:  the game logic may have moved it
	 * (a motion actuator on a kinematic body, or an explicit SetPosition()), and
	 * Bullet's controller reads its ghost transform in the same way. */
	m_origin = b3Body_GetPosition(body);

	/* The body may only have been teleported:  the mover keeps its own state, so
	 * start from wherever the body is.
	 *
	 * A jump is applied here rather than in Jump() so that the ground state of
	 * this step is used, exactly like BlenderBulletCharacterController does. */
	if (m_wantJump) {
		const bool canJump = (m_onGround && m_maxJumps > 0) || m_jumps < m_maxJumps;
		if (canJump) {
			m_verticalVelocity = m_jumpSpeed;
			m_jumps++;
		}
		m_wantJump = false;
	}

	/* Semi implicit Euler on the vertical axis, with Bullet's clamps (upwards by
	 * the jump speed, downwards by the fall speed). */
	m_verticalVelocity -= m_gravity * timeStep;
	if (m_verticalVelocity > 0.0f && m_verticalVelocity > m_jumpSpeed)
		m_verticalVelocity = m_jumpSpeed;
	if (m_verticalVelocity < 0.0f && fabsf(m_verticalVelocity) > fabsf(m_fallSpeed))
		m_verticalVelocity = -fabsf(m_fallSpeed);

	b3Vec3 target;
	target.x = (float)m_walkDirection[0] * timeStep;
	target.y = (float)m_walkDirection[1] * timeStep;
	target.z = (float)m_walkDirection[2] * timeStep + m_verticalVelocity * timeStep;

	/* The move query must not see the character's own capsule. */
	b3QueryFilter filter = b3DefaultQueryFilter();
	filter.categoryBits = ~(uint64_t)0;
	filter.maskBits = ~B3D_CHARACTER_CATEGORY;

	/* Collide, solve, cast:  gather the planes around the capsule, solve them
	 * into the shortest separating translation and sweep the capsule there,
	 * sliding along whatever is in the way.  This is the loop Box3D's mover API
	 * is meant to be driven with. */
	const b3Pos start = m_origin;
	MoveCapsule(target, filter);

	const b3Vec3 achieved = b3SubPos(m_origin, start);

	/* Step up:  a character walking into a low step would be stopped by its
	 * vertical face, because Box3D's mover only slides along what it hits.  When
	 * the horizontal move was blocked, the same move is retried from the step
	 * height above:  Box3D's own casts supply everything this needs. */
	if (m_onGround && m_stepHeight > 0.0f) {
		const float wanted = sqrtf(target.x * target.x + target.y * target.y);
		const float done = sqrtf(achieved.x * achieved.x + achieved.y * achieved.y);

		if (wanted > 1.0e-5f && done < wanted - 1.0e-5f) {
			const b3Pos blocked = m_origin;
			const b3Vec3 up = b3dCharacterMakeVector(0.0f, 0.0f, m_stepHeight);
			const float upFraction = b3World_CastMover(world, m_origin, &m_mover, up, filter,
			                                           FilterOwnShape, this);

			if (upFraction > 0.0f) {
				m_origin = b3OffsetPos(m_origin, b3MulSV(upFraction, up));

				b3Vec3 flat = b3dCharacterMakeVector(target.x, target.y, 0.0f);
				MoveCapsule(flat, filter);

				/* Settle on to whatever the step is made of. */
				SnapToGround(filter);

				const b3Vec3 stepped = b3SubPos(m_origin, start);
				const float steppedDone = sqrtf(stepped.x * stepped.x + stepped.y * stepped.y);

				/* Keep the detour only when it really got further; a wall is not a
				 * step and the character has to stay where it was. */
				if (steppedDone <= done + 1.0e-3f)
					m_origin = blocked;
			}
		}
	}

	/* Ground state:  the capsule is standing on something when a walkable plane
	 * touches it.  A capsule that rests exactly on a floor can report no plane at
	 * all, so the ground is confirmed with a short cast straight down, which also
	 * glues the character to stairs and slopes while it walks. */
	B3DCharacterPlanes ground;
	ground.count = 0;
	ground.walkable = false;
	b3World_CollideMover(world, m_origin, &m_mover, filter, GatherPlanes, &ground);

	bool grounded = ground.walkable;

	if (m_verticalVelocity <= 0.0f && SnapToGround(filter))
		grounded = true;

	m_onGround = grounded;

	if (m_onGround) {
		if (m_verticalVelocity < 0.0f)
			m_verticalVelocity = 0.0f;
		/* Touching the ground resets the jump counter, see updateAction(). */
		m_jumps = 0;
	}

	/* Give the body the place the mover ended up in.  The rotation is left to the
	 * game logic:  a character does not spin when it is pushed. */
	b3Body_SetTransform(body, m_origin, b3Body_GetRotation(body));
}

void B3DCharacter::MoveCapsule(const b3Vec3& target, const b3QueryFilter& filter)
{
	if (!m_env || !m_env->IsWorldAlive())
		return;

	const b3WorldId world = m_env->GetWorldId();
	b3Vec3 delta = target;

	for (int iteration = 0; iteration < B3D_CHARACTER_ITERATIONS; iteration++) {
		B3DCharacterPlanes buffer;
		buffer.count = 0;
		buffer.walkable = false;

		b3World_CollideMover(world, m_origin, &m_mover, filter, GatherPlanes, &buffer);

		/* The planes only correct the translation (they separate a capsule that
		 * overlaps something).  With no planes at all b3SolvePlanes() hands the
		 * wanted translation straight back, so free air is moved through as well. */
		b3PlaneSolverResult result = b3SolvePlanes(delta, buffer.planes, buffer.count);

		if (b3LengthSquared(result.delta) < B3D_CHARACTER_EPSILON)
			break;

		const float fraction = b3World_CastMover(world, m_origin, &m_mover, result.delta, filter,
		                                         FilterOwnShape, this);

		if (!(fraction > 0.0f))   /* zero or a NaN, nothing can be moved */
			break;

		m_origin = b3OffsetPos(m_origin, b3MulSV(fraction, result.delta));

		if (fraction >= 1.0f)
			break;

		/* Whatever was blocked is retried with the remaining translation. */
		delta = b3MulSV(1.0f - fraction, result.delta);
	}
}

bool B3DCharacter::SnapToGround(const b3QueryFilter& filter)
{
	if (!m_env || !m_env->IsWorldAlive())
		return false;

	/* A little further than the step height, so that a character standing exactly
	 * on a surface always finds it. */
	const float distance = ((m_stepHeight > 0.0f) ? m_stepHeight : 0.05f) + 0.05f;
	const b3Vec3 down = b3dCharacterMakeVector(0.0f, 0.0f, -distance);

	const float fraction = b3World_CastMover(m_env->GetWorldId(), m_origin, &m_mover, down, filter,
	                                         FilterOwnShape, this);
	if (fraction >= 1.0f)
		return false;

	m_origin = b3OffsetPos(m_origin, b3MulSV(fraction, down));
	return true;
}

void B3DCharacter::Jump()
{
	/* Applied in the next Update(), which knows whether the character stands on
	 * something. */
	m_wantJump = true;
}