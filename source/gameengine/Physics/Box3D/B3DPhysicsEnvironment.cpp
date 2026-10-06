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

/** \file B3DPhysicsEnvironment.cpp
 *  \ingroup physbox3d
 */

#include "B3DPhysicsEnvironment.h"
#include "B3DPhysicsController.h"

#include "PHY_IMotionState.h"
#include "PHY_Pro.h"

#include "KX_GameObject.h"
#include "KX_ClientObjectInfo.h"
#include "B3DMotionState.h"
#include "B3DCharacter.h"
#include "B3DVehicle.h"
#include "KX_BlenderSceneConverter.h"
#include "KX_KetsjiEngine.h"
#include "KX_PythonInit.h"
#include "RAS_MeshObject.h"
#include "RAS_Polygon.h"

#include "MT_MinMax.h"

#include "DNA_scene_types.h"
#include "DNA_object_types.h"
#include "DNA_constraint_types.h"
#include "DNA_meshdata_types.h"

extern "C" {
#include "BLI_utildefines.h"
#include "BKE_object.h"
#include "BKE_DerivedMesh.h"
#include "BKE_cdderivedmesh.h"
}

#include <stdio.h>
#include <string.h>
#include <math.h>
#include <utility>

/* Contacts polled per body and step; a body that touches more shapes than this
 * still reports the first B3D_MAX_POLLED_CONTACTS of them. */
#define B3D_MAX_POLLED_CONTACTS 16

/* Same value as Bullet's CCD_CONSTRAINT_DISABLE_LINKED_COLLISION: the linked
 * bodies of the constraint do not collide with each other. */
#define B3D_CONSTRAINT_DISABLE_LINKED_COLLISION 0x80

/* b3ParallelJoint is a spring; used for the angular only constraint it has to be
 * stiff enough to behave like the rigid constraint Bullet builds for it. */
#define B3D_ANGULAR_JOINT_MAX_TORQUE 1.0e6f

/* Half extents below this are degenerate (a plane has a zero thickness bound
 * box) and Box3D hulls built from coplanar points are not usable, so a thin
 * slab is used instead. */
#define B3D_MIN_HALF_EXTENT 0.01f

static inline b3Vec3 b3v(float x, float y, float z)
{
	b3Vec3 v;
	v.x = x;
	v.y = y;
	v.z = z;
	return v;
}

/* ------------------------------------------------------------------------- */

B3DPhysicsEnvironment::B3DPhysicsEnvironment()
	: m_worldId(b3_nullWorldId),
	  m_worldAlive(false),
	  m_subStepCount(4),
	  m_debugMode(0),
	  m_useFixedTimeStep(false),
	  m_fixedTimeStep(0.0f),
	  m_accumulator(0.0f),
	  m_compoundChildReported(false),
	  m_meshBoundsReported(false),
	  m_meshDynamicReported(false),
	  m_sensorProxyReported(false),
	  m_nextJointId(1),
	  m_jointMappingReported(false),
	  m_deactivationLinear(0.8f),
	  m_deactivationAngular(1.0f),
	  m_deactivationTime(2.0f),
	  m_ccdMode(1)
{
	m_gravity.setValue(0.0f, 0.0f, -9.81f);

	b3WorldDef wd = b3DefaultWorldDef();

	/* Box3D has no up vector, Blender and the BGE are Z up. */
	wd.gravity = b3v(0.0f, 0.0f, -9.81f);
	wd.enableSleep = true;
	wd.enableContinuous = true;

	/* Box3D spawns its own threads when workerCount > 1 and no task callbacks are
	 * given.  b3World_Step() must not run inside a job, so the BGE keeps the
	 * default single threaded for now. */
	wd.workerCount = 1;

	m_worldId = b3CreateWorld(&wd);
	m_worldAlive = b3World_IsValid(m_worldId);

	if (!m_worldAlive) {
		m_worldId = b3_nullWorldId;
		printf("B3DPhysicsEnvironment: failed to create a Box3D world\n");
	}
}

B3DPhysicsEnvironment::~B3DPhysicsEnvironment()
{
	/* Vehicles own wheel bodies and joints inside the world, so they have to be
	 * destroyed while the world is still alive. */
	for (size_t i = 0; i < m_vehicles.size(); i++)
		delete m_vehicles[i];
	m_vehicles.clear();

	/* Destroying the world destroys every body, shape and joint in it.  The flag
	 * is cleared first so that controllers deleted afterwards do not touch the
	 * freed world (see B3DPhysicsController::~B3DPhysicsController). */
	if (m_worldAlive && b3World_IsValid(m_worldId)) {
		b3DestroyWorld(m_worldId);
	}
	m_worldId = b3_nullWorldId;
	m_worldAlive = false;

	for (std::map<int, B3DJoint*>::iterator it = m_joints.begin(); it != m_joints.end(); ++it)
		delete it->second;
	m_joints.clear();

	m_controllers.clear();
}

B3DPhysicsEnvironment* B3DPhysicsEnvironment::Create(Scene *blenderscene, bool visualizePhysics)
{
	B3DPhysicsEnvironment *env = new B3DPhysicsEnvironment();

	/* Blender and Box3D both work in metres, so b3SetLengthUnitsPerMeter() is
	 * deliberately left at its default.  It would have to be called before any
	 * other Box3D function if a scene ever needed different units. */

	/* The Scene -> Physics panel settings, applied exactly where
	 * CcdPhysicsEnvironment::Create() applies them. */
	if (blenderscene) {
		env->SetDeactivationLinearTreshold(blenderscene->gm.lineardeactthreshold);
		env->SetDeactivationAngularTreshold(blenderscene->gm.angulardeactthreshold);
		env->SetDeactivationTime(blenderscene->gm.deactivationtime);
	}

	if (visualizePhysics)
		env->SetDebugMode(1);

	return env;
}

void B3DPhysicsEnvironment::AddController(B3DPhysicsController *ctrl)
{
	if (!ctrl)
		return;

	m_controllers.insert(ctrl);

	/* The scene settings arrive before the objects are converted, so every body
	 * that appears later gets the current sleep threshold here. */
	b3BodyId body = ctrl->GetBodyId();
	if (b3Body_IsValid(body)) {
		float threshold = m_deactivationLinear;
		if (!(threshold > 0.0f))
			threshold = m_deactivationAngular;
		b3Body_SetSleepThreshold(body, threshold);
	}
}

void B3DPhysicsEnvironment::RemoveController(B3DPhysicsController *ctrl)
{
	m_controllers.erase(ctrl);
}

bool B3DPhysicsEnvironment::HasController(B3DPhysicsController *ctrl) const
{
	return ctrl && m_controllers.find(ctrl) != m_controllers.end();
}

/* -------------------------------------------------------------------------
 * Frame stepping
 * ------------------------------------------------------------------------- */

void B3DPhysicsEnvironment::BeginFrame()
{
	/* Box3D accumulates and clears forces inside b3World_Step(), there is no
	 * separate frame boundary to maintain. */
}

void B3DPhysicsEnvironment::EndFrame()
{
}

bool B3DPhysicsEnvironment::ProceedDeltaTime(double curTime, float timeStep, float interval)
{
	(void)curTime;
	(void)interval;

	if (!m_worldAlive)
		return false;

	std::set<B3DPhysicsController*>::iterator it;

	/* Constraints whose limits arrived after their creation (the 6DOF ones) are
	 * built here, before the first step that uses them. */
	CreatePendingJoints();

	/* Blender's velocity clamps are applied right before the solve, the same
	 * place CcdPhysicsController::SimulationTick() does it. */
	for (it = m_controllers.begin(); it != m_controllers.end(); ++it)
		(*it)->ApplyVelocityClamps();

	const int subSteps = (m_subStepCount > 0) ? m_subStepCount : 1;

	if (m_useFixedTimeStep && m_fixedTimeStep > 0.0f) {
		/* Box3D expects a fixed time step; accumulate so that a variable frame
		 * time still produces deterministic steps. */
		m_accumulator += timeStep;

		int steps = 0;
		while (m_accumulator >= m_fixedTimeStep && steps < 8) {
			UpdateCharacters(m_fixedTimeStep);
			b3World_Step(m_worldId, m_fixedTimeStep, subSteps);
			m_accumulator -= m_fixedTimeStep;
			steps++;
		}

		if (steps == 8) {
			/* The simulation cannot keep up, drop the backlog instead of
			 * spiralling into ever longer frames. */
			m_accumulator = 0.0f;
		}
	}
	else {
		UpdateCharacters(timeStep);
		b3World_Step(m_worldId, timeStep, subSteps);
	}

	/* Contacts and sensor overlaps of the step that just ran.  This has to happen
	 * before the next b3World_Step(): Box3D only keeps the events of the current
	 * step. */
	ProcessEvents();

	/* Write the simulated transforms back into the motion states.  Everything
	 * else (scene graph update, sensor transforms) happens in
	 * KX_Scene::UpdateParents(), which runs right after this call. */
	for (it = m_controllers.begin(); it != m_controllers.end(); ++it)
		(*it)->SynchronizeMotionStates(timeStep);

	SyncVehicles(timeStep);

	return true;
}

void B3DPhysicsEnvironment::SetFixedTimeStep(bool useFixedTimeStep, float fixedTimeStep)
{
	m_useFixedTimeStep = useFixedTimeStep;
	m_fixedTimeStep = fixedTimeStep;
	m_accumulator = 0.0f;
}

float B3DPhysicsEnvironment::GetFixedTimeStep()
{
	return m_useFixedTimeStep ? m_fixedTimeStep : 0.0f;
}

void B3DPhysicsEnvironment::SetNumTimeSubSteps(int numTimeSubSteps)
{
	/* Box3D clamps internally as well; keep the reported value sane because
	 * KX_ObjectActuator divides by GetNumTimeSubSteps(). */
	m_subStepCount = (numTimeSubSteps > 0) ? numTimeSubSteps : 1;
}

void B3DPhysicsEnvironment::SetNumIterations(int numIter)
{
	/* Bullet's setNumIterations() is a solver iteration count.  Box3D's solver
	 * has no such knob:  accuracy comes from the number of sub-steps, so that is
	 * what a game setting this affects. */
	SetNumTimeSubSteps(numIter);
}

void B3DPhysicsEnvironment::ApplySleepThresholds()
{
	/* Box3D has one speed threshold per body where Bullet has a linear and an
	 * angular one.  The linear threshold is the meaningful one for a sleeping
	 * body, so it wins whenever the scene sets it; a scene that only tunes the
	 * angular value still gets that value. */
	float threshold = m_deactivationLinear;
	if (!(threshold > 0.0f))
		threshold = m_deactivationAngular;

	for (std::set<B3DPhysicsController*>::iterator it = m_controllers.begin(); it != m_controllers.end(); ++it) {
		b3BodyId body = (*it)->GetBodyId();
		if (b3Body_IsValid(body))
			b3Body_SetSleepThreshold(body, threshold);
	}
}

void B3DPhysicsEnvironment::SetDeactivationLinearTreshold(float linTresh)
{
	m_deactivationLinear = linTresh;
	ApplySleepThresholds();
}

void B3DPhysicsEnvironment::SetDeactivationAngularTreshold(float angTresh)
{
	m_deactivationAngular = angTresh;
	ApplySleepThresholds();
}

void B3DPhysicsEnvironment::SetDeactivationTime(float dTime)
{
	m_deactivationTime = dTime;

	/* Bullet's deactivation time is how long a body has to stay below the
	 * thresholds before it sleeps; Box3D sleeps as soon as a body is slow enough.
	 * Zero still means "no sleeping", which Box3D spells as a world switch. */
	if (m_worldAlive)
		b3World_EnableSleeping(m_worldId, dTime > 0.0f);
}

void B3DPhysicsEnvironment::SetCcdMode(int ccdMode)
{
	m_ccdMode = ccdMode;

	/* b3World_EnableContinuous() is the world wide CCD switch.  Box3D's per-body
	 * "bullet" flag (b3Body_SetBullet()) is what actually sweeps a fast body, but
	 * the BGE has no per-object CCD setting in 2.79, so the mode is applied to the
	 * world and the bullet flag stays off. */
	if (m_worldAlive)
		b3World_EnableContinuous(m_worldId, ccdMode != 0);
}

/* ---- physics visualization (Render -> Show Physics Visualization) ---------- */

/** b3HexColor is a packed 0xRRGGBB value, the rasterizer wants 0..1 floats. */
static MT_Vector3 b3dDrawColor(b3HexColor color)
{
	const float inv = 1.0f / 255.0f;
	return MT_Vector3(float((int(color) >> 16) & 0xFF) * inv,
	                  float((int(color) >> 8) & 0xFF) * inv,
	                  float(int(color) & 0xFF) * inv);
}

/** Box3D hands out b3Pos (double in large world mode) and b3Vec3 values. */
template <class T> static MT_Vector3 b3dDrawPoint(const T& p)
{
	return MT_Vector3(p.x, p.y, p.z);
}

static void b3dDrawLine(const MT_Vector3& from, const MT_Vector3& to, const MT_Vector3& color)
{
	KX_RasterizerDrawDebugLine(from, to, color);
}

/** A circle around center in one of the three world planes (0: xy, 1: xz, 2: yz). */
static void b3dDrawRing(const MT_Vector3& center, float radius, int axis, const MT_Vector3& color)
{
	const int segments = 24;
	MT_Vector3 previous;
	for (int i = 0; i <= segments; i++) {
		const float angle = 6.283185307179586f * float(i) / float(segments);
		const float ca = cosf(angle) * radius;
		const float sa = sinf(angle) * radius;
		MT_Vector3 point = center;
		if (axis == 0)
			point += MT_Vector3(ca, sa, 0.0f);
		else if (axis == 1)
			point += MT_Vector3(ca, 0.0f, sa);
		else
			point += MT_Vector3(0.0f, ca, sa);
		if (i > 0)
			b3dDrawLine(previous, point, color);
		previous = point;
	}
}

static void b3dDrawSphereAt(const MT_Vector3& center, float radius, const MT_Vector3& color)
{
	for (int axis = 0; axis < 3; axis++)
		b3dDrawRing(center, radius, axis, color);
}

/** The twelve edges of an axis aligned box. */
static void b3dDrawBoxAt(const MT_Vector3& center, const MT_Vector3& extents, const MT_Vector3& color)
{
	MT_Vector3 corner[8];
	for (int i = 0; i < 8; i++) {
		corner[i] = center + MT_Vector3((i & 1) ? extents.x() : -extents.x(),
		                                (i & 2) ? extents.y() : -extents.y(),
		                                (i & 4) ? extents.z() : -extents.z());
	}
	static const int edges[12][2] = {
		{0, 1}, {2, 3}, {4, 5}, {6, 7},
		{0, 2}, {1, 3}, {4, 6}, {5, 7},
		{0, 4}, {1, 5}, {2, 6}, {3, 7},
	};
	for (int i = 0; i < 12; i++)
		b3dDrawLine(corner[edges[i][0]], corner[edges[i][1]], color);
}

static void b3dDrawSegmentCb(b3Pos p1, b3Pos p2, b3HexColor color, void *context)
{
	b3dDrawLine(b3dDrawPoint(p1), b3dDrawPoint(p2), b3dDrawColor(color));
}

static void b3dDrawPointCb(b3Pos p, float size, b3HexColor color, void *context)
{
	const MT_Vector3 center = b3dDrawPoint(p);
	const MT_Vector3 rgb = b3dDrawColor(color);
	const float s = (size > 0.0f) ? size : 0.05f;
	b3dDrawLine(center - MT_Vector3(s, 0.0f, 0.0f), center + MT_Vector3(s, 0.0f, 0.0f), rgb);
	b3dDrawLine(center - MT_Vector3(0.0f, s, 0.0f), center + MT_Vector3(0.0f, s, 0.0f), rgb);
	b3dDrawLine(center - MT_Vector3(0.0f, 0.0f, s), center + MT_Vector3(0.0f, 0.0f, s), rgb);
}

static void b3dDrawSphereCb(b3Pos p, float radius, b3HexColor color, float alpha, void *context)
{
	b3dDrawSphereAt(b3dDrawPoint(p), radius, b3dDrawColor(color));
}

static void b3dDrawCapsuleCb(b3Pos p1, b3Pos p2, float radius, b3HexColor color, float alpha, void *context)
{
	const MT_Vector3 a = b3dDrawPoint(p1);
	const MT_Vector3 b = b3dDrawPoint(p2);
	const MT_Vector3 rgb = b3dDrawColor(color);
	b3dDrawSphereAt(a, radius, rgb);
	b3dDrawSphereAt(b, radius, rgb);
	/* the four sides of the capsule in the plane perpendicular to its axis */
	const MT_Vector3 axis = b - a;
	const MT_Vector3 up = (fabsf(axis.z()) > 0.9f * axis.length()) ? MT_Vector3(1.0f, 0.0f, 0.0f)
	                                                              : MT_Vector3(0.0f, 0.0f, 1.0f);
	const MT_Vector3 side1 = MT_Vector3(axis.cross(up)).safe_normalized() * radius;
	const MT_Vector3 side2 = MT_Vector3(axis.cross(side1)).safe_normalized() * radius;
	b3dDrawLine(a + side1, b + side1, rgb);
	b3dDrawLine(a - side1, b - side1, rgb);
	b3dDrawLine(a + side2, b + side2, rgb);
	b3dDrawLine(a - side2, b - side2, rgb);
}

static void b3dDrawBoundsCb(b3AABB aabb, b3HexColor color, void *context)
{
	const MT_Vector3 lower = b3dDrawPoint(aabb.lowerBound);
	const MT_Vector3 upper = b3dDrawPoint(aabb.upperBound);
	b3dDrawBoxAt((lower + upper) * 0.5f, (upper - lower) * 0.5f, b3dDrawColor(color));
}

static void b3dDrawBoxCb(b3Vec3 extents, b3WorldTransform transform, b3HexColor color, void *context)
{
	const MT_Vector3 center = b3dDrawPoint(transform.p);
	const MT_Vector3 rgb = b3dDrawColor(color);
	MT_Vector3 corner[8];
	for (int i = 0; i < 8; i++) {
		const b3Vec3 local = b3v((i & 1) ? extents.x : -extents.x,
		                         (i & 2) ? extents.y : -extents.y,
		                         (i & 4) ? extents.z : -extents.z);
		corner[i] = center + b3dDrawPoint(b3RotateVector(transform.q, local));
	}
	static const int edges[12][2] = {
		{0, 1}, {2, 3}, {4, 5}, {6, 7},
		{0, 2}, {1, 3}, {4, 6}, {5, 7},
		{0, 4}, {1, 5}, {2, 6}, {3, 7},
	};
	for (int i = 0; i < 12; i++)
		b3dDrawLine(corner[edges[i][0]], corner[edges[i][1]], rgb);
}

static void b3dDrawTransformCb(b3WorldTransform transform, void *context)
{
	/* body axes and centre of mass, always in rgb = xyz */
	const float scale = 0.3f * b3GetLengthUnitsPerMeter();
	const MT_Vector3 origin = b3dDrawPoint(transform.p);
	b3dDrawLine(origin, origin + b3dDrawPoint(b3RotateVector(transform.q, b3v(scale, 0.0f, 0.0f))),
	            MT_Vector3(1.0f, 0.0f, 0.0f));
	b3dDrawLine(origin, origin + b3dDrawPoint(b3RotateVector(transform.q, b3v(0.0f, scale, 0.0f))),
	            MT_Vector3(0.0f, 1.0f, 0.0f));
	b3dDrawLine(origin, origin + b3dDrawPoint(b3RotateVector(transform.q, b3v(0.0f, 0.0f, scale))),
	            MT_Vector3(0.0f, 0.0f, 1.0f));
}

static bool b3dDrawShapeCb(void *userShape, b3WorldTransform transform, b3HexColor color, void *context)
{
	/* Shapes are only handed over when the world has a createDebugShape()
	 * callback to tessellate them; without one the per shape bounding box from
	 * drawBounds is what the user sees. */
	const b3DebugShape *shape = static_cast<const b3DebugShape*>(userShape);
	if (shape && shape->type == b3_sphereShape && shape->sphere) {
		const MT_Vector3 center = b3dDrawPoint(transform.p) +
		                          b3dDrawPoint(b3RotateVector(transform.q, shape->sphere->center));
		b3dDrawSphereAt(center, shape->sphere->radius, b3dDrawColor(color));
	}
	return true;
}

static void b3dDrawString(b3Pos p, const char *text, b3HexColor color, void *context)
{
	/* the rasterizer has no world space text */
}

void B3DPhysicsEnvironment::DebugDrawWorld()
{
	if (m_debugMode <= 0 || !m_worldAlive)
		return;

	/* b3World_Draw() asserts that the drawing bounds are a valid AABB, so build
	 * one around the bodies this environment owns. */
	bool haveBounds = false;
	MT_Vector3 lower, upper;
	for (std::set<B3DPhysicsController*>::iterator it = m_controllers.begin(); it != m_controllers.end(); ++it) {
		b3BodyId body = (*it)->GetBodyId();
		if (!b3Body_IsValid(body))
			continue;
		const MT_Vector3 position = b3dDrawPoint(b3Body_GetPosition(body));
		if (!haveBounds) {
			lower = upper = position;
			haveBounds = true;
		}
		else {
			for (int i = 0; i < 3; i++) {
				if (position[i] < lower[i]) lower[i] = position[i];
				if (position[i] > upper[i]) upper[i] = position[i];
			}
		}
	}

	if (!haveBounds)
		return;

	const MT_Vector3 margin(10.0f, 10.0f, 10.0f);
	lower -= margin;
	upper += margin;

	b3DebugDraw draw = b3DefaultDebugDraw();
	draw.drawingBounds.lowerBound = b3v(lower.x(), lower.y(), lower.z());
	draw.drawingBounds.upperBound = b3v(upper.x(), upper.y(), upper.z());
	draw.DrawShapeFcn = &b3dDrawShapeCb;
	draw.DrawSegmentFcn = &b3dDrawSegmentCb;
	draw.DrawTransformFcn = &b3dDrawTransformCb;
	draw.DrawPointFcn = &b3dDrawPointCb;
	draw.DrawSphereFcn = &b3dDrawSphereCb;
	draw.DrawCapsuleFcn = &b3dDrawCapsuleCb;
	draw.DrawBoundsFcn = &b3dDrawBoundsCb;
	draw.DrawBoxFcn = &b3dDrawBoxCb;
	draw.DrawStringFcn = &b3dDrawString;
	draw.drawShapes = true;
	draw.drawBounds = true;
	draw.drawJoints = true;
	draw.drawJointExtras = true;
	draw.drawContacts = true;
	draw.drawContactNormals = true;
	draw.drawContactForces = false;
	draw.drawFrictionForces = false;
	draw.drawContactFeatures = false;
	draw.drawGraphColors = false;
	draw.drawIslands = false;
	draw.drawMass = false;
	draw.drawBodyNames = false;
	draw.forceScale = 1.0f;
	draw.jointScale = 1.0f;
	draw.context = NULL;

	b3World_Draw(m_worldId, &draw, ~(uint64_t)0);
}

void B3DPhysicsEnvironment::SetGravity(float x, float y, float z)
{
	m_gravity.setValue(x, y, z);
	if (m_worldAlive)
		b3World_SetGravity(m_worldId, b3v(x, y, z));
}

void B3DPhysicsEnvironment::GetGravity(MT_Vector3& grav)
{
	grav = m_gravity;
}

/* -------------------------------------------------------------------------
 * Joints
 * ------------------------------------------------------------------------- */

/* Bullet puts the hinge axis on the local Z of the joint frame
 * (btHingeConstraint's hinge axis is Z) and so does Box3D's revolute, spherical
 * and parallel joint:  the twist of b3SphericalJoint is about the frame's Z and
 * b3ParallelJoint aligns the two frames' Z axes.  The BGE passes the constraint
 * frame as three axis vectors, so every Box3D joint is built with the same
 * convention, the "main" axis of the BGE becomes the frame's Z. */
static b3Transform b3dMakeJointFrame(const MT_Vector3& axis, const MT_Vector3& axis1, const MT_Vector3& axis2,
                                     const b3Vec3& pivot)
{
	b3Matrix3 m;
	m.cx = b3v((float)axis1.x(), (float)axis1.y(), (float)axis1.z());
	m.cy = b3v((float)axis2.x(), (float)axis2.y(), (float)axis2.z());
	m.cz = b3v((float)axis.x(), (float)axis.y(), (float)axis.z());

	b3Transform t;
	t.p = pivot;
	t.q = b3MakeQuatFromMatrix(&m);
	return t;
}

/* b3PrismaticJoint slides along the X axis of its local frame (the revolute and
 * spherical joints rotate about Z), so the free degree of freedom of a 6DOF
 * sliding constraint has to end up as the frame's X.  A cyclic permutation of
 * the three frame axes keeps the frame right handed. */
static b3Transform b3dMakeSlidingFrame(const MT_Vector3& axis, const MT_Vector3& axis1, const MT_Vector3& axis2,
                                       int freeDof, const b3Vec3& pivot)
{
	const MT_Vector3 *axes[3] = { &axis, &axis1, &axis2 };
	int free = (freeDof >= 0 && freeDof < 3) ? freeDof : 0;

	/* b3dMakeJointFrame() puts its first argument into the frame's Z, so the
	 * prismatic slide axis has to be built here:  the matrix needs the free axis
	 * in its X column, with the remaining two in cyclic order to stay right
	 * handed. */
	MT_Vector3 x = *axes[free];
	MT_Vector3 y = *axes[(free + 1) % 3];
	MT_Vector3 z = *axes[(free + 2) % 3];

	b3Matrix3 m;
	m.cx = b3v((float)x.x(), (float)x.y(), (float)x.z());
	m.cy = b3v((float)y.x(), (float)y.y(), (float)y.z());
	m.cz = b3v((float)z.x(), (float)z.y(), (float)z.z());

	b3Transform t;
	t.p = pivot;
	t.q = b3MakeQuatFromMatrix(&m);
	return t;
}

static b3Transform b3dBodyTransform(b3BodyId bodyId)
{
	b3Transform t;
	b3Pos p = b3Body_GetPosition(bodyId);
	t.p = b3v((float)p.x, (float)p.y, (float)p.z);
	t.q = b3Body_GetRotation(bodyId);
	return t;
}

/* A degree of freedom is free when its lower limit is above its upper limit,
 * exactly like Bullet's 6DOF limits ("minLimit > maxLimit means free"). */
static bool b3dDofIsFree(const B3DPhysicsEnvironment::B3DJoint *jointDef, int dof)
{
	return jointDef->haveLimits[dof] && jointDef->lower[dof] > jointDef->upper[dof];
}

static bool b3dDofIsLocked(const B3DPhysicsEnvironment::B3DJoint *jointDef, int dof)
{
	return jointDef->haveLimits[dof] && jointDef->lower[dof] == jointDef->upper[dof];
}

B3DPhysicsEnvironment::B3DJoint* B3DPhysicsEnvironment::FindJoint(int constraintId) const
{
	std::map<int, B3DJoint*>::const_iterator it = m_joints.find(constraintId);
	return (it == m_joints.end()) ? NULL : it->second;
}

int B3DPhysicsEnvironment::CreateConstraint(PHY_IPhysicsController *ctrl, PHY_IPhysicsController *ctrl2,
                                            PHY_ConstraintType type,
                                            float pivotX, float pivotY, float pivotZ,
                                            float axis0X, float axis0Y, float axis0Z,
                                            float axis1X, float axis1Y, float axis1Z,
                                            float axis2X, float axis2Y, float axis2Z, int flag)
{
	if (!m_worldAlive)
		return 0;

	B3DPhysicsController *b3dA = dynamic_cast<B3DPhysicsController*>(ctrl);
	B3DPhysicsController *b3dB = dynamic_cast<B3DPhysicsController*>(ctrl2);

	if (!b3dA || !b3Body_IsValid(b3dA->GetBodyId()))
		return 0;

	if (type == PHY_VEHICLE_CONSTRAINT) {
		/* Box3D has no raycast vehicle: every wheel becomes a real wheel body
		 * driven by a wheel joint (see B3DVehicle), the BGE side of vehicles is
		 * unchanged.  A vehicle on a static chassis makes no sense either. */
		if (!b3dA->IsDynamic() && !b3dA->IsKinematic())
			return 0;
		B3DVehicle *vehicle = new B3DVehicle(m_worldId, b3dA->GetBodyId(), m_nextJointId++, type);
		m_vehicles.push_back(vehicle);
		return vehicle->GetUserConstraintId();
	}

	/* A constraint between two static bodies can never do anything, Bullet
	 * refuses it as well. */
	bool staticA = !b3dA->IsDynamic() && !b3dA->IsKinematic();
	bool staticB = (b3dB == NULL) || (!b3dB->IsDynamic() && !b3dB->IsKinematic());
	if (staticA && staticB)
		return 0;

	B3DJoint *jointDef = new B3DJoint();
	jointDef->id = m_nextJointId++;
	jointDef->type = type;
	jointDef->kind = B3D_JOINT_NONE;
	jointDef->ctrlA = b3dA;
	jointDef->ctrlB = b3dB;
	jointDef->anchorBody = b3_nullBodyId;
	jointDef->pivotA = b3v(pivotX, pivotY, pivotZ);
	jointDef->axisA = MT_Vector3(axis0X, axis0Y, axis0Z);
	jointDef->axis1A = MT_Vector3(axis1X, axis1Y, axis1Z);
	jointDef->axis2A = MT_Vector3(axis2X, axis2Y, axis2Z);
	jointDef->flags = flag;
	jointDef->created = false;
	jointDef->freeLinearDof = -1;
	jointDef->freeAngularDof = -1;
	jointDef->joint = b3_nullJointId;
	for (int i = 0; i < 6; i++) {
		jointDef->haveLimits[i] = false;
		jointDef->lower[i] = 0.0f;
		jointDef->upper[i] = 0.0f;
		jointDef->haveMotor[i] = false;
		jointDef->motorSpeed[i] = 0.0f;
		jointDef->motorForce[i] = 0.0f;
		jointDef->haveSpring[i] = false;
		jointDef->springStiffness[i] = 0.0f;
		jointDef->springDamping[i] = 0.0f;
	}

	/* The pin and hinge joints do not depend on limits that arrive later, the
	 * 6DOF one does, see EnsureJointCreated(). */
	switch (type) {
		case PHY_POINT2POINT_CONSTRAINT:
			jointDef->kind = B3D_JOINT_SPHERICAL;
			break;
		case PHY_LINEHINGE_CONSTRAINT:
			jointDef->kind = B3D_JOINT_REVOLUTE;
			break;
		case PHY_ANGULAR_CONSTRAINT:
			jointDef->kind = B3D_JOINT_PARALLEL;
			break;
		case PHY_CONE_TWIST_CONSTRAINT:
			jointDef->kind = B3D_JOINT_SPHERICAL;
			break;
		case PHY_GENERIC_6DOF_CONSTRAINT:
			jointDef->kind = B3D_JOINT_NONE;
			break;
		default:
			/* PHY_VEHICLE_CONSTRAINT and anything unknown have no Box3D
			 * counterpart. */
			delete jointDef;
			return 0;
	}

	m_joints[jointDef->id] = jointDef;

	if (jointDef->kind != B3D_JOINT_NONE && !EnsureJointCreated(jointDef)) {
		m_joints.erase(jointDef->id);
		delete jointDef;
		return 0;
	}

	return jointDef->id;
}

b3JointId B3DPhysicsEnvironment::CreateMappedJoint(B3DJoint *jointDef)
{
	b3BodyId bodyA = jointDef->ctrlA->GetBodyId();
	b3BodyId bodyB = b3_nullBodyId;

	b3Transform localA;
	if (jointDef->kind == B3D_JOINT_PRISMATIC) {
		localA = b3dMakeSlidingFrame(jointDef->axisA, jointDef->axis1A, jointDef->axis2A,
		                             (jointDef->freeLinearDof >= 0) ? jointDef->freeLinearDof : 0,
		                             jointDef->pivotA);
	}
	else {
		localA = b3dMakeJointFrame(jointDef->axisA, jointDef->axis1A, jointDef->axis2A, jointDef->pivotA);
	}

	if (jointDef->ctrlB) {
		bodyB = jointDef->ctrlB->GetBodyId();
	}
	else if (b3Body_IsValid(jointDef->anchorBody)) {
		bodyB = jointDef->anchorBody;
	}
	else {
		/* Box3D joints always connect two bodies, so a constraint to "nothing"
		 * gets a shapeless static body at the pivot:  without shapes it never
		 * reaches the broad phase, so it cannot be seen or touched, and a static
		 * body is exactly "the world". */
		b3BodyDef bd = b3DefaultBodyDef();
		bd.type = b3_staticBody;

		b3Transform t = b3MulTransforms(b3dBodyTransform(bodyA), localA);
		b3Pos p;
		p.x = t.p.x;
		p.y = t.p.y;
		p.z = t.p.z;
		bd.position = p;
		bd.rotation = t.q;

		jointDef->anchorBody = b3CreateBody(m_worldId, &bd);
		if (!b3Body_IsValid(jointDef->anchorBody))
			return b3_nullJointId;
		bodyB = jointDef->anchorBody;
	}

	b3Transform localB = b3InvMulTransforms(b3dBodyTransform(bodyB),
	                                        b3MulTransforms(b3dBodyTransform(bodyA), localA));

	/* Bullet exposes the flag as "disable collision between linked bodies",
	 * Box3D spells the same thing collideConnected = false. */
	bool collideConnected = (jointDef->flags & B3D_CONSTRAINT_DISABLE_LINKED_COLLISION) == 0;

	b3JointId joint = b3_nullJointId;

	switch (jointDef->kind) {
		case B3D_JOINT_REVOLUTE: {
			b3RevoluteJointDef def = b3DefaultRevoluteJointDef();
			def.base.bodyIdA = bodyA;
			def.base.bodyIdB = bodyB;
			def.base.localFrameA = localA;
			def.base.localFrameB = localB;
			def.base.collideConnected = collideConnected;
			joint = b3CreateRevoluteJoint(m_worldId, &def);
			break;
		}
		case B3D_JOINT_SPHERICAL: {
			b3SphericalJointDef def = b3DefaultSphericalJointDef();
			def.base.bodyIdA = bodyA;
			def.base.bodyIdB = bodyB;
			def.base.localFrameA = localA;
			def.base.localFrameB = localB;
			def.base.collideConnected = collideConnected;
			joint = b3CreateSphericalJoint(m_worldId, &def);
			break;
		}
		case B3D_JOINT_PRISMATIC: {
			b3PrismaticJointDef def = b3DefaultPrismaticJointDef();
			def.base.bodyIdA = bodyA;
			def.base.bodyIdB = bodyB;
			def.base.localFrameA = localA;
			def.base.localFrameB = localB;
			def.base.collideConnected = collideConnected;
			joint = b3CreatePrismaticJoint(m_worldId, &def);
			break;
		}
		case B3D_JOINT_WELD: {
			b3WeldJointDef def = b3DefaultWeldJointDef();
			def.base.bodyIdA = bodyA;
			def.base.bodyIdB = bodyB;
			def.base.localFrameA = localA;
			def.base.localFrameB = localB;
			def.base.collideConnected = collideConnected;
			joint = b3CreateWeldJoint(m_worldId, &def);
			break;
		}
		case B3D_JOINT_PARALLEL: {
			b3ParallelJointDef def = b3DefaultParallelJointDef();
			def.base.bodyIdA = bodyA;
			def.base.bodyIdB = bodyB;
			def.base.localFrameA = localA;
			def.base.localFrameB = localB;
			def.base.collideConnected = collideConnected;
			/* The parallel joint is a spring, a stiff one behaves like the
			 * angular only hinge Bullet builds for PHY_ANGULAR_CONSTRAINT. */
			def.maxTorque = B3D_ANGULAR_JOINT_MAX_TORQUE;
			joint = b3CreateParallelJoint(m_worldId, &def);
			break;
		}
		default:
			break;
	}

	if (!b3Joint_IsValid(joint)) {
		if (b3Body_IsValid(jointDef->anchorBody)) {
			b3DestroyBody(jointDef->anchorBody);
			jointDef->anchorBody = b3_nullBodyId;
		}
		return b3_nullJointId;
	}

	b3Joint_SetUserData(joint, jointDef);

	return joint;
}

bool B3DPhysicsEnvironment::EnsureJointCreated(B3DJoint *jointDef)
{
	if (jointDef->created)
		return true;

	if (jointDef->kind == B3D_JOINT_NONE) {
		/* Map the six degrees of freedom of a 6DOF constraint onto the joints
		 * Box3D provides.  "free" is lower > upper, "locked" is lower == upper
		 * and anything else is a limited axis.  The BGE sends the limits of all
		 * six degrees of freedom in a row, so wait for the last of them before
		 * deciding. */
		for (int i = 0; i < 6; i++) {
			if (!jointDef->haveLimits[i])
				return false;
		}

		int linearFree = -1, linearLimited = 0, linearLocked = 0;
		int angularFree = -1, angularLimited = 0, angularLocked = 0;
		for (int i = 0; i < 3; i++) {
			if (b3dDofIsFree(jointDef, i)) {
				linearFree = i;
			}
			else if (b3dDofIsLocked(jointDef, i)) {
				linearLocked++;
			}
			else {
				linearLimited++;
			}

			if (b3dDofIsFree(jointDef, 3 + i)) {
				angularFree = i;
			}
			else if (b3dDofIsLocked(jointDef, 3 + i)) {
				angularLocked++;
			}
			else {
				angularLimited++;
			}
		}

		if (linearFree < 0 && linearLimited == 0 && angularFree < 0 && angularLimited == 0) {
			/* everything locked */
			jointDef->kind = B3D_JOINT_WELD;
		}
		else if (linearFree == 0 || linearFree == 1 || linearFree == 2) {
			if (linearLocked + 1 == 3 && angularLocked == 3) {
				jointDef->kind = B3D_JOINT_PRISMATIC;
				jointDef->freeLinearDof = linearFree;
			}
			else {
				return false;
			}
		}
		else if (angularFree >= 0) {
			if (linearLocked == 3 && angularLocked + 1 == 3) {
				jointDef->kind = B3D_JOINT_REVOLUTE;
				jointDef->freeAngularDof = angularFree;
			}
			else if (linearLocked == 3 && linearLimited == 0) {
				/* The pivot is locked and the swing is free: a ball socket.  Its
				 * angles act as limits of the swing and the twist. */
				jointDef->kind = B3D_JOINT_SPHERICAL;
			}
			else {
				return false;
			}
		}
		else if (angularLimited > 0 && linearLocked == 3 && linearLimited == 0) {
			jointDef->kind = B3D_JOINT_SPHERICAL;
		}
		else {
			return false;
		}
	}

	if (jointDef->kind == B3D_JOINT_NONE)
		return false;

	b3JointId joint = CreateMappedJoint(jointDef);
	if (!b3Joint_IsValid(joint)) {
		if (!m_jointMappingReported) {
			printf("B3DPhysics: this Box3D build cannot express constraint type %d\n",
			       (int)jointDef->type);
			m_jointMappingReported = true;
		}
		return false;
	}

	jointDef->joint = joint;
	jointDef->created = true;

	ApplyLimits(jointDef);
	ApplyMotors(jointDef);
	ApplySprings(jointDef);
	return true;
}

void B3DPhysicsEnvironment::DestroyJoint(B3DJoint *jointDef)
{
	if (b3Joint_IsValid(jointDef->joint)) {
		b3DestroyJoint(jointDef->joint, true);
		jointDef->joint = b3_nullJointId;
	}
	if (b3Body_IsValid(jointDef->anchorBody)) {
		b3DestroyBody(jointDef->anchorBody);
		jointDef->anchorBody = b3_nullBodyId;
	}
	jointDef->created = false;
}

/* The limits of a 6DOF constraint arrive after its creation, so the joints that
 * could not be mapped yet are built on the first step that follows. */
void B3DPhysicsEnvironment::CreatePendingJoints()
{
	for (std::map<int, B3DJoint*>::iterator it = m_joints.begin(); it != m_joints.end(); ++it) {
		B3DJoint *jointDef = it->second;
		if (jointDef->created)
			continue;
		if (!EnsureJointCreated(jointDef) && !m_jointMappingReported) {
			printf("B3DPhysics: this Box3D build cannot express constraint type %d\n",
			       (int)jointDef->type);
			m_jointMappingReported = true;
		}
	}
}

void B3DPhysicsEnvironment::ApplyLimits(B3DJoint *jointDef)
{
	switch (jointDef->kind) {
		case B3D_JOINT_REVOLUTE: {
			/* The BGE addresses the hinge angle as degree of freedom 3. */
			int dof = (jointDef->freeAngularDof >= 0) ? 3 + jointDef->freeAngularDof : 3;
			if (jointDef->haveLimits[dof]) {
				float lower = jointDef->lower[dof];
				float upper = jointDef->upper[dof];
				/* lower > upper means free, see the BGE documentation. */
				bool enable = lower <= upper;
				if (lower > upper) {
					lower = 0.0f;
					upper = 0.0f;
				}
				b3RevoluteJoint_EnableLimit(jointDef->joint, true);
				b3RevoluteJoint_SetLimits(jointDef->joint, lower, upper);
				if (!enable)
					b3RevoluteJoint_EnableLimit(jointDef->joint, false);
			}
			/* An angular axis that is locked is not expressible with a revolute
			 * joint, Box3D only allows limits around its one free axis. */
			break;
		}
		case B3D_JOINT_PRISMATIC: {
			int dof = (jointDef->freeLinearDof >= 0) ? jointDef->freeLinearDof : 0;
			if (jointDef->haveLimits[dof]) {
				float lower = jointDef->lower[dof];
				float upper = jointDef->upper[dof];
				bool enable = lower <= upper;
				if (lower > upper) {
					lower = 0.0f;
					upper = 0.0f;
				}
				b3PrismaticJoint_EnableLimit(jointDef->joint, true);
				b3PrismaticJoint_SetLimits(jointDef->joint, lower, upper);
				if (!enable)
					b3PrismaticJoint_EnableLimit(jointDef->joint, false);
			}
			break;
		}
		case B3D_JOINT_SPHERICAL: {
			/* Cone twist: Bullet uses the X and Y Euler limits as the cone and Z
			 * as the twist, Box3D has one cone angle and a twist range. */
			if (jointDef->type == PHY_CONE_TWIST_CONSTRAINT) {
				float cone = -1.0f;
				for (int i = 3; i <= 4; i++) {
					if (jointDef->haveLimits[i] && jointDef->lower[i] <= jointDef->upper[i]) {
						float limit = jointDef->upper[i];
						if (limit > 0.0f && (cone < 0.0f || limit < cone))
							cone = limit;
					}
				}
				if (cone >= 0.0f) {
					b3SphericalJoint_SetConeLimit(jointDef->joint, cone);
					b3SphericalJoint_EnableConeLimit(jointDef->joint, true);
				}

				if (jointDef->haveLimits[5]) {
					float lower = jointDef->lower[5];
					float upper = jointDef->upper[5];
					if (lower <= upper) {
						b3SphericalJoint_SetTwistLimits(jointDef->joint, lower, upper);
						b3SphericalJoint_EnableTwistLimit(jointDef->joint, true);
					}
				}
			}
			break;
		}
		default:
			break;
	}
}

void B3DPhysicsEnvironment::ApplyMotors(B3DJoint *jointDef)
{
	/* The BGE uses 6,7,8 for the linear and 9,10,11 for the angular motors of a
	 * 6DOF constraint, Bullet takes a target velocity and a maximum force. */
	for (int i = 0; i < 3; i++) {
		int linear = i;
		if (!jointDef->haveMotor[linear])
			continue;
		if (jointDef->kind != B3D_JOINT_PRISMATIC)
			continue;
		if (jointDef->freeLinearDof != i)
			continue;
		b3PrismaticJoint_SetMotorSpeed(jointDef->joint, jointDef->motorSpeed[linear]);
		b3PrismaticJoint_SetMaxMotorForce(jointDef->joint, jointDef->motorForce[linear]);
		b3PrismaticJoint_EnableMotor(jointDef->joint, jointDef->motorForce[linear] > 0.0f);
	}

	for (int i = 0; i < 3; i++) {
		int angular = 3 + i;
		if (!jointDef->haveMotor[angular])
			continue;
		if (jointDef->kind == B3D_JOINT_REVOLUTE) {
			if (jointDef->freeAngularDof >= 0 && jointDef->freeAngularDof != i)
				continue;
			b3RevoluteJoint_SetMotorSpeed(jointDef->joint, jointDef->motorSpeed[angular]);
			b3RevoluteJoint_SetMaxMotorTorque(jointDef->joint, jointDef->motorForce[angular]);
			b3RevoluteJoint_EnableMotor(jointDef->joint, jointDef->motorForce[angular] > 0.0f);
		}
	}
}

void B3DPhysicsEnvironment::ApplySprings(B3DJoint *jointDef)
{
	for (int i = 0; i < 3; i++) {
		int linear = i;
		if (jointDef->haveSpring[linear] && jointDef->kind == B3D_JOINT_PRISMATIC &&
		    jointDef->freeLinearDof == i) {
			b3PrismaticJoint_SetSpringHertz(jointDef->joint, jointDef->springStiffness[linear]);
			b3PrismaticJoint_SetSpringDampingRatio(jointDef->joint, jointDef->springDamping[linear]);
			b3PrismaticJoint_EnableSpring(jointDef->joint, true);
		}
	}

	for (int i = 0; i < 3; i++) {
		int angular = 3 + i;
		if (jointDef->haveSpring[angular] && jointDef->kind == B3D_JOINT_REVOLUTE) {
			if (jointDef->freeAngularDof >= 0 && jointDef->freeAngularDof != i)
				continue;
			b3RevoluteJoint_SetSpringHertz(jointDef->joint, jointDef->springStiffness[angular]);
			b3RevoluteJoint_SetSpringDampingRatio(jointDef->joint, jointDef->springDamping[angular]);
			b3RevoluteJoint_EnableSpring(jointDef->joint, true);
		}
	}
}

void B3DPhysicsEnvironment::RemoveConstraintById(int constraintid)
{
	for (size_t i = 0; i < m_vehicles.size(); i++) {
		if (m_vehicles[i]->GetUserConstraintId() == constraintid) {
			delete m_vehicles[i];
			m_vehicles.erase(m_vehicles.begin() + i);
			return;
		}
	}

	std::map<int, B3DJoint*>::iterator it = m_joints.find(constraintid);
	if (it == m_joints.end())
		return;

	DestroyJoint(it->second);
	delete it->second;
	m_joints.erase(it);
}

void B3DPhysicsEnvironment::SetupObjectConstraints(KX_GameObject *obj_src, KX_GameObject *obj_dest,
                                                   bRigidBodyJointConstraint *dat)
{
	PHY_IPhysicsController *phy_src = obj_src->GetPhysicsController();
	PHY_IPhysicsController *phy_dest = obj_dest->GetPhysicsController();

	if (!phy_src || !phy_dest)
		return;

	/* The Blender constraint stores the constraint frame as Euler angles and the
	 * pivot in the source object's local space, exactly what CreateConstraint()
	 * wants:  the scale is taken into account the way Bullet does it. */
	MT_Matrix3x3 localCFrame(MT_Vector3(dat->axX, dat->axY, dat->axZ));
	MT_Vector3 axis0 = localCFrame.getColumn(0);
	MT_Vector3 axis1 = localCFrame.getColumn(1);
	MT_Vector3 axis2 = localCFrame.getColumn(2);
	MT_Vector3 scale = obj_src->NodeGetWorldScaling();

	int constraintId = CreateConstraint(
		phy_src, phy_dest, (PHY_ConstraintType)dat->type,
		(float)(dat->pivX * scale.x()), (float)(dat->pivY * scale.y()), (float)(dat->pivZ * scale.z()),
		(float)(axis0.x() * scale.x()), (float)(axis0.y() * scale.y()), (float)(axis0.z() * scale.z()),
		(float)(axis1.x() * scale.x()), (float)(axis1.y() * scale.y()), (float)(axis1.z() * scale.z()),
		(float)(axis2.x() * scale.x()), (float)(axis2.y() * scale.y()), (float)(axis2.z() * scale.z()),
		dat->flag);

	if (!constraintId)
		return;

	/* Which degrees of freedom the Blender constraint exposes through the "limit"
	 * flags, see CcdPhysicsEnvironment::SetupObjectConstraints(). */
	int dof = 0;
	int dof_max = 0;
	int dofbit = 0;

	switch (dat->type) {
		case PHY_GENERIC_6DOF_CONSTRAINT:
			dof_max = 6;
			dofbit = 1;
			break;
		case PHY_CONE_TWIST_CONSTRAINT:
			dof = 3;
			dof_max = 6;
			dofbit = 1 << 3;
			break;
		case PHY_LINEHINGE_CONSTRAINT:
		case PHY_ANGULAR_CONSTRAINT:
			dof = 3;
			dof_max = 4;
			dofbit = 1 << 3;
			break;
		default:
			break;
	}

	for (; dof < dof_max; dof++) {
		if (dat->flag & dofbit)
			SetConstraintParam(constraintId, dof, dat->minLimit[dof], dat->maxLimit[dof]);
		else	/* minLimit > maxLimit means free (no limit) for this degree of freedom */
			SetConstraintParam(constraintId, dof, 1.0f, -1.0f);
		dofbit <<= 1;
	}
}

float B3DPhysicsEnvironment::GetAppliedImpulse(int constraintid)
{
	B3DJoint *jointDef = FindJoint(constraintid);
	if (!jointDef || !jointDef->created || !b3Joint_IsValid(jointDef->joint))
		return 0.0f;

	/* Bullet reports the impulse the solver applied to the constraint.  Box3D
	 * exposes the constraint force and torque of the last step instead, this is
	 * its magnitude. */
	b3Vec3 force = b3Joint_GetConstraintForce(jointDef->joint);
	b3Vec3 torque = b3Joint_GetConstraintTorque(jointDef->joint);
	float linear = sqrtf(force.x * force.x + force.y * force.y + force.z * force.z);
	float angular = sqrtf(torque.x * torque.x + torque.y * torque.y + torque.z * torque.z);
	return linear + angular;
}

PHY_IVehicle* B3DPhysicsEnvironment::GetVehicleConstraint(int constraintId)
{
	for (size_t i = 0; i < m_vehicles.size(); i++) {
		if (m_vehicles[i]->GetUserConstraintId() == constraintId)
			return m_vehicles[i];
	}

	return NULL;
}

void B3DPhysicsEnvironment::SyncVehicles(float timeStep)
{
	/* Box3D wheels are bodies, Bullet's are raycast results: the visual wheel
	 * objects have to be moved to the wheel bodies after every step, exactly like
	 * CcdPhysicsEnvironment::ProceedDeltaTime() does it. */
	for (size_t i = 0; i < m_vehicles.size(); i++)
		m_vehicles[i]->SyncWheels(timeStep);
}

PHY_ICharacter* B3DPhysicsEnvironment::GetCharacterController(KX_GameObject *ob)
{
	if (!ob)
		return NULL;

	B3DPhysicsController *ctrl = static_cast<B3DPhysicsController*>(ob->GetPhysicsController());
	return ctrl ? ctrl->GetCharacter() : NULL;
}

void B3DPhysicsEnvironment::UpdateCharacters(float timeStep)
{
	/* A character moves itself with Box3D's capsule mover, so it has to be put in
	 * place before the step that is supposed to see it. */
	for (std::set<B3DPhysicsController*>::iterator it = m_controllers.begin(); it != m_controllers.end(); ++it) {
		B3DCharacter *character = (*it)->GetCharacter();
		if (character)
			character->Update(timeStep);
	}
}

/* RayTest() lives further down, together with the rest of the query code. */

/* -------------------------------------------------------------------------
 * Queries
 * ------------------------------------------------------------------------- */

struct B3DRayCastContext
{
	PHY_IRayCastFilterCallback *filterCallback;
	PHY_RayCastResult *result;
	B3DPhysicsEnvironment *env;
	float bestFraction;
	bool hit;
};

/** Callback of b3World_CastRay(): keeps the closest hit that passes the filter. */
static float b3dRayCastCallback(b3ShapeId shapeId, b3Pos point, b3Vec3 normal, float fraction,
                                uint64_t userMaterialId, int triangleIndex, int childIndex,
                                void *context)
{
	B3DRayCastContext *ctx = static_cast<B3DRayCastContext*>(context);
	(void)userMaterialId;
	(void)triangleIndex;
	(void)childIndex;

	B3DPhysicsController *ctrl = ctx->env->ControllerFromShape(shapeId);
	if (!ctrl)
		return -1.0f;

	/* Sensors (ghosts) are transparent for rays, exactly like Bullet's
	 * SensorFilter, and so is the controller the caller asked to ignore. */
	if (ctrl->IsSensor())
		return -1.0f;
	if (ctx->filterCallback->m_ignoreController == ctrl)
		return -1.0f;
	if (!ctx->filterCallback->needBroadphaseRayCast(ctrl))
		return -1.0f;

	if (!ctx->hit || fraction < ctx->bestFraction) {
		ctx->hit = true;
		ctx->bestFraction = fraction;
		ctx->result->m_controller = ctrl;
		ctx->result->m_hitPoint[0] = (MT_Scalar)point.x;
		ctx->result->m_hitPoint[1] = (MT_Scalar)point.y;
		ctx->result->m_hitPoint[2] = (MT_Scalar)point.z;
		ctx->result->m_hitNormal[0] = (MT_Scalar)normal.x;
		ctx->result->m_hitNormal[1] = (MT_Scalar)normal.y;
		ctx->result->m_hitNormal[2] = (MT_Scalar)normal.z;
		/* Box3D reports the material and the triangle per hit; the UV and polygon
		 * lookup Bullet performs for meshes has no counterpart here. */
		ctx->result->m_meshObject = NULL;
		ctx->result->m_polygon = -1;
		ctx->result->m_hitUVOK = 0;
	}

	/* Clip the ray to this hit so that only closer hits are considered. */
	return fraction;
}

PHY_IPhysicsController* B3DPhysicsEnvironment::RayTest(PHY_IRayCastFilterCallback& filterCallback,
                                                      float fromX, float fromY, float fromZ,
                                                      float toX, float toY, float toZ)
{
	if (!m_worldAlive)
		return NULL;

	b3Pos origin;
	origin.x = fromX;
	origin.y = fromY;
	origin.z = fromZ;
	b3Vec3 translation = b3v(toX - fromX, toY - fromY, toZ - fromZ);

	b3QueryFilter filter;
	/* Accept every category; the per controller filtering happens in the
	 * callback above. */
	filter.categoryBits = ~(uint64_t)0;
	filter.maskBits = ~(uint64_t)0;
	filter.id = 0;
	filter.name = NULL;

	PHY_RayCastResult result;
	memset(&result, 0, sizeof(result));

	B3DRayCastContext ctx;
	ctx.filterCallback = &filterCallback;
	ctx.result = &result;
	ctx.env = this;
	ctx.bestFraction = 1.0f;
	ctx.hit = false;

	b3World_CastRay(m_worldId, origin, translation, filter, b3dRayCastCallback, &ctx);

	if (!ctx.hit)
		return NULL;

	filterCallback.reportHit(&result);
	return result.m_controller;
}

bool B3DPhysicsEnvironment::CullingTest(PHY_CullingCallback callback, void *userData,
                                        MT_Vector4* planeNormals, int planeNumber, int occlusionRes,
                                        const int *viewport, float modelview[16], float projection[16])
{
	/* No DBVT based occlusion culling for Box3D; KX_Scene falls back to its
	 * brute force visibility pass when this returns false. */
	(void)callback; (void)userData; (void)planeNormals; (void)planeNumber;
	(void)occlusionRes; (void)viewport; (void)modelview; (void)projection;
	return false;
}

void B3DPhysicsEnvironment::SetConstraintParam(int constraintId, int param, float value, float value1)
{
	B3DJoint *jointDef = FindJoint(constraintId);
	if (!jointDef)
		return;

	/* The limits of a 6DOF constraint are its mapping, so they have to be
	 * collected before the Box3D joint can be built, see EnsureJointCreated(). */
	if (param >= 0 && param <= 5) {
		jointDef->haveLimits[param] = true;
		jointDef->lower[param] = value;
		jointDef->upper[param] = value1;
		if (!jointDef->created)
			EnsureJointCreated(jointDef);
		else
			ApplyLimits(jointDef);
		return;
	}

	if (param >= 6 && param <= 8) {
		int dof = param - 6;
		jointDef->haveMotor[dof] = true;
		jointDef->motorSpeed[dof] = value;
		jointDef->motorForce[dof] = value1;
		if (jointDef->created)
			ApplyMotors(jointDef);
		return;
	}

	if (param >= 9 && param <= 11) {
		int dof = param - 9;
		jointDef->haveMotor[3 + dof] = true;
		jointDef->motorSpeed[3 + dof] = value;
		jointDef->motorForce[3 + dof] = value1;
		if (jointDef->created)
			ApplyMotors(jointDef);
		return;
	}

	if (param >= 12 && param <= 17) {
		int dof = param - 12;
		jointDef->haveSpring[dof] = true;
		/* Bullet takes a stiffness and a damping here, Box3D a frequency and a
		 * damping ratio; the value is used as the frequency and clamped to the
		 * range Box3D accepts. */
		jointDef->springStiffness[dof] = (value > 0.0f) ? value : 0.0f;
		jointDef->springDamping[dof] = (value1 > 0.0f) ? value1 : 0.0f;
		if (jointDef->created)
			ApplySprings(jointDef);
		return;
	}
}

float B3DPhysicsEnvironment::GetConstraintParam(int constraintId, int param)
{
	B3DJoint *jointDef = FindJoint(constraintId);
	if (!jointDef || !jointDef->created)
		return 0.0f;

	/* Mirrors Bullet: 0..2 are the relative linear positions, 3..5 the relative
	 * angular positions of the constraint. */
	switch (jointDef->kind) {
		case B3D_JOINT_PRISMATIC:
			if (param == jointDef->freeLinearDof)
				return b3PrismaticJoint_GetTranslation(jointDef->joint);
			return 0.0f;
		case B3D_JOINT_REVOLUTE: {
			int dof = (jointDef->freeAngularDof >= 0) ? 3 + jointDef->freeAngularDof : 3;
			if (param == dof)
				return b3RevoluteJoint_GetAngle(jointDef->joint);
			return 0.0f;
		}
		case B3D_JOINT_SPHERICAL:
			if (param == 5)
				return b3SphericalJoint_GetTwistAngle(jointDef->joint);
			return 0.0f;
		default:
			return 0.0f;
	}
}

/* -------------------------------------------------------------------------
 * Sensors and callbacks
 * ------------------------------------------------------------------------- */

void B3DPhysicsEnvironment::AddSensor(PHY_IPhysicsController *ctrl)
{
	B3DPhysicsController *b3dctrl = dynamic_cast<B3DPhysicsController*>(ctrl);
	if (!b3dctrl)
		return;

	/* Unlike Bullet, a Box3D body is part of its world from the moment it is
	 * created, so there is nothing to insert here; this only has to make sure the
	 * controller is stepped. */
	AddController(b3dctrl);
	b3dctrl->SetInWorld(true);
}

void B3DPhysicsEnvironment::RemoveSensor(PHY_IPhysicsController *ctrl)
{
	/* A Box3D body cannot be taken out of its world without being destroyed, and
	 * the owning object still needs its motion state synchronised, so the
	 * controller stays registered.  Sensor shapes never generate a collision
	 * response, so leaving them in the world is harmless. */
	(void)ctrl;
}

void B3DPhysicsEnvironment::AddTouchCallback(int response_class, PHY_ResponseCallback callback, void *user)
{
	m_touchCallbacks[response_class] = callback;
	m_touchCallbackUser[response_class] = user;
}

bool B3DPhysicsEnvironment::RequestCollisionCallback(PHY_IPhysicsController *ctrl)
{
	B3DPhysicsController *b3dctrl = dynamic_cast<B3DPhysicsController*>(ctrl);
	if (!b3dctrl)
		return false;

	/* Several sensors can watch the same object; Register() is the reference
	 * counted equivalent of CcdPhysicsController::Register(). */
	return b3dctrl->Register();
}

bool B3DPhysicsEnvironment::RemoveCollisionCallback(PHY_IPhysicsController *ctrl)
{
	B3DPhysicsController *b3dctrl = dynamic_cast<B3DPhysicsController*>(ctrl);
	if (!b3dctrl)
		return false;

	return b3dctrl->Unregister();
}

B3DPhysicsController* B3DPhysicsEnvironment::ControllerFromShape(b3ShapeId shapeId) const
{
	if (!b3Shape_IsValid(shapeId))
		return NULL;

	b3BodyId bodyId = b3Shape_GetBody(shapeId);
	if (!b3Body_IsValid(bodyId))
		return NULL;

	/* Every body this environment creates stores its controller as user data. */
	return static_cast<B3DPhysicsController*>(b3Body_GetUserData(bodyId));
}

/** Is this controller the proxy of a Near or Radar sensor? Those carry a
 *  KX_ClientObjectInfo of type SENSOR, see KX_TouchEventManager. */
static bool b3dIsBroadPhaseSensor(B3DPhysicsController *ctrl)
{
	if (!ctrl)
		return false;
	const KX_ClientObjectInfo *info = static_cast<const KX_ClientObjectInfo*>(ctrl->GetNewClientInfo());
	return info && info->m_type == KX_ClientObjectInfo::SENSOR;
}

/**
 * Fill in the contact information handed to the touch callbacks.
 *
 * For a real contact Box3D provides the manifold, so the point and the normal
 * are exact.  Sensor (ghost) overlaps have no manifold, there the two body
 * centres are used instead, which is what the shape of the data allows.
 */
static void b3dMakeCollData(b3Pos posA, b3Pos posB, const b3Vec3 *normal, const b3Pos *point,
                            PHY_CollData& collData){
	if (point && normal) {
		collData.m_point1.setValue((MT_Scalar)point->x, (MT_Scalar)point->y, (MT_Scalar)point->z);
		collData.m_point2 = collData.m_point1;
		collData.m_normal.setValue((MT_Scalar)normal->x, (MT_Scalar)normal->y, (MT_Scalar)normal->z);
		return;
	}

	b3Vec3 dir = b3SubPos(posB, posA);
	float len = b3Length(dir);
	if (len > 1e-6f)
		dir = b3MulSV(1.0f / len, dir);
	else
		dir = b3v(0.0f, 0.0f, 1.0f);

	/* point1 is on the object the callback is about, so mirror the orientation
	 * of the normal the same way CcdPhysicsEnvironment::CallbackTriggers does. */
	collData.m_point1.setValue((MT_Scalar)posA.x, (MT_Scalar)posA.y, (MT_Scalar)posA.z);
	collData.m_point2.setValue((MT_Scalar)posB.x, (MT_Scalar)posB.y, (MT_Scalar)posB.z);
	collData.m_normal.setValue((MT_Scalar)dir.x, (MT_Scalar)dir.y, (MT_Scalar)dir.z);
}

void B3DPhysicsEnvironment::ProcessEvents()
{
	if (!m_worldAlive)
		return;

	PHY_ResponseCallback objectCb = NULL;
	void *objectUser = NULL;
	PHY_ResponseCallback broadPhaseCb = NULL;
	void *broadPhaseUser = NULL;

	std::map<int, PHY_ResponseCallback>::iterator cbIt = m_touchCallbacks.find(PHY_OBJECT_RESPONSE);
	if (cbIt != m_touchCallbacks.end()) {
		objectCb = cbIt->second;
		objectUser = m_touchCallbackUser[PHY_OBJECT_RESPONSE];
	}
	cbIt = m_touchCallbacks.find(PHY_BROADPH_RESPONSE);
	if (cbIt != m_touchCallbacks.end()) {
		broadPhaseCb = cbIt->second;
		broadPhaseUser = m_touchCallbackUser[PHY_BROADPH_RESPONSE];
	}

	if (!objectCb && !broadPhaseCb)
		return;

	/* Box3D reports the *begin* and the *end* of a touch, while the BGE (and
	 * Bullet, which reports every manifold of every step) expects a report for as
	 * long as two objects overlap.  Contacts are therefore polled instead of
	 * listened to: b3Body_GetContactData() returns the manifolds of every
	 * touching contact, which also covers the static triangle meshes that never
	 * produce contact events.  The sensor proxies (Near/Radar and ghosts) have no
	 * contacts at all, their overlaps come from the sensor events and are kept
	 * in m_activeTouches. */

	/* ---- sensor overlaps: ghosts and the Near/Radar proxies ---- */
	b3SensorEvents sensors = b3World_GetSensorEvents(m_worldId);
	for (int i = 0; i < sensors.beginCount; i++) {
		const b3SensorBeginTouchEvent& ev = sensors.beginEvents[i];

		B3DPhysicsController *ctrlA = ControllerFromShape(ev.sensorShapeId);
		B3DPhysicsController *ctrlB = ControllerFromShape(ev.visitorShapeId);
		if (!ctrlA || !ctrlB || ctrlA == ctrlB)
			continue;

		/* The broad phase callback expects the sensor proxy first, exactly like
		 * the object1/object2 order KX_TouchEventManager::newBroadphaseResponse
		 * checks.  Its result also decides whether the BGE is interested in the
		 * pair at all: the Near/Radar filter rejects the sensor's own object and
		 * objects outside its filter, and Bullet would never create such a
		 * pair. */
		B3DPhysicsController *first = ctrlA;
		B3DPhysicsController *second = ctrlB;
		if (!b3dIsBroadPhaseSensor(ctrlA) && b3dIsBroadPhaseSensor(ctrlB)) {
			first = ctrlB;
			second = ctrlA;
		}

		PHY_CollData collData;
		b3dMakeCollData(b3Body_GetPosition(b3Shape_GetBody(ev.sensorShapeId)),
		                b3Body_GetPosition(b3Shape_GetBody(ev.visitorShapeId)),
		                NULL, NULL, collData);

		bool keep = true;
		if (broadPhaseCb && b3dIsBroadPhaseSensor(first))
			keep = broadPhaseCb(broadPhaseUser, first, second, &collData) != 0;

		if (keep)
			m_activeTouches.insert(std::make_pair(ctrlA, ctrlB));
	}

	for (int i = 0; i < sensors.endCount; i++) {
		const b3SensorEndTouchEvent& ev = sensors.endEvents[i];
		B3DPhysicsController *ctrlA = ControllerFromShape(ev.sensorShapeId);
		B3DPhysicsController *ctrlB = ControllerFromShape(ev.visitorShapeId);
		if (ctrlA && ctrlB)
			m_activeTouches.erase(std::make_pair(ctrlA, ctrlB));
	}

	/* A controller that left the environment (or was destroyed) cannot be
	 * reported any more, drop its pairs. */
	for (std::set<std::pair<B3DPhysicsController*, B3DPhysicsController*> >::iterator it =
	         m_activeTouches.begin(); it != m_activeTouches.end(); ) {
		if (m_controllers.find(it->first) == m_controllers.end() ||
		    m_controllers.find(it->second) == m_controllers.end())
			m_activeTouches.erase(it++);
		else
			++it;
	}

	if (!objectCb)
		return;

	/* ---- touching contacts, polled from the bodies that own a collision
	 * sensor (a handful of objects in a typical scene) ---- */
	for (std::set<B3DPhysicsController*>::iterator it = m_controllers.begin();
	     it != m_controllers.end(); ++it) {
		B3DPhysicsController *ctrlA = *it;
		if (!ctrlA->Registered() || ctrlA->IsSensor())
			continue;

		b3BodyId bodyA = ctrlA->GetBodyId();
		if (!b3Body_IsValid(bodyA))
			continue;

		b3ContactData data[B3D_MAX_POLLED_CONTACTS];
		int count = b3Body_GetContactData(bodyA, data, B3D_MAX_POLLED_CONTACTS);
		for (int i = 0; i < count; i++) {
			/* The contact is reported from body A's side: find the partner. */
			b3ShapeId other = (data[i].shapeIdA.index1 == ctrlA->GetShapeId().index1)
			                      ? data[i].shapeIdB
			                      : data[i].shapeIdA;
			B3DPhysicsController *ctrlB = ControllerFromShape(other);
			if (!ctrlB || ctrlB == ctrlA)
				continue;

			/* Two sensor shapes never see each other (Bullet keeps them apart
			 * with the collision filter groups). */
			if (ctrlA->IsSensor() && ctrlB->IsSensor())
				continue;

			PHY_CollData collData;
			bool haveManifold = false;
			if (data[i].manifoldCount > 0) {
				b3Manifold manifold = data[i].manifolds[0];
				b3Pos point = b3OffsetPos(b3Body_GetPosition(bodyA),
				                          manifold.points[0].anchorA);
				b3dMakeCollData(b3Body_GetPosition(bodyA),
				                b3Body_GetPosition(b3Shape_GetBody(other)),
				                &manifold.normal, &point, collData);
				haveManifold = true;
			}
			if (!haveManifold) {
				b3dMakeCollData(b3Body_GetPosition(bodyA),
				                b3Body_GetPosition(b3Shape_GetBody(other)),
				                NULL, NULL, collData);
			}

			m_activeTouches.insert(std::make_pair(ctrlA, ctrlB));

			if (ctrlB->Registered())
				objectCb(objectUser, ctrlA, ctrlB, &collData);
			else
				objectCb(objectUser, ctrlB, ctrlA, &collData);
		}
	}

	/* ---- proxy overlaps kept from the sensor events ---- */
	for (std::set<std::pair<B3DPhysicsController*, B3DPhysicsController*> >::iterator it =
	         m_activeTouches.begin(); it != m_activeTouches.end(); ++it) {
		B3DPhysicsController *ctrlA = it->first;
		B3DPhysicsController *ctrlB = it->second;

		/* Contacts were already reported above, only the pairs that have no
		 * contact (the sensor proxies) are left. */
		if (!ctrlA->IsSensor() && !ctrlB->IsSensor())
			continue;
		if (!ctrlA->Registered() && !ctrlB->Registered())
			continue;

		PHY_CollData collData;
		b3dMakeCollData(b3Body_GetPosition(b3Shape_GetBody(ctrlA->GetShapeId())),
		                b3Body_GetPosition(b3Shape_GetBody(ctrlB->GetShapeId())),
		                NULL, NULL, collData);

		objectCb(objectUser, ctrlA, ctrlB, &collData);
	}
}

PHY_IPhysicsController* B3DPhysicsEnvironment::CreateSphereController(float radius, const MT_Vector3& position)
{
	/* Proxy used by the Near sensor: a static sensor sphere at the object's
	 * position, see CcdPhysicsEnvironment::CreateSphereController(). */
	if (!m_worldAlive)
		return NULL;

	B3DMotionState *motionState = new B3DMotionState();
	motionState->SetWorldPosition((float)position[0], (float)position[1], (float)position[2]);

	B3DPhysicsController *ctrl = new B3DPhysicsController(this, motionState, false);
	ctrl->SetSensor(true);

	B3DShapeDesc desc;
	desc.shapeType = PHY_SHAPE_SPHERE;
	desc.radius = radius;
	desc.isSensor = true;

	/* No mass: a static sensor needs none, and gravity must not move it. */
	if (!ctrl->Build(desc, 0.0f,
	                 MT_Vector3(1.0, 1.0, 1.0), MT_Vector3(1.0, 1.0, 1.0), true,
	                 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
	                 radius, 0.0f)) {
		delete ctrl;
		return NULL;
	}

	return ctrl;
}

PHY_IPhysicsController* B3DPhysicsEnvironment::CreateConeController(float coneradius, float coneheight)
{
	/* Proxy used by the Radar sensor.  The cone opens along -Z, like the shape
	 * B3DPhysicsController builds for PHY_SHAPE_CONE. */
	if (!m_worldAlive)
		return NULL;

	B3DMotionState *motionState = new B3DMotionState();

	B3DPhysicsController *ctrl = new B3DPhysicsController(this, motionState, false);
	ctrl->SetSensor(true);

	B3DShapeDesc desc;
	desc.shapeType = PHY_SHAPE_CONE;
	desc.radius = coneradius;
	desc.height = coneheight;
	desc.isSensor = true;

	if (!ctrl->Build(desc, 0.0f,
	                 MT_Vector3(1.0, 1.0, 1.0), MT_Vector3(1.0, 1.0, 1.0), true,
	                 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
	                 coneradius, 0.0f)) {
		delete ctrl;
		return NULL;
	}

	return ctrl;
}

void B3DPhysicsEnvironment::MergeEnvironment(PHY_IPhysicsEnvironment *other_env)
{
	/* A Box3D body belongs to the world that created it, so merging means moving
	 * the controllers over; B3DPhysicsController::SetPhysicsEnvironment()
	 * rebuilds each body in this world. */
	B3DPhysicsEnvironment *other = dynamic_cast<B3DPhysicsEnvironment*>(other_env);
	if (!other) {
		printf("B3DPhysicsEnvironment::MergeEnvironment: refusing to merge a foreign physics environment\n");
		return;
	}

	/* Copy first: SetPhysicsEnvironment() mutates both sets. */
	std::set<B3DPhysicsController*> others = other->m_controllers;
	std::set<B3DPhysicsController*>::iterator it;
	for (it = others.begin(); it != others.end(); ++it)
		(*it)->SetPhysicsEnvironment(this);
}

/* -------------------------------------------------------------------------
 * Blender object -> Box3D body conversion
 * ------------------------------------------------------------------------- */

/**
 * Collect the collision geometry of a mesh based physics object.
 *
 * Mirrors CcdShapeConstructionInfo::UpdateMesh(): the tessellated faces of the
 * derived mesh are used, only faces whose RAS_Polygon is flagged as a collider
 * contribute (the "No Physics" material option clears that flag), and the
 * vertices are compacted down to the ones a collider face actually references so
 * that loose vertices cannot inflate a convex hull.
 *
 * \param out_vertices  xyz triples, local object space
 * \param out_indices   three indices per triangle (also filled for hulls, and
 *                      ignored by the caller in that case)
 * \return true when at least one collider triangle was collected
 */
static bool b3dCollectCollisionGeometry(RAS_MeshObject *meshobj, DerivedMesh *dm,
                                        std::vector<float>& out_vertices,
                                        std::vector<int32_t>& out_indices)
{
	out_vertices.clear();
	out_indices.clear();

	if (!meshobj || !meshobj->HasColliderPolygon())
		return false;

	bool free_dm = false;
	if (!dm) {
		dm = CDDM_from_mesh(meshobj->GetMesh());
		free_dm = true;
	}
	if (!dm)
		return false;

	/* Some meshes with modifiers report no tessfaces, DM_ensure_tessface fixes it. */
	DM_ensure_tessface(dm);

	MVert *mvert = dm->getVertArray(dm);
	MFace *mface = dm->getTessFaceArray(dm);
	const int numpolys = dm->getNumTessFaces(dm);
	const int numverts = dm->getNumVerts(dm);

	if (!mvert || !mface || numpolys <= 0 || numverts <= 0) {
		if (free_dm)
			dm->release(dm);
		return false;
	}

	/* Tessfaces can come from a different polygon order than the render mesh, the
	 * original indices are needed to reach the collider flag. */
	const int *index_mf_to_mpoly = (const int *)dm->getTessFaceDataArray(dm, CD_ORIGINDEX);
	const int *index_mp_to_orig = (const int *)dm->getPolyDataArray(dm, CD_ORIGINDEX);
	if (!index_mf_to_mpoly)
		index_mp_to_orig = NULL;

	std::vector<int32_t> remap((size_t)numverts, -1);

	for (int p = 0; p < numpolys; p++) {
		MFace *mf = &mface[p];
		const int origi = index_mf_to_mpoly
		                      ? DM_origindex_mface_mpoly(index_mf_to_mpoly, index_mp_to_orig, p)
		                      : p;
		RAS_Polygon *poly = (origi != ORIGINDEX_NONE) ? meshobj->GetPolygon(origi) : NULL;

		if (!poly || !poly->IsCollider())
			continue;

		/* Mirror Blender's convention: v4 == 0 means the face is a triangle. */
		const bool isQuad = (mf->v4 != 0);
		int raw[4];
		raw[0] = mf->v1;
		raw[1] = mf->v2;
		raw[2] = mf->v3;
		raw[3] = mf->v4;

		int32_t tri[4];
		const int rawCount = isQuad ? 4 : 3;
		for (int k = 0; k < rawCount; k++) {
			const int rv = raw[k];
			if (rv < 0 || rv >= numverts) {
				tri[k] = 0;
				continue;
			}
			int32_t& slot = remap[(size_t)rv];
			if (slot < 0) {
				slot = (int32_t)(out_vertices.size() / 3);
				const float *co = mvert[rv].co;
				out_vertices.push_back(co[0]);
				out_vertices.push_back(co[1]);
				out_vertices.push_back(co[2]);
			}
			tri[k] = slot;
		}

		out_indices.push_back(tri[0]);
		out_indices.push_back(tri[1]);
		out_indices.push_back(tri[2]);
		if (isQuad) {
			out_indices.push_back(tri[0]);
			out_indices.push_back(tri[2]);
			out_indices.push_back(tri[3]);
		}
	}

	if (free_dm)
		dm->release(dm);

	return !out_indices.empty();
}

void B3DPhysicsEnvironment::ConvertObject(KX_GameObject *gameobj,
                                          RAS_MeshObject *meshobj,
                                          DerivedMesh *dm,
                                          KX_Scene *kxscene,
                                          PHY_ShapeProps *shapeprops,
                                          PHY_MaterialProps *smmaterial,
                                          PHY_IMotionState *motionstate,
                                          int activeLayerBitInfo,
                                          bool isCompoundChild,
                                          bool hasCompoundChildren)
{
	/* The mesh based bounds are cooked from meshobj/dm, the compound assembly path
	 * is still handled as separate bodies (see the controller section below). */
	(void)kxscene;
	(void)hasCompoundChildren;

	Object *blenderobject = gameobj->GetBlenderObject();

	bool isb3ddyna = (blenderobject->gameflag & OB_DYNAMIC) != 0;
	bool isb3dsensor = (blenderobject->gameflag & OB_SENSOR) != 0;
	bool isb3dsoftbody = (blenderobject->gameflag & OB_SOFT_BODY) != 0;
	bool isb3drigidbody = (blenderobject->gameflag & OB_RIGID_BODY) != 0;
	bool isb3dcharacter = (blenderobject->gameflag & OB_CHARACTER) != 0;

	/* Box3D has no soft bodies, cloth or fluids at all.  Objects flagged as soft
	 * bodies get no Box3D representation; this is the documented feature
	 * difference between the two engines. */
	if (isb3dsoftbody) {
		delete motionstate;
		return;
	}

	/* ---- collision shape description (mirrors CcdPhysicsEnvironment) ---- */

	char bounds = isb3ddyna ? OB_BOUND_SPHERE : OB_BOUND_TRIANGLE_MESH;
	if (!(blenderobject->gameflag & OB_BOUNDS)) {
		if (blenderobject->gameflag & OB_SOFT_BODY)
			bounds = OB_BOUND_TRIANGLE_MESH;
		else if (blenderobject->gameflag & OB_CHARACTER)
			bounds = OB_BOUND_SPHERE;
	}
	else {
		if (ELEM(blenderobject->collision_boundtype, OB_BOUND_CONVEX_HULL, OB_BOUND_TRIANGLE_MESH)
		    && blenderobject->type != OB_MESH)
		{
			/* Triangle meshes and hulls need a mesh object, fall back to a sphere. */
			bounds = OB_BOUND_SPHERE;
		}
		else {
			bounds = blenderobject->collision_boundtype;
		}
	}

	float bounds_extends[3];
	BoundBox *bb = BKE_object_boundbox_get(blenderobject);
	if (bb == NULL) {
		bounds_extends[0] = bounds_extends[1] = bounds_extends[2] = 1.0f;
	}
	else {
		bounds_extends[0] = 0.5f * fabsf(bb->vec[0][0] - bb->vec[4][0]);
		bounds_extends[1] = 0.5f * fabsf(bb->vec[0][1] - bb->vec[2][1]);
		bounds_extends[2] = 0.5f * fabsf(bb->vec[0][2] - bb->vec[1][2]);
	}

	B3DShapeDesc shape;
	shape.scaling = gameobj->NodeGetWorldScaling();
	shape.friction = smmaterial->m_friction;
	shape.restitution = smmaterial->m_restitution;
	shape.isSensor = isb3dsensor || isb3dcharacter;

	/* Blender's 16 bit collision group/mask mapped onto Box3D's 64 bit filter.
	 * A zero group means "no filtering", which Box3D spells as a single category
	 * bit and an all ones mask. */
	shape.categoryBits = (blenderobject->col_group != 0) ? (uint64_t)(uint16_t)blenderobject->col_group : (uint64_t)1;
	shape.maskBits = (blenderobject->col_mask != 0) ? (uint64_t)(uint16_t)blenderobject->col_mask : ~(uint64_t)0;

	if (isb3dcharacter) {
		/* A character capsule lives in its own collision category.  The mover
		 * queries of B3DCharacter mask that bit out, which is how a character
		 * stops colliding with itself:  Box3D's b3World_CollideMover() only takes
		 * a bit filter, no "ignore this body" argument.  Rays and the Near/Radar
		 * sensors use the default filter and still see the character. */
		shape.categoryBits = B3D_CHARACTER_CATEGORY;
		shape.maskBits = ~B3D_CHARACTER_CATEGORY;
	}

	switch (bounds) {
		case OB_BOUND_SPHERE:
			shape.shapeType = PHY_SHAPE_SPHERE;
			/* Blender exposes the sphere radius as the object "inertia", the same
			 * source CcdPhysicsEnvironment uses. */
			shape.radius = (blenderobject->inertia > 0.0f)
			                   ? blenderobject->inertia
			                   : MT_max(bounds_extends[0], MT_max(bounds_extends[1], bounds_extends[2]));
			break;

		case OB_BOUND_BOX:
			shape.shapeType = PHY_SHAPE_BOX;
			shape.halfExtents.setValue(MT_max(bounds_extends[0], B3D_MIN_HALF_EXTENT),
			                           MT_max(bounds_extends[1], B3D_MIN_HALF_EXTENT),
			                           MT_max(bounds_extends[2], B3D_MIN_HALF_EXTENT));
			break;

		case OB_BOUND_CYLINDER:
			shape.shapeType = PHY_SHAPE_CYLINDER;
			shape.radius = MT_max(bounds_extends[0], bounds_extends[1]);
			shape.height = 2.0f * MT_max(bounds_extends[2], B3D_MIN_HALF_EXTENT);
			break;

		case OB_BOUND_CONE:
			shape.shapeType = PHY_SHAPE_CONE;
			shape.radius = MT_max(bounds_extends[0], bounds_extends[1]);
			shape.height = 2.0f * MT_max(bounds_extends[2], B3D_MIN_HALF_EXTENT);
			break;

		case OB_BOUND_CAPSULE:
		{
			float radius = MT_max(bounds_extends[0], bounds_extends[1]);
			float height = 2.0f * (bounds_extends[2] - radius);
			shape.shapeType = PHY_SHAPE_CAPSULE;
			shape.radius = radius;
			shape.height = (height < 0.0f) ? 0.0f : height;
			break;
		}

		case OB_BOUND_CONVEX_HULL:
		case OB_BOUND_TRIANGLE_MESH:
		{
			/* Both bounds are cooked from the collider polygons of the mesh.
			 * Box3D only creates mesh contacts on static bodies, so a dynamic
			 * object with mesh bounds gets the convex hull of the same geometry
			 * (which is what Bullet's GImpact path approximates as well). */
			const bool wantTriangles = (bounds == OB_BOUND_TRIANGLE_MESH) && !isb3ddyna;

			std::vector<float> vertices;
			std::vector<int32_t> indices;
			if (b3dCollectCollisionGeometry(meshobj, dm, vertices, indices)
			    && (vertices.size() / 3) >= 4)
			{
				if (wantTriangles) {
					shape.shapeType = PHY_SHAPE_MESH;
					shape.meshVertices.swap(vertices);
					shape.meshIndices.swap(indices);
				}
				else {
					shape.shapeType = PHY_SHAPE_POLYTOPE;
					shape.hullPoints.swap(vertices);
					if (bounds == OB_BOUND_TRIANGLE_MESH && !m_meshDynamicReported) {
						m_meshDynamicReported = true;
						printf("B3DPhysics: Box3D only collides triangle meshes on static bodies, "
						       "dynamic objects use the convex hull of the mesh instead\n");
					}
				}
				break;
			}

			/* No usable collider geometry (no mesh, no collider polygons, fewer
			 * than four points): keep the object collidable with its bound box. */
			shape.shapeType = PHY_SHAPE_BOX;
			shape.halfExtents.setValue(MT_max(bounds_extends[0], B3D_MIN_HALF_EXTENT),
			                           MT_max(bounds_extends[1], B3D_MIN_HALF_EXTENT),
			                           MT_max(bounds_extends[2], B3D_MIN_HALF_EXTENT));
			if (!m_meshBoundsReported) {
				m_meshBoundsReported = true;
				printf("B3DPhysics: no collider geometry for a mesh based collision bound, "
				       "the object bounding box is used instead\n");
			}
			break;
		}
	}

	/* ---- controller ---- */

	/* A compound child does not get a body of its own: its shape is attached to
	 * the parent body with the child's placement relative to the parent baked in,
	 * exactly like CcdPhysicsEnvironment does with btCompoundShape.  Box3D only
	 * allows b3CompoundShape on static bodies, so a dynamic parent gets several
	 * shapes on one body instead, which is the same thing to the solver. */
	if (isCompoundChild) {
		Object *blenderparent = blenderobject->parent;
		while (blenderparent && blenderparent->parent)
			blenderparent = blenderparent->parent;

		KX_GameObject *parent = NULL;
		if (blenderparent) {
			KX_BlenderSceneConverter *converter =
			    (KX_BlenderSceneConverter *)KX_GetActiveEngine()->GetSceneConverter();
			parent = converter ? converter->FindGameObject(blenderparent) : NULL;
		}

		B3DPhysicsController *parentCtrl =
		    parent ? static_cast<B3DPhysicsController *>(parent->GetPhysicsController()) : NULL;

		if (!parentCtrl) {
			/* No physics on the root parent (an empty, a soft body parent, ...):
			 * the child would be unreachable for the solver, so it gets its own
			 * static body to stay collidable. */
			if (!m_compoundChildReported) {
				m_compoundChildReported = true;
				printf("B3DPhysics: a compound child has no physical parent, "
				       "it becomes a separate body\n");
			}
		}
		else {
			/* Relative placement: the parent's scale and rotation are undone, the
			 * child keeps its own world scale through shape.scaling. */
			SG_Node *gameNode = gameobj->GetSGNode();
			SG_Node *parentNode = parent->GetSGNode();

			MT_Vector3 parentScale = parentNode->GetWorldScaling();
			for (int i = 0; i < 3; i++)
				parentScale[i] = (fabs(parentScale[i]) > 1e-9) ? 1.0 / parentScale[i] : 1.0;

			MT_Matrix3x3 parentInvRot = parentNode->GetWorldOrientation().transposed();

			B3DChildTransform local;
			MT_Vector3 delta = gameNode->GetWorldPosition() - parentNode->GetWorldPosition();
			for (int i = 0; i < 3; i++)
				delta[i] *= parentScale[i];
			local.position = parentInvRot * delta;
			local.rotation = parentInvRot * gameNode->GetWorldOrientation();

			if (parentCtrl->AddChildShape(shape, local)) {
				/* The child has no controller, so the motion state (and the body it
				 * described) is not needed. */
				delete motionstate;
				return;
			}

			if (!m_compoundChildReported) {
				m_compoundChildReported = true;
				printf("B3DPhysics: could not attach a compound child shape, "
				       "it becomes a separate body\n");
			}
		}
	}

	bool isDynamic = isb3ddyna && !isCompoundChild && !isb3dcharacter;
	/* A character is moved by its own mover, so its body is kinematic:  nothing in
	 * the solver may move it, but the game logic can still place it. */
	bool isKinematic = isCompoundChild || isb3dcharacter;

	B3DPhysicsController *ctrl = new B3DPhysicsController(this, motionstate, isDynamic);
	ctrl->SetSensor(isb3dsensor);
	ctrl->SetKinematic(isKinematic);
	/* Blender's "Compound" option on the root object makes it a compound root even
	 * before it has children, which is what allows dynamic parenting later on. */
	ctrl->SetCompoundRoot(hasCompoundChildren);

	MT_Vector3 linearFactor(1.0, 1.0, 1.0);
	MT_Vector3 angularFactor(1.0, 1.0, 1.0);
	if (blenderobject->gameflag2 & OB_LOCK_RIGID_BODY_X_AXIS) linearFactor[0] = 0.0;
	if (blenderobject->gameflag2 & OB_LOCK_RIGID_BODY_Y_AXIS) linearFactor[1] = 0.0;
	if (blenderobject->gameflag2 & OB_LOCK_RIGID_BODY_Z_AXIS) linearFactor[2] = 0.0;
	if (blenderobject->gameflag2 & OB_LOCK_RIGID_BODY_X_ROT_AXIS) angularFactor[0] = 0.0;
	if (blenderobject->gameflag2 & OB_LOCK_RIGID_BODY_Y_ROT_AXIS) angularFactor[1] = 0.0;
	if (blenderobject->gameflag2 & OB_LOCK_RIGID_BODY_Z_ROT_AXIS) angularFactor[2] = 0.0;

	const float mass = isb3ddyna ? shapeprops->m_mass : 0.0f;

	/* Bullet inverts Blender's drag to get a damping factor, keep that so that the
	 * "Damping" and "Rotation Damping" sliders keep their meaning. */
	const float linearDamping = 1.0f - shapeprops->m_lin_drag;
	const float angularDamping = 1.0f - shapeprops->m_ang_drag;

	if (!ctrl->Build(shape, mass, linearFactor, angularFactor, isb3drigidbody,
	                 linearDamping, angularDamping,
	                 shapeprops->m_clamp_vel_min, shapeprops->m_clamp_vel_max,
	                 shapeprops->m_clamp_angvel_min, shapeprops->m_clamp_angvel_max,
	                 blenderobject->inertia, blenderobject->margin))
	{
		/* Build() failed before the object took ownership; the motion state goes
		 * with the controller. */
		delete ctrl;
		return;
	}

	gameobj->SetPhysicsController(ctrl, isDynamic);

	/* Dynamic objects have their animation recorded, matching Bullet. */
	if (isDynamic)
		gameobj->SetRecordAnimation(true);

	ctrl->SetNewClientInfo(gameobj->getClientInfo());

	/* A "Character" object gets the Box3D capsule mover (PHY_ICharacter).  The
	 * mover capsule is the same shape the body carries, in world units, and it is
	 * what the character collides with the world as. */
	if (isb3dcharacter) {
		const float scaleX = fabsf((float)shape.scaling[0]);
		const float scaleY = fabsf((float)shape.scaling[1]);
		const float scaleZ = fabsf((float)shape.scaling[2]);
		const float radiusScale = MT_max(scaleX, scaleY);
		const float halfCylinder = (shape.shapeType == PHY_SHAPE_CAPSULE) ? 0.5f * shape.height * scaleZ : 0.0f;

		b3Capsule mover;
		mover.radius = MT_max(shape.radius * radiusScale, B3D_MIN_HALF_EXTENT);
		mover.center1 = b3v(0.0f, 0.0f, -halfCylinder);
		mover.center2 = b3v(0.0f, 0.0f, halfCylinder);

		ctrl->SetCharacter(new B3DCharacter(this, ctrl, mover,
		                                    shapeprops->m_step_height,
		                                    shapeprops->m_jump_speed,
		                                    shapeprops->m_fall_speed,
		                                    shapeprops->m_max_jumps));
	}

	/* "No sleeping" is what OB_COLLISION_RESPONSE means in the Blender UI. */
	if ((blenderobject->gameflag & OB_COLLISION_RESPONSE) != 0)
		b3Body_EnableSleep(ctrl->GetBodyId(), false);

	/* Sensor objects are registered when a collision sensor asks for them, all
	 * other objects are registered as soon as they are on an active layer. */
	if (!isb3dsensor && (blenderobject->lay & activeLayerBitInfo) != 0)
		AddController(ctrl);
}
