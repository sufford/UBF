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

/** \file B3DPhysicsController.cpp
 *  \ingroup physbox3d
 */

#include "B3DPhysicsController.h"
#include "B3DPhysicsEnvironment.h"
#include "B3DCharacter.h"
#include "B3DMotionState.h"

#include "PHY_IMotionState.h"
#include "PHY_IPhysicsEnvironment.h"

#include <cmath>
#include <cstdio>
#include <cstring>

/* -------------------------------------------------------------------------
 * MT (Blender) <-> Box3D conversion helpers
 *
 * Box3D's b3Quat is { b3Vec3 v; float s; } i.e. (x, y, z, w), which is the same
 * order KX_MotionState uses.  b3Matrix3 is column major while MT_Matrix3x3 is
 * row major, so the matrix helpers below transpose.
 *
 * Note that compound literals like (b3Vec3){1,2,3} are a C99 construct that is
 * not portable to C++, so every value is built field by field.
 * ------------------------------------------------------------------------- */

#define B3D_PI 3.14159265358979323846f

/* Number of segments used when a cylinder or cone has to be tessellated into a
 * convex hull.  16 keeps Box3D's hull vertex/face/edge counts well inside the
 * limits its own b3CreateCylinder() allows (up to 32 segments). */
#define B3D_RING_SEGMENTS 16

/** Set once the first time a non zero collision margin has to be reported as
 *  unsupported, so that the message does not repeat for every object. */
static bool b3dMarginWarningIssued = false;

static inline b3Vec3 makeB3Vector(float x, float y, float z)
{
	b3Vec3 v;
	v.x = x;
	v.y = y;
	v.z = z;
	return v;
}

static inline b3Quat makeB3Quat(float x, float y, float z, float w)
{
	b3Quat q;
	q.v.x = x;
	q.v.y = y;
	q.v.z = z;
	q.s = w;
	return q;
}

static inline b3Transform makeB3IdentityTransform()
{
	b3Transform t;
	t.p = makeB3Vector(0.0f, 0.0f, 0.0f);
	t.q = makeB3Quat(0.0f, 0.0f, 0.0f, 1.0f);
	return t;
}

static inline b3Vec3 vecToB3(const MT_Vector3& v)
{
	return makeB3Vector((float)v[0], (float)v[1], (float)v[2]);
}

static inline MT_Vector3 vecToMT(const b3Vec3& v)
{
	return MT_Vector3((MT_Scalar)v.x, (MT_Scalar)v.y, (MT_Scalar)v.z);
}

/** MT_Matrix3x3 (row major) -> b3Matrix3 (column major). */
static b3Matrix3 matToB3(const MT_Matrix3x3& m)
{
	b3Matrix3 r;
	r.cx = makeB3Vector((float)m[0][0], (float)m[1][0], (float)m[2][0]);
	r.cy = makeB3Vector((float)m[0][1], (float)m[1][1], (float)m[2][1]);
	r.cz = makeB3Vector((float)m[0][2], (float)m[1][2], (float)m[2][2]);
	return r;
}

static b3Quat quatFromMTMatrix(const MT_Matrix3x3& m)
{
	b3Matrix3 bm = matToB3(m);
	return b3MakeQuatFromMatrix(&bm);
}

static MT_Matrix3x3 matToMT(const b3Quat& q)
{
	/* Box3D and moto both use the standard active rotation convention
	 * (b3RotateVector(q, v) == MT_Matrix3x3(q) * v), so moto's own
	 * quaternion constructor can be used instead of hand rolling the matrix. */
	MT_Quaternion mq((MT_Scalar)q.v.x, (MT_Scalar)q.v.y, (MT_Scalar)q.v.z, (MT_Scalar)q.s);
	return MT_Matrix3x3(mq);
}

static inline float vecLength(const b3Vec3& v)
{
	return sqrtf(v.x * v.x + v.y * v.y + v.z * v.z);
}

/* ------------------------------------------------------------------------- */

B3DShapeDesc::B3DShapeDesc()
	: shapeType(PHY_SHAPE_NONE),
	  radius(0.0f),
	  height(0.0f),
	  friction(0.5f),
	  restitution(0.0f),
	  isSensor(false),
	  categoryBits(1),
	  maskBits(~(uint64_t)0)
{
	halfExtents.setValue(0.0f, 0.0f, 0.0f);
	scaling.setValue(1.0f, 1.0f, 1.0f);
}

B3DChildTransform::B3DChildTransform()
{
	position.setValue(0.0f, 0.0f, 0.0f);
	rotation.setIdentity();
}

B3DChildShapeDesc::B3DChildShapeDesc()
	: owner(NULL)
{
}

B3DPhysicsController::B3DPhysicsController(B3DPhysicsEnvironment *env, PHY_IMotionState *motionstate, bool isDynamic)
	: m_env(env),
	  m_motionState(motionstate),
	  m_bodyId(b3_nullBodyId),
	  m_shapeId(b3_nullShapeId),
	  m_clientInfo(NULL),
	  m_isDynamic(isDynamic),
	  m_isKinematic(false),
	  m_isRigidBody(true),
	  m_isCompoundRoot(false),
	  m_isSensor(false),
	  m_suspended(false),
	  m_inWorld(false),
	  m_mass(0.0f),
	  m_linearDamping(0.0f),
	  m_angularDamping(0.0f),
	  m_clampVelMin(0.0f),
	  m_clampVelMax(0.0f),
	  m_clampAngVelMin(0.0f),
	  m_clampAngVelMax(0.0f),
	  m_radius(0.0f),
	  m_margin(0.0f),
	  m_applyAngularLocks(true),
	  m_registerCount(0),
	  m_character(NULL)
{
	m_scaling.setValue(1.0f, 1.0f, 1.0f);
	m_linearFactor.setValue(1.0f, 1.0f, 1.0f);
	m_angularFactor.setValue(1.0f, 1.0f, 1.0f);
}

B3DPhysicsController::~B3DPhysicsController()
{
	if (m_env)
		m_env->RemoveController(this);

	/* EndObject() destroys the controller while the world is still alive, so the
	 * body has to be released here or it would linger as invisible geometry.
	 * If the environment went away first the whole world (and every body in it)
	 * is already gone, so nothing may be touched any more. */
	if (m_env && m_env->IsWorldAlive() && b3Body_IsValid(m_bodyId)) {
		b3DestroyBody(m_bodyId);
	}
	m_bodyId = b3_nullBodyId;
	m_shapeId = b3_nullShapeId;
	m_childShapes.clear();

	/* The shapes referenced the cooked meshes, so they have to go after the body
	 * (and therefore after every shape that used them) was destroyed. */
	ReleaseCookedMeshes();

	if (m_motionState)
		delete m_motionState;

	/* The character mover only holds handles that are gone with the body. */
	delete m_character;
	m_character = NULL;
}

void B3DPhysicsController::ReleaseCookedMeshes()
{
	for (size_t i = 0; i < m_cookedMeshes.size(); i++) {
		if (m_cookedMeshes[i])
			b3DestroyMesh(m_cookedMeshes[i]);
	}
	m_cookedMeshes.clear();
}

/* -------------------------------------------------------------------------
 * Body and shape construction
 * ------------------------------------------------------------------------- */

bool B3DPhysicsController::CreateBody()
{
	if (!m_env || !m_motionState || !m_env->IsWorldAlive())
		return false;

	float pos[3];
	float quat[4];
	m_motionState->GetWorldPosition(pos[0], pos[1], pos[2]);
	m_motionState->GetWorldOrientation(quat[0], quat[1], quat[2], quat[3]);

	b3BodyDef bd = b3DefaultBodyDef();

	if (m_isDynamic)
		bd.type = b3_dynamicBody;
	else if (m_isKinematic)
		bd.type = b3_kinematicBody;
	else
		bd.type = b3_staticBody;

	bd.position = makeB3Vector(pos[0], pos[1], pos[2]);
	bd.rotation = makeB3Quat(quat[0], quat[1], quat[2], quat[3]);
	bd.linearDamping = m_linearDamping;
	bd.angularDamping = m_angularDamping;
	bd.gravityScale = 1.0f;
	bd.enableSleep = true;
	bd.isAwake = true;
	bd.userData = this;

	/* Blender's per axis locks.  Box3D can only carry them in the body
	 * definition, there is no b3Body_SetMotionLocks() to change them later. */
	bd.motionLocks.linearX = (m_linearFactor[0] == 0.0);
	bd.motionLocks.linearY = (m_linearFactor[1] == 0.0);
	bd.motionLocks.linearZ = (m_linearFactor[2] == 0.0);
	bd.motionLocks.angularX = m_applyAngularLocks && (m_angularFactor[0] == 0.0);
	bd.motionLocks.angularY = m_applyAngularLocks && (m_angularFactor[1] == 0.0);
	bd.motionLocks.angularZ = m_applyAngularLocks && (m_angularFactor[2] == 0.0);

	m_bodyId = b3CreateBody(m_env->GetWorldId(), &bd);
	if (!b3Body_IsValid(m_bodyId)) {
		m_bodyId = b3_nullBodyId;
		printf("B3DPhysicsController: Box3D refused to create a body (out of worlds?)\n");
		return false;
	}

	b3Body_SetUserData(m_bodyId, this);
	return true;
}

b3ShapeId B3DPhysicsController::CreateShape(const B3DShapeDesc& shape)
{
	return CreateShapeOnBody(shape, NULL);
}

b3ShapeId B3DPhysicsController::CreateShapeOnBody(const B3DShapeDesc& shape, const B3DChildTransform *local)
{
	b3ShapeDef sd = b3DefaultShapeDef();

	sd.baseMaterial.friction = shape.friction;
	sd.baseMaterial.restitution = shape.restitution;
	sd.filter.categoryBits = shape.categoryBits;
	sd.filter.maskBits = shape.maskBits;
	/* Unit density: Box3D turns this into a volume derived mass which is then
	 * rescaled to the mass Blender asked for in ApplyMass(). */
	sd.density = 1.0f;
	sd.isSensor = shape.isSensor;
	/* Box3D only records an overlap for a sensor shape when the *visitor* shape
	 * has sensor events enabled as well (see src/sensor.c).  Mesh and height
	 * field shapes are excluded: the sensor query builds a point proxy of the
	 * visitor and that proxy only exists for convex shapes, a mesh visitor trips
	 * B3_ASSERT in b3MakeShapeProxy().  Contacts are therefore used for mesh
	 * geometry, see B3DPhysicsEnvironment::ProcessEvents(). */
	sd.enableSensorEvents = (shape.shapeType != PHY_SHAPE_MESH);
	/* Contact events are off by default in Box3D and are only produced for
	 * kinematic and dynamic shapes.  The BGE listens to them for collision
	 * sensors, so they are switched on for exactly those shapes (ignored by
	 * Box3D for sensors and static shapes). */
	sd.enableContactEvents = !sd.isSensor && (m_isDynamic || m_isKinematic);
	sd.updateBodyMass = true;

	b3Transform identity = makeB3IdentityTransform();

	/* A compound child is placed by its local transform, a plain shape sits at the
	 * body origin. */
	b3Transform localTransform = identity;
	bool hasLocal = (local != NULL);
	if (hasLocal) {
		localTransform.p = vecToB3(local->position);
		localTransform.q = quatFromMTMatrix(local->rotation);
	}

	/* Box3D has no negative or non-uniform scale for spheres and capsules, but
	 * hull shapes accept a scale vector, so the absolute scale is split between
	 * the two paths below. */
	float absScale[3];
	for (int i = 0; i < 3; i++) {
		absScale[i] = fabsf((float)shape.scaling[i]);
		if (absScale[i] <= 1e-9f)
			absScale[i] = 1.0f;
	}
	b3Vec3 hullScale = makeB3Vector(absScale[0], absScale[1], absScale[2]);

	switch (shape.shapeType) {
		case PHY_SHAPE_BOX:
		{
			/* b3MakeBoxHull() returns a hull by value and the shape clones it, so
			 * b3DestroyHull() must not be called on it.  The scale is applied by
			 * Box3D, which also handles mirrored (negative) scale. */
			b3BoxHull box = b3MakeBoxHull((float)shape.halfExtents[0],
			                              (float)shape.halfExtents[1],
			                              (float)shape.halfExtents[2]);
			return b3CreateTransformedHullShape(m_bodyId, &sd, &box.base, localTransform, hullScale);
		}

		case PHY_SHAPE_CYLINDER:
		{
			/* Box3D's own b3CreateCylinder() is Y aligned while Blender and Bullet
			 * (btCylinderShapeZ) are Z aligned, so the ring is generated directly
			 * in Z here. */
			b3Vec3 points[B3D_RING_SEGMENTS * 2];
			int n = 0;
			const float hz = 0.5f * shape.height;
			for (int i = 0; i < B3D_RING_SEGMENTS; i++) {
				float a = 2.0f * B3D_PI * (float)i / (float)B3D_RING_SEGMENTS;
				float x = cosf(a) * shape.radius;
				float y = sinf(a) * shape.radius;
				points[n++] = makeB3Vector(x, y, -hz);
				points[n++] = makeB3Vector(x, y, hz);
			}

			b3HullData *hull = b3CreateHull(points, n, n);
			if (!hull)
				return b3_nullShapeId;

			b3ShapeId id = b3CreateTransformedHullShape(m_bodyId, &sd, hull, localTransform, hullScale);
			b3DestroyHull(hull);
			return id;
		}

		case PHY_SHAPE_CONE:
		{
			/* btConeShapeZ() is Z aligned and centred on the origin, with the base
			 * at -h/2 and the tip at +h/2.  Box3D's b3CreateCone() asserts
			 * radius2 > 0 and is Y aligned, so the ring is generated here with a
			 * small but non zero tip radius. */
			b3Vec3 points[B3D_RING_SEGMENTS * 2];
			int n = 0;
			const float hz = 0.5f * shape.height;
			const float tipRadius = (shape.radius > 0.0f) ? (shape.radius * 0.02f) : 0.001f;
			for (int i = 0; i < B3D_RING_SEGMENTS; i++) {
				float a = 2.0f * B3D_PI * (float)i / (float)B3D_RING_SEGMENTS;
				float ca = cosf(a);
				float sa = sinf(a);
				points[n++] = makeB3Vector(ca * shape.radius, sa * shape.radius, -hz);
				points[n++] = makeB3Vector(ca * tipRadius, sa * tipRadius, hz);
			}

			b3HullData *hull = b3CreateHull(points, n, n);
			if (!hull)
				return b3_nullShapeId;

			b3ShapeId id = b3CreateTransformedHullShape(m_bodyId, &sd, hull, localTransform, hullScale);
			b3DestroyHull(hull);
			return id;
		}

		case PHY_SHAPE_SPHERE:
		{
			/* Box3D spheres cannot be scaled non uniformly, so the scale is
			 * averaged.  Bullet would turn the sphere into an ellipsoid here. */
			float uniform = (absScale[0] + absScale[1] + absScale[2]) / 3.0f;
			b3Sphere sphere;
			/* No transform argument exists for a sphere, so a compound child is
			 * placed by moving its centre. */
			sphere.center = hasLocal ? localTransform.p : makeB3Vector(0.0f, 0.0f, 0.0f);
			sphere.radius = shape.radius * uniform;
			return b3CreateSphereShape(m_bodyId, &sd, &sphere);
		}

		case PHY_SHAPE_CAPSULE:
		{
			/* btCapsuleShapeZ(radius, height): two hemispheres whose centres are
			 * height apart along Z. */
			float rxy = 0.5f * (absScale[0] + absScale[1]);
			float hz = 0.5f * shape.height * absScale[2];
			b3Capsule capsule;
			/* Like the sphere there is no transform argument, so a compound child's
			 * rotation and offset are applied to the two segment centres. */
			b3Vec3 axis = makeB3Vector(0.0f, 0.0f, hz);
			if (hasLocal) {
				axis = b3RotateVector(localTransform.q, axis);
				capsule.center1 = makeB3Vector(localTransform.p.x - axis.x,
				                               localTransform.p.y - axis.y,
				                               localTransform.p.z - axis.z);
				capsule.center2 = makeB3Vector(localTransform.p.x + axis.x,
				                               localTransform.p.y + axis.y,
				                               localTransform.p.z + axis.z);
			}
			else {
				capsule.center1 = makeB3Vector(0.0f, 0.0f, -hz);
				capsule.center2 = makeB3Vector(0.0f, 0.0f, hz);
			}
			capsule.radius = shape.radius * rxy;
			return b3CreateCapsuleShape(m_bodyId, &sd, &capsule);
		}

		case PHY_SHAPE_POLYTOPE:
		{
			/* Blender's "Convex Hull" collision bounds, cooked from the vertices
			 * of the collider polygons.  b3CreateHull() builds the hull and the
			 * shape clones it, so the temporary hull is destroyed right away. */
			const int count = (int)(shape.hullPoints.size() / 3);
			if (count < 4)
				return b3_nullShapeId;

			std::vector<b3Vec3> points((size_t)count);
			for (int i = 0; i < count; i++) {
				points[(size_t)i] = makeB3Vector(shape.hullPoints[(size_t)(i * 3 + 0)],
				                                 shape.hullPoints[(size_t)(i * 3 + 1)],
				                                 shape.hullPoints[(size_t)(i * 3 + 2)]);
			}

			b3HullData *hull = b3CreateHull(&points[0], count, count);
			if (!hull)
				return b3_nullShapeId;

			b3ShapeId id = b3CreateTransformedHullShape(m_bodyId, &sd, hull, localTransform, hullScale);
			b3DestroyHull(hull);
			return id;
		}

		case PHY_SHAPE_MESH:
		{
			/* Blender's "Triangle Mesh" collision bounds (static geometry).  Box3D
			 * bakes the triangles into a BVH and the shape keeps a pointer to it,
			 * so the result is stored on the controller and released with it. */
			const int vertexCount = (int)(shape.meshVertices.size() / 3);
			const int triangleCount = (int)(shape.meshIndices.size() / 3);
			if (vertexCount < 3 || triangleCount < 1)
				return b3_nullShapeId;

			std::vector<b3Vec3> vertices((size_t)vertexCount);
			for (int i = 0; i < vertexCount; i++) {
				b3Vec3 v = makeB3Vector(shape.meshVertices[(size_t)(i * 3 + 0)],
				                        shape.meshVertices[(size_t)(i * 3 + 1)],
				                        shape.meshVertices[(size_t)(i * 3 + 2)]);
				/* A mesh shape has no local transform argument, so a compound
				 * child's placement is baked into its vertices (the scale is still
				 * passed separately). */
				if (hasLocal) {
					v = b3RotateVector(localTransform.q, v);
					v.x += localTransform.p.x;
					v.y += localTransform.p.y;
					v.z += localTransform.p.z;
				}
				vertices[(size_t)i] = v;
			}

			/* The indices vector is already the layout Box3D wants. */
			std::vector<int32_t> indices(shape.meshIndices);

			b3MeshDef md;
			memset(&md, 0, sizeof(md));
			md.vertices = &vertices[0];
			md.indices = &indices[0];
			md.materialIndices = NULL;
			md.weldTolerance = 0.0f;
			md.vertexCount = vertexCount;
			md.triangleCount = triangleCount;
			/* The BGE mesh already shares vertices between faces (it comes from
			 * the Blender mesh, not from the split render mesh), so no welding is
			 * needed and edge adjacency is not used by collision. */
			md.weldVertices = false;
			md.useMedianSplit = false;
			md.identifyEdges = false;

			b3MeshData *cooked = b3CreateMesh(&md, NULL, 0);
			if (!cooked) {
				printf("B3DPhysicsController: Box3D refused to cook a triangle mesh "
				       "(%d vertices, %d triangles)\n", vertexCount, triangleCount);
				return b3_nullShapeId;
			}
			m_cookedMeshes.push_back(cooked);

			return b3CreateMeshShape(m_bodyId, &sd, cooked, hullScale);
		}

		default:
			break;
	}

	return b3_nullShapeId;
}

void B3DPhysicsController::ApplyMass(float mass)
{
	if (!m_isDynamic || !b3Body_IsValid(m_bodyId) || mass <= 0.0f)
		return;

	/* Box3D derives the mass from the shape density.  ApplyMassFromShapes() has
	 * already run when the shape was created, so read the volume based result
	 * back and rescale mass and inertia to the mass Blender asked for.  Keeping
	 * Box3D's tensor shape is better than Blender's crude mass/3 guess: it is a
	 * real volume integral of the actual collision shape. */
	b3MassData md = b3Body_GetMassData(m_bodyId);
	if (md.mass <= 1e-9f)
		return;

	const float k = mass / md.mass;
	md.mass = mass;
	md.inertia.cx.x *= k;
	md.inertia.cx.y *= k;
	md.inertia.cx.z *= k;
	md.inertia.cy.x *= k;
	md.inertia.cy.y *= k;
	md.inertia.cy.z *= k;
	md.inertia.cz.x *= k;
	md.inertia.cz.y *= k;
	md.inertia.cz.z *= k;
	b3Body_SetMassData(m_bodyId, md);
}

bool B3DPhysicsController::AddChildShape(const B3DShapeDesc& shape, const B3DChildTransform& local)
{
	if (!m_env || !m_env->IsWorldAlive() || !b3Body_IsValid(m_bodyId))
		return false;

	b3ShapeId id = CreateShapeOnBody(shape, &local);
	if (!b3Shape_IsValid(id)) {
		printf("B3DPhysicsController: could not attach a compound child shape\n");
		return false;
	}

	m_childShapes.push_back(id);

	B3DChildShapeDesc rec;
	rec.shape = shape;
	rec.local = local;
	m_childDescs.push_back(rec);

	/* The body's mass properties change with every shape, so Box3D's volume
	 * derived result is recomputed and Blender's mass is re-applied on top of it
	 * (a static or kinematic parent ignores both). */
	b3Body_ApplyMassFromShapes(m_bodyId);
	ApplyMass(m_mass);
	return true;
}

void B3DPhysicsController::ReattachChildShapes()
{
	/* Iterate over a copy: AddChildShape() appends to m_childDescs. */
	std::vector<B3DChildShapeDesc> children = m_childDescs;
	m_childShapes.clear();
	for (size_t i = 0; i < children.size(); i++)
		AddChildShape(children[i].shape, children[i].local);
}

bool B3DPhysicsController::Build(const B3DShapeDesc& shape,
                                 float mass,
                                 const MT_Vector3& linearFactor, const MT_Vector3& angularFactor,
                                 bool applyAngularLocks,
                                 float linearDamping, float angularDamping,
                                 float linVelMin, float linVelMax,
                                 float angVelMin, float angVelMax,
                                 float radius, float margin)
{
	/* Remember everything: GetReplica()/PostProcessReplica() rebuild from here. */
	m_shapeDesc = shape;
	m_mass = mass;
	m_linearFactor = linearFactor;
	m_angularFactor = angularFactor;
	m_applyAngularLocks = applyAngularLocks;
	m_linearDamping = linearDamping;
	m_angularDamping = angularDamping;
	m_clampVelMin = linVelMin;
	m_clampVelMax = linVelMax;
	m_clampAngVelMin = angVelMin;
	m_clampAngVelMax = angVelMax;
	m_radius = radius;
	m_margin = margin;
	m_scaling = shape.scaling;
	m_isSensor = shape.isSensor;

	if (!CreateBody())
		return false;

	m_shapeId = CreateShape(shape);
	if (!b3Shape_IsValid(m_shapeId)) {
		m_shapeId = b3_nullShapeId;
		b3DestroyBody(m_bodyId);
		m_bodyId = b3_nullBodyId;
		printf("B3DPhysicsController: failed to create a Box3D shape for bounds type %d\n",
		       (int)shape.shapeType);
		return false;
	}

	b3Shape_SetUserData(m_shapeId, this);
	ApplyMass(m_mass);

	/* Box3D has no per shape collision margin, the engine wide B3_LINEAR_SLOP is
	 * used instead.  m_margin is kept so that the Python collisionMargin
	 * property keeps working, but it cannot influence the simulation. */
	if (m_margin > 0.0f && !b3dMarginWarningIssued) {
		b3dMarginWarningIssued = true;
		printf("B3DPhysics: collision margin is not supported by Box3D and is ignored\n");
	}

	return true;
}

/* -------------------------------------------------------------------------
 * Motion state synchronisation
 * ------------------------------------------------------------------------- */

bool B3DPhysicsController::SynchronizeMotionStates(float time)
{
	(void)time;

	if (!m_motionState || !b3Body_IsValid(m_bodyId) || m_suspended)
		return false;

	/* Mirrors CcdPhysicsController: static bodies are not written back, the
	 * scene graph is the authority for them (see KX_GameObject::UpdateTransform,
	 * which calls SetTransform() for every non dynamic controller). */
	b3BodyType type = b3Body_GetType(m_bodyId);
	if (type == b3_staticBody)
		return false;

	b3Pos p = b3Body_GetPosition(m_bodyId);
	b3Quat q = b3Body_GetRotation(m_bodyId);

	m_motionState->SetWorldPosition((float)p.x, (float)p.y, (float)p.z);
	m_motionState->SetWorldOrientation(q.v.x, q.v.y, q.v.z, q.s);
	m_motionState->CalculateWorldTransformations();
	return true;
}

void B3DPhysicsController::WriteMotionStateToDynamics(bool nondynaonly)
{
	if (!m_motionState || !b3Body_IsValid(m_bodyId))
		return;

	/* Never let a stale scene graph transform overwrite the simulation. */
	if (nondynaonly && b3Body_GetType(m_bodyId) == b3_dynamicBody)
		return;

	float pos[3];
	float quat[4];
	m_motionState->GetWorldPosition(pos[0], pos[1], pos[2]);
	m_motionState->GetWorldOrientation(quat[0], quat[1], quat[2], quat[3]);

	b3Body_SetTransform(m_bodyId,
	                    makeB3Vector(pos[0], pos[1], pos[2]),
	                    makeB3Quat(quat[0], quat[1], quat[2], quat[3]));
	b3Body_SetAwake(m_bodyId, true);
}

void B3DPhysicsController::WriteDynamicsToMotionState()
{
	SynchronizeMotionStates(0.0f);
}

void B3DPhysicsController::SetTransform()
{
	WriteMotionStateToDynamics(false);
}

void B3DPhysicsController::PostProcessReplica(PHY_IMotionState *motionstate, PHY_IPhysicsController *parentctrl)
{
	(void)parentctrl;

	/* The replica produced by GetReplica() has no motion state of its own yet and
	 * no Box3D body, so both are created here from the copied description. */
	if (m_motionState && m_motionState != motionstate)
		delete m_motionState;
	m_motionState = motionstate;

	if (!Build(m_shapeDesc, m_mass, m_linearFactor, m_angularFactor, m_applyAngularLocks,
	           m_linearDamping, m_angularDamping,
	           m_clampVelMin, m_clampVelMax, m_clampAngVelMin, m_clampAngVelMax,
	           m_radius, m_margin))
	{
		return;
	}

	/* Reattach the compound children, if the original had any. */
	if (!m_childDescs.empty())
		ReattachChildShapes();

	if (m_env)
		m_env->AddController(this);
}

void B3DPhysicsController::SetPhysicsEnvironment(PHY_IPhysicsEnvironment *env)
{
	/* Called when a scene is merged or lib-loaded.  Only a Box3D environment can
	 * host a Box3D body; anything else is refused instead of being cast blindly. */
	B3DPhysicsEnvironment *b3denv = dynamic_cast<B3DPhysicsEnvironment*>(env);
	if (!b3denv) {
		if (env)
			printf("B3DPhysicsController: refusing to move a Box3D body into a foreign physics environment\n");
		return;
	}
	if (m_env == b3denv)
		return;

	/* A Box3D body belongs to the world that created it, so moving the object to
	 * another environment means rebuilding the body there.  The simulated state
	 * is carried across so that a lib-loaded object does not visibly jump. */
	b3Pos position = makeB3Vector(0.0f, 0.0f, 0.0f);
	b3Quat rotation = makeB3Quat(0.0f, 0.0f, 0.0f, 1.0f);
	b3Vec3 linearVelocity = makeB3Vector(0.0f, 0.0f, 0.0f);
	b3Vec3 angularVelocity = makeB3Vector(0.0f, 0.0f, 0.0f);

	const bool haveState = (m_env && m_env->IsWorldAlive() && b3Body_IsValid(m_bodyId));
	if (haveState) {
		position = b3Body_GetPosition(m_bodyId);
		rotation = b3Body_GetRotation(m_bodyId);
		linearVelocity = b3Body_GetLinearVelocity(m_bodyId);
		angularVelocity = b3Body_GetAngularVelocity(m_bodyId);
	}

	if (m_env) {
		m_env->RemoveController(this);
		if (m_env->IsWorldAlive() && b3Body_IsValid(m_bodyId))
			b3DestroyBody(m_bodyId);
	}
	m_bodyId = b3_nullBodyId;
	m_shapeId = b3_nullShapeId;
	m_childShapes.clear();

	/* The shapes that referenced the cooked meshes are gone with the body, so the
	 * meshes can be freed and cooked again for the new environment. */
	ReleaseCookedMeshes();

	m_env = b3denv;

	if (!Build(m_shapeDesc, m_mass, m_linearFactor, m_angularFactor, m_applyAngularLocks,
	           m_linearDamping, m_angularDamping,
	           m_clampVelMin, m_clampVelMax, m_clampAngVelMin, m_clampAngVelMax,
	           m_radius, m_margin))
	{
		printf("B3DPhysicsController: failed to rebuild the body in the target environment\n");
		return;
	}

	if (haveState && b3Body_IsValid(m_bodyId)) {
		b3Body_SetTransform(m_bodyId, position, rotation);
		if (m_isDynamic) {
			b3Body_SetLinearVelocity(m_bodyId, linearVelocity);
			b3Body_SetAngularVelocity(m_bodyId, angularVelocity);
		}
	}

	/* The new body has only the primary shape so far. */
	if (!m_childDescs.empty())
		ReattachChildShapes();

	m_env->AddController(this);
}

PHY_IPhysicsController* B3DPhysicsController::GetReplica()
{
	/* A plain copy; the body is created in PostProcessReplica() once the replica's
	 * motion state exists.  The copy ctor duplicates the motion state pointer,
	 * which the original still owns, so it is cleared here without deleting it. */
	B3DPhysicsController *replica = new B3DPhysicsController(*this);
	replica->m_motionState = NULL;
	replica->m_bodyId = b3_nullBodyId;
	replica->m_shapeId = b3_nullShapeId;
	replica->m_inWorld = false;
	replica->m_suspended = false;

	/* Runtime handles must never be shared with a copy: the shape ids belong to the
	 * original's body and the cooked meshes are owned (and freed) by the original.
	 * The descriptions are kept, so PostProcessReplica() can rebuild both. */
	replica->m_childShapes.clear();
	replica->m_cookedMeshes.clear();
	replica->m_childDescs.clear();
	return replica;
}

PHY_IPhysicsController* B3DPhysicsController::GetReplicaForSensors()
{
	/* Only the Near and Radar sensor proxies are duplicated this way; a scene
	 * object controller is replicated by KX_Scene through GetReplica().  The
	 * replica is a full controller of its own: its own motion state (standalone,
	 * not a scene graph node) and its own body in the same world. */
	if (!m_env || !m_env->IsWorldAlive())
		return NULL;

	B3DMotionState *motionState = new B3DMotionState();

	if (b3Body_IsValid(m_bodyId)) {
		b3Pos p = b3Body_GetPosition(m_bodyId);
		b3Quat q = b3Body_GetRotation(m_bodyId);
		motionState->SetWorldPosition(p.x, p.y, p.z);
		motionState->SetWorldOrientation(q.v.x, q.v.y, q.v.z, q.s);
	}

	B3DPhysicsController *replica = new B3DPhysicsController(m_env, motionState, false);

	/* The proxies are static sensor shapes: no contacts, only overlap events. */
	B3DShapeDesc desc = m_shapeDesc;
	replica->SetSensor(true);
	replica->SetInWorld(true);
	replica->SetCompoundRoot(false);

	if (!replica->Build(desc, 0.0f,
	                    m_linearFactor, m_angularFactor, m_applyAngularLocks,
	                    0.0f, 0.0f,
	                    m_clampVelMin, m_clampVelMax,
	                    m_clampAngVelMin, m_clampAngVelMax,
	                    m_radius, m_margin)) {
		delete replica;
		return NULL;
	}

	return replica;
}

void B3DPhysicsController::SetRadius(float radius)
{
	m_radius = radius;

	/* The Near sensor widens/narrows its detection sphere by changing the radius
	 * at run time, exactly like btSphereShape::setUnscaledRadius().  Only a sphere
	 * proxy can be resized; anything else keeps the new value for later use. */
	if (m_shapeDesc.shapeType != PHY_SHAPE_SPHERE)
		return;
	if (!m_env || !m_env->IsWorldAlive() || !b3Body_IsValid(m_bodyId))
		return;

	m_shapeDesc.radius = radius;

	if (b3Shape_IsValid(m_shapeId))
		b3DestroyShape(m_shapeId, false);
	m_shapeId = b3_nullShapeId;

	B3DShapeDesc shape = m_shapeDesc;
	m_shapeId = CreateShape(shape);
}

/* -------------------------------------------------------------------------
 * Kinematic interface
 * ------------------------------------------------------------------------- */

void B3DPhysicsController::EnsureMovable()
{
	if (!b3Body_IsValid(m_bodyId) || m_isSensor)
		return;

	/* Mirrors the CF_KINEMATIC_OBJECT promotion in CcdPhysicsController: once
	 * logic has moved a static object it has to be simulated as kinematic. */
	if (b3Body_GetType(m_bodyId) == b3_staticBody) {
		b3Body_SetType(m_bodyId, b3_kinematicBody);
		b3Body_SetAwake(m_bodyId, true);
	}
}

void B3DPhysicsController::SetPosition(const MT_Vector3& pos)
{
	if (!b3Body_IsValid(m_bodyId))
		return;

	EnsureMovable();
	if (b3Body_GetType(m_bodyId) == b3_staticBody)
		return;   /* kinematic bodies take their transform from the scene graph */

	b3Quat q = b3Body_GetRotation(m_bodyId);
	b3Body_SetTransform(m_bodyId, makeB3Vector((float)pos[0], (float)pos[1], (float)pos[2]), q);
}

void B3DPhysicsController::GetPosition(MT_Vector3& pos) const
{
	if (b3Body_IsValid(m_bodyId)) {
		b3Pos p = b3Body_GetPosition(m_bodyId);
		pos.setValue((MT_Scalar)p.x, (MT_Scalar)p.y, (MT_Scalar)p.z);
	}
}

void B3DPhysicsController::RelativeTranslate(const MT_Vector3& dloc, bool local)
{
	if (!b3Body_IsValid(m_bodyId))
		return;

	EnsureMovable();
	if (b3Body_GetType(m_bodyId) == b3_staticBody)
		return;

	b3Quat q = b3Body_GetRotation(m_bodyId);
	b3Vec3 d = vecToB3(dloc);
	if (local)
		d = b3RotateVector(q, d);

	b3Pos p = b3Body_GetPosition(m_bodyId);
	b3Body_SetTransform(m_bodyId, makeB3Vector(p.x + d.x, p.y + d.y, p.z + d.z), q);
}

MT_Matrix3x3 B3DPhysicsController::GetOrientation()
{
	if (!b3Body_IsValid(m_bodyId))
		return MT_Matrix3x3(MT_Scalar(1.0), MT_Scalar(0.0), MT_Scalar(0.0),
		                    MT_Scalar(0.0), MT_Scalar(1.0), MT_Scalar(0.0),
		                    MT_Scalar(0.0), MT_Scalar(0.0), MT_Scalar(1.0));

	return matToMT(b3Body_GetRotation(m_bodyId));
}

void B3DPhysicsController::SetOrientation(const MT_Matrix3x3& orn)
{
	if (!b3Body_IsValid(m_bodyId))
		return;

	EnsureMovable();
	if (b3Body_GetType(m_bodyId) == b3_staticBody)
		return;

	b3Pos p = b3Body_GetPosition(m_bodyId);
	b3Quat q = quatFromMTMatrix(orn);
	b3Body_SetTransform(m_bodyId, p, q);
}

void B3DPhysicsController::RelativeRotate(const MT_Matrix3x3& rot, bool local)
{
	if (!b3Body_IsValid(m_bodyId))
		return;

	EnsureMovable();
	if (b3Body_GetType(m_bodyId) == b3_staticBody)
		return;

	/* Bullet computes current * (current^-1 * delta * current) in world mode,
	 * which reduces to delta * current because the scene graph orientation and
	 * the body orientation are kept in sync. */
	b3Quat qd = quatFromMTMatrix(rot);
	b3Quat qc = b3Body_GetRotation(m_bodyId);
	b3Quat qn = local ? b3MulQuat(qc, qd) : b3MulQuat(qd, qc);

	b3Pos p = b3Body_GetPosition(m_bodyId);
	b3Body_SetTransform(m_bodyId, p, b3NormalizeQuat(qn));
}

void B3DPhysicsController::SetScaling(const MT_Vector3& scale)
{
	/* Box3D shapes clone their geometry, so an already created shape cannot be
	 * rescaled.  Bounds are built with the object scale baked in at creation
	 * time, which covers the common case; runtime rescaling of a live object is
	 * not applied yet. */
	m_scaling = scale;
}

/* -------------------------------------------------------------------------
 * Mass, damping, activation
 * ------------------------------------------------------------------------- */

MT_Scalar B3DPhysicsController::GetMass()
{
	if (b3Body_IsValid(m_bodyId))
		return (MT_Scalar)b3Body_GetMass(m_bodyId);
	return (MT_Scalar)m_mass;
}

void B3DPhysicsController::SetMass(MT_Scalar newmass)
{
	m_mass = (float)newmass;
	if (!b3Body_IsValid(m_bodyId) || m_suspended)
		return;
	if (newmass <= 1e-5f || GetMass() <= 1e-5f)
		return;

	ApplyMass((float)newmass);
	b3Body_SetAwake(m_bodyId, true);
}

void B3DPhysicsController::SetDamping(float linear, float angular)
{
	m_linearDamping = linear;
	m_angularDamping = angular;
	if (b3Body_IsValid(m_bodyId)) {
		b3Body_SetLinearDamping(m_bodyId, linear);
		b3Body_SetAngularDamping(m_bodyId, angular);
	}
}

void B3DPhysicsController::SetLinearDamping(float damping)
{
	SetDamping(damping, GetAngularDamping());
}

void B3DPhysicsController::SetAngularDamping(float damping)
{
	SetDamping(GetLinearDamping(), damping);
}

void B3DPhysicsController::SuspendDynamics(bool ghost)
{
	(void)ghost;

	if (!b3Body_IsValid(m_bodyId) || m_suspended)
		return;
	/* Sensors are never removed from the world, matching CcdPhysicsEnvironment. */
	if (m_isSensor)
		return;

	m_suspended = true;
	b3Body_Disable(m_bodyId);
}

void B3DPhysicsController::RestoreDynamics()
{
	if (!b3Body_IsValid(m_bodyId) || !m_suspended)
		return;

	/* Pick up whatever logic did to the object while it was suspended. */
	WriteMotionStateToDynamics(false);
	if (m_isDynamic)
		ApplyMass(m_mass);

	m_suspended = false;
	b3Body_Enable(m_bodyId);
	b3Body_SetAwake(m_bodyId, true);

	/* The body is simulated again, so the environment has to write its motion
	 * state back (a controller that left the set would leave the object frozen). */
	if (m_env && !m_env->HasController(this))
		m_env->AddController(this);
}

void B3DPhysicsController::SetActive(bool active)
{
	/* CcdPhysicsController implements this as a no-op as well. */
	(void)active;
}

void B3DPhysicsController::RefreshCollisions()
{
	/* Box3D rebuilds contacts every step, there is no broad phase proxy to
	 * refresh.  Nothing to do. */
}

/* -------------------------------------------------------------------------
 * Forces, impulses, velocities
 * ------------------------------------------------------------------------- */

void B3DPhysicsController::ApplyForce(const MT_Vector3& force, bool local)
{
	if (!b3Body_IsValid(m_bodyId))
		return;

	EnsureMovable();
	if (b3Body_GetType(m_bodyId) == b3_staticBody)
		return;

	b3Vec3 f = vecToB3(force);
	if (local)
		f = b3RotateVector(b3Body_GetRotation(m_bodyId), f);

	b3Body_ApplyForceToCenter(m_bodyId, f, true);
}

void B3DPhysicsController::ApplyTorque(const MT_Vector3& torque, bool local)
{
	if (!b3Body_IsValid(m_bodyId))
		return;

	EnsureMovable();
	if (b3Body_GetType(m_bodyId) == b3_staticBody)
		return;

	b3Vec3 t = vecToB3(torque);
	if (local)
		t = b3RotateVector(b3Body_GetRotation(m_bodyId), t);

	b3Body_ApplyTorque(m_bodyId, t, true);
}

void B3DPhysicsController::ApplyImpulse(const MT_Point3& attach, const MT_Vector3& impulse, bool local)
{
	if (!b3Body_IsValid(m_bodyId))
		return;

	b3Vec3 imp = vecToB3(impulse);
	if (imp.x * imp.x + imp.y * imp.y + imp.z * imp.z <= 1e-12f)
		return;

	EnsureMovable();
	if (b3Body_GetType(m_bodyId) == b3_staticBody)
		return;

	b3Pos p = b3Body_GetPosition(m_bodyId);
	b3Quat q = b3Body_GetRotation(m_bodyId);

	b3Vec3 impWorld;
	b3Vec3 pointWorld;
	if (local) {
		/* attach is an offset from the body origin and the impulse is in body
		 * space, exactly like CcdPhysicsController::ApplyImpulse(). */
		impWorld = b3RotateVector(q, imp);
		b3Vec3 rel = b3RotateVector(q, makeB3Vector((float)attach[0], (float)attach[1], (float)attach[2]));
		pointWorld = makeB3Vector(p.x + rel.x, p.y + rel.y, p.z + rel.z);
	} else {
		impWorld = imp;
		pointWorld = makeB3Vector((float)attach[0], (float)attach[1], (float)attach[2]);
	}

	b3Body_ApplyLinearImpulse(m_bodyId, impWorld, pointWorld, true);
}

void B3DPhysicsController::SetLinearVelocity(const MT_Vector3& lin_vel, bool local)
{
	if (!b3Body_IsValid(m_bodyId))
		return;

	EnsureMovable();
	if (b3Body_GetType(m_bodyId) == b3_staticBody)
		return;

	b3Vec3 v = vecToB3(lin_vel);

	/* Tiniest velocities cause instabilities, refuse them like Bullet does. */
	float len2 = v.x * v.x + v.y * v.y + v.z * v.z;
	if (len2 > 0.0f && len2 <= 1e-12f)
		v = makeB3Vector(0.0f, 0.0f, 0.0f);
	else if (local)
		v = b3RotateVector(b3Body_GetRotation(m_bodyId), v);

	b3Body_SetLinearVelocity(m_bodyId, v);
	b3Body_SetAwake(m_bodyId, true);
}

void B3DPhysicsController::SetAngularVelocity(const MT_Vector3& ang_vel, bool local)
{
	if (!b3Body_IsValid(m_bodyId))
		return;

	EnsureMovable();
	if (b3Body_GetType(m_bodyId) == b3_staticBody)
		return;

	b3Vec3 w = vecToB3(ang_vel);

	float len2 = w.x * w.x + w.y * w.y + w.z * w.z;
	if (len2 > 0.0f && len2 <= 1e-12f)
		w = makeB3Vector(0.0f, 0.0f, 0.0f);
	else if (local)
		w = b3RotateVector(b3Body_GetRotation(m_bodyId), w);

	b3Body_SetAngularVelocity(m_bodyId, w);
	b3Body_SetAwake(m_bodyId, true);
}

void B3DPhysicsController::ResolveCombinedVelocities(float linvelX, float linvelY, float linvelZ,
                                                     float angVelX, float angVelY, float angVelZ)
{
	/* CcdPhysicsController implements this as a no-op as well. */
	(void)linvelX; (void)linvelY; (void)linvelZ;
	(void)angVelX; (void)angVelY; (void)angVelZ;
}

MT_Vector3 B3DPhysicsController::GetLinearVelocity()
{
	if (!b3Body_IsValid(m_bodyId))
		return MT_Vector3(0.0f, 0.0f, 0.0f);
	return vecToMT(b3Body_GetLinearVelocity(m_bodyId));
}

MT_Vector3 B3DPhysicsController::GetAngularVelocity()
{
	if (!b3Body_IsValid(m_bodyId))
		return MT_Vector3(0.0f, 0.0f, 0.0f);
	return vecToMT(b3Body_GetAngularVelocity(m_bodyId));
}

MT_Vector3 B3DPhysicsController::GetVelocity(const MT_Point3& pos)
{
	if (!b3Body_IsValid(m_bodyId))
		return MT_Vector3(0.0f, 0.0f, 0.0f);

	/* Bullet's getVelocityInLocalPoint() treats the argument as an offset from
	 * the body origin; Box3D wants an absolute world point. */
	b3Pos p = b3Body_GetPosition(m_bodyId);
	b3Vec3 worldPoint = makeB3Vector(p.x + (float)pos[0], p.y + (float)pos[1], p.z + (float)pos[2]);
	return vecToMT(b3Body_GetWorldPointVelocity(m_bodyId, worldPoint));
}

MT_Vector3 B3DPhysicsController::GetLocalInertia()
{
	if (!b3Body_IsValid(m_bodyId))
		return MT_Vector3(0.0f, 0.0f, 0.0f);

	b3Matrix3 inertia = b3Body_GetLocalRotationalInertia(m_bodyId);
	return MT_Vector3((MT_Scalar)inertia.cx.x, (MT_Scalar)inertia.cy.y, (MT_Scalar)inertia.cz.z);
}

void B3DPhysicsController::SetRigidBody(bool rigid)
{
	m_isRigidBody = rigid;

	if (!b3Body_IsValid(m_bodyId))
		return;

	if (!rigid) {
		/* Bullet locks all angular axes for a non rigid dynamic object.  Box3D
		 * cannot change motion locks after body creation, so only the rotation
		 * itself can be stopped here. */
		b3Body_SetAngularVelocity(m_bodyId, makeB3Vector(0.0f, 0.0f, 0.0f));
	}
}

void B3DPhysicsController::ApplyVelocityClamps()
{
	if (!b3Body_IsValid(m_bodyId))
		return;
	if (b3Body_GetType(m_bodyId) == b3_staticBody)
		return;

	if (m_clampVelMax > 0.0f || m_clampVelMin > 0.0f) {
		b3Vec3 v = b3Body_GetLinearVelocity(m_bodyId);
		float len = vecLength(v);

		if (m_clampVelMax > 0.0f && len > m_clampVelMax) {
			float s = m_clampVelMax / len;
			b3Body_SetLinearVelocity(m_bodyId, makeB3Vector(v.x * s, v.y * s, v.z * s));
		} else if (m_clampVelMin > 0.0f && len > 1e-6f && len < m_clampVelMin) {
			float s = m_clampVelMin / len;
			b3Body_SetLinearVelocity(m_bodyId, makeB3Vector(v.x * s, v.y * s, v.z * s));
		}
	}

	if (m_clampAngVelMax > 0.0f || m_clampAngVelMin > 0.0f) {
		b3Vec3 w = b3Body_GetAngularVelocity(m_bodyId);
		float len = vecLength(w);

		if (m_clampAngVelMax > 0.0f && len > m_clampAngVelMax) {
			float s = m_clampAngVelMax / len;
			b3Body_SetAngularVelocity(m_bodyId, makeB3Vector(w.x * s, w.y * s, w.z * s));
		} else if (m_clampAngVelMin > 0.0f && len > 1e-6f && len < m_clampAngVelMin) {
			float s = m_clampAngVelMin / len;
			b3Body_SetAngularVelocity(m_bodyId, makeB3Vector(w.x * s, w.y * s, w.z * s));
		}
	}
}

/* -------------------------------------------------------------------------
 * Not implemented for Box3D yet
 * ------------------------------------------------------------------------- */

void B3DPhysicsController::AddCompoundChild(PHY_IPhysicsController *child)
{
	/* Dynamic parenting (KX_GameObject::SetParent with the compound flag): the
	 * child's shape joins this body and the child leaves the simulation, exactly
	 * like CcdPhysicsController::AddCompoundChild does.  IsCompound() is true for
	 * an object the converter built as a compound root as well, so a parent that
	 * has no children yet still accepts one. */
	B3DPhysicsController *childCtrl = dynamic_cast<B3DPhysicsController *>(child);
	if (!childCtrl || childCtrl == this || !IsCompound())
		return;
	if (!b3Body_IsValid(m_bodyId) || !b3Body_IsValid(childCtrl->m_bodyId))
		return;
	if (!childCtrl->m_env || !childCtrl->m_env->IsWorldAlive())
		return;

	/* Relative placement, measured between the two bodies so that the simulated
	 * state is used rather than the scene graph.  The parent's scale is undone
	 * because the child shape carries its own world scale. */
	b3Pos rootPos = b3Body_GetPosition(m_bodyId);
	b3Quat rootRot = b3Body_GetRotation(m_bodyId);
	b3Pos childPos = b3Body_GetPosition(childCtrl->m_bodyId);
	b3Quat childRot = b3Body_GetRotation(childCtrl->m_bodyId);

	b3Vec3 delta = b3Sub(childPos, rootPos);
	float invScale[3];
	for (int i = 0; i < 3; i++) {
		invScale[i] = fabsf((float)m_shapeDesc.scaling[i]);
		invScale[i] = (invScale[i] > 1e-9f) ? 1.0f / invScale[i] : 1.0f;
	}
	delta.x *= invScale[0];
	delta.y *= invScale[1];
	delta.z *= invScale[2];

	B3DChildTransform local;
	local.position = vecToMT(b3InvRotateVector(rootRot, delta));
	local.rotation = matToMT(b3MulQuat(b3Conjugate(rootRot), childRot));

	if (!AddChildShape(childCtrl->m_shapeDesc, local))
		return;

	/* Remember the owner so that RemoveCompoundChild() can find the shape again.
	 * The child's own body is disabled rather than taken out of the environment's
	 * controller set, so that RestoreDynamics() can bring it back on its own. */
	m_childDescs.back().owner = child;
	childCtrl->SuspendDynamics(false);
}

void B3DPhysicsController::RemoveCompoundChild(PHY_IPhysicsController *child)
{
	if (!child)
		return;

	for (size_t i = 0; i < m_childDescs.size() && i < m_childShapes.size(); i++) {
		if (m_childDescs[i].owner != child)
			continue;

		/* Destroy the shape without letting Box3D recompute the mass yet: the
		 * rebuild below does it once for all remaining shapes. */
		if (b3Shape_IsValid(m_childShapes[i]))
			b3DestroyShape(m_childShapes[i], false);

		m_childShapes.erase(m_childShapes.begin() + i);
		m_childDescs.erase(m_childDescs.begin() + i);

		if (b3Body_IsValid(m_bodyId)) {
			b3Body_ApplyMassFromShapes(m_bodyId);
			ApplyMass(m_mass);
		}
		return;
	}
}

bool B3DPhysicsController::ReinstancePhysicsShape(KX_GameObject *from_gameobj, RAS_MeshObject *from_meshobj)
{
	/* Replacing the collision shape of a live body needs the mesh paths, which
	 * are not implemented yet. */
	(void)from_gameobj;
	(void)from_meshobj;
	return false;
}

void B3DPhysicsController::ReplicateConstraints(KX_GameObject *gameobj, std::vector<KX_GameObject*> constobj)
{
	(void)gameobj;
	(void)constobj;
}
