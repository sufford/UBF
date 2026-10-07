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

/** \file JoltPhysicsController.cpp
 *  \ingroup physjolt
 */

#include "JoltPhysicsController.h"
#include "JoltPhysicsEnvironment.h"
#include "JoltMotionState.h"

#include "PHY_IMotionState.h"
#include "PHY_IPhysicsEnvironment.h"

#include "KX_GameObject.h"
#include "KX_Scene.h"
#include "KX_BlenderSceneConverter.h"
#include "KX_KetsjiEngine.h"

#include "MT_MinMax.h"

#include <Jolt/Physics/Body/Body.h>
#include <Jolt/Physics/Body/BodyInterface.h>
#include <Jolt/Physics/Body/BodyLock.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Body/MassProperties.h>
#include <Jolt/Physics/Body/MotionProperties.h>
#include <Jolt/Physics/Body/MotionQuality.h>
#include <Jolt/Physics/Body/MotionType.h>
#include <Jolt/Physics/Body/AllowedDOFs.h>
#include <Jolt/Physics/EActivation.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Collision/Shape/SphereShape.h>
#include <Jolt/Physics/Collision/Shape/CapsuleShape.h>
#include <Jolt/Physics/Collision/Shape/CylinderShape.h>
#include <Jolt/Physics/Collision/Shape/ConvexHullShape.h>
#include <Jolt/Physics/Collision/Shape/MeshShape.h>
#include <Jolt/Physics/Collision/Shape/RotatedTranslatedShape.h>
#include <Jolt/Physics/Collision/Shape/ScaledShape.h>
#include <Jolt/Geometry/IndexedTriangle.h>
#include <Jolt/Math/Math.h>

#include <stdio.h>
#include <math.h>


/* -------------------------------------------------------------------------
 * moto (Blender) <-> Jolt conversion helpers
 *
 * Jolt's quaternion is (x, y, z, w), the same order PHY_IMotionState uses.
 * JPH::RVec3 is the "large world" position type: it is JPH::Vec3 unless
 * JPH_DOUBLE_PRECISION is set, so both are handled through the same helpers.
 * ------------------------------------------------------------------------- */

/* Half extents and radii below this are degenerate (a plane has a zero
 * thickness bound box, and Jolt's CapsuleShape asserts on a non positive
 * half height), so a thin slab is used instead. */
#define JOLT_MIN_HALF_EXTENT 0.01f

/* Number of segments used when a cone has to be tessellated into a convex hull;
 * Jolt has no cone shape of its own. */
#define JOLT_CONE_SEGMENTS 16

static inline JPH::Vec3 vecToJolt(const MT_Vector3& v)
{
	return JPH::Vec3((float)v[0], (float)v[1], (float)v[2]);
}

static inline MT_Vector3 vecToMT(const JPH::Vec3& v)
{
	return MT_Vector3((MT_Scalar)v.GetX(), (MT_Scalar)v.GetY(), (MT_Scalar)v.GetZ());
}

static inline MT_Vector3 posToMT(const JPH::RVec3& v)
{
	return MT_Vector3((MT_Scalar)v.GetX(), (MT_Scalar)v.GetY(), (MT_Scalar)v.GetZ());
}

static inline float vecLength(const JPH::Vec3& v)
{
	return sqrtf(v.GetX() * v.GetX() + v.GetY() * v.GetY() + v.GetZ() * v.GetZ());
}

/** Jolt's cylinder, capsule and cone are aligned with their local Y axis, while
 *  BGE (and therefore every bound box Blender reports) is Z up.  This is the
 *  rotation that maps the shape's local Y onto world Z; Bullet solves the same
 *  problem by using its *Z shape variants. */
static inline JPH::Quat zUpShapeRotation()
{
	return JPH::Quat::sRotation(JPH::Vec3::sAxisX(), 0.5f * JPH::JPH_PI);
}

/* ------------------------------------------------------------------------- */

JoltShapeDesc::JoltShapeDesc()
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

JoltChildTransform::JoltChildTransform()
{
	position.setValue(0.0f, 0.0f, 0.0f);
	rotation.setIdentity();
}

JoltChildShapeDesc::JoltChildShapeDesc()
	: owner(NULL)
{
}

/* ------------------------------------------------------------------------- */

JoltPhysicsController::JoltPhysicsController(JoltPhysicsEnvironment *env, PHY_IMotionState *motionstate, bool isDynamic)
	: m_env(env),
	  m_motionState(motionstate),
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
	  m_registerCount(0)
{
	m_scaling.setValue(1.0f, 1.0f, 1.0f);
	m_linearFactor.setValue(1.0f, 1.0f, 1.0f);
	m_angularFactor.setValue(1.0f, 1.0f, 1.0f);

	/* The environment has to be able to invalidate this controller even when its
	 * body never enters the simulated set (sensors, inactive layers, suspension),
	 * see JoltPhysicsEnvironment::AddKnownController(). */
	if (m_env)
		m_env->AddKnownController(this);
}

JoltPhysicsController::~JoltPhysicsController()
{
	if (m_env) {
		m_env->RemoveController(this);
		m_env->RemoveKnownController(this);
	}

	/* EndObject() destroys the controller while the world is still alive, so the
	 * body has to be released here or it would linger as invisible geometry.
	 * If the environment went away first the whole physics system (and every
	 * body in it) is already gone, so nothing may be touched any more. */
	if (m_env && m_env->IsWorldAlive() && IsBodyAlive()) {
		/* A suspended controller already removed its body from the world; asking
		 * Jolt to remove it again would touch a body that is not in the broad
		 * phase (Jolt asserts on that).  The body itself still has to be
		 * destroyed, otherwise it stays allocated in the body manager. */
		if (m_inWorld)
			m_env->GetBodyInterface().RemoveBody(m_bodyId);
		m_env->GetBodyInterface().DestroyBody(m_bodyId);
	}
	m_bodyId = JPH::BodyID();
	m_inWorld = false;
	m_shape = nullptr;
	m_childShapes.clear();

	if (m_motionState)
		delete m_motionState;
	m_motionState = NULL;
}

JPH::EMotionType JoltPhysicsController::EffectiveMotionType() const
{
	if (!m_env || !m_env->IsWorldAlive() || !IsBodyAlive())
		return JPH::EMotionType::Static;
	return m_env->GetBodyInterface().GetMotionType(m_bodyId);
}

JPH::BodyInterface& JoltPhysicsController::GetBodyInterface() const
{
	return m_env->GetBodyInterface();
}

bool JoltPhysicsController::IsBodyAlive() const
{
	/* JPH::BodyID has no "still valid" query of its own: an id that was never
	 * handed out (or was already recycled) has index ~0.  Every caller also has
	 * to check that the environment is alive, see the destructor. */
	return !m_bodyId.IsInvalid();
}

JPH::EMotionType JoltPhysicsController::MotionType() const
{
	if (!m_isDynamic)
		return m_isKinematic ? JPH::EMotionType::Kinematic : JPH::EMotionType::Static;
	return JPH::EMotionType::Dynamic;
}

/* -------------------------------------------------------------------------
 * Shape construction
 * ------------------------------------------------------------------------- */

bool JoltPhysicsController::BuildShapeFromDesc(const JoltShapeDesc& shape, JPH::RefConst<JPH::Shape>& out_shape) const
{
	using namespace JPH;

	const float sx = fabsf((float)shape.scaling[0]);
	const float sy = fabsf((float)shape.scaling[1]);
	const float sz = fabsf((float)shape.scaling[2]);

	/* A Blender object scale is baked into the Jolt shape, because a Jolt shape
	 * has no scale of its own other than a ScaledShape decorator.  Baking keeps
	 * the convex radius and the mass properties consistent with the visible size;
	 * averaging the scale would not, so every case below picks the axes its own
	 * geometry uses. */
	const float radialScale = MT_max(sx, sy);

	RefConst<Shape> base;

	switch (shape.shapeType) {
		case PHY_SHAPE_SPHERE:
		{
			const float radius = MT_max((float)shape.radius * MT_max(radialScale, sz), JOLT_MIN_HALF_EXTENT);
			base = new SphereShape(radius);
			break;
		}

		case PHY_SHAPE_BOX:
		{
			const Vec3 half_extent(MT_max((float)shape.halfExtents[0] * sx, JOLT_MIN_HALF_EXTENT),
			                       MT_max((float)shape.halfExtents[1] * sy, JOLT_MIN_HALF_EXTENT),
			                       MT_max((float)shape.halfExtents[2] * sz, JOLT_MIN_HALF_EXTENT));
			base = new BoxShape(half_extent);
			break;
		}

		case PHY_SHAPE_CYLINDER:
		{
			const float half_height = MT_max(0.5f * (float)shape.height * sz, JOLT_MIN_HALF_EXTENT);
			const float radius = MT_max((float)shape.radius * radialScale, JOLT_MIN_HALF_EXTENT);
			base = new RotatedTranslatedShape(Vec3::sZero(), zUpShapeRotation(),
			                                  new CylinderShape(half_height, radius));
			break;
		}

		case PHY_SHAPE_CAPSULE:
		{
			/* Jolt asserts on a non positive half height of the cylinder part, so
			 * a capsule whose height collapsed to its diameter becomes a sphere
			 * like capsule instead of an invalid shape. */
			const float half_height = MT_max(0.5f * (float)shape.height * sz, JOLT_MIN_HALF_EXTENT);
			const float radius = MT_max((float)shape.radius * radialScale, JOLT_MIN_HALF_EXTENT);
			base = new RotatedTranslatedShape(Vec3::sZero(), zUpShapeRotation(),
			                                  new CapsuleShape(half_height, radius));
			break;
		}

		case PHY_SHAPE_CONE:
		{
			/* No cone shape in Jolt: tessellate it into a convex hull, the same
			 * way the Box3D backend does. */
			const float radius = MT_max((float)shape.radius * radialScale, JOLT_MIN_HALF_EXTENT);
			const float half_height = MT_max(0.5f * (float)shape.height * sz, JOLT_MIN_HALF_EXTENT);

			Array<Vec3> points;
			points.reserve(JOLT_CONE_SEGMENTS + 1);
			points.push_back(Vec3(0.0f, half_height, 0.0f));
			for (int i = 0; i < JOLT_CONE_SEGMENTS; i++) {
				const float a = (2.0f * JPH_PI * i) / JOLT_CONE_SEGMENTS;
				points.push_back(Vec3(radius * cosf(a), -half_height, radius * sinf(a)));
			}

			ConvexHullShapeSettings settings(points);
			ShapeSettings::ShapeResult result = settings.Create();
			if (result.HasError()) {
				printf("JoltPhysics: could not build a convex hull for a cone bound (%s), a box is used instead\n",
				       result.GetError().c_str());
				return false;
			}
			base = result.Get();
			break;
		}

		case PHY_SHAPE_POLYTOPE:
		{
			const size_t num_points = shape.hullPoints.size() / 3;
			if (num_points < 4)
				return false;

			Array<Vec3> points;
			points.reserve((int)num_points);
			for (size_t i = 0; i < num_points; i++) {
				points.push_back(Vec3((float)shape.hullPoints[i * 3 + 0] * sx,
				                      (float)shape.hullPoints[i * 3 + 1] * sy,
				                      (float)shape.hullPoints[i * 3 + 2] * sz));
			}

			ConvexHullShapeSettings settings(points);
			ShapeSettings::ShapeResult result = settings.Create();
			if (result.HasError()) {
				printf("JoltPhysics: could not build a convex hull (%s), a box is used instead\n",
				       result.GetError().c_str());
				return false;
			}
			base = result.Get();
			break;
		}

		case PHY_SHAPE_MESH:
		{
			const size_t num_verts = shape.meshVertices.size() / 3;
			const size_t num_indices = shape.meshIndices.size();
			if (num_verts < 3 || num_indices < 3)
				return false;

			VertexList vertices;
			vertices.reserve((int)num_verts);
			for (size_t i = 0; i < num_verts; i++) {
				vertices.push_back(Float3((float)shape.meshVertices[i * 3 + 0] * sx,
				                          (float)shape.meshVertices[i * 3 + 1] * sy,
				                          (float)shape.meshVertices[i * 3 + 2] * sz));
			}

			IndexedTriangleList triangles;
			triangles.reserve((int)(num_indices / 3));
			for (size_t i = 0; i + 2 < num_indices; i += 3) {
				triangles.push_back(IndexedTriangle((uint32)shape.meshIndices[i + 0],
				                                    (uint32)shape.meshIndices[i + 1],
				                                    (uint32)shape.meshIndices[i + 2]));
			}

			MeshShapeSettings settings(vertices, triangles);
			ShapeSettings::ShapeResult result = settings.Create();
			if (result.HasError()) {
				printf("JoltPhysics: could not cook a triangle mesh (%s), a box is used instead\n",
				       result.GetError().c_str());
				return false;
			}
			base = result.Get();
			break;
		}

		default:
			return false;
	}

	out_shape = base;
	return true;
}

/* -------------------------------------------------------------------------
 * Body construction
 * ------------------------------------------------------------------------- */

bool JoltPhysicsController::Build(const JoltShapeDesc& shape,
                                  float mass,
                                  const MT_Vector3& linearFactor,
                                  const MT_Vector3& angularFactor,
                                  bool applyAngularLocks,
                                  float linearDamping,
                                  float angularDamping,
                                  float linVelMin,
                                  float linVelMax,
                                  float angVelMin,
                                  float angVelMax,
                                  float radius,
                                  float margin)
{
	using namespace JPH;

	/* Everything below is also what a rebuild (replica, environment change) needs,
	 * so the description is stored before any early out. */
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

	if (!m_env || !m_env->IsWorldAlive())
		return false;

	if (!BuildShapeFromDesc(shape, m_shape)) {
		/* The description was not usable (a degenerate hull, a mesh Jolt could not
		 * cook).  The caller falls back to a box, so nothing is created here. */
		return false;
	}

	if (!m_motionState)
		return false;

	float pos[3];
	float quat[4];
	m_motionState->GetWorldPosition(pos[0], pos[1], pos[2]);
	m_motionState->GetWorldOrientation(quat[0], quat[1], quat[2], quat[3]);

	BodyCreationSettings settings(m_shape,
	                              RVec3((float)pos[0], (float)pos[1], (float)pos[2]),
	                              Quat(quat[0], quat[1], quat[2], quat[3]).Normalized(),
	                              MotionType(),
	                              m_isDynamic ? JoltLayers::MOVING : JoltLayers::NON_MOVING);

	settings.mFriction = MT_max(shape.friction, 0.0f);
	settings.mRestitution = (shape.restitution < 0.0f) ? 0.0f : ((shape.restitution > 1.0f) ? 1.0f : shape.restitution);
	settings.mLinearDamping = MT_max(linearDamping, 0.0f);
	settings.mAngularDamping = MT_max(angularDamping, 0.0f);
	settings.mIsSensor = m_isSensor;
	settings.mAllowSleeping = true;
	settings.mCollideKinematicVsNonDynamic = false;
	settings.mGravityFactor = m_isDynamic ? 1.0f : 0.0f;
	settings.mAllowedDOFs = AllowedDOFsFromFactors(linearFactor, angularFactor);
	/* The environment finds the controller of a body (ray hits, contacts) through
	 * the body's user data. */
	settings.mUserData = (uint64)(uintptr_t)this;

	/* Jolt's maximum velocities have the same meaning as Blender's clamps, but
	 * Blender lets the clamp be disabled (0).  The Blender clamp is applied
	 * explicitly in ApplyVelocityClamps() so that the setting keeps its meaning;
	 * Jolt's own default is used as the hard safety net. */
	if (linVelMax > 0.0f)
		settings.mMaxLinearVelocity = linVelMax;
	if (angVelMax > 0.0f)
		settings.mMaxAngularVelocity = angVelMax;

	/* Blender's mass is authoritative: let Jolt calculate the inertia from the
	 * shape but scale it so that the mass comes out as the value the user set.
	 * This is EOverrideMassProperties::CalculateInertia. */
	if (m_isDynamic) {
		if (mass <= 1e-5f) {
			/* A dynamic object with no mass cannot be simulated; treat it as
			 * static like Bullet does (it ends up with a zero mass shape). */
			printf("JoltPhysics: a dynamic object has no mass, it is created as a static body\n");
			settings.mMotionType = EMotionType::Static;
			settings.mGravityFactor = 0.0f;
			if (settings.mObjectLayer == JoltLayers::MOVING)
				settings.mObjectLayer = JoltLayers::NON_MOVING;
		}
		else {
			settings.mOverrideMassProperties = EOverrideMassProperties::CalculateInertia;
			settings.mMassPropertiesOverride.mMass = mass;
		}
	}

	/* A sensor may not be simulated as a dynamic body: Jolt would still integrate
	 * it.  BGE sensors are trigger volumes, so they become kinematic. */
	if (m_isSensor && settings.mMotionType == EMotionType::Dynamic)
		settings.mMotionType = EMotionType::Kinematic;

	m_bodyId = m_env->GetBodyInterface().CreateAndAddBody(settings, EActivation::Activate);
	if (m_bodyId.IsInvalid())
		return false;

	m_inWorld = true;
	m_suspended = false;
	return true;
}

JPH::EAllowedDOFs JoltPhysicsController::AllowedDOFsFromFactors(const MT_Vector3& linearFactor, const MT_Vector3& angularFactor) const
{
	using namespace JPH;

	uint8 dofs = 0;
	if (linearFactor[0] > 0.5)
		dofs |= (uint8)EAllowedDOFs::TranslationX;
	if (linearFactor[1] > 0.5)
		dofs |= (uint8)EAllowedDOFs::TranslationY;
	if (linearFactor[2] > 0.5)
		dofs |= (uint8)EAllowedDOFs::TranslationZ;
	if (angularFactor[0] > 0.5)
		dofs |= (uint8)EAllowedDOFs::RotationX;
	if (angularFactor[1] > 0.5)
		dofs |= (uint8)EAllowedDOFs::RotationY;
	if (angularFactor[2] > 0.5)
		dofs |= (uint8)EAllowedDOFs::RotationZ;

	/* "No degrees of freedom at all" is not a valid body: Jolt asserts on it.  A
	 * fully locked object keeps its translations, which is what Bullet does with
	 * a linear factor of zero (the body is simply never moved and never rotated,
	 * because the solver cannot apply any impulse with a zero factor). */
	if (dofs == 0)
		dofs = (uint8)EAllowedDOFs::All;

	return (EAllowedDOFs)dofs;
}

void JoltPhysicsController::ApplyMass(float mass)
{
	using namespace JPH;

	if (!IsBodyAlive() || !m_env || !m_env->IsWorldAlive() || !m_shape)
		return;

	BodyLockWrite lock(m_env->GetBodyLockInterface(), m_bodyId);
	if (!lock.Succeeded())
		return;

	Body& body = lock.GetBody();
	MotionProperties *mp = body.GetMotionProperties();
	if (!mp)
		return;

	MassProperties mass_properties = m_shape->GetMassProperties();
	mass_properties.ScaleToMass(mass);
	mp->SetMassProperties(mp->GetAllowedDOFs(), mass_properties);
}

/* -------------------------------------------------------------------------
 * Transform synchronization
 * ------------------------------------------------------------------------- */

bool JoltPhysicsController::SynchronizeMotionStates(float time)
{
	(void)time;

	if (!m_motionState || !m_env || !m_env->IsWorldAlive() || !IsBodyAlive() || m_suspended)
		return false;

	/* Mirrors CcdPhysicsController: static bodies are not written back, the
	 * scene graph is the authority for them (see KX_GameObject::UpdateTransform,
	 * which calls SetTransform() for every non dynamic controller).  Kinematic
	 * bodies are written back as well: nothing in the solver moves them, but the
	 * scene graph still wants to know where they ended up. */
	if (MotionType() == JPH::EMotionType::Static)
		return false;

	JPH::RVec3 position;
	JPH::Quat rotation;
	m_env->GetBodyInterface().GetPositionAndRotation(m_bodyId, position, rotation);

	m_motionState->SetWorldPosition((float)position.GetX(), (float)position.GetY(), (float)position.GetZ());
	m_motionState->SetWorldOrientation(rotation.GetX(), rotation.GetY(), rotation.GetZ(), rotation.GetW());
	m_motionState->CalculateWorldTransformations();
	return true;
}

void JoltPhysicsController::WriteMotionStateToDynamics(bool nondynaonly)
{
	if (!m_motionState || !m_env || !m_env->IsWorldAlive() || !IsBodyAlive())
		return;

	/* Never let a stale scene graph transform overwrite the simulation. */
	if (nondynaonly && MotionType() == JPH::EMotionType::Dynamic)
		return;

	float pos[3];
	float quat[4];
	m_motionState->GetWorldPosition(pos[0], pos[1], pos[2]);
	m_motionState->GetWorldOrientation(quat[0], quat[1], quat[2], quat[3]);

	m_env->GetBodyInterface().SetPositionAndRotation(m_bodyId,
	                                                 JPH::RVec3(pos[0], pos[1], pos[2]),
	                                                 JPH::Quat(quat[0], quat[1], quat[2], quat[3]).Normalized(),
	                                                 JPH::EActivation::Activate);
}

void JoltPhysicsController::WriteDynamicsToMotionState()
{
	SynchronizeMotionStates(0.0f);
}

void JoltPhysicsController::SetTransform()
{
	WriteMotionStateToDynamics(false);
}

/* -------------------------------------------------------------------------
 * Position, orientation, scaling
 * ------------------------------------------------------------------------- */

void JoltPhysicsController::SetPosition(const MT_Vector3& pos)
{
	if (!m_env || !m_env->IsWorldAlive() || !IsBodyAlive())
		return;

	JPH::Quat rotation = m_env->GetBodyInterface().GetRotation(m_bodyId);
	m_env->GetBodyInterface().SetPositionAndRotation(m_bodyId,
	                                                 JPH::RVec3((float)pos[0], (float)pos[1], (float)pos[2]),
	                                                 rotation,
	                                                 JPH::EActivation::Activate);
}

void JoltPhysicsController::GetPosition(MT_Vector3& pos) const
{
	if (!m_env || !m_env->IsWorldAlive() || !IsBodyAlive()) {
		pos.setValue(0.0f, 0.0f, 0.0f);
		return;
	}
	pos = posToMT(m_env->GetBodyInterface().GetPosition(m_bodyId));
}

void JoltPhysicsController::RelativeTranslate(const MT_Vector3& dloc, bool local)
{
	if (!m_env || !m_env->IsWorldAlive() || !IsBodyAlive())
		return;

	const JPH::RVec3 position = m_env->GetBodyInterface().GetPosition(m_bodyId);
	JPH::Vec3 delta = vecToJolt(dloc);

	if (local)
		delta = m_env->GetBodyInterface().GetRotation(m_bodyId) * delta;

	const JPH::Quat rotation = m_env->GetBodyInterface().GetRotation(m_bodyId);
	m_env->GetBodyInterface().SetPositionAndRotation(m_bodyId,
	                                                 position + JPH::RVec3(delta),
	                                                 rotation,
	                                                 JPH::EActivation::Activate);
}

MT_Matrix3x3 JoltPhysicsController::GetOrientation()
{
	if (!m_env || !m_env->IsWorldAlive() || !IsBodyAlive()) {
		MT_Matrix3x3 identity;
		identity.setIdentity();
		return identity;
	}

	const JPH::Quat q = m_env->GetBodyInterface().GetRotation(m_bodyId);
	const MT_Quaternion mq((MT_Scalar)q.GetX(), (MT_Scalar)q.GetY(), (MT_Scalar)q.GetZ(), (MT_Scalar)q.GetW());
	return MT_Matrix3x3(mq);
}

void JoltPhysicsController::SetOrientation(const MT_Matrix3x3& orn)
{
	if (!m_env || !m_env->IsWorldAlive() || !IsBodyAlive())
		return;

	/* moto has no quaternion constructor from a matrix, see MT_Matrix3x3.h. */
	const MT_Quaternion mq = orn.getRotation();
	const JPH::RVec3 position = m_env->GetBodyInterface().GetPosition(m_bodyId);
	m_env->GetBodyInterface().SetPositionAndRotation(m_bodyId,
	                                                 position,
	                                                 JPH::Quat((float)mq[0], (float)mq[1], (float)mq[2], (float)mq[3]).Normalized(),
	                                                 JPH::EActivation::Activate);
}

void JoltPhysicsController::RelativeRotate(const MT_Matrix3x3& drot, bool local)
{
	if (!m_env || !m_env->IsWorldAlive() || !IsBodyAlive())
		return;

	const MT_Quaternion mq = drot.getRotation();
	const JPH::Quat qd((float)mq[0], (float)mq[1], (float)mq[2], (float)mq[3]);

	const JPH::Quat qc = m_env->GetBodyInterface().GetRotation(m_bodyId);
	const JPH::Quat qn = local ? (qc * qd) : (qd * qc);

	const JPH::RVec3 position = m_env->GetBodyInterface().GetPosition(m_bodyId);
	m_env->GetBodyInterface().SetPositionAndRotation(m_bodyId, position, qn.Normalized(), JPH::EActivation::Activate);
}

void JoltPhysicsController::SetScaling(const MT_Vector3& scale)
{
	/* Jolt bakes the scale into the shape at creation time, so a live body cannot
	 * be rescaled.  The bound boxes Blender reports already carry the object
	 * scale, so ConvertObject bakes the current scale in; a runtime rescale is
	 * recorded for a later rebuild but is not applied to the running shape. */
	m_scaling = scale;
}

/* -------------------------------------------------------------------------
 * Mass, damping, activation
 * ------------------------------------------------------------------------- */

MT_Scalar JoltPhysicsController::GetMass()
{
	if (m_env && m_env->IsWorldAlive() && IsBodyAlive() && EffectiveMotionType() == JPH::EMotionType::Dynamic) {
		JPH::BodyLockRead lock(m_env->GetBodyLockInterface(), m_bodyId);
		if (lock.Succeeded()) {
			const JPH::MotionProperties *mp = lock.GetBody().GetMotionProperties();
			if (mp && mp->GetInverseMass() > 0.0f)
				return (MT_Scalar)(1.0f / mp->GetInverseMass());
		}
	}
	return (MT_Scalar)m_mass;
}

void JoltPhysicsController::SetMass(MT_Scalar newmass)
{
	m_mass = (float)newmass;

	if (!m_env || !m_env->IsWorldAlive() || !IsBodyAlive() || m_suspended)
		return;
	/* A static or kinematic body has no mass to change. */
	if (EffectiveMotionType() != JPH::EMotionType::Dynamic)
		return;
	if (newmass <= 1e-5f)
		return;

	ApplyMass((float)newmass);
	m_env->GetBodyInterface().ActivateBody(m_bodyId);
}

void JoltPhysicsController::SetDamping(float linear, float angular)
{
	m_linearDamping = linear;
	m_angularDamping = angular;

	if (!m_env || !m_env->IsWorldAlive() || !IsBodyAlive())
		return;

	/* BodyInterface has no damping setter: damping lives on MotionProperties,
	 * which is only reachable through a body lock. */
	JPH::BodyLockWrite lock(m_env->GetBodyLockInterface(), m_bodyId);
	if (!lock.Succeeded())
		return;

	JPH::MotionProperties *mp = lock.GetBody().GetMotionProperties();
	if (!mp)
		return;

	mp->SetLinearDamping(MT_max(linear, 0.0f));
	mp->SetAngularDamping(MT_max(angular, 0.0f));
}

void JoltPhysicsController::SetLinearDamping(float damping)
{
	SetDamping(damping, GetAngularDamping());
}

void JoltPhysicsController::SetAngularDamping(float damping)
{
	SetDamping(GetLinearDamping(), damping);
}

void JoltPhysicsController::SuspendDynamics(bool ghost)
{
	(void)ghost;

	if (!m_env || !m_env->IsWorldAlive() || !IsBodyAlive() || m_suspended)
		return;
	/* Sensors are never removed from the world, matching CcdPhysicsEnvironment. */
	if (m_isSensor)
		return;

	m_suspended = true;
	if (m_inWorld) {
		m_env->GetBodyInterface().RemoveBody(m_bodyId);
		m_inWorld = false;
	}
	m_env->RemoveController(this);
}

void JoltPhysicsController::RestoreDynamics()
{
	if (!m_env || !m_env->IsWorldAlive() || !IsBodyAlive() || !m_suspended)
		return;

	/* Pick up whatever logic did to the object while it was suspended. */
	WriteMotionStateToDynamics(false);

	m_suspended = false;
	if (!m_inWorld) {
		m_env->GetBodyInterface().AddBody(m_bodyId, JPH::EActivation::Activate);
		m_inWorld = true;
	}

	/* The body is simulated again, so the environment has to write its motion
	 * state back (a controller that left the set would leave the object frozen). */
	if (!m_env->HasController(this))
		m_env->AddController(this);
}

void JoltPhysicsController::SetActive(bool active)
{
	/* CcdPhysicsController implements this as a no-op as well. */
	(void)active;
}

void JoltPhysicsController::SetInWorld(bool in_world)
{
	if (!m_env || !m_env->IsWorldAlive() || !IsBodyAlive())
		return;
	if (m_suspended)
		return;
	if (in_world == m_inWorld)
		return;

	if (in_world) {
		m_env->GetBodyInterface().AddBody(m_bodyId, JPH::EActivation::Activate);
		m_inWorld = true;
	}
	else {
		m_env->GetBodyInterface().RemoveBody(m_bodyId);
		m_inWorld = false;
	}
}

void JoltPhysicsController::OnEnvironmentDestroyed()
{
	/* The physics system that owns the body is being destroyed, so the body id and
	 * the shape must not be touched again.  The motion state stays: it belongs to
	 * the scene graph node and is deleted with this controller. */
	m_env = NULL;
	m_bodyId = JPH::BodyID();
	m_shape = nullptr;
	m_childShapes.clear();
	m_inWorld = false;
	m_suspended = false;
}

void JoltPhysicsController::SetAllowSleeping(bool allow)
{
	if (!m_env || !m_env->IsWorldAlive() || !IsBodyAlive())
		return;

	JPH::BodyLockWrite lock(m_env->GetBodyLockInterface(), m_bodyId);
	if (!lock.Succeeded())
		return;

	/* Body::SetAllowSleeping() writes through the motion properties, and
	 * ActivateBody() dereferences them as well, so neither is legal for a static
	 * body. */
	if (lock.GetBody().IsStatic())
		return;

	lock.GetBody().SetAllowSleeping(allow);
	if (!allow)
		m_env->GetBodyInterface().ActivateBody(m_bodyId);
}

void JoltPhysicsController::RefreshCollisions()
{
	if (!m_env || !m_env->IsWorldAlive() || !IsBodyAlive())
		return;

	/* Called after the collision group/mask of an object changed.  Jolt's broad
	 * phase caches the layer of a body at insertion time, so re-inserting is what
	 * makes a layer change visible.  Until the mask based layer setup exists this
	 * only resets the sleep timer of the body.
	 * A static body is skipped: it has no motion properties to activate and it
	 * has no contacts to recompute. */
	if (EffectiveMotionType() == JPH::EMotionType::Static)
		return;

	m_env->GetBodyInterface().ActivateBody(m_bodyId);
}

/* -------------------------------------------------------------------------
 * Forces, impulses, velocities
 * ------------------------------------------------------------------------- */

void JoltPhysicsController::ApplyForce(const MT_Vector3& force, bool local)
{
	if (!m_env || !m_env->IsWorldAlive() || !IsBodyAlive())
		return;
	if (EffectiveMotionType() != JPH::EMotionType::Dynamic)
		return;

	JPH::Vec3 f = vecToJolt(force);
	if (local)
		f = m_env->GetBodyInterface().GetRotation(m_bodyId) * f;

	m_env->GetBodyInterface().AddForce(m_bodyId, f);
}

void JoltPhysicsController::ApplyTorque(const MT_Vector3& torque, bool local)
{
	if (!m_env || !m_env->IsWorldAlive() || !IsBodyAlive())
		return;
	if (EffectiveMotionType() != JPH::EMotionType::Dynamic)
		return;

	JPH::Vec3 t = vecToJolt(torque);
	if (local)
		t = m_env->GetBodyInterface().GetRotation(m_bodyId) * t;

	m_env->GetBodyInterface().AddTorque(m_bodyId, t);
}

void JoltPhysicsController::ApplyImpulse(const MT_Point3& attach, const MT_Vector3& impulse, bool local)
{
	if (!m_env || !m_env->IsWorldAlive() || !IsBodyAlive())
		return;

	JPH::Vec3 imp = vecToJolt(impulse);
	if (imp.LengthSq() <= 1e-12f)
		return;
	if (EffectiveMotionType() != JPH::EMotionType::Dynamic)
		return;

	const JPH::Quat q = m_env->GetBodyInterface().GetRotation(m_bodyId);
	const JPH::RVec3 p = m_env->GetBodyInterface().GetPosition(m_bodyId);
	JPH::Vec3 point(0.0f, 0.0f, 0.0f);
	if (local) {
		/* attach is an offset from the body origin and the impulse is in body
		 * space, exactly like CcdPhysicsController::ApplyImpulse(). */
		imp = q * imp;
		point = p + q * JPH::Vec3((float)attach[0], (float)attach[1], (float)attach[2]);
	}
	else {
		point = JPH::Vec3((float)attach[0], (float)attach[1], (float)attach[2]);
	}

	m_env->GetBodyInterface().AddImpulse(m_bodyId, imp, JPH::RVec3(point));
}

void JoltPhysicsController::SetLinearVelocity(const MT_Vector3& lin_vel, bool local)
{
	if (!m_env || !m_env->IsWorldAlive() || !IsBodyAlive())
		return;
	if (EffectiveMotionType() == JPH::EMotionType::Static)
		return;

	JPH::Vec3 v = vecToJolt(lin_vel);
	if (local)
		v = m_env->GetBodyInterface().GetRotation(m_bodyId) * v;

	m_env->GetBodyInterface().SetLinearVelocity(m_bodyId, v);
	m_env->GetBodyInterface().ActivateBody(m_bodyId);
}

void JoltPhysicsController::SetAngularVelocity(const MT_Vector3& ang_vel, bool local)
{
	if (!m_env || !m_env->IsWorldAlive() || !IsBodyAlive())
		return;
	if (EffectiveMotionType() == JPH::EMotionType::Static)
		return;

	JPH::Vec3 w = vecToJolt(ang_vel);
	if (local)
		w = m_env->GetBodyInterface().GetRotation(m_bodyId) * w;

	m_env->GetBodyInterface().SetAngularVelocity(m_bodyId, w);
	m_env->GetBodyInterface().ActivateBody(m_bodyId);
}

void JoltPhysicsController::ResolveCombinedVelocities(float linvelX, float linvelY, float linvelZ,
                                                     float angVelX, float angVelY, float angVelZ)
{
	/* Bullet uses this to combine the velocities of two bodies that are about to
	 * be joined by a constraint; with no constraints there is nothing to
	 * combine. */
	(void)linvelX; (void)linvelY; (void)linvelZ;
	(void)angVelX; (void)angVelY; (void)angVelZ;
}

MT_Vector3 JoltPhysicsController::GetLinearVelocity()
{
	if (!m_env || !m_env->IsWorldAlive() || !IsBodyAlive())
		return MT_Vector3(0.0f, 0.0f, 0.0f);
	if (EffectiveMotionType() == JPH::EMotionType::Static)
		return MT_Vector3(0.0f, 0.0f, 0.0f);

	return vecToMT(m_env->GetBodyInterface().GetLinearVelocity(m_bodyId));
}

MT_Vector3 JoltPhysicsController::GetAngularVelocity()
{
	if (!m_env || !m_env->IsWorldAlive() || !IsBodyAlive())
		return MT_Vector3(0.0f, 0.0f, 0.0f);
	if (EffectiveMotionType() == JPH::EMotionType::Static)
		return MT_Vector3(0.0f, 0.0f, 0.0f);

	return vecToMT(m_env->GetBodyInterface().GetAngularVelocity(m_bodyId));
}

MT_Vector3 JoltPhysicsController::GetVelocity(const MT_Point3& pos)
{
	if (!m_env || !m_env->IsWorldAlive() || !IsBodyAlive())
		return MT_Vector3(0.0f, 0.0f, 0.0f);
	if (EffectiveMotionType() == JPH::EMotionType::Static)
		return MT_Vector3(0.0f, 0.0f, 0.0f);

	/* Velocity of a point on the body: v + w x r. */
	const MT_Vector3 lin = GetLinearVelocity();
	const MT_Vector3 ang = GetAngularVelocity();

	MT_Vector3 body_pos;
	GetPosition(body_pos);
	const MT_Vector3 r = pos - body_pos;

	return lin + ang.cross(r);
}

MT_Vector3 JoltPhysicsController::GetLocalInertia()
{
	if (!m_env || !m_env->IsWorldAlive() || !IsBodyAlive())
		return MT_Vector3(0.0f, 0.0f, 0.0f);

	JPH::BodyLockRead lock(m_env->GetBodyLockInterface(), m_bodyId);
	if (!lock.Succeeded())
		return MT_Vector3(0.0f, 0.0f, 0.0f);

	const JPH::MotionProperties *mp = lock.GetBody().GetMotionProperties();
	if (!mp)
		return MT_Vector3(0.0f, 0.0f, 0.0f);

	const JPH::Vec3 inv = mp->GetInverseInertiaDiagonal();
	return MT_Vector3((inv.GetX() > 0.0f) ? (MT_Scalar)(1.0f / inv.GetX()) : MT_Scalar(0.0),
	                  (inv.GetY() > 0.0f) ? (MT_Scalar)(1.0f / inv.GetY()) : MT_Scalar(0.0),
	                  (inv.GetZ() > 0.0f) ? (MT_Scalar)(1.0f / inv.GetZ()) : MT_Scalar(0.0));
}

void JoltPhysicsController::SetRadius(float radius)
{
	m_radius = radius;
	/* The shape of a live body cannot be resized in place; ConvertObject bakes the
	 * radius in at creation time. */
}

void JoltPhysicsController::SetMargin(float margin)
{
	m_margin = margin;
	/* Jolt has no per body collision margin: the "convex radius" of a shape is
	 * fixed when the shape is built (and is Jolt's own default for primitives).
	 * ConvertObject passes Blender's margin through, and it is reported back by
	 * GetMargin(), but it does not reach the solver yet. */
}

/* -------------------------------------------------------------------------
 * Velocity clamps
 * ------------------------------------------------------------------------- */

void JoltPhysicsController::ApplyVelocityClamps()
{
	if (!m_env || !m_env->IsWorldAlive() || !IsBodyAlive())
		return;
	if (EffectiveMotionType() == JPH::EMotionType::Static)
		return;

	if (m_clampVelMax > 0.0f || m_clampVelMin > 0.0f) {		const JPH::Vec3 v = m_env->GetBodyInterface().GetLinearVelocity(m_bodyId);
		const float len = vecLength(v);

		if (m_clampVelMax > 0.0f && len > m_clampVelMax) {
			m_env->GetBodyInterface().SetLinearVelocity(m_bodyId, v * (m_clampVelMax / len));
		}
		else if (m_clampVelMin > 0.0f && len > 1e-6f && len < m_clampVelMin) {
			m_env->GetBodyInterface().SetLinearVelocity(m_bodyId, v * (m_clampVelMin / len));
		}
	}

	if (m_clampAngVelMax > 0.0f || m_clampAngVelMin > 0.0f) {
		const JPH::Vec3 w = m_env->GetBodyInterface().GetAngularVelocity(m_bodyId);
		const float len = vecLength(w);

		if (m_clampAngVelMax > 0.0f && len > m_clampAngVelMax) {
			m_env->GetBodyInterface().SetAngularVelocity(m_bodyId, w * (m_clampAngVelMax / len));
		}
		else if (m_clampAngVelMin > 0.0f && len > 1e-6f && len < m_clampAngVelMin) {
			m_env->GetBodyInterface().SetAngularVelocity(m_bodyId, w * (m_clampAngVelMin / len));
		}
	}
}

/* -------------------------------------------------------------------------
 * Replication and environment changes
 * ------------------------------------------------------------------------- */

bool JoltPhysicsController::RebuildBody()
{
	if (!m_env || !m_env->IsWorldAlive())
		return false;

	if (IsBodyAlive()) {
		if (m_inWorld) {
			m_env->GetBodyInterface().RemoveBody(m_bodyId);
			m_inWorld = false;
		}
		m_env->GetBodyInterface().DestroyBody(m_bodyId);
		m_bodyId = JPH::BodyID();
	}
	m_shape = nullptr;

	return Build(m_shapeDesc, m_mass, m_linearFactor, m_angularFactor, m_applyAngularLocks,
	             m_linearDamping, m_angularDamping,
	             m_clampVelMin, m_clampVelMax, m_clampAngVelMin, m_clampAngVelMax,
	             m_radius, m_margin);
}

void JoltPhysicsController::PostProcessReplica(PHY_IMotionState *motionstate, PHY_IPhysicsController *parentctrl)
{
	(void)parentctrl;

	/* The replica produced by GetReplica() has no motion state of its own yet and
	 * no body, so both are created here from the copied description. */
	if (m_motionState && m_motionState != motionstate)
		delete m_motionState;
	m_motionState = motionstate;

	if (!Build(m_shapeDesc, m_mass, m_linearFactor, m_angularFactor, m_applyAngularLocks,
	           m_linearDamping, m_angularDamping,
	           m_clampVelMin, m_clampVelMax, m_clampAngVelMin, m_clampAngVelMax,
	           m_radius, m_margin))
	{
		printf("JoltPhysics: could not build the body of a replicated object\n");
		return;
	}

	if (m_env)
		m_env->AddController(this);

	/* A replica is not created by the environment, so it has to announce itself
	 * (the copy constructor does not run the normal constructor). */
	if (m_env)
		m_env->AddKnownController(this);
}

void JoltPhysicsController::SetPhysicsEnvironment(PHY_IPhysicsEnvironment *env)
{
	/* Called when a scene is merged or lib-loaded.  Only a Jolt environment can
	 * host a Jolt body; anything else is refused instead of being cast blindly. */
	JoltPhysicsEnvironment *joltenv = dynamic_cast<JoltPhysicsEnvironment*>(env);
	if (!joltenv) {
		if (env)
			printf("JoltPhysicsController: refusing to move a Jolt body into a foreign physics environment\n");
		return;
	}
	if (m_env == joltenv)
		return;

	/* A Jolt body belongs to the physics system that created it, so moving the
	 * object to another environment means rebuilding the body there.  The
	 * simulated state is carried across so that a lib-loaded object does not
	 * visibly jump. */
	JPH::RVec3 position = JPH::RVec3::sZero();
	JPH::Quat rotation = JPH::Quat::sIdentity();
	JPH::Vec3 linear_velocity = JPH::Vec3::sZero();
	JPH::Vec3 angular_velocity = JPH::Vec3::sZero();

	const bool have_state = (m_env && m_env->IsWorldAlive() && IsBodyAlive() && m_inWorld);
	/* Reading a velocity dereferences the motion properties, which only exist for
	 * a simulated body. */
	const bool have_velocity = have_state && EffectiveMotionType() != JPH::EMotionType::Static;
	if (have_state) {
		m_env->GetBodyInterface().GetPositionAndRotation(m_bodyId, position, rotation);
		if (have_velocity) {
			linear_velocity = m_env->GetBodyInterface().GetLinearVelocity(m_bodyId);
			angular_velocity = m_env->GetBodyInterface().GetAngularVelocity(m_bodyId);
		}
	}

	if (m_env) {
		m_env->RemoveController(this);
		m_env->RemoveKnownController(this);
		if (m_env->IsWorldAlive() && IsBodyAlive() && m_inWorld) {
			m_env->GetBodyInterface().RemoveBody(m_bodyId);
			m_inWorld = false;
		}
		if (m_env->IsWorldAlive() && IsBodyAlive())
			m_env->GetBodyInterface().DestroyBody(m_bodyId);
	}
	m_bodyId = JPH::BodyID();
	m_shape = nullptr;

	m_env = joltenv;

	if (!Build(m_shapeDesc, m_mass, m_linearFactor, m_angularFactor, m_applyAngularLocks,
	           m_linearDamping, m_angularDamping,
	           m_clampVelMin, m_clampVelMax, m_clampAngVelMin, m_clampAngVelMax,
	           m_radius, m_margin))
	{
		printf("JoltPhysicsController: failed to rebuild the body in the target environment\n");
		return;
	}

	if (have_state && IsBodyAlive()) {
		m_env->GetBodyInterface().SetPositionAndRotation(m_bodyId, position, rotation, JPH::EActivation::DontActivate);
		/* The rebuild can have produced a static body (a dynamic object without
		 * mass), and a static body has no motion properties to write to. */
		if (EffectiveMotionType() != JPH::EMotionType::Static) {
			m_env->GetBodyInterface().SetLinearVelocity(m_bodyId, linear_velocity);
			m_env->GetBodyInterface().SetAngularVelocity(m_bodyId, angular_velocity);
		}
	}

	m_env->AddController(this);
	m_env->AddKnownController(this);
}

PHY_IPhysicsController* JoltPhysicsController::GetReplica()
{
	/* A plain copy; the body is created in PostProcessReplica() once the replica's
	 * motion state exists.  The copy ctor duplicates the motion state pointer,
	 * which the original still owns, so it is cleared here without deleting it. */
	JoltPhysicsController *replica = new JoltPhysicsController(*this);
	replica->m_motionState = NULL;
	replica->m_bodyId = JPH::BodyID();
	replica->m_shape = nullptr;
	replica->m_inWorld = false;
	replica->m_suspended = false;

	/* Runtime handles must never be shared with a copy: the shape belongs to the
	 * original's body and the children are owned (and reattached) by the original.
	 * The descriptions are kept, so PostProcessReplica() can rebuild everything. */
	replica->m_childShapes.clear();
	return replica;
}

PHY_IPhysicsController* JoltPhysicsController::GetReplicaForSensors()
{
	/* Only the Near and Radar sensor proxies are duplicated this way; a scene
	 * object controller is replicated by KX_Scene through GetReplica().  The
	 * replica is a full controller of its own with a standalone motion state, so
	 * it is created from scratch rather than copied: a sensor proxy is not part
	 * of the scene graph. */
	JoltMotionState *motionstate = new JoltMotionState();
	motionstate->SetWorldScaling((float)m_scaling[0], (float)m_scaling[1], (float)m_scaling[2]);

	JoltPhysicsController *replica = new JoltPhysicsController(m_env, motionstate, false);
	replica->SetSensor(true);
	replica->SetKinematic(true);

	if (m_env && m_env->IsWorldAlive()) {
		if (!replica->Build(m_shapeDesc, 0.0f, m_linearFactor, m_angularFactor, m_applyAngularLocks,
		                    0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, m_radius, m_margin))
		{
			printf("JoltPhysics: could not build a body for a sensor proxy\n");
		}
	}

	return replica;
}

/* -------------------------------------------------------------------------
 * Sensors and collision callbacks
 * ------------------------------------------------------------------------- */

void JoltPhysicsController::AddCompoundChild(PHY_IPhysicsController *child)
{
	(void)child;
	printf("JoltPhysics: dynamic compound parenting is not supported yet\n");
}

void JoltPhysicsController::RemoveCompoundChild(PHY_IPhysicsController *child)
{
	(void)child;
}

bool JoltPhysicsController::AddChildShape(const JoltShapeDesc& shape, const JoltChildTransform& local)
{
	/* Jolt bodies carry exactly one Shape, so absorbing a child means building a
	 * compound shape for the parent.  Until that exists the caller falls back to
	 * giving the child its own body (see ConvertObject), which keeps it
	 * collidable.  The description is recorded so that the compound shape can be
	 * built without changing the converter once it is implemented. */
	JoltChildShapeDesc desc;
	desc.shape = shape;
	desc.local = local;
	desc.owner = NULL;
	m_childShapes.push_back(desc);
	return false;
}

void* JoltPhysicsController::GetNewClientInfo()
{
	return m_clientInfo;
}

void JoltPhysicsController::SetNewClientInfo(void *clientinfo)
{
	m_clientInfo = clientinfo;
}

bool JoltPhysicsController::ReinstancePhysicsShape(KX_GameObject *from_gameobj, RAS_MeshObject *from_meshobj)
{
	/* CcdPhysicsController recreates the collision shape from the game object's new
	 * mesh.  Jolt can do the same (the shape is rebuilt from the new bound box),
	 * but the bound box has to be recomputed from the mesh first, which is what
	 * ConvertObject does.  Deferred, so the object keeps its current shape. */
	(void)from_gameobj;
	(void)from_meshobj;
	return false;
}

void JoltPhysicsController::ReplicateConstraints(KX_GameObject *gameobj, std::vector<KX_GameObject*> constobj)
{
	(void)gameobj;
	(void)constobj;
}