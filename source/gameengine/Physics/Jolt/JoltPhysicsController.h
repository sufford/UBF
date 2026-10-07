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

/** \file JoltPhysicsController.h
 *  \ingroup physjolt
 *
 * Jolt implementation of PHY_IPhysicsController.
 *
 * One controller owns exactly one JPH::BodyID, one JPH::Shape (as its
 * BodyCreationSettings) and the motion state the scene graph handed over.  It is
 * the Jolt counterpart of CcdPhysicsController and was modelled on
 * B3DPhysicsController, which solved the same BGE semantics for Box3D.
 */

#ifndef __JOLTPHYSICSCONTROLLER_H__
#define __JOLTPHYSICSCONTROLLER_H__

#include <vector>

#include "PHY_IPhysicsController.h"
#include "PHY_DynamicTypes.h"

#include "MT_Vector3.h"
#include "MT_Point3.h"
#include "MT_Matrix3x3.h"

#include <Jolt/Jolt.h>
#include <Jolt/Core/Reference.h>
#include <Jolt/Physics/Body/AllowedDOFs.h>
#include <Jolt/Physics/Body/BodyID.h>
#include <Jolt/Physics/Body/MotionType.h>
#include <Jolt/Physics/Collision/Shape/Shape.h>

class JoltPhysicsEnvironment;
class PHY_IMotionState;
class PHY_IPhysicsEnvironment;
class KX_GameObject;
class RAS_MeshObject;

namespace JPH {
class BodyInterface;
}

/**
 * Description of the collision shape Jolt should build for one Blender object.
 *
 * The Blender to shape mapping lives in JoltPhysicsEnvironment::ConvertObject
 * (mirroring what CcdPhysicsEnvironment does).  The result is handed to the
 * controller instead of a ready made JPH::Shape, because a controller has to be
 * able to rebuild its own shape when it is replicated for a group instance or
 * moved to another environment by MergeEnvironment().
 *
 * The geometry of the mesh based bounds travels in the vectors below: they hold
 * the *used* vertices only (the ones a collider polygon references), in local
 * object space and before scaling.
 */
struct JoltShapeDesc
{
	PHY_ShapeType shapeType;   /* PHY_SHAPE_BOX / SPHERE / CYLINDER / CONE / CAPSULE
	                            * / POLYTOPE / MESH                                    */
	MT_Vector3 halfExtents;    /* box half extents, local (unscaled) space             */
	float radius;              /* sphere / cylinder / cone / capsule radius            */
	float height;              /* cylinder / cone height, capsule cylinder part        */
	MT_Vector3 scaling;        /* object world scale, baked into the shape             */
	float friction;
	float restitution;
	bool isSensor;

	/** Blender's 16 bit collision group/mask.  Kept so that the layer mapping can
	 *  be filled in without touching ConvertObject again; Jolt's current layer
	 *  setup does not read them yet, see the note in JoltPhysicsEnvironment.h. */
	uint64_t categoryBits;
	uint64_t maskBits;

	/** Convex hull input, xyz triples.  Used by PHY_SHAPE_POLYTOPE. */
	std::vector<float> hullPoints;
	/** Triangle mesh input, xyz triples, used by PHY_SHAPE_MESH. */
	std::vector<float> meshVertices;
	/** Triangle indices into meshVertices, three per triangle. */
	std::vector<int32_t> meshIndices;

	JoltShapeDesc();
};

/**
 * Placement of a compound child shape relative to its parent body.
 *
 * Only position and rotation are relative; every shape bakes its own *world*
 * scale (see JoltShapeDesc::scaling) because a Jolt shape carries no scale of
 * its own beyond what is handed to it at construction time.
 */
struct JoltChildTransform
{
	MT_Vector3 position;
	MT_Matrix3x3 rotation;

	JoltChildTransform();
};

/** A compound child: its shape and where it sits on the parent body. */
struct JoltChildShapeDesc
{
	JoltShapeDesc shape;
	JoltChildTransform local;
	/** Controller of the child object, so the parent can tell it that its shape
	 *  was absorbed (and therefore needs no body of its own). */
	PHY_IPhysicsController *owner;

	JoltChildShapeDesc();
};

class JoltPhysicsController : public PHY_IPhysicsController
{
public:
	JoltPhysicsController(JoltPhysicsEnvironment *env, PHY_IMotionState *motionstate, bool isDynamic);
	virtual ~JoltPhysicsController();

	/** Create the JPH::Shape and the body, then add it to the environment.
	 *  \return false when no body could be created; the controller is then dead
	 *          and the caller must delete it. */
	bool Build(const JoltShapeDesc& shape,
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
	           float margin);

	/** Blender's velocity clamps are applied right before the solve, the same
	 *  place CcdPhysicsController::SimulationTick() does it. */
	void ApplyVelocityClamps();

	JPH::BodyID GetBodyId() const { return m_bodyId; }
	JPH::BodyInterface& GetBodyInterface() const;

	/** Put the body into the simulated set, or take it out again.  An object that
	 *  is not on an active scene layer, and a sensor before something asks for it,
	 *  must not take part in the simulation, which is what CcdPhysicsEnvironment
	 *  expresses by never calling AddCcdPhysicsController() for it. */
	void SetInWorld(bool in_world);

	bool BuildShapeFromDesc(const JoltShapeDesc& shape, JPH::RefConst<JPH::Shape>& out_shape) const;

	void SetSensor(bool sensor) { m_isSensor = sensor; }
	void SetKinematic(bool kinematic) { m_isKinematic = kinematic; }
	void SetCompoundRoot(bool compound) { m_isCompoundRoot = compound; }

	/** Attach a child shape to this body.  Not implemented yet, see the .cpp. */
	bool AddChildShape(const JoltShapeDesc& shape, const JoltChildTransform& local);

	/** (Re)create this body from the description Build() stored.  Used for
	 *  replicas and when the object moves to another environment. */
	bool RebuildBody();

	/** Called by the environment destructor: the physics system this controller
	 *  belongs to is about to be destroyed, so from here on nothing may be
	 *  simulated.  The controller may outlive the environment. */
	void OnEnvironmentDestroyed();

	/** Reference counted "this object is watched by a collision sensor", the same
	 *  contract CcdPhysicsController::Register() implements. */
	bool Register() { m_registerCount++; return true; }
	bool Unregister() { if (m_registerCount > 0) m_registerCount--; return true; }

	/** Blender's "No sleeping" option (OB_COLLISION_RESPONSE). */
	void SetAllowSleeping(bool allow);

	/* ---- PHY_IController ---- */

	virtual void* GetNewClientInfo();
	virtual void SetNewClientInfo(void *clientinfo);

	/* ---- PHY_IPhysicsController ---- */

	virtual bool SynchronizeMotionStates(float time);
	virtual void WriteMotionStateToDynamics(bool nondynaonly);
	virtual void WriteDynamicsToMotionState();
	virtual PHY_IMotionState* GetMotionState() { return m_motionState; }
	virtual void PostProcessReplica(PHY_IMotionState *motionstate, PHY_IPhysicsController *parentctrl);
	virtual void SetPhysicsEnvironment(PHY_IPhysicsEnvironment *env);

	virtual void RelativeTranslate(const MT_Vector3& dloc, bool local);
	virtual void RelativeRotate(const MT_Matrix3x3& drot, bool local);
	virtual MT_Matrix3x3 GetOrientation();
	virtual void SetOrientation(const MT_Matrix3x3& orn);
	virtual void SetPosition(const MT_Vector3& pos);
	virtual void GetPosition(MT_Vector3& pos) const;
	virtual void SetScaling(const MT_Vector3& scale);
	virtual void SetTransform();

	virtual MT_Scalar GetMass();
	virtual void SetMass(MT_Scalar newmass);

	virtual void ApplyImpulse(const MT_Point3& attach, const MT_Vector3& impulse, bool local);
	virtual void ApplyTorque(const MT_Vector3& torque, bool local);
	virtual void ApplyForce(const MT_Vector3& force, bool local);
	virtual void SetAngularVelocity(const MT_Vector3& ang_vel, bool local);
	virtual void SetLinearVelocity(const MT_Vector3& lin_vel, bool local);
	virtual void ResolveCombinedVelocities(float linvelX, float linvelY, float linvelZ,
	                                       float angVelX, float angVelY, float angVelZ);

	virtual float GetLinearDamping() const { return m_linearDamping; }
	virtual float GetAngularDamping() const { return m_angularDamping; }
	virtual void SetLinearDamping(float damping);
	virtual void SetAngularDamping(float damping);
	virtual void SetDamping(float linear, float angular);

	virtual void RefreshCollisions();
	virtual void SuspendDynamics(bool ghost = false);
	virtual void RestoreDynamics();
	virtual void SetActive(bool active);

	virtual MT_Vector3 GetLinearVelocity();
	virtual MT_Vector3 GetAngularVelocity();
	virtual MT_Vector3 GetVelocity(const MT_Point3& pos);
	virtual MT_Vector3 GetLocalInertia();

	virtual void SetRigidBody(bool rigid) { m_isRigidBody = rigid; }

	virtual PHY_IPhysicsController* GetReplica();
	virtual PHY_IPhysicsController* GetReplicaForSensors();

	virtual void CalcXform() {}
	virtual void SetMargin(float margin);
	virtual float GetMargin() const { return m_margin; }
	virtual float GetRadius() const { return m_radius; }
	virtual void SetRadius(float radius);

	virtual float GetLinVelocityMin() const { return m_clampVelMin; }
	virtual void SetLinVelocityMin(float val) { m_clampVelMin = val; }
	virtual float GetLinVelocityMax() const { return m_clampVelMax; }
	virtual void SetLinVelocityMax(float val) { m_clampVelMax = val; }

	virtual void SetAngularVelocityMin(float val) { m_clampAngVelMin = val; }
	virtual float GetAngularVelocityMin() const { return m_clampAngVelMin; }
	virtual void SetAngularVelocityMax(float val) { m_clampAngVelMax = val; }
	virtual float GetAngularVelocityMax() const { return m_clampAngVelMax; }

	virtual void AddCompoundChild(PHY_IPhysicsController *child);
	virtual void RemoveCompoundChild(PHY_IPhysicsController *child);

	virtual bool IsDynamic() { return m_isDynamic; }
	virtual bool IsCompound() { return m_isCompoundRoot; }
	virtual bool IsSuspended() const { return m_suspended; }

	virtual bool ReinstancePhysicsShape(KX_GameObject *from_gameobj, RAS_MeshObject *from_meshobj);
	virtual void ReplicateConstraints(KX_GameObject *gameobj, std::vector<KX_GameObject*> constobj);

	/* ---- helpers used by JoltPhysicsEnvironment ---- */

	bool IsBodyAlive() const;
	/** True while the body is part of the simulated set of this environment. */
	bool IsInWorld() const { return m_inWorld; }

private:
	JoltPhysicsEnvironment *m_env;
	PHY_IMotionState *m_motionState;
	JPH::BodyID m_bodyId;
	/** Keeps the shape alive even if the body is temporarily removed. */
	JPH::RefConst<JPH::Shape> m_shape;

	JPH::EMotionType MotionType() const;

	/** The motion type Jolt actually gave the body.
	 *
	 * MotionType() reports what the controller was built from, which can differ:
	 * a dynamic object with no usable mass is created as a static body.  Every
	 * call that reaches Jolt's motion properties (velocities, forces, sleeping,
	 * activation) has to ask the body itself, because Jolt only allocates those
	 * properties for non-static bodies and dereferences them without a check. */
	JPH::EMotionType EffectiveMotionType() const;

	/** Blender's linear/angular lock axes -> Jolt's allowed DOF bit mask. */
	JPH::EAllowedDOFs AllowedDOFsFromFactors(const MT_Vector3& linearFactor, const MT_Vector3& angularFactor) const;

	/** Recompute the mass properties of the live body from Blender's mass. */
	void ApplyMass(float mass);

	void *m_clientInfo;

	bool m_isDynamic;
	bool m_isKinematic;
	bool m_isRigidBody;
	bool m_isCompoundRoot;
	bool m_isSensor;
	bool m_suspended;
	bool m_inWorld;

	float m_mass;
	float m_linearDamping;
	float m_angularDamping;
	float m_clampVelMin;
	float m_clampVelMax;
	float m_clampAngVelMin;
	float m_clampAngVelMax;
	float m_radius;
	float m_margin;

	MT_Vector3 m_scaling;
	MT_Vector3 m_linearFactor;
	MT_Vector3 m_angularFactor;
	bool m_applyAngularLocks;

	/** Last shape description, kept so that replicas can rebuild the shape. */
	JoltShapeDesc m_shapeDesc;

	/** Children whose shapes the environment asked to attach.  Jolt bodies carry
	 *  exactly one Shape, so this is the input for a compound shape; see
	 *  AddChildShape(). */
	std::vector<JoltChildShapeDesc> m_childShapes;

	/** Number of touch sensors that asked for collision callbacks on this
	 *  controller.  Not wired to Jolt's ContactListener yet. */
	int m_registerCount;
};

#endif  /* __JOLTPHYSICSCONTROLLER_H__ */
