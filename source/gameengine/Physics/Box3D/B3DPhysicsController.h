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

/** \file B3DPhysicsController.h
 *  \ingroup physbox3d
 *
 * Box3D implementation of PHY_IPhysicsController.
 * One controller owns exactly one Box3D rigid body and the shapes attached
 * to it.  It is the Box3D counterpart of CcdPhysicsController.
 */

#ifndef __B3DPHYSICSCONTROLLER_H__
#define __B3DPHYSICSCONTROLLER_H__

#include <vector>

#include "PHY_IPhysicsController.h"
#include "PHY_DynamicTypes.h"

#include "MT_Vector3.h"
#include "MT_Point3.h"
#include "MT_Matrix3x3.h"

#include "box3d/box3d.h"

class B3DPhysicsEnvironment;
class PHY_IMotionState;
class PHY_IPhysicsEnvironment;
class KX_GameObject;
class RAS_MeshObject;

/**
 * Description of the collision shape Box3D should build for one Blender object.
 *
 * The Blender to shape mapping lives in B3DPhysicsEnvironment::ConvertObject
 * (mirroring what CcdPhysicsEnvironment does).  The result is handed to the
 * controller instead of a ready made b3ShapeId, because a controller must be
 * able to rebuild its own shape when it is replicated for a group instance.
 *
 * The geometry of the mesh based bounds travels in the vectors below: they hold
 * the *used* vertices only (the ones a collider polygon references), in local
 * object space and before scaling, because the shape is scaled by Box3D.
 */
struct B3DShapeDesc
{
	PHY_ShapeType shapeType;   /* PHY_SHAPE_BOX / SPHERE / CYLINDER / CONE / CAPSULE
	                            * / CONVEX_HULL / MESH                             */
	MT_Vector3 halfExtents;    /* box half extents, local (unscaled) space       */
	float radius;              /* sphere / cylinder / cone / capsule radius      */
	float height;              /* cylinder / cone height, capsule cylinder part  */
	MT_Vector3 scaling;        /* object world scale, baked into the shape       */
	float friction;
	float restitution;
	bool isSensor;
	uint64_t categoryBits;
	uint64_t maskBits;

	/** Convex hull input, xyz triples.  Used by PHY_SHAPE_CONVEX_HULL. */
	std::vector<float> hullPoints;
	/** Triangle mesh input, xyz triples, used by PHY_SHAPE_MESH. */
	std::vector<float> meshVertices;
	/** Triangle indices into meshVertices, three per triangle. */
	std::vector<int32_t> meshIndices;

	B3DShapeDesc();
};

/**
 * Placement of a compound child shape relative to its parent body.
 *
 * Only position and rotation are relative; every shape bakes its own *world*
 * scale (see B3DShapeDesc::scaling) because a Box3D body has no scale of its
 * own.  This is the same split Bullet makes with btCompoundShape::addChildShape
 * plus btCollisionShape::setLocalScaling.
 */
struct B3DChildTransform
{
	MT_Vector3 position;
	MT_Matrix3x3 rotation;

	B3DChildTransform();
};

/** A compound child: its shape and where it sits on the parent body. */
struct B3DChildShapeDesc
{
	B3DShapeDesc shape;
	B3DChildTransform local;
	/** Controller the shape was taken from when the child was parented at run
	 *  time, NULL for the children the converter attached.  Used by
	 *  RemoveCompoundChild() to find the shape again. */
	PHY_IPhysicsController *owner;

	B3DChildShapeDesc();
};

class B3DPhysicsController : public PHY_IPhysicsController
{
public:
	B3DPhysicsController(B3DPhysicsEnvironment *env, PHY_IMotionState *motionstate, bool isDynamic);
	virtual ~B3DPhysicsController();

	/**
	 * Create the body and its shape.
	 *
	 * \param shape          the collision shape to build
	 * \param mass           Blender mass, 0 for static bodies
	 * \param linearFactor   1 = free, 0 = locked, per axis
	 * \param angularFactor  1 = free, 0 = locked, per axis
	 * \param applyAngularLocks  only rigid bodies honour the angular locks
	 * \param linearDamping  already converted to Box3D damping units
	 * \param angularDamping already converted to Box3D damping units
	 * \return true if the body and shape were created
	 */
	bool Build(const B3DShapeDesc& shape,
	           float mass,
	           const MT_Vector3& linearFactor, const MT_Vector3& angularFactor,
	           bool applyAngularLocks,
	           float linearDamping, float angularDamping,
	           float linVelMin, float linVelMax,
	           float angVelMin, float angVelMax,
	           float radius, float margin);

	/** Clamp the linear/angular velocity to the values Blender asked for. */
	void ApplyVelocityClamps();

	b3BodyId GetBodyId() const { return m_bodyId; }
	b3ShapeId GetShapeId() const { return m_shapeId; }
	bool IsSensor() const { return m_isSensor; }
	void SetSensor(bool sensor) { m_isSensor = sensor; }
	bool IsInWorld() const { return m_inWorld; }
	void SetInWorld(bool inworld) { m_inWorld = inworld; }
	void SetKinematic(bool kinematic) { m_isKinematic = kinematic; }
	bool IsKinematic() const { return m_isKinematic; }

	/** The character mover of this controller, NULL for everything that is not a
	 *  "Character" physics object. */
	class B3DCharacter* GetCharacter() const { return m_character; }
	void SetCharacter(class B3DCharacter* character) { m_character = character; }

	/* ---- PHY_IPhysicsController ---- */

	virtual bool SynchronizeMotionStates(float time);
	virtual void WriteMotionStateToDynamics(bool nondynaonly);
	virtual void WriteDynamicsToMotionState();
	virtual PHY_IMotionState* GetMotionState() { return m_motionState; }
	virtual void PostProcessReplica(PHY_IMotionState *motionstate, PHY_IPhysicsController *parentctrl);
	virtual void SetPhysicsEnvironment(PHY_IPhysicsEnvironment *env);

	/* kinematic methods */
	virtual void RelativeTranslate(const MT_Vector3& dloc, bool local);
	virtual void RelativeRotate(const MT_Matrix3x3& rot, bool local);
	virtual MT_Matrix3x3 GetOrientation();
	virtual void SetOrientation(const MT_Matrix3x3& orn);
	virtual void SetPosition(const MT_Vector3& pos);
	virtual void GetPosition(MT_Vector3& pos) const;
	virtual void SetScaling(const MT_Vector3& scale);
	virtual void SetTransform();

	virtual MT_Scalar GetMass();
	virtual void SetMass(MT_Scalar newmass);

	/* physics methods */
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

	/* reading out information from physics */
	virtual MT_Vector3 GetLinearVelocity();
	virtual MT_Vector3 GetAngularVelocity();
	virtual MT_Vector3 GetVelocity(const MT_Point3& pos);
	virtual MT_Vector3 GetLocalInertia();

	virtual void SetRigidBody(bool rigid);

	virtual PHY_IPhysicsController* GetReplica();
	/** Replica used by the Near and Radar sensors for their sphere/cone proxy,
	 *  see CcdPhysicsController::GetReplicaForSensors(). */
	virtual PHY_IPhysicsController* GetReplicaForSensors();

	virtual void CalcXform() { WriteMotionStateToDynamics(false); }
	virtual void SetMargin(float margin) { m_margin = margin; }
	virtual float GetMargin() const { return m_margin; }
	virtual float GetRadius() const { return m_radius; }
	virtual void SetRadius(float radius);

	/* ---- collision callbacks ----
	 * A touch sensor asks the environment for collision callbacks on this
	 * controller; several sensors can share one object, so the registrations are
	 * counted exactly like CcdPhysicsController does it. */
	bool Register()
	{
		return (m_registerCount++ == 0);
	}
	bool Unregister()
	{
		return (--m_registerCount == 0);
	}
	bool Registered() const { return m_registerCount != 0; }

	virtual float GetLinVelocityMin() const { return m_clampVelMin; }
	virtual void SetLinVelocityMin(float val) { m_clampVelMin = val; }
	virtual float GetLinVelocityMax() const { return m_clampVelMax; }
	virtual void SetLinVelocityMax(float val) { m_clampVelMax = val; }
	virtual void SetAngularVelocityMin(float val) { m_clampAngVelMin = val; }
	virtual float GetAngularVelocityMin() const { return m_clampAngVelMin; }
	virtual void SetAngularVelocityMax(float val) { m_clampAngVelMax = val; }
	virtual float GetAngularVelocityMax() const { return m_clampAngVelMax; }

	/* Shape control.  Box3D only allows b3CompoundShape on static bodies, so a
	 * compound is built the way Box3D intends multi part bodies to be built:
	 * every child becomes another shape on the parent body with the child's
	 * local transform baked in. */
	virtual void AddCompoundChild(PHY_IPhysicsController *child);
	virtual void RemoveCompoundChild(PHY_IPhysicsController *child);

	/**
	 * Attach one more shape to this controller's body.
	 *
	 * \param shape  the child shape, in the child's own local space
	 * \param local  child placement relative to the parent body
	 * \return true when the shape was created and attached
	 */
	bool AddChildShape(const B3DShapeDesc& shape, const B3DChildTransform& local);

	virtual bool IsDynamic() { return m_isDynamic; }
	virtual bool IsCompound() { return m_isCompoundRoot || !m_childShapes.empty(); }

	/** Mark this object as a compound root even before any child arrives (the
	 *  converter sets it for objects using Blender's "Compound" option, which is
	 *  what makes later dynamic parenting possible). */
	void SetCompoundRoot(bool compound) { m_isCompoundRoot = compound; }
	virtual bool IsSuspended() const { return m_suspended; }

	virtual bool ReinstancePhysicsShape(KX_GameObject *from_gameobj, RAS_MeshObject *from_meshobj);
	virtual void ReplicateConstraints(KX_GameObject *gameobj, std::vector<KX_GameObject*> constobj);

	/* PHY_IController */
	virtual void* GetNewClientInfo() { return m_clientInfo; }
	virtual void SetNewClientInfo(void *clientinfo) { m_clientInfo = clientinfo; }

	/** Promote a static body to kinematic so that logic can move it.  Mirrors the
	 *  btCollisionObject::CF_KINEMATIC_OBJECT promotion in CcdPhysicsController. */
	void EnsureMovable();

private:
	bool CreateBody();
	b3ShapeId CreateShape(const B3DShapeDesc& shape);
	/** Shared shape builder.  A non NULL local transform attaches the shape at
	 *  that place instead of at the body origin (used by compound children). */
	b3ShapeId CreateShapeOnBody(const B3DShapeDesc& shape, const B3DChildTransform *local);
	void ApplyMass(float mass);
	/** Free the cooked triangle meshes owned by this controller. */
	void ReleaseCookedMeshes();
	/** Attach the recorded compound children to the current body.  Used after the
	 *  body was rebuilt (replica, environment change). */
	void ReattachChildShapes();

	B3DPhysicsEnvironment *m_env;
	PHY_IMotionState *m_motionState;   /* owned */
	b3BodyId m_bodyId;
	b3ShapeId m_shapeId;
	/** Extra shapes on the same body: the compound children. */
	std::vector<b3ShapeId> m_childShapes;
	/** The compound children the shapes above were built from, so that a rebuilt
	 *  body (replica, environment change) can attach them again. */
	std::vector<B3DChildShapeDesc> m_childDescs;
	/** Cooked triangle meshes owned by this controller.  b3CreateMeshShape() keeps
	 *  references to them instead of cloning, so they must outlive the shapes. */
	std::vector<b3MeshData*> m_cookedMeshes;
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
	B3DShapeDesc m_shapeDesc;

	/** Number of touch sensors that asked for collision callbacks on this
	 *  controller, see Register()/Unregister(). */
	int m_registerCount;

	/** Character mover, owned, only set for "Character" physics objects. */
	class B3DCharacter* m_character;
};

#endif  /* __B3DPHYSICSCONTROLLER_H__ */
