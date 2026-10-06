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

/** \file B3DPhysicsEnvironment.h
 *  \ingroup physbox3d
 *
 * Box3D implementation of PHY_IPhysicsEnvironment.
 *
 * One environment owns exactly one Box3D world (b3WorldId) and every
 * B3DPhysicsController that lives in it.  It is the Box3D counterpart of
 * CcdPhysicsEnvironment.  Engine selection happens in
 * KX_BlenderSceneConverter::ConvertScene(), see WOPHY_BOX3D.
 */

#ifndef __B3DPHYSICSENVIRONMENT_H__
#define __B3DPHYSICSENVIRONMENT_H__

#include <map>
#include <set>
#include <vector>

#include "PHY_IPhysicsEnvironment.h"
#include "PHY_Pro.h"

#include "MT_Vector3.h"

#include "box3d/box3d.h"

class B3DPhysicsController;
class B3DVehicle;
struct Scene;

class B3DPhysicsEnvironment : public PHY_IPhysicsEnvironment
{
public:
	/** Factory used by KX_BlenderSceneConverter::ConvertScene(). */
	static B3DPhysicsEnvironment* Create(Scene *blenderscene, bool visualizePhysics);

	B3DPhysicsEnvironment();
	virtual ~B3DPhysicsEnvironment();

	b3WorldId GetWorldId() const { return m_worldId; }
	/** False once b3DestroyWorld() has run; controllers use this to avoid
	 *  touching Box3D after the world they belonged to is gone. */
	bool IsWorldAlive() const { return m_worldAlive; }

	/** Controller behind a Box3D shape, or NULL when the shape is gone.  Public
	 *  because the Box3D query callbacks are plain functions. */
	B3DPhysicsController* ControllerFromShape(b3ShapeId shapeId) const;

	void AddController(B3DPhysicsController *ctrl);
	void RemoveController(B3DPhysicsController *ctrl);
	/** True while the controller is part of this environment's simulated set. */
	bool HasController(B3DPhysicsController *ctrl) const;

	/* ---- PHY_IPhysicsEnvironment ---- */

	virtual void BeginFrame();
	virtual void EndFrame();
	virtual bool ProceedDeltaTime(double curTime, float timeStep, float interval);

	virtual void SetFixedTimeStep(bool useFixedTimeStep, float fixedTimeStep);
	virtual float GetFixedTimeStep();

	virtual int GetDebugMode() const { return m_debugMode; }
	virtual void SetDebugMode(int debugMode) { m_debugMode = debugMode; }
	/** Render the world through b3World_Draw(): shape bounds, contacts, joints
	 *  and body axes, all as lines for the BGE rasterizer. */
	virtual void DebugDrawWorld();

	virtual void SetNumTimeSubSteps(int numTimeSubSteps);
	virtual int GetNumTimeSubSteps() { return m_subStepCount; }

	/** Box3D has no solver iteration count; the sub-step count is the accuracy
	 *  knob of its solver, so this sets the same value as SetNumTimeSubSteps(). */
	virtual void SetNumIterations(int numIter);

	/** Box3D can only switch sleeping on and off globally, a body that does not
	 *  sleep keeps the world awake, so a time of zero disables sleeping. */
	virtual void SetDeactivationTime(float dTime);
	/** Box3D has a single sleep speed threshold, see ApplySleepThresholds(). */
	virtual void SetDeactivationLinearTreshold(float linTresh);
	virtual void SetDeactivationAngularTreshold(float angTresh);

	/** Box3D's continuous collision is a world switch plus a per-body "bullet"
	 *  flag; the BGE only offers this global mode, so a non zero mode turns on
	 *  continuous collision for the whole world. */
	virtual void SetCcdMode(int ccdMode);

	virtual void SetGravity(float x, float y, float z);
	virtual void GetGravity(MT_Vector3& grav);

	virtual int CreateConstraint(PHY_IPhysicsController *ctrl, PHY_IPhysicsController *ctrl2,
	                             PHY_ConstraintType type,
	                             float pivotX, float pivotY, float pivotZ,
	                             float axis0X, float axis0Y, float axis0Z,
	                             float axis1X, float axis1Y, float axis1Z,
	                             float axis2X, float axis2Y, float axis2Z, int flag);
	virtual void RemoveConstraintById(int constraintid);

	virtual PHY_IVehicle* GetVehicleConstraint(int constraintId);
	virtual PHY_ICharacter* GetCharacterController(KX_GameObject *ob);
	/** Run every character mover, called right before each world step. */
	void UpdateCharacters(float timeStep);
	/** Move the visual wheel objects of every vehicle, after each world step. */
	void SyncVehicles(float timeStep);

	virtual PHY_IPhysicsController* RayTest(PHY_IRayCastFilterCallback& filterCallback,
	                                       float fromX, float fromY, float fromZ,
	                                       float toX, float toY, float toZ);

	virtual bool CullingTest(PHY_CullingCallback callback, void *userData,
	                         MT_Vector4* planeNormals, int planeNumber, int occlusionRes,
	                         const int *viewport, float modelview[16], float projection[16]);

	virtual void AddSensor(PHY_IPhysicsController *ctrl);
	virtual void RemoveSensor(PHY_IPhysicsController *ctrl);
	virtual void AddTouchCallback(int response_class, PHY_ResponseCallback callback, void *user);
	virtual bool RequestCollisionCallback(PHY_IPhysicsController *ctrl);
	virtual bool RemoveCollisionCallback(PHY_IPhysicsController *ctrl);

	virtual PHY_IPhysicsController* CreateSphereController(float radius, const MT_Vector3& position);
	virtual PHY_IPhysicsController* CreateConeController(float coneradius, float coneheight);

	virtual void SetConstraintParam(int constraintId, int param, float value, float value1);
	virtual float GetConstraintParam(int constraintId, int param);
	virtual float GetAppliedImpulse(int constraintId);

	/* Constraints that the scene converter found in the .blend. */
	virtual void SetupObjectConstraints(KX_GameObject *obj_src, KX_GameObject *obj_dest,
	                                    bRigidBodyJointConstraint *dat);

	virtual void MergeEnvironment(PHY_IPhysicsEnvironment *other_env);

	virtual void ConvertObject(KX_GameObject *gameobj,
	                           RAS_MeshObject *meshobj,
	                           DerivedMesh *dm,
	                           KX_Scene *kxscene,
	                           PHY_ShapeProps *shapeprops,
	                           PHY_MaterialProps *smmaterial,
	                           PHY_IMotionState *motionstate,
	                           int activeLayerBitInfo,
	                           bool isCompoundChild,
	                           bool hasCompoundChildren);

	/** Box3D joint a BGE constraint was mapped onto.  Box3D has no generic six
	 *  degree of freedom joint, so a 6DOF constraint becomes the closest Box3D
	 *  joint its limits allow, see B3DPhysicsEnvironment::EnsureJointCreated(). */
	enum B3DJointKind
	{
		B3D_JOINT_NONE = 0,
		B3D_JOINT_REVOLUTE,   /* one free angular axis, pivot and other axes locked */
		B3D_JOINT_SPHERICAL,  /* pivot locked, rotation free */
		B3D_JOINT_PRISMATIC,  /* one free linear axis, rotation locked */
		B3D_JOINT_WELD,       /* every axis locked */
		B3D_JOINT_PARALLEL,   /* angular spring that aligns the two frames' Z axes */
	};

	/** One constraint created through CreateConstraint().  The limits of a
	 *  constraint arrive in a series of SetConstraintParam() calls *after* it has
	 *  been created, so the Box3D joint itself is built lazily once the limits
	 *  are known. */
	struct B3DJoint
	{
		int id;
		PHY_ConstraintType type;      /* the BGE constraint type */
		B3DJointKind kind;            /* Box3D joint it maps onto, may be resolved later */
		B3DPhysicsController *ctrlA;
		B3DPhysicsController *ctrlB;  /* NULL when the constraint is tied to the world */
		b3BodyId anchorBody;          /* static, shapeless body standing in for the world */
		b3Vec3 pivotA;                /* pivot in A's local space */
		MT_Vector3 axisA;             /* main axis (hinge axis, cone axis) in A's local space */
		MT_Vector3 axis1A, axis2A;    /* the two other local frame axes of A */
		int flags;
		bool created;
		int freeLinearDof;            /* 0..2 for a prismatic joint, -1 unknown */
		int freeAngularDof;           /* 0..2 for a revolute joint, -1 unknown */
		bool haveLimits[6];
		float lower[6];
		float upper[6];
		bool haveMotor[6];
		float motorSpeed[6];
		float motorForce[6];
		bool haveSpring[6];
		float springStiffness[6];
		float springDamping[6];
		b3JointId joint;
	};

private:
	/** Drain the contact and sensor events of the last step and feed them to the
	 *  touch and broad phase callbacks. */
	void ProcessEvents();

	B3DJoint* FindJoint(int constraintId) const;
	/** Build the Box3D joint of a constraint, using the limits collected so far.
	 *  Returns false when Box3D cannot express the constraint. */
	bool EnsureJointCreated(B3DJoint *jointDef);
	b3JointId CreateMappedJoint(B3DJoint *jointDef);
	void DestroyJoint(B3DJoint *jointDef);
	/** Build the joints whose limits arrived after their creation. */
	void CreatePendingJoints();
	/** Yaw/limit handling shared by the revolute and prismatic mapping. */
	void ApplyLimits(B3DJoint *jointDef);
	void ApplyMotors(B3DJoint *jointDef);
	void ApplySprings(B3DJoint *jointDef);

	std::map<int, B3DJoint*> m_joints;
	int m_nextJointId;
	bool m_jointMappingReported;

	/** Vehicles (wheel joint rigs) alive in this environment. */
	std::vector<B3DVehicle*> m_vehicles;

	b3WorldId m_worldId;
	bool m_worldAlive;

	std::set<B3DPhysicsController*> m_controllers;

	/** Controller pairs that currently overlap through a sensor shape (the
	 *  Near/Radar proxies and ghost objects).  Box3D only reports the begin and
	 *  the end of such an overlap, so the pairs in between are remembered here
	 *  and reported on every step, which is what the BGE's sensors and the Python
	 *  collision callbacks expect.  Touching contacts are polled separately, see
	 *  ProcessEvents(). */
	std::set<std::pair<B3DPhysicsController*, B3DPhysicsController*> > m_activeTouches;

	/* Box3D sub-steps each b3World_Step() call.  4 is Box3D's recommended value
	 * and is also what GetNumTimeSubSteps() reports. */
	int m_subStepCount;

	int m_debugMode;

	/* Scene settings of the Scene -> Physics panel.  Box3D has one sleep speed
	 * threshold per body and a world wide sleeping switch, so the linear and the
	 * angular threshold are collapsed into one value, see
	 * ApplySleepThresholds(). */
	float m_deactivationLinear;
	float m_deactivationAngular;
	float m_deactivationTime;
	int m_ccdMode;

	/** Put the Box3D sleep threshold on every body. */
	void ApplySleepThresholds();

	bool m_useFixedTimeStep;
	float m_fixedTimeStep;
	float m_accumulator;

	MT_Vector3 m_gravity;

	/* Touch/object callbacks registered by KX_TouchEventManager.  Contacts and
	 * overlaps are drained from Box3D after every step, see ProcessEvents(). */
	std::map<int, PHY_ResponseCallback> m_touchCallbacks;
	std::map<int, void*> m_touchCallbackUser;

	bool m_compoundChildReported;
	bool m_meshBoundsReported;
	bool m_meshDynamicReported;
	bool m_sensorProxyReported;
	bool m_contactReportingReported;
};

#endif  /* __B3DPHYSICSENVIRONMENT_H__ */
