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

/** \file JoltPhysicsEnvironment.h
 *  \ingroup physjolt
 *
 * Jolt implementation of PHY_IPhysicsEnvironment.
 *
 * One environment owns exactly one JPH::PhysicsSystem and every
 * JoltPhysicsController that lives in it.  It is the Jolt counterpart of
 * CcdPhysicsEnvironment and mirrors B3DPhysicsEnvironment, which did the same
 * job for Box3D.  Engine selection happens in
 * KX_BlenderSceneConverter::ConvertScene(), see WOPHY_JOLT.
 *
 * ---------------------------------------------------------------------------
 * Known limitations of this first cut (Phase 1), all deliberate:
 *
 *  * Object layers: Jolt is initialised with the two layer scheme from Jolt's
 *    own HelloWorld sample (NON_MOVING / MOVING), so every moving body collides
 *    with every other moving body regardless of Blender's 16 bit collision
 *    group/mask.  JoltShapeDesc keeps the Blender bits so that the mask based
 *    layer setup can be filled in without touching ConvertObject again.
 *  * Compound children are not attached to the parent body: Jolt bodies carry a
 *    single Shape, so absorbing a child needs a (mutable) compound shape.  Until
 *    that exists ConvertObject falls back to giving the child its own body,
 *    which keeps it collidable.
 *  * Constraints, vehicles, characters and touch/sensor callbacks are not
 *    implemented yet; the corresponding entry points report "unsupported"
 *    instead of silently doing nothing.
 * ---------------------------------------------------------------------------
 */

#ifndef __JOLTPHYSICSENVIRONMENT_H__
#define __JOLTPHYSICSENVIRONMENT_H__

#include <map>
#include <set>
#include <vector>

#include "PHY_IPhysicsEnvironment.h"
#include "PHY_Pro.h"

#include "MT_Vector3.h"

#include <Jolt/Jolt.h>
#include <Jolt/Core/TempAllocator.h>
#include <Jolt/Core/JobSystemThreadPool.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/Physics/Collision/BroadPhase/BroadPhaseLayer.h>
#include <Jolt/Physics/Collision/ObjectLayer.h>

class JoltPhysicsController;
struct Scene;

/** Jolt object layers.  See the limitation note above. */
namespace JoltLayers
{
	static constexpr JPH::ObjectLayer NON_MOVING = 0;
	static constexpr JPH::ObjectLayer MOVING = 1;
	static constexpr JPH::ObjectLayer NUM_LAYERS = 2;
}

namespace JoltBroadPhaseLayers
{
	static constexpr JPH::BroadPhaseLayer NON_MOVING(0);
	static constexpr JPH::BroadPhaseLayer MOVING(1);
	static constexpr JPH::uint NUM_LAYERS = 2;
}

/** Object layer -> broad phase layer mapping. */
class JoltBPLayerInterface final : public JPH::BroadPhaseLayerInterface
{
public:
	JoltBPLayerInterface();

	virtual JPH::uint GetNumBroadPhaseLayers() const override;
	virtual JPH::BroadPhaseLayer GetBroadPhaseLayer(JPH::ObjectLayer inLayer) const override;

#if defined(JPH_EXTERNAL_PROFILE) || defined(JPH_PROFILE_ENABLED)
	virtual const char *GetBroadPhaseLayerName(JPH::BroadPhaseLayer inLayer) const override;
#endif

private:
	JPH::BroadPhaseLayer mObjectToBroadPhase[JoltLayers::NUM_LAYERS];
};

/** Decides whether an object layer may collide with a broad phase layer. */
class JoltObjectVsBroadPhaseLayerFilter final : public JPH::ObjectVsBroadPhaseLayerFilter
{
public:
	virtual bool ShouldCollide(JPH::ObjectLayer inLayer1, JPH::BroadPhaseLayer inLayer2) const override;
};

/** Decides whether two object layers may collide. */
class JoltObjectLayerPairFilter final : public JPH::ObjectLayerPairFilter
{
public:
	virtual bool ShouldCollide(JPH::ObjectLayer inObject1, JPH::ObjectLayer inObject2) const override;
};

class JoltPhysicsEnvironment : public PHY_IPhysicsEnvironment
{
public:
	/** Factory used by KX_BlenderSceneConverter::ConvertScene(). */
	static JoltPhysicsEnvironment* Create(Scene *blenderscene, bool visualizePhysics);

	JoltPhysicsEnvironment();
	virtual ~JoltPhysicsEnvironment();

	/** Jolt's global state (allocator hook, factory, type registration) is per
	 *  process, not per world; the constructor calls this. */
	static void EnsureJoltInitialised();

	JPH::PhysicsSystem& GetPhysicsSystem() { return m_physicsSystem; }
	JPH::BodyInterface& GetBodyInterface();
	const JPH::BodyLockInterfaceLocking& GetBodyLockInterface();

	/** False once the environment is shutting down; controllers use this to avoid
	 *  touching a physics system that is being torn down. */
	bool IsWorldAlive() const { return m_worldAlive; }

	/** Controller behind a Jolt body, or NULL when the body is unknown. */
	JoltPhysicsController* ControllerFromBody(const JPH::BodyID& bodyId) const;

	void AddController(JoltPhysicsController *ctrl);
	void RemoveController(JoltPhysicsController *ctrl);
	/** True while the controller is part of this environment's simulated set. */
	bool HasController(JoltPhysicsController *ctrl) const;

	/** Register a controller that this environment built, whether or not it ended
	 *  up in the simulated set above.  Sensors, objects on inactive layers and
	 *  suspended bodies are not simulated, but they still hold a pointer back to
	 *  this environment (and own a Jolt body), so the environment has to be able
	 *  to invalidate every one of them when it is destroyed. */
	void AddKnownController(JoltPhysicsController *ctrl);
	void RemoveKnownController(JoltPhysicsController *ctrl);

	/* ---- PHY_IPhysicsEnvironment ---- */

	virtual void BeginFrame();
	virtual void EndFrame();
	virtual bool ProceedDeltaTime(double curTime, float timeStep, float interval);

	virtual void SetFixedTimeStep(bool useFixedTimeStep, float fixedTimeStep);
	virtual float GetFixedTimeStep();

	virtual int GetDebugMode() const { return m_debugMode; }
	virtual void SetDebugMode(int debugMode) { m_debugMode = debugMode; }
	virtual void DebugDrawWorld();

	virtual void SetNumTimeSubSteps(int numTimeSubSteps);
	virtual int GetNumTimeSubSteps() { return m_subStepCount; }

	/** Jolt solves with a velocity and a position iteration count; this maps both
	 *  onto the requested iteration count. */
	virtual void SetNumIterations(int numIter);

	virtual void SetDeactivationTime(float dTime);
	virtual void SetDeactivationLinearTreshold(float linTresh);
	virtual void SetDeactivationAngularTreshold(float angTresh);

	virtual void SetContactBreakingTreshold(float contactBreakingTreshold);
	virtual void SetCcdMode(int ccdMode);
	virtual void SetSolverType(int solverType);
	virtual void SetSolverSorConstant(float sor);

	virtual void SetGravity(float x, float y, float z);
	virtual void GetGravity(MT_Vector3& grav);

	virtual int CreateConstraint(PHY_IPhysicsController *ctrl,
	                             PHY_IPhysicsController *ctrl2,
	                             PHY_ConstraintType type,
	                             float pivotX, float pivotY, float pivotZ,
	                             float axis0X, float axis0Y, float axis0Z,
	                             float axis1X, float axis1Y, float axis1Z,
	                             float axis2X, float axis2Y, float axis2Z,
	                             int flag);

	virtual void RemoveConstraintById(int constraintid);
	virtual float GetAppliedImpulse(int constraintid);

	virtual PHY_IVehicle* GetVehicleConstraint(int constraintId);
	virtual PHY_ICharacter* GetCharacterController(KX_GameObject *ob);

	virtual PHY_IPhysicsController* RayTest(PHY_IRayCastFilterCallback& filterCallback,
	                                        float fromX, float fromY, float fromZ,
	                                        float toX, float toY, float toZ);

	virtual bool CullingTest(PHY_CullingCallback callback, void *userData,
	                         MT_Vector4* planeNormals, int planeNumber, int occlusionRes,
	                         const int *viewport, float modelview[16], float projection[16]);

	virtual void SetConstraintParam(int constraintId, int param, float value, float value1);
	virtual float GetConstraintParam(int constraintId, int param);

	virtual void AddSensor(PHY_IPhysicsController *ctrl);
	virtual void RemoveSensor(PHY_IPhysicsController *ctrl);
	virtual void AddTouchCallback(int response_class, PHY_ResponseCallback callback, void *user);
	virtual bool RequestCollisionCallback(PHY_IPhysicsController *ctrl);
	virtual bool RemoveCollisionCallback(PHY_IPhysicsController *ctrl);

	virtual PHY_IPhysicsController* CreateSphereController(float radius, const MT_Vector3& position);
	virtual PHY_IPhysicsController* CreateConeController(float coneradius, float coneheight);

	virtual void MergeEnvironment(PHY_IPhysicsEnvironment *other_env);

	virtual void SetupObjectConstraints(KX_GameObject *obj_src, KX_GameObject *obj_dest,
	                                    bRigidBodyJointConstraint *dat);

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

private:
	JPH::PhysicsSystem m_physicsSystem;
	JoltBPLayerInterface m_broadPhaseLayerInterface;
	JoltObjectVsBroadPhaseLayerFilter m_objectVsBroadPhaseFilter;
	JoltObjectLayerPairFilter m_objectLayerPairFilter;

	/** Pre-allocated so that a physics step does not allocate; see Jolt's
	 *  TempAllocator documentation. */
	JPH::TempAllocatorImpl *m_tempAllocator;
	/** Runs Jolt's internal jobs.  Owned; one pool per environment keeps a
	 *  scene's physics independent of every other scene. */
	JPH::JobSystemThreadPool *m_jobSystem;

	/** Controllers whose motion states are written back by the solver. */
	std::set<JoltPhysicsController*> m_controllers;
	/** Every controller this environment created, including the ones that are not
	 *  simulated; see AddKnownController(). */
	std::set<JoltPhysicsController*> m_knownControllers;

	/** Body count at the last OptimizeBroadPhase() call, see ProceedDeltaTime(). */
	JPH::uint m_optimisedBodyCount;

	bool m_worldAlive;
	int m_subStepCount;
	int m_debugMode;

	bool m_useFixedTimeStep;
	float m_fixedTimeStep;
	float m_accumulator;
	/** Upper bound on fixed steps per frame, so that a slow frame drops the
	 *  backlog instead of spiralling into ever longer frames. */
	int m_maxStepsPerFrame;

	MT_Vector3 m_gravity;

	/* Blender's scene settings, see JoltPhysicsEnvironment::Create(). */
	float m_deactivationLinear;
	float m_deactivationAngular;
	float m_deactivationTime;
	int m_ccdMode;
	float m_contactBreakingTreshold;
	int m_solverType;
	float m_sor;

	/* Touch/object callbacks registered by KX_TouchEventManager.  The maps are
	 * kept so that the API is complete, but nothing drains them yet: contact
	 * reporting needs a JPH::ContactListener, see the limitation note above. */
	std::map<int, PHY_ResponseCallback> m_touchCallbacks;
	std::map<int, void*> m_touchCallbackUser;

	/* Report each unsupported feature once instead of once per frame. */
	bool m_constraintReported;
	bool m_vehicleReported;
	bool m_characterReported;
	bool m_contactReportingReported;
	bool m_compoundChildReported;
	bool m_meshBoundsReported;
	bool m_meshDynamicReported;
	bool m_softBodyReported;

	void ApplyDeactivationSettings();
	void ReportUnsupported(const char* feature, bool& flag);
	void SyncMotionStates(float timeStep);
};

#endif  /* __JOLTPHYSICSENVIRONMENT_H__ */
