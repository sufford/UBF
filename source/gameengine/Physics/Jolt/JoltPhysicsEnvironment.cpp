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

/** \file JoltPhysicsEnvironment.cpp
 *  \ingroup physjolt
 */

#include "JoltPhysicsEnvironment.h"
#include "JoltPhysicsEnvironmentFactory.h"
#include "JoltPhysicsController.h"
#include "JoltMotionState.h"


#include "PHY_IMotionState.h"
#include "PHY_Pro.h"

#include "KX_GameObject.h"
#include "KX_ClientObjectInfo.h"
#include "KX_BlenderSceneConverter.h"
#include "KX_KetsjiEngine.h"
#include "KX_PythonInit.h"
#include "RAS_MeshObject.h"
#include "RAS_Polygon.h"

#include "MT_MinMax.h"

#include "DNA_scene_types.h"
#include "DNA_object_types.h"
#include "DNA_meshdata_types.h"

extern "C" {
#include "BLI_utildefines.h"
#include "BKE_object.h"
#include "BKE_DerivedMesh.h"
#include "BKE_cdderivedmesh.h"
}

#include <Jolt/Core/Factory.h>
#include <Jolt/Core/IssueReporting.h>
#include <Jolt/Core/Memory.h>
#include <Jolt/RegisterTypes.h>
#include <Jolt/Physics/Body/BodyFilter.h>
#include <Jolt/Physics/Collision/CastResult.h>
#include <Jolt/Physics/Collision/NarrowPhaseQuery.h>
#include <Jolt/Physics/Collision/RayCast.h>

#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdarg.h>
#include <set>
#include <thread>

/* Half extents below this are degenerate (a plane has a zero thickness bound
 * box), so a thin slab is used instead.  Same value as the Box3D backend. */
#define JOLT_MIN_HALF_EXTENT 0.01f

/* Upper bound on the number of fixed steps a single frame may run, so that a
 * slow frame drops the backlog instead of spiralling into ever longer frames. */
#define JOLT_MAX_STEPS_PER_FRAME 8

/* Capacity of the physics system.  BGE streams objects in and out of a scene, so
 * these have to cover a whole level rather than one test scene. */
#define JOLT_MAX_BODIES 65536
#define JOLT_MAX_BODY_PAIRS 65536
#define JOLT_MAX_CONTACT_CONSTRAINTS 10240

/* Temporary allocation during a step.  Jolt asks for this up front so that the
 * step itself does not allocate. */
#define JOLT_TEMP_ALLOCATOR_SIZE (16 * 1024 * 1024)

/* Jolt runs its own job system; a handful of workers is enough for BGE and
 * avoids oversubscribing a machine that is also rendering. */
#define JOLT_MAX_JOB_THREADS 4

/* -------------------------------------------------------------------------
 * Jolt process global state
 *
 * Jolt's allocator hook, type factory and collision dispatch registration are
 * per process, not per physics world, and have to happen before the first Jolt
 * call.  They are never torn down: Blender has no single shutdown point that is
 * guaranteed to come after every physics environment (and both Blender and the
 * standalone player run a process lifetime that is short enough for this to be
 * irrelevant).
 * ------------------------------------------------------------------------- */

static bool s_joltInitialised = false;

static void JoltTraceImpl(const char* inFMT, ...)
{
	va_list list;
	va_start(list, inFMT);
	char buffer[1024];
	vsnprintf(buffer, sizeof(buffer), inFMT, list);
	va_end(list);
	printf("Jolt: %s\n", buffer);
}

#ifdef JPH_ENABLE_ASSERTS
static bool JoltAssertFailedImpl(const char* inExpression, const char* inMessage, const char* inFile, JPH::uint inLine)
{
	/* Report and continue: a Blender build must never stop at a Jolt assert. */
	printf("Jolt assert: %s:%u: (%s) %s\n",
	       inFile, (unsigned)inLine, inExpression, (inMessage != NULL) ? inMessage : "");
	return true;
}
#endif /* JPH_ENABLE_ASSERTS */

void JoltPhysicsEnvironment::EnsureJoltInitialised()
{
	if (s_joltInitialised)
		return;
	s_joltInitialised = true;

	JPH::RegisterDefaultAllocator();

	JPH::Trace = JoltTraceImpl;
	JPH_IF_ENABLE_ASSERTS(JPH::AssertFailed = JoltAssertFailedImpl;)

	JPH::Factory::sInstance = new JPH::Factory();
	JPH::RegisterTypes();
}

/* -------------------------------------------------------------------------
 * Object layers
 *
 * See the limitation note in the header: these are the two layers of Jolt's own
 * HelloWorld sample, not Blender's 16 bit collision group/mask.
 * ------------------------------------------------------------------------- */

JoltBPLayerInterface::JoltBPLayerInterface()
{
	mObjectToBroadPhase[JoltLayers::NON_MOVING] = JoltBroadPhaseLayers::NON_MOVING;
	mObjectToBroadPhase[JoltLayers::MOVING] = JoltBroadPhaseLayers::MOVING;
}

JPH::uint JoltBPLayerInterface::GetNumBroadPhaseLayers() const
{
	return JoltBroadPhaseLayers::NUM_LAYERS;
}

JPH::BroadPhaseLayer JoltBPLayerInterface::GetBroadPhaseLayer(JPH::ObjectLayer inLayer) const
{
	JPH_ASSERT(inLayer < JoltLayers::NUM_LAYERS);
	return mObjectToBroadPhase[inLayer];
}

#if defined(JPH_EXTERNAL_PROFILE) || defined(JPH_PROFILE_ENABLED)
const char* JoltBPLayerInterface::GetBroadPhaseLayerName(JPH::BroadPhaseLayer inLayer) const
{
	switch ((JPH::BroadPhaseLayer::Type)inLayer) {
		case (JPH::BroadPhaseLayer::Type)JoltBroadPhaseLayers::NON_MOVING:
			return "NON_MOVING";
		case (JPH::BroadPhaseLayer::Type)JoltBroadPhaseLayers::MOVING:
			return "MOVING";
		default:
			JPH_ASSERT(false);
			return "INVALID";
	}
}
#endif

bool JoltObjectVsBroadPhaseLayerFilter::ShouldCollide(JPH::ObjectLayer inLayer1, JPH::BroadPhaseLayer inLayer2) const
{
	switch (inLayer1) {
		case JoltLayers::NON_MOVING:
			/* Static geometry only has to be tested against the moving tree. */
			return inLayer2 == JoltBroadPhaseLayers::MOVING;
		case JoltLayers::MOVING:
			return true;
		default:
			JPH_ASSERT(false);
			return false;
	}
}

bool JoltObjectLayerPairFilter::ShouldCollide(JPH::ObjectLayer inObject1, JPH::ObjectLayer inObject2) const
{
	switch (inObject1) {
		case JoltLayers::NON_MOVING:
			return inObject2 == JoltLayers::MOVING;
		case JoltLayers::MOVING:
			return true;
		default:
			JPH_ASSERT(false);
			return false;
	}
}

/* -------------------------------------------------------------------------
 * Ray cast filtering
 * ------------------------------------------------------------------------- */

/** Ignores a growing set of bodies, so that a ray can be re-cast past a hit the
 *  application filter rejected.  Jolt's own IgnoreMultipleBodiesFilter does the
 *  same, but writes to a fixed size array whose size has to be guessed. */
class JoltIgnoreBodiesFilter final : public JPH::BodyFilter
{
public:
	void Add(const JPH::BodyID& inBodyID)
	{
		mIgnored.insert(inBodyID.GetIndex());
	}

	virtual bool ShouldCollide(const JPH::BodyID& inBodyID) const override
	{
		return mIgnored.find(inBodyID.GetIndex()) == mIgnored.end();
	}

	virtual bool ShouldCollideLocked(const JPH::Body& inBody) const override
	{
		/* CcdPhysicsEnvironment masks the sensor filter out of its ray test
		 * (m_collisionFilterMask = AllFilter ^ SensorFilter), so a ray must not
		 * report a sensor either. */
		if (inBody.IsSensor())
			return false;
		return mIgnored.find(inBody.GetID().GetIndex()) == mIgnored.end();
	}

private:
	std::set<JPH::uint32> mIgnored;
};

/* -------------------------------------------------------------------------
 * Environment
 * ------------------------------------------------------------------------- */

JoltPhysicsEnvironment::JoltPhysicsEnvironment()
	: m_tempAllocator(NULL),
	  m_jobSystem(NULL),
	  m_optimisedBodyCount(0),
	  m_worldAlive(false),
	  m_subStepCount(1),
	  m_debugMode(0),
	  m_useFixedTimeStep(false),
	  m_fixedTimeStep(0.0f),
	  m_accumulator(0.0f),
	  m_maxStepsPerFrame(JOLT_MAX_STEPS_PER_FRAME),
	  m_deactivationLinear(0.8f),
	  m_deactivationAngular(1.0f),
	  m_deactivationTime(2.0f),
	  m_ccdMode(0),
	  m_contactBreakingTreshold(0.02f),
	  m_solverType(1),
	  m_sor(1.0f),
	  m_constraintReported(false),
	  m_vehicleReported(false),
	  m_characterReported(false),
	  m_contactReportingReported(false),
	  m_compoundChildReported(false),
	  m_meshBoundsReported(false),
	  m_meshDynamicReported(false),
	  m_softBodyReported(false)
{
	EnsureJoltInitialised();

	m_gravity.setValue(0.0f, 0.0f, -9.81f);

	m_tempAllocator = new JPH::TempAllocatorImpl(JOLT_TEMP_ALLOCATOR_SIZE);

	int num_threads = (int)std::thread::hardware_concurrency() - 1;
	if (num_threads < 1)
		num_threads = 1;
	if (num_threads > JOLT_MAX_JOB_THREADS)
		num_threads = JOLT_MAX_JOB_THREADS;

	/* 2048 jobs and 8 barriers are the values Jolt's own samples use. */
	m_jobSystem = new JPH::JobSystemThreadPool(2048, 8, num_threads);

	m_physicsSystem.Init(JOLT_MAX_BODIES, 0, JOLT_MAX_BODY_PAIRS, JOLT_MAX_CONTACT_CONSTRAINTS,
	                     m_broadPhaseLayerInterface, m_objectVsBroadPhaseFilter, m_objectLayerPairFilter);
	m_physicsSystem.SetGravity(JPH::Vec3((float)m_gravity[0], (float)m_gravity[1], (float)m_gravity[2]));

	m_worldAlive = true;
}

JoltPhysicsEnvironment* JoltPhysicsEnvironment::Create(Scene *blenderscene, bool visualizePhysics)
{
	JoltPhysicsEnvironment *env = new JoltPhysicsEnvironment();

	/* Jolt and Blender both work in metres, so no unit conversion is needed. */

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

PHY_IPhysicsEnvironment* PHY_JoltCreateEnvironment(Scene *blenderscene, bool visualizePhysics)
{
	JoltPhysicsEnvironment *env = JoltPhysicsEnvironment::Create(blenderscene, visualizePhysics);
	return env;
}

JoltPhysicsEnvironment::~JoltPhysicsEnvironment()
{
	/* From here on every controller has to treat the physics system as gone.
	 * The controllers may well outlive the environment (KX_Scene destroys objects
	 * and the physics environment in its own order), so their pointer back to
	 * this object must not be left dangling. */
	m_worldAlive = false;

	/* Every controller has to be invalidated, not just the simulated ones: a
	 * sensor, an object that was built off the active layer or a suspended body
	 * keeps its pointer to this environment and its body id, and the whole
	 * physics system (including those bodies) dies with this object. */
	std::set<JoltPhysicsController*>::iterator it;
	for (it = m_knownControllers.begin(); it != m_knownControllers.end(); ++it)
		(*it)->OnEnvironmentDestroyed();
	m_knownControllers.clear();
	m_controllers.clear();

	m_physicsSystem.SetContactListener(NULL);

	delete m_jobSystem;
	m_jobSystem = NULL;
	delete m_tempAllocator;
	m_tempAllocator = NULL;
}

JPH::BodyInterface& JoltPhysicsEnvironment::GetBodyInterface()
{
	return m_physicsSystem.GetBodyInterface();
}

const JPH::BodyLockInterfaceLocking& JoltPhysicsEnvironment::GetBodyLockInterface()
{
	return m_physicsSystem.GetBodyLockInterface();
}

JoltPhysicsController* JoltPhysicsEnvironment::ControllerFromBody(const JPH::BodyID& bodyId) const
{
	if (bodyId.IsInvalid() || !m_worldAlive)
		return NULL;

	/* Every body this environment creates stores its controller as user data. */
	const JPH::uint64 user_data = m_physicsSystem.GetBodyInterfaceNoLock().GetUserData(bodyId);
	return reinterpret_cast<JoltPhysicsController*>((uintptr_t)user_data);
}

void JoltPhysicsEnvironment::AddController(JoltPhysicsController *ctrl)
{
	if (ctrl)
		m_controllers.insert(ctrl);
}

void JoltPhysicsEnvironment::RemoveController(JoltPhysicsController *ctrl)
{
	m_controllers.erase(ctrl);
}

bool JoltPhysicsEnvironment::HasController(JoltPhysicsController *ctrl) const
{
	return m_controllers.find(ctrl) != m_controllers.end();
}

void JoltPhysicsEnvironment::AddKnownController(JoltPhysicsController *ctrl)
{
	if (ctrl)
		m_knownControllers.insert(ctrl);
}

void JoltPhysicsEnvironment::RemoveKnownController(JoltPhysicsController *ctrl)
{
	m_knownControllers.erase(ctrl);
}

void JoltPhysicsEnvironment::ReportUnsupported(const char *feature, bool& flag)
{
	if (flag)
		return;
	flag = true;
	printf("JoltPhysics: %s is not implemented by the Jolt backend yet\n", feature);
}

void JoltPhysicsEnvironment::ApplyDeactivationSettings()
{
	/* Blender's thresholds are velocities in m/s and Jolt uses a single sleep
	 * threshold velocity; the smaller of the two Blender values is the one that
	 * decides when an object stops being simulated early, so it is used. */
	if (!m_worldAlive)
		return;

	float threshold = m_deactivationLinear;
	if (!(threshold > 0.0f))
		threshold = m_deactivationAngular;
	if (!(threshold > 0.0f))
		threshold = 0.03f;

	JPH::PhysicsSettings settings = m_physicsSystem.GetPhysicsSettings();
	settings.mPointVelocitySleepThreshold = threshold;
	if (m_deactivationTime > 0.0f)
		settings.mTimeBeforeSleep = m_deactivationTime;
	m_physicsSystem.SetPhysicsSettings(settings);
}

/* -------------------------------------------------------------------------
 * Frame and step
 * ------------------------------------------------------------------------- */

void JoltPhysicsEnvironment::BeginFrame()
{
	/* Jolt accumulates and clears forces inside Update(), there is no separate
	 * frame boundary to maintain. */
}

void JoltPhysicsEnvironment::EndFrame()
{
}

void JoltPhysicsEnvironment::SyncMotionStates(float timeStep)
{
	std::set<JoltPhysicsController*>::iterator it;
	for (it = m_controllers.begin(); it != m_controllers.end(); ++it)
		(*it)->SynchronizeMotionStates(timeStep);
}

bool JoltPhysicsEnvironment::ProceedDeltaTime(double curTime, float timeStep, float interval)
{
	(void)curTime;
	(void)interval;

	if (!m_worldAlive)
		return false;
	if (timeStep <= 0.0f)
		return true;


	std::set<JoltPhysicsController*>::iterator it;

	/* Blender's velocity clamps are applied right before the solve, the same
	 * place CcdPhysicsController::SimulationTick() does it. */
	for (it = m_controllers.begin(); it != m_controllers.end(); ++it)
		(*it)->ApplyVelocityClamps();

	/* Jolt keeps static bodies in a separate broad phase tree that is only
	 * rebuilt by OptimizeBroadPhase().  BGE adds and removes whole object trees
	 * (AddObject, lib load) far more often than the tree is queried, so one
	 * rebuild per frame in which the body count changed is a good trade. */
	const JPH::uint num_bodies = m_physicsSystem.GetNumBodies();
	if (num_bodies != m_optimisedBodyCount) {
		m_physicsSystem.OptimizeBroadPhase();
		m_optimisedBodyCount = num_bodies;
	}

	const int sub_steps = (m_subStepCount > 0) ? m_subStepCount : 1;

	if (m_useFixedTimeStep && m_fixedTimeStep > 0.0f) {
		/* Jolt does not accumulate a fixed step itself, so that a variable frame
		 * time still produces deterministic steps. */
		m_accumulator += timeStep;

		int steps = 0;
		while (m_accumulator >= m_fixedTimeStep && steps < m_maxStepsPerFrame) {
			m_physicsSystem.Update(m_fixedTimeStep, sub_steps, m_tempAllocator, m_jobSystem);
			m_accumulator -= m_fixedTimeStep;
			steps++;
		}

		if (steps == m_maxStepsPerFrame) {
			/* The simulation cannot keep up, drop the backlog. */
			m_accumulator = 0.0f;
		}
	}
	else {
		m_physicsSystem.Update(timeStep, sub_steps, m_tempAllocator, m_jobSystem);
	}

	/* Write the simulated transforms back into the motion states.  Everything
	 * else (scene graph update, sensor transforms) happens in
	 * KX_Scene::UpdateParents(), which runs right after this call. */
	SyncMotionStates(timeStep);

	return true;
}

void JoltPhysicsEnvironment::SetFixedTimeStep(bool useFixedTimeStep, float fixedTimeStep)
{
	m_useFixedTimeStep = useFixedTimeStep;
	m_fixedTimeStep = fixedTimeStep;
	m_accumulator = 0.0f;
}

float JoltPhysicsEnvironment::GetFixedTimeStep()
{
	return m_useFixedTimeStep ? m_fixedTimeStep : 0.0f;
}

void JoltPhysicsEnvironment::SetNumTimeSubSteps(int numTimeSubSteps)
{
	m_subStepCount = (numTimeSubSteps > 0) ? numTimeSubSteps : 1;
}

void JoltPhysicsEnvironment::SetNumIterations(int numIter)
{
	if (!m_worldAlive || numIter <= 0)
		return;

	/* Blender has one iteration count; Jolt splits it into velocity and position
	 * iterations.  The position iterations are what removes penetration, so they
	 * get a third of the budget. */
	JPH::PhysicsSettings settings = m_physicsSystem.GetPhysicsSettings();
	settings.mNumVelocitySteps = (JPH::uint)numIter;
	settings.mNumPositionSteps = (JPH::uint)MT_max(numIter / 3, 1);
	m_physicsSystem.SetPhysicsSettings(settings);
}

void JoltPhysicsEnvironment::SetDeactivationTime(float dTime)
{
	m_deactivationTime = dTime;
}

void JoltPhysicsEnvironment::SetDeactivationLinearTreshold(float linTresh)
{
	m_deactivationLinear = linTresh;
	ApplyDeactivationSettings();
}

void JoltPhysicsEnvironment::SetDeactivationAngularTreshold(float angTresh)
{
	m_deactivationAngular = angTresh;
	ApplyDeactivationSettings();
}

void JoltPhysicsEnvironment::SetContactBreakingTreshold(float contactBreakingTreshold)
{
	m_contactBreakingTreshold = contactBreakingTreshold;
}

void JoltPhysicsEnvironment::SetCcdMode(int ccdMode)
{
	m_ccdMode = ccdMode;
}

void JoltPhysicsEnvironment::SetSolverType(int solverType)
{
	m_solverType = solverType;
}

void JoltPhysicsEnvironment::SetSolverSorConstant(float sor)
{
	m_sor = sor;
}

void JoltPhysicsEnvironment::SetGravity(float x, float y, float z)
{
	m_gravity.setValue(x, y, z);
	if (m_worldAlive)
		m_physicsSystem.SetGravity(JPH::Vec3(x, y, z));
}

void JoltPhysicsEnvironment::GetGravity(MT_Vector3& grav)
{
	grav = m_gravity;
}

void JoltPhysicsEnvironment::DebugDrawWorld()
{
	/* Jolt can debug render through JPH::DebugRenderer, but that needs a renderer
	 * implementation wired into Blender's viewport; not part of this cut. */
}

/* -------------------------------------------------------------------------
 * Ray test
 * ------------------------------------------------------------------------- */

PHY_IPhysicsController* JoltPhysicsEnvironment::RayTest(PHY_IRayCastFilterCallback& filterCallback,
                                                       float fromX, float fromY, float fromZ,
                                                       float toX, float toY, float toZ)
{
	if (!m_worldAlive)
		return NULL;

	const JPH::RVec3 origin((float)fromX, (float)fromY, (float)fromZ);
	const JPH::Vec3 direction((float)(toX - fromX), (float)(toY - fromY), (float)(toZ - fromZ));

	if (direction.LengthSq() <= 1e-12f)
		return NULL;

	const JPH::NarrowPhaseQuery& query = m_physicsSystem.GetNarrowPhaseQuery();

	JoltIgnoreBodiesFilter body_filter;

	/* The controller the caller wants to see through, e.g. the object casting the
	 * ray itself. */
	if (filterCallback.m_ignoreController) {
		JoltPhysicsController *ignore = dynamic_cast<JoltPhysicsController*>(filterCallback.m_ignoreController);
		if (ignore && ignore->IsBodyAlive())
			body_filter.Add(ignore->GetBodyId());
	}

	/* Cast, and re-cast past a hit the application filter rejects.  Bullet does
	 * the same by walking its ray result callback list; the iteration cap only
	 * guards against a filter that rejects everything. */
	PHY_RayCastResult result;
	memset(&result, 0, sizeof(result));
	result.m_controller = NULL;
	result.m_meshObject = NULL;
	result.m_polygon = 0;
	result.m_hitUVOK = 0;

	for (int iteration = 0; iteration < 64; iteration++) {
		JPH::RRayCast ray(origin, direction);
		JPH::RayCastResult hit;

		if (!query.CastRay(ray, hit, JPH::BroadPhaseLayerFilter(), JPH::ObjectLayerFilter(), body_filter))
			return NULL;

		JoltPhysicsController *ctrl = ControllerFromBody(hit.mBodyID);
		if (ctrl && filterCallback.needBroadphaseRayCast(ctrl)) {
			const JPH::Vec3 hit_point = ray.GetPointOnRay(hit.mFraction);

			result.m_controller = ctrl;
			result.m_hitPoint.setValue(hit_point.GetX(), hit_point.GetY(), hit_point.GetZ());

			/* The surface normal needs the sub shape and the body itself. */
			JPH::Vec3 normal(0.0f, 0.0f, 0.0f);
			{
				JPH::BodyLockRead lock(GetBodyLockInterface(), hit.mBodyID);
				if (lock.Succeeded())
					normal = lock.GetBody().GetWorldSpaceSurfaceNormal(hit.mSubShapeID2, JPH::RVec3(hit_point));
			}

			/* A degenerate normal would make the callers rotate an object into an
			 * arbitrary direction; Bullet falls back to +X in the same situation. */
			if (normal.LengthSq() <= 1e-12f)
				normal = JPH::Vec3(1.0f, 0.0f, 0.0f);

			result.m_hitNormal.setValue(normal.GetX(), normal.GetY(), normal.GetZ());

			/* Jolt can also report the sub shape that was hit, but BGE expects a
			 * RAS_MeshObject and a polygon index plus UV coordinates, which only a
			 * mesh shape can provide.  Reporting "no mesh" keeps the callers on
			 * their non mesh path instead of reading garbage. */
			filterCallback.reportHit(&result);
			return ctrl;
		}

		if (ctrl && ctrl->IsBodyAlive())
			body_filter.Add(ctrl->GetBodyId());
		else
			body_filter.Add(hit.mBodyID);
	}

	return NULL;
}

bool JoltPhysicsEnvironment::CullingTest(PHY_CullingCallback callback, void *userData,
                                         MT_Vector4* planeNormals, int planeNumber, int occlusionRes,
                                         const int *viewport, float modelview[16], float projection[16])
{
	/* No DBVT based occlusion culling for Jolt; KX_Scene falls back to its brute
	 * force visibility pass when this returns false. */
	(void)callback; (void)userData; (void)planeNormals; (void)planeNumber;
	(void)occlusionRes; (void)viewport; (void)modelview; (void)projection;
	return false;
}

/* -------------------------------------------------------------------------
 * Sensors, proxies
 * ------------------------------------------------------------------------- */

void JoltPhysicsEnvironment::AddSensor(PHY_IPhysicsController *ctrl)
{
	JoltPhysicsController *joltctrl = dynamic_cast<JoltPhysicsController*>(ctrl);
	if (!joltctrl)
		return;

	/* A Jolt body is part of its physics system from the moment it is created, so
	 * there is nothing to insert here; this only has to make sure the controller
	 * is stepped. */
	joltctrl->SetSensor(true);
	joltctrl->SetInWorld(true);
	AddController(joltctrl);
}

void JoltPhysicsEnvironment::RemoveSensor(PHY_IPhysicsController *ctrl)
{
	/* A Jolt body cannot be taken out of its physics system without being
	 * destroyed, and the owning object still needs its motion state synchronised,
	 * so the controller stays registered.  A sensor shape never generates a
	 * collision response, so leaving it in the world is harmless. */
	(void)ctrl;
}

void JoltPhysicsEnvironment::AddTouchCallback(int response_class, PHY_ResponseCallback callback, void *user)
{
	m_touchCallbacks[response_class] = callback;
	m_touchCallbackUser[response_class] = user;

	ReportUnsupported("collision and touch event reporting", m_contactReportingReported);
}

bool JoltPhysicsEnvironment::RequestCollisionCallback(PHY_IPhysicsController *ctrl)
{
	JoltPhysicsController *joltctrl = dynamic_cast<JoltPhysicsController*>(ctrl);
	if (!joltctrl)
		return false;

	ReportUnsupported("collision and touch event reporting", m_contactReportingReported);
	return joltctrl->Register();
}

bool JoltPhysicsEnvironment::RemoveCollisionCallback(PHY_IPhysicsController *ctrl)
{
	JoltPhysicsController *joltctrl = dynamic_cast<JoltPhysicsController*>(ctrl);
	if (!joltctrl)
		return false;

	return joltctrl->Unregister();
}

PHY_IPhysicsController* JoltPhysicsEnvironment::CreateSphereController(float radius, const MT_Vector3& position)
{
	/* Proxy used by the Near sensor: a static sensor sphere at the object's
	 * position, see CcdPhysicsEnvironment::CreateSphereController(). */
	if (!m_worldAlive)
		return NULL;

	JoltMotionState *motionstate = new JoltMotionState();
	motionstate->SetWorldPosition((float)position[0], (float)position[1], (float)position[2]);

	JoltPhysicsController *ctrl = new JoltPhysicsController(this, motionstate, false);
	ctrl->SetSensor(true);

	JoltShapeDesc desc;
	desc.shapeType = PHY_SHAPE_SPHERE;
	desc.radius = radius;
	desc.isSensor = true;

	/* No mass: a static sensor needs none, and gravity must not move it. */
	if (!ctrl->Build(desc, 0.0f,
	                 MT_Vector3(1.0, 1.0, 1.0), MT_Vector3(1.0, 1.0, 1.0), true,
	                 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
	                 radius, 0.0f))
	{
		delete ctrl;
		return NULL;
	}

	ctrl->SetRadius(radius);
	AddController(ctrl);
	return ctrl;
}

PHY_IPhysicsController* JoltPhysicsEnvironment::CreateConeController(float coneradius, float coneheight)
{
	/* Proxy used by the Radar sensor.  The cone opens along -Z, like the shape
	 * JoltPhysicsController builds for PHY_SHAPE_CONE. */
	if (!m_worldAlive)
		return NULL;

	JoltMotionState *motionstate = new JoltMotionState();

	JoltPhysicsController *ctrl = new JoltPhysicsController(this, motionstate, false);
	ctrl->SetSensor(true);

	JoltShapeDesc desc;
	desc.shapeType = PHY_SHAPE_CONE;
	desc.radius = coneradius;
	desc.height = coneheight;
	desc.isSensor = true;

	if (!ctrl->Build(desc, 0.0f,
	                 MT_Vector3(1.0, 1.0, 1.0), MT_Vector3(1.0, 1.0, 1.0), true,
	                 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
	                 coneradius, 0.0f))
	{
		delete ctrl;
		return NULL;
	}

	ctrl->SetRadius(coneradius);
	AddController(ctrl);
	return ctrl;
}

/* -------------------------------------------------------------------------
 * Constraints and characters
 *
 * Neither is implemented by this cut.  The entry points report once instead of
 * silently returning a handle that never does anything.
 * ------------------------------------------------------------------------- */

int JoltPhysicsEnvironment::CreateConstraint(PHY_IPhysicsController *ctrl,
                                             PHY_IPhysicsController *ctrl2,
                                             PHY_ConstraintType type,
                                             float pivotX, float pivotY, float pivotZ,
                                             float axis0X, float axis0Y, float axis0Z,
                                             float axis1X, float axis1Y, float axis1Z,
                                             float axis2X, float axis2Y, float axis2Z,
                                             int flag)
{
	(void)ctrl; (void)ctrl2; (void)type;
	(void)pivotX; (void)pivotY; (void)pivotZ;
	(void)axis0X; (void)axis0Y; (void)axis0Z;
	(void)axis1X; (void)axis1Y; (void)axis1Z;
	(void)axis2X; (void)axis2Y; (void)axis2Z;
	(void)flag;

	ReportUnsupported("constraints", m_constraintReported);
	return -1;
}

void JoltPhysicsEnvironment::RemoveConstraintById(int constraintid)
{
	(void)constraintid;
}

float JoltPhysicsEnvironment::GetAppliedImpulse(int constraintid)
{
	(void)constraintid;
	return 0.0f;
}

void JoltPhysicsEnvironment::SetConstraintParam(int constraintId, int param, float value, float value1)
{
	(void)constraintId; (void)param; (void)value; (void)value1;
}

float JoltPhysicsEnvironment::GetConstraintParam(int constraintId, int param)
{
	(void)constraintId; (void)param;
	return 0.0f;
}

void JoltPhysicsEnvironment::SetupObjectConstraints(KX_GameObject *obj_src, KX_GameObject *obj_dest,
                                                    bRigidBodyJointConstraint *dat)
{
	(void)obj_src; (void)obj_dest; (void)dat;
}

PHY_IVehicle* JoltPhysicsEnvironment::GetVehicleConstraint(int constraintId)
{
	(void)constraintId;
	ReportUnsupported("vehicle constraints", m_vehicleReported);
	return NULL;
}

PHY_ICharacter* JoltPhysicsEnvironment::GetCharacterController(KX_GameObject *ob)
{
	(void)ob;
	ReportUnsupported("character controllers", m_characterReported);
	return NULL;
}

/* -------------------------------------------------------------------------
 * Scene merging
 * ------------------------------------------------------------------------- */

void JoltPhysicsEnvironment::MergeEnvironment(PHY_IPhysicsEnvironment *other_env)
{
	/* A Jolt body belongs to the physics system that created it, so merging means
	 * moving the controllers over; JoltPhysicsController::SetPhysicsEnvironment()
	 * rebuilds each body in this system. */
	JoltPhysicsEnvironment *other = dynamic_cast<JoltPhysicsEnvironment*>(other_env);
	if (!other) {
		printf("JoltPhysicsEnvironment::MergeEnvironment: refusing to merge a foreign physics environment\n");
		return;
	}

	/* Copy first: SetPhysicsEnvironment() mutates both sets. */
	std::set<JoltPhysicsController*> others = other->m_controllers;
	std::set<JoltPhysicsController*>::iterator it;
	for (it = others.begin(); it != others.end(); ++it)
		(*it)->SetPhysicsEnvironment(this);
}

/* -------------------------------------------------------------------------
 * Blender object -> Jolt body conversion
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
static bool joltCollectCollisionGeometry(RAS_MeshObject *meshobj, DerivedMesh *dm,
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

void JoltPhysicsEnvironment::ConvertObject(KX_GameObject *gameobj,
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
	(void)kxscene;
	(void)hasCompoundChildren;


	Object *blenderobject = gameobj->GetBlenderObject();

	bool isjoltdyna = (blenderobject->gameflag & OB_DYNAMIC) != 0;
	bool isjoltsensor = (blenderobject->gameflag & OB_SENSOR) != 0;
	bool isjoltsoftbody = (blenderobject->gameflag & OB_SOFT_BODY) != 0;
	bool isjoltrigidbody = (blenderobject->gameflag & OB_RIGID_BODY) != 0;
	bool isjoltcharacter = (blenderobject->gameflag & OB_CHARACTER) != 0;

	/* Jolt has soft bodies, but they are a different simulation from the rigid
	 * body one and are not wired to BGE's soft body flags yet.  An object flagged
	 * as a soft body gets no Jolt representation; this is a documented feature
	 * difference for this cut. */
	if (isjoltsoftbody) {
		ReportUnsupported("soft bodies", m_softBodyReported);
		delete motionstate;
		return;
	}

	if (isjoltcharacter)
		ReportUnsupported("character controllers", m_characterReported);

	/* ---- collision shape description (mirrors CcdPhysicsEnvironment) ---- */

	char bounds = isjoltdyna ? OB_BOUND_SPHERE : OB_BOUND_TRIANGLE_MESH;
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

	JoltShapeDesc shape;
	shape.scaling = gameobj->NodeGetWorldScaling();
	shape.friction = smmaterial->m_friction;
	shape.restitution = smmaterial->m_restitution;
	shape.isSensor = isjoltsensor || isjoltcharacter;

	/* Blender's 16 bit collision group/mask are kept for the mask based layer
	 * setup; the current two layer scheme does not read them, see the note in
	 * JoltPhysicsEnvironment.h. */
	shape.categoryBits = (blenderobject->col_group != 0) ? (uint64_t)(uint16_t)blenderobject->col_group : (uint64_t)1;
	shape.maskBits = (blenderobject->col_mask != 0) ? (uint64_t)(uint16_t)blenderobject->col_mask : ~(uint64_t)0;

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
			shape.halfExtents.setValue(MT_max(bounds_extends[0], JOLT_MIN_HALF_EXTENT),
			                           MT_max(bounds_extends[1], JOLT_MIN_HALF_EXTENT),
			                           MT_max(bounds_extends[2], JOLT_MIN_HALF_EXTENT));
			break;

		case OB_BOUND_CYLINDER:
			shape.shapeType = PHY_SHAPE_CYLINDER;
			shape.radius = MT_max(bounds_extends[0], bounds_extends[1]);
			shape.height = 2.0f * MT_max(bounds_extends[2], JOLT_MIN_HALF_EXTENT);
			break;

		case OB_BOUND_CONE:
			shape.shapeType = PHY_SHAPE_CONE;
			shape.radius = MT_max(bounds_extends[0], bounds_extends[1]);
			shape.height = 2.0f * MT_max(bounds_extends[2], JOLT_MIN_HALF_EXTENT);
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
			/* Both bounds are cooked from the collider polygons of the mesh.  Jolt
			 * only creates mesh contacts on static bodies, so a dynamic object with
			 * mesh bounds gets the convex hull of the same geometry (which is what
			 * Bullet's GImpact path approximates as well). */
			const bool wantTriangles = (bounds == OB_BOUND_TRIANGLE_MESH) && !isjoltdyna;

			std::vector<float> vertices;
			std::vector<int32_t> indices;
			if (joltCollectCollisionGeometry(meshobj, dm, vertices, indices)
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
						printf("JoltPhysics: a triangle mesh bound on a dynamic object is used as its "
						       "convex hull, Jolt only collides mesh shapes on static bodies\n");
					}
				}
				break;
			}

			/* No usable collider geometry (no mesh, no collider polygons, fewer
			 * than four points): keep the object collidable with its bound box. */
			shape.shapeType = PHY_SHAPE_BOX;
			shape.halfExtents.setValue(MT_max(bounds_extends[0], JOLT_MIN_HALF_EXTENT),
			                           MT_max(bounds_extends[1], JOLT_MIN_HALF_EXTENT),
			                           MT_max(bounds_extends[2], JOLT_MIN_HALF_EXTENT));
			if (!m_meshBoundsReported) {
				m_meshBoundsReported = true;
				printf("JoltPhysics: no collider geometry for a mesh based collision bound, "
				       "the object bounding box is used instead\n");
			}
			break;
		}
	}

	/* ---- controller ---- */

	/* A compound child would have its shape attached to the parent body, but a
	 * Jolt body carries exactly one shape, so absorbing a child needs a compound
	 * shape (see JoltPhysicsController::AddChildShape()).  Until that exists the
	 * child becomes a separate body, which keeps it collidable. */
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

		JoltPhysicsController *parentCtrl =
		    parent ? static_cast<JoltPhysicsController*>(parent->GetPhysicsController()) : NULL;

		if (!parentCtrl || !parentCtrl->AddChildShape(shape, JoltChildTransform())) {
			ReportUnsupported("compound child shapes", m_compoundChildReported);
		}
	}

	bool isDynamic = isjoltdyna && !isCompoundChild && !isjoltcharacter;
	/* A character is moved by its own mover, so its body is kinematic: nothing in
	 * the solver may move it, but the game logic can still place it. */
	bool isKinematic = isCompoundChild || isjoltcharacter;

	JoltPhysicsController *ctrl = new JoltPhysicsController(this, motionstate, isDynamic);
	ctrl->SetSensor(isjoltsensor);
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

	const float mass = isjoltdyna ? shapeprops->m_mass : 0.0f;

	/* Bullet inverts Blender's drag to get a damping factor, keep that so that the
	 * "Damping" and "Rotation Damping" sliders keep their meaning. */
	const float linearDamping = 1.0f - shapeprops->m_lin_drag;
	const float angularDamping = 1.0f - shapeprops->m_ang_drag;

	if (!ctrl->Build(shape, mass, linearFactor, angularFactor, isjoltrigidbody,
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

	/* Characters would get their PHY_ICharacter mover here; without one a
	 * character object still exists as a kinematic body but the game logic cannot
	 * move it through the character API.  Already reported above. */

	/* "No sleeping" is what OB_COLLISION_RESPONSE means in the Blender UI. */
	if ((blenderobject->gameflag & OB_COLLISION_RESPONSE) != 0)
		ctrl->SetAllowSleeping(false);

	/* Sensor objects are registered when a collision sensor asks for them, all
	 * other objects are registered as soon as they are on an active layer.  An
	 * object that is not on an active scene layer does not take part in the
	 * simulation at all, which is what CcdPhysicsEnvironment expresses by not
	 * calling AddCcdPhysicsController() for it. */
	if (isjoltsensor || (blenderobject->lay & activeLayerBitInfo) != 0) {
		if (!isjoltsensor)
			AddController(ctrl);
	}
	else {
		ctrl->SetInWorld(false);
	}
}