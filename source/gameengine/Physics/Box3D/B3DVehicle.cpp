/** \file B3DVehicle.cpp
 *  \ingroup physbox3d
 *
 * Box3D implementation of PHY_IVehicle over wheel joints, see B3DVehicle.h.
 */

#include "B3DVehicle.h"

#include "PHY_IMotionState.h"

#include <math.h>

/* box3d.h only carries the C API, this is the same tiny helper the rest of the
 * backend uses to spell a b3Vec3. */
static inline b3Vec3 b3v(float x, float y, float z)
{
	b3Vec3 result;
	result.x = x;
	result.y = y;
	result.z = z;
	return result;
}

/* Suspension:  Bullet and the BGE pass a stiffness coefficient (btVehicleTuning
 * defaults to 5.88), Box3D wants a frequency in Hz.  For a car suspension the
 * numbers happen to live in the same range, so the value is taken as Hz. */
#define B3D_VEHICLE_HERTZ_FACTOR 1.0f
#define B3D_VEHICLE_MIN_HERTZ 0.2f
#define B3D_VEHICLE_MAX_HERTZ 30.0f

/* The BGE's applyEngineForce() is a force, Box3D's spin motor turns at a speed:
 * the force times the tyre radius becomes the max spin torque, and the motor
 * runs against this speed cap. */
#define B3D_VEHICLE_SPIN_SPEED 60.0f

/* How much of the chassis mass a wheel gets.  A Bullet raycast wheel has no mass
 * at all, but a Box3D suspension spring is mass normalized against the joint's
 * own effective mass: a nearly massless wheel cannot carry the chassis, so the
 * wheel has to be a real fraction of it. */
#define B3D_VEHICLE_WHEEL_MASS_FRACTION 0.25f
/* Fallback density when the chassis mass is unknown. */
#define B3D_VEHICLE_WHEEL_DENSITY 1.0f

#define B3D_VEHICLE_STEERING_LIMIT 3.15f
/* Bullet steers a wheel by setting its angle, Box3D's wheel joint steers with a
 * torque limited spring, so the torque has to be a sane one for a car instead of
 * a value that rips the wheel out of the chassis. */
#define B3D_VEHICLE_MAX_STEERING_TORQUE 50.0f

static b3Vec3 b3dVehicleVec(const MT_Vector3& v)
{
	return b3v((float)v.x(), (float)v.y(), (float)v.z());
}

static float b3dVehicleClamp(float value, float lower, float upper)
{
	if (value < lower)
		return lower;
	if (value > upper)
		return upper;
	return value;
}

/** Joint frame basis:  X is the suspension direction, Z the spin axis. */
static b3Quat b3dVehicleFrame(const b3Vec3& down, const b3Vec3& axle)
{
	b3Matrix3 matrix;
	matrix.cx = down;
	matrix.cz = axle;
	matrix.cy = b3Cross(axle, down);
	return b3MakeQuatFromMatrix(&matrix);
}

B3DVehicle::B3DVehicle(b3WorldId worldId, b3BodyId chassisBody, int constraintId, int constraintType)
    : m_worldId(worldId),
      m_chassisBody(chassisBody),
      m_constraintId(constraintId),
      m_constraintType(constraintType),
      m_coordRight(0),
      m_coordUp(1),
      m_coordForward(2)
{
}

B3DVehicle::~B3DVehicle()
{
	for (size_t i = 0; i < m_wheels.size(); i++) {
		Wheel *wheel = m_wheels[i];
		if (b3Joint_IsValid(wheel->joint))
			b3DestroyJoint(wheel->joint, true);
		if (b3Body_IsValid(wheel->body))
			b3DestroyBody(wheel->body);
		/* the motion states belong to KX_VehicleWrapper */
		delete wheel;
	}
	m_wheels.clear();
}

B3DVehicle::Wheel *B3DVehicle::GetWheel(int wheelIndex) const
{
	if (wheelIndex < 0 || wheelIndex >= (int)m_wheels.size())
		return NULL;
	return m_wheels[wheelIndex];
}

bool B3DVehicle::CreateWheelBody(const MT_Vector3& connectionPoint, const MT_Vector3& downDirection,
                                 const MT_Vector3& axleDirection, float suspensionRestLength,
                                 float wheelRadius, bool hasSteering, Wheel& wheel)
{
	const b3Vec3 worldDown = b3Normalize(b3dVehicleVec(downDirection));
	const b3Vec3 worldAxle = b3Normalize(b3dVehicleVec(axleDirection));

	if (b3Dot(worldDown, worldDown) < 0.5f || b3Dot(worldAxle, worldAxle) < 0.5f)
		return false;

	/* The wheel hangs below its attachment point by the suspension rest length,
	 * which is the neutral position the joint spring holds it at. */
	const b3Pos anchor = b3Body_GetWorldPoint(m_chassisBody, b3dVehicleVec(connectionPoint));

	b3BodyDef bd = b3DefaultBodyDef();
	bd.type = b3_dynamicBody;
	bd.position.x = anchor.x + worldDown.x * suspensionRestLength;
	bd.position.y = anchor.y + worldDown.y * suspensionRestLength;
	bd.position.z = anchor.z + worldDown.z * suspensionRestLength;
	bd.rotation = b3Quat_identity;
	bd.linearDamping = 0.0f;
	bd.angularDamping = 0.05f;
	bd.enableSleep = false;
	bd.userData = NULL;

	wheel.body = b3CreateBody(m_worldId, &bd);
	if (!b3Body_IsValid(wheel.body))
		return false;

	/* The tyre: a short capsule along the axle rolls like a cylinder. */
	const float halfWidth = 0.05f * wheelRadius;
	b3Capsule capsule;
	capsule.center1 = b3MulSV(-halfWidth, worldAxle);
	capsule.center2 = b3MulSV(halfWidth, worldAxle);
	capsule.radius = wheelRadius;

	b3ShapeDef sd = b3DefaultShapeDef();
	sd.baseMaterial.friction = 1.0f;
	sd.baseMaterial.restitution = 0.0f;
	/* The mass comes from the density: Box3D has no massless bodies and the
	 * suspension depends on the wheel having a usable one. */
	const float chassisMass = b3Body_GetMass(m_chassisBody);
	float density = B3D_VEHICLE_WHEEL_DENSITY;
	if (chassisMass > 0.0f) {
		const float pi = 3.14159265358979f;
		const float volume = pi * wheelRadius * wheelRadius * (2.0f * halfWidth)
		                     + (4.0f / 3.0f) * pi * wheelRadius * wheelRadius * wheelRadius;
		if (volume > 1e-6f)
			density = B3D_VEHICLE_WHEEL_MASS_FRACTION * chassisMass / volume;
	}
	sd.density = density;
	sd.enableSensorEvents = false;
	sd.enableContactEvents = false;
	sd.updateBodyMass = true;

	wheel.shape = b3CreateCapsuleShape(wheel.body, &sd, &capsule);

	b3WheelJointDef jd = b3DefaultWheelJointDef();
	jd.base.bodyIdA = m_chassisBody;
	jd.base.bodyIdB = wheel.body;
	jd.base.collideConnected = false;
	jd.base.localFrameA.p = b3dVehicleVec(connectionPoint);
	jd.base.localFrameA.q = b3dVehicleFrame(b3RotateVector(b3Conjugate(b3Body_GetRotation(m_chassisBody)), worldDown),
	                                        b3RotateVector(b3Conjugate(b3Body_GetRotation(m_chassisBody)), worldAxle));
	/* The suspension is at rest where the two joint frames coincide.  The frame
	 * positions are body local vectors (the orientation of the frames carries the
	 * axes), so the B frame goes back up along the suspension by the rest length,
	 * which puts it exactly on the attachment point.  The wheel body itself hangs
	 * that far below it. */
	jd.base.localFrameB.p = b3MulSV(-suspensionRestLength, worldDown);
	jd.base.localFrameB.q = b3dVehicleFrame(worldDown, worldAxle);
	jd.base.forceThreshold = 0.0f;
	jd.base.torqueThreshold = 0.0f;

	jd.enableSuspensionSpring = true;
	jd.suspensionHertz = b3dVehicleClamp(5.88f * B3D_VEHICLE_HERTZ_FACTOR,
	                                     B3D_VEHICLE_MIN_HERTZ, B3D_VEHICLE_MAX_HERTZ);
	jd.suspensionDampingRatio = 0.83f;
	jd.enableSuspensionLimit = false;

	jd.enableSpinMotor = true;
	jd.maxSpinTorque = 0.0f;
	jd.spinSpeed = 0.0f;

	jd.enableSteering = hasSteering;
	jd.steeringHertz = 2.0f;
	jd.steeringDampingRatio = 0.7f;
	jd.enableSteeringLimit = hasSteering;
	jd.lowerSteeringLimit = -B3D_VEHICLE_STEERING_LIMIT;
	jd.upperSteeringLimit = B3D_VEHICLE_STEERING_LIMIT;
	jd.targetSteeringAngle = 0.0f;
	jd.maxSteeringTorque = B3D_VEHICLE_MAX_STEERING_TORQUE;

	wheel.joint = b3CreateWheelJoint(m_worldId, &jd);
	if (!b3Joint_IsValid(wheel.joint))
		return false;

	wheel.axle = worldAxle;
	wheel.radius = wheelRadius;
	wheel.rotation = 0.0f;
	wheel.hasSteering = hasSteering;

	return true;
}

void B3DVehicle::AddWheel(PHY_IMotionState *motionState, MT_Vector3 connectionPoint,
                          MT_Vector3 downDirection, MT_Vector3 axleDirection,
                          float suspensionRestLength, float wheelRadius, bool hasSteering)
{
	if (!b3Body_IsValid(m_chassisBody) || !(wheelRadius > 0.0f))
		return;

	Wheel *wheel = new Wheel();
	wheel->motionState = motionState;
	wheel->body = b3_nullBodyId;
	wheel->joint = b3_nullJointId;
	wheel->shape = b3_nullShapeId;
	wheel->axle = b3Vec3_zero;
	wheel->radius = wheelRadius;
	wheel->rotation = 0.0f;
	wheel->hasSteering = hasSteering;

	if (!CreateWheelBody(connectionPoint, downDirection, axleDirection,
	                     suspensionRestLength, wheelRadius, hasSteering, *wheel)) {
		if (b3Joint_IsValid(wheel->joint))
			b3DestroyJoint(wheel->joint, true);
		if (b3Body_IsValid(wheel->body))
			b3DestroyBody(wheel->body);
		delete wheel;
		return;
	}

	m_wheels.push_back(wheel);
}

void B3DVehicle::SyncWheels(float timeStep)
{
	for (size_t i = 0; i < m_wheels.size(); i++) {
		Wheel *wheel = m_wheels[i];
		if (!wheel->motionState || !b3Body_IsValid(wheel->body))
			continue;

		const b3Pos position = b3Body_GetPosition(wheel->body);
		const b3Quat rotation = b3Body_GetRotation(wheel->body);
		wheel->motionState->SetWorldPosition(position.x, position.y, position.z);
		wheel->motionState->SetWorldOrientation(rotation.v.x, rotation.v.y, rotation.v.z, rotation.s);

		/* Bullet reports the wheel's spin angle, Box3D has no such counter, so it
		 * is accumulated from the angular velocity about the axle. */
		const b3Vec3 omega = b3Body_GetAngularVelocity(wheel->body);
		wheel->rotation += b3Dot(omega, wheel->axle) * timeStep;
	}
}

int B3DVehicle::GetNumWheels() const
{
	return (int)m_wheels.size();
}

void B3DVehicle::GetWheelPosition(int wheelIndex, float& posX, float& posY, float& posZ) const
{
	Wheel *wheel = GetWheel(wheelIndex);
	if (!wheel || !b3Body_IsValid(wheel->body))
		return;

	const b3Pos position = b3Body_GetPosition(wheel->body);
	posX = (float)position.x;
	posY = (float)position.y;
	posZ = (float)position.z;
}

void B3DVehicle::GetWheelOrientationQuaternion(int wheelIndex, float& quatX, float& quatY, float& quatZ, float& quatW) const
{
	Wheel *wheel = GetWheel(wheelIndex);
	if (!wheel || !b3Body_IsValid(wheel->body))
		return;

	const b3Quat rotation = b3Body_GetRotation(wheel->body);
	quatX = (float)rotation.v.x;
	quatY = (float)rotation.v.y;
	quatZ = (float)rotation.v.z;
	quatW = (float)rotation.s;
}

float B3DVehicle::GetWheelRotation(int wheelIndex) const
{
	Wheel *wheel = GetWheel(wheelIndex);
	return wheel ? wheel->rotation : 0.0f;
}

void B3DVehicle::SetSteeringValue(float steering, int wheelIndex)
{
	Wheel *wheel = GetWheel(wheelIndex);
	if (!wheel || !b3Joint_IsValid(wheel->joint))
		return;

	/* The BGE asks for an angle, so the joint must not clamp it to its default
	 * limits; the wheel joint's steering is a spring, hence the torque. */
	if (b3Body_IsValid(m_chassisBody))
		b3Body_SetAwake(m_chassisBody, true);
	if (b3Body_IsValid(wheel->body))
		b3Body_SetAwake(wheel->body, true);

	b3WheelJoint_EnableSteering(wheel->joint, true);
	b3WheelJoint_EnableSteeringLimit(wheel->joint, false);
	b3WheelJoint_SetMaxSteeringTorque(wheel->joint, B3D_VEHICLE_MAX_STEERING_TORQUE);
	b3WheelJoint_SetTargetSteeringAngle(wheel->joint, steering);
}

void B3DVehicle::ApplyEngineForce(float force, int wheelIndex)
{
	Wheel *wheel = GetWheel(wheelIndex);
	if (!wheel || !b3Joint_IsValid(wheel->joint))
		return;

	/* Driving wakes the rig up: a resting car is asleep and Box3D ignores the
	 * motor of a sleeping island. */
	if (b3Body_IsValid(m_chassisBody))
		b3Body_SetAwake(m_chassisBody, true);
	if (b3Body_IsValid(wheel->body))
		b3Body_SetAwake(wheel->body, true);

	const float torque = fabsf(force) * wheel->radius;
	b3WheelJoint_EnableSpinMotor(wheel->joint, true);
	b3WheelJoint_SetMaxSpinTorque(wheel->joint, torque);
	b3WheelJoint_SetSpinMotorSpeed(wheel->joint,
	                               (force < 0.0f) ? -B3D_VEHICLE_SPIN_SPEED : B3D_VEHICLE_SPIN_SPEED);
}

void B3DVehicle::ApplyBraking(float braking, int wheelIndex)
{
	Wheel *wheel = GetWheel(wheelIndex);
	if (!wheel || !b3Joint_IsValid(wheel->joint))
		return;

	/* Box3D has no brake: a motor that wants zero speed and fights with a torque
	 * proportional to the braking force is the same thing. */
	if (b3Body_IsValid(m_chassisBody))
		b3Body_SetAwake(m_chassisBody, true);
	if (b3Body_IsValid(wheel->body))
		b3Body_SetAwake(wheel->body, true);

	const float torque = fabsf(braking) * wheel->radius;
	b3WheelJoint_EnableSpinMotor(wheel->joint, true);
	b3WheelJoint_SetMaxSpinTorque(wheel->joint, torque);
	b3WheelJoint_SetSpinMotorSpeed(wheel->joint, 0.0f);
}

void B3DVehicle::SetWheelFriction(float friction, int wheelIndex)
{
	Wheel *wheel = GetWheel(wheelIndex);
	if (!wheel || !b3Shape_IsValid(wheel->shape))
		return;

	b3Shape_SetFriction(wheel->shape, friction);
}

void B3DVehicle::SetSuspensionStiffness(float suspensionStiffness, int wheelIndex)
{
	Wheel *wheel = GetWheel(wheelIndex);
	if (!wheel || !b3Joint_IsValid(wheel->joint))
		return;

	b3WheelJoint_SetSuspensionHertz(wheel->joint,
	                                b3dVehicleClamp(suspensionStiffness * B3D_VEHICLE_HERTZ_FACTOR,
	                                                B3D_VEHICLE_MIN_HERTZ, B3D_VEHICLE_MAX_HERTZ));
}

void B3DVehicle::SetSuspensionDamping(float suspensionDamping, int wheelIndex)
{
	Wheel *wheel = GetWheel(wheelIndex);
	if (!wheel || !b3Joint_IsValid(wheel->joint))
		return;

	b3WheelJoint_SetSuspensionDampingRatio(wheel->joint, b3dVehicleClamp(suspensionDamping, 0.0f, 1.0f));
}

void B3DVehicle::SetSuspensionCompression(float suspensionCompression, int wheelIndex)
{
	/* Bullet has separate relaxation and compression damping, Box3D has a single
	 * damping ratio: the relaxation one (SetSuspensionDamping) wins, this is a
	 * documented no-op. */
	(void)suspensionCompression;
	(void)wheelIndex;
}

void B3DVehicle::SetRollInfluence(float rollInfluence, int wheelIndex)
{
	/* no roll influence in a Box3D wheel joint, the tyre is a real body */
	(void)rollInfluence;
	(void)wheelIndex;
}

void B3DVehicle::SetCoordinateSystem(int rightIndex, int upIndex, int forwardIndex)
{
	/* the wheels carry explicit frames, so this is only remembered */
	m_coordRight = rightIndex;
	m_coordUp = upIndex;
	m_coordForward = forwardIndex;
}
