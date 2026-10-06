/** \file B3DVehicle.h
 *  \ingroup physbox3d
 *
 * Box3D implementation of PHY_IVehicle.
 *
 * Bullet builds a vehicle out of ray casts against the ground, Box3D has no such
 * helper: here every wheel is a real rigid body (a short capsule along the axle)
 * held to the chassis by a b3WheelJoint, which is how Box3D's own car samples do
 * it.  The BGE side of vehicles does not change at all:
 * bge.constraints.createConstraint(chassis, -1, VEHICLE_CONSTRAINT),
 * bge.constraints.getVehicleConstraint(id) and KX_VehicleWrapper keep working,
 * the wheel objects are moved by SyncWheels() like Bullet's raycast wheels.
 */

#ifndef __B3DVEHICLE_H__
#define __B3DVEHICLE_H__

#include <vector>

#include "PHY_IVehicle.h"
#include "PHY_DynamicTypes.h"

#include "MT_Vector3.h"

#include "box3d/box3d.h"

class PHY_IMotionState;

class B3DVehicle : public PHY_IVehicle
{
public:
	B3DVehicle(b3WorldId worldId, b3BodyId chassisBody, int constraintId, int constraintType);
	virtual ~B3DVehicle();

	/** Move the visual wheel objects, called after every world step. */
	void SyncWheels(float timeStep);

	/* ---- PHY_IVehicle ---- */

	virtual void AddWheel(PHY_IMotionState *motionState,
	                      MT_Vector3 connectionPoint,
	                      MT_Vector3 downDirection,
	                      MT_Vector3 axleDirection,
	                      float suspensionRestLength,
	                      float wheelRadius,
	                      bool hasSteering);

	virtual int GetNumWheels() const;

	virtual void GetWheelPosition(int wheelIndex, float& posX, float& posY, float& posZ) const;
	virtual void GetWheelOrientationQuaternion(int wheelIndex, float& quatX, float& quatY, float& quatZ, float& quatW) const;
	virtual float GetWheelRotation(int wheelIndex) const;

	virtual int GetUserConstraintId() const { return m_constraintId; }
	virtual int GetUserConstraintType() const { return m_constraintType; }

	virtual void SetSteeringValue(float steering, int wheelIndex);
	virtual void ApplyEngineForce(float force, int wheelIndex);
	virtual void ApplyBraking(float braking, int wheelIndex);
	virtual void SetWheelFriction(float friction, int wheelIndex);
	virtual void SetSuspensionStiffness(float suspensionStiffness, int wheelIndex);
	virtual void SetSuspensionDamping(float suspensionDamping, int wheelIndex);
	virtual void SetSuspensionCompression(float suspensionCompression, int wheelIndex);
	virtual void SetRollInfluence(float rollInfluence, int wheelIndex);
	virtual void SetCoordinateSystem(int rightIndex, int upIndex, int forwardIndex);

private:
	struct Wheel {
		PHY_IMotionState *motionState;
		b3BodyId body;
		b3JointId joint;
		b3ShapeId shape;
		b3Vec3 axle;      /**< spin axis, world space, kept for the spin angle */
		float radius;
		float rotation;   /**< accumulated spin angle, Bullet's m_rotation */
		bool hasSteering;
	};

	Wheel *GetWheel(int wheelIndex) const;
	bool CreateWheelBody(const MT_Vector3& connectionPoint, const MT_Vector3& downDirection,
	                     const MT_Vector3& axleDirection, float suspensionRestLength,
	                     float wheelRadius, bool hasSteering, Wheel& wheel);

	b3WorldId m_worldId;
	b3BodyId m_chassisBody;
	int m_constraintId;
	int m_constraintType;

	/* The BGE never calls SetCoordinateSystem(), the wheels carry their own
	 * frames around, so these are only remembered. */
	int m_coordRight, m_coordUp, m_coordForward;

	std::vector<Wheel*> m_wheels;
};

#endif  /* __B3DVEHICLE_H__ */
