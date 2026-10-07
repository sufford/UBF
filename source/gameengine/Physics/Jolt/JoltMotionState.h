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

/** \file JoltMotionState.h
 *  \ingroup physjolt
 *
 * Standalone motion state for the controllers Jolt creates on its own, i.e. the
 * sphere and cone proxies behind the Near and Radar sensors.
 *
 * Those proxies are not tied to a scene graph node: KX_NearSensor (and
 * KX_RadarSensor) copy the parent object's transform into the motion state every
 * frame and the physics reads it back, so this class only stores the latest
 * transform.  Bullet uses DefaultMotionState for the same job, but that one
 * lives in CcdPhysicsController.h and would drag Bullet types into this backend.
 *
 * Unlike B3DMotionState this class needs no physics engine types at all: Jolt's
 * quaternion is (x, y, z, w), the same order PHY_IMotionState uses, and the
 * matrix conversion is plain moto.
 */

#ifndef __JOLTMOTIONSTATE_H__
#define __JOLTMOTIONSTATE_H__

#include "PHY_IMotionState.h"

#include "MT_Point3.h"
#include "MT_Quaternion.h"
#include "MT_Matrix3x3.h"
#include "MT_Vector3.h"

class JoltMotionState : public PHY_IMotionState
{
public:
	JoltMotionState()
	{
		m_position.setValue(0.0f, 0.0f, 0.0f);
		m_orientation.setValue(0.0f, 0.0f, 0.0f, 1.0f);
		m_scaling.setValue(1.0f, 1.0f, 1.0f);
	}

	virtual ~JoltMotionState() {}

	virtual void GetWorldPosition(float& posX, float& posY, float& posZ)
	{
		posX = (float)m_position[0];
		posY = (float)m_position[1];
		posZ = (float)m_position[2];
	}

	virtual void GetWorldScaling(float& scaleX, float& scaleY, float& scaleZ)
	{
		scaleX = (float)m_scaling[0];
		scaleY = (float)m_scaling[1];
		scaleZ = (float)m_scaling[2];
	}

	virtual void GetWorldOrientation(float& quatIma0, float& quatIma1, float& quatIma2, float& quatReal)
	{
		quatIma0 = (float)m_orientation[0];
		quatIma1 = (float)m_orientation[1];
		quatIma2 = (float)m_orientation[2];
		quatReal = (float)m_orientation[3];
	}

	/** ori is the 12 float, column major array MT_Matrix3x3::getValue() writes. */
	virtual void GetWorldOrientation(float *ori)
	{
		MT_Matrix3x3 mat(m_orientation);
		mat.getValue(ori);
	}

	/** The reverse of GetWorldOrientation(float*), used by the Near and Radar
	 *  sensors to push their parent's orientation in.  getValue() writes column
	 *  major, MT_Matrix3x3's constructor takes (xx, xy, xz, yx, ...). */
	virtual void SetWorldOrientation(const float *ori)
	{
		MT_Matrix3x3 mat(ori[0], ori[4], ori[8],
		                 ori[1], ori[5], ori[9],
		                 ori[2], ori[6], ori[10]);

		/* moto and Jolt both use the standard active rotation convention, so the
		 * quaternion can be used as is.  moto has no quaternion constructor from a
		 * matrix, MT_Matrix3x3::getRotation() is the conversion. */
		MT_Quaternion q = mat.getRotation();
		m_orientation.setValue(q[0], q[1], q[2], q[3]);
	}

	virtual void SetWorldPosition(float posX, float posY, float posZ)
	{
		m_position.setValue(posX, posY, posZ);
	}

	virtual void SetWorldOrientation(float quatIma0, float quatIma1, float quatIma2, float quatReal)
	{
		m_orientation.setValue(quatIma0, quatIma1, quatIma2, quatReal);
	}

	virtual void CalculateWorldTransformations() {}

	void SetWorldScaling(float scaleX, float scaleY, float scaleZ)
	{
		m_scaling.setValue(scaleX, scaleY, scaleZ);
	}

private:
	MT_Point3 m_position;
	MT_Quaternion m_orientation;
	MT_Vector3 m_scaling;
};

#endif  /* __JOLTMOTIONSTATE_H__ */
