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

/** \file B3DMotionState.h
 *  \ingroup physbox3d
 *
 * Standalone motion state for the controllers Box3D creates on its own, i.e.
 * the sphere and cone proxies behind the Near and Radar sensors.
 *
 * Those proxies are not tied to a scene graph node: KX_NearSensor (and
 * KX_RadarSensor) copy the parent object's transform into the motion state every
 * frame and the physics reads it back, so this class only stores the latest
 * transform.  Bullet uses DefaultMotionState for the same job, but that one
 * lives in CcdPhysicsController.h and would drag Bullet types into this backend.
 */

#ifndef __B3DMOTIONSTATE_H__
#define __B3DMOTIONSTATE_H__

#include "PHY_IMotionState.h"

#include "MT_Point3.h"
#include "MT_Quaternion.h"
#include "MT_Matrix3x3.h"
#include "MT_Vector3.h"

#include "box3d/box3d.h"

class B3DMotionState : public PHY_IMotionState
{
public:
	B3DMotionState()
	{
		m_position.setValue(0.0f, 0.0f, 0.0f);
		m_orientation.setValue(0.0f, 0.0f, 0.0f, 1.0f);
		m_scaling.setValue(1.0f, 1.0f, 1.0f);
	}

	virtual ~B3DMotionState() {}

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
	 *  sensors to push their parent's orientation in. */
	virtual void SetWorldOrientation(const float *ori)
	{
		b3Matrix3 mat;
		mat.cx.x = ori[0];
		mat.cx.y = ori[1];
		mat.cx.z = ori[2];
		mat.cy.x = ori[4];
		mat.cy.y = ori[5];
		mat.cy.z = ori[6];
		mat.cz.x = ori[8];
		mat.cz.y = ori[9];
		mat.cz.z = ori[10];

		/* Box3D and MT agree on the active rotation convention, so the
		 * quaternion can be used as is. */
		b3Quat q = b3MakeQuatFromMatrix(&mat);
		m_orientation.setValue(q.v.x, q.v.y, q.v.z, q.s);
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

#endif  /* __B3DMOTIONSTATE_H__ */
