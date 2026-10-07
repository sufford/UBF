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

/** \file JoltPhysicsEnvironmentFactory.h
 *  \ingroup physjolt
 *
 * Factory for the Jolt physics environment.
 *
 * KX_BlenderSceneConverter picks the physics engine of a scene and is compiled
 * as C++14, while Jolt (and therefore JoltPhysicsEnvironment.h) needs C++17.
 * Including the environment header there would drag C++17 into the whole
 * converter library, so the converter only sees this plain declaration.
 *
 * The implementation lives in JoltPhysicsEnvironment.cpp, which is part of
 * ge_phys_jolt.
 */

#ifndef __JOLTPHYSICSENVIRONMENTFACTORY_H__
#define __JOLTPHYSICSENVIRONMENTFACTORY_H__

#include "PHY_IPhysicsEnvironment.h"

struct Scene;

/**
 * Create a Jolt physics environment for \a blenderscene.
 *
 * \param blenderscene     the Blender scene the environment belongs to, may be
 *                         NULL.  Its Scene -> Physics settings are applied the
 *                         same way CcdPhysicsEnvironment::Create() applies them.
 * \param visualizePhysics enable debug drawing (--show-physics on the command
 *                         line); accepted for interface compatibility, the Jolt
 *                         backend has no Blender debug renderer yet.
 */
PHY_IPhysicsEnvironment* PHY_JoltCreateEnvironment(Scene *blenderscene, bool visualizePhysics);

#endif  /* __JOLTPHYSICSENVIRONMENTFACTORY_H__ */
