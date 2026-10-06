# ***** BEGIN GPL LICENSE BLOCK *****
#
# This program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public License
# as published by the Free Software Foundation; either version 2
# of the License, or (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software Foundation,
# Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
#
# ***** END GPL LICENSE BLOCK *****

# Box3D is a C library, its public headers are included from C++ here.
remove_strict_flags()

set(INC
	.
	../common
	../../Converter
	../../Expressions
	../../GameLogic
	../../Ketsji
	../../Rasterizer
	../../SceneGraph
	../../../blender/blenkernel
	../../../blender/blenlib
	../../../blender/gpu
	../../../blender/makesdna
	../../../../intern/container
	../../../../intern/guardedalloc
	../../../../intern/glew-mx
	../../../../intern/string
)

# The vendored Box3D source tree lives in intern/rigidbody_box3d and is built as
# the bf_intern_rigidbody_box3d library.  Its public headers are used here.
set(INC_SYS
	../../../../intern/moto/include
	${GLEW_INCLUDE_PATH}
	${PYTHON_INCLUDE_DIRS}
	${CMAKE_SOURCE_DIR}/intern/rigidbody_box3d/include
)

set(SRC
	B3DCharacter.cpp
	B3DPhysicsController.cpp
	B3DPhysicsEnvironment.cpp
	B3DVehicle.cpp

	B3DCharacter.h
	B3DPhysicsController.h
	B3DPhysicsEnvironment.h
	B3DVehicle.h
)

add_definitions(${GL_DEFINITIONS})

blender_add_lib(ge_phys_box3d "${SRC}" "${INC}" "${INC_SYS}")
