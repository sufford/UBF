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

# Jolt is C++17 with its own warning level, its own idea of what a "default"
# compiler configuration is and a lot of non standard pragmas.
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

# The vendored Jolt source tree lives in intern/rigidbody_jolt and is built as the
# bf_intern_jolt library.
#
# Jolt's ABI depends on how it is compiled: JPH_USE_AVX2 and friends are derived
# from the compiler's own predefined macros, and JPH_ENABLE_ASSERTS,
# JPH_DEBUG_RENDERER and JPH_DOUBLE_PRECISION change class layouts.  This
# directory therefore has to be compiled with exactly the same defines as the
# library, which bf_intern_jolt_config carries as an INTERFACE target.  The
# define list lives in one place so the two can never drift apart.
#
# (The include path is also listed in INC_SYS below so that this library can be
# configured even when bf_intern_jolt was not added to the build.)
set(INC_SYS
	../../../../intern/moto/include
	${GLEW_INCLUDE_PATH}
	${PYTHON_INCLUDE_DIRS}
	${CMAKE_SOURCE_DIR}/intern/rigidbody_jolt/thirdparty
)

set(SRC
	JoltMotionState.h
	JoltPhysicsController.cpp
	JoltPhysicsController.h
	JoltPhysicsEnvironment.cpp
	JoltPhysicsEnvironment.h
	JoltPhysicsEnvironmentFactory.h
)

add_definitions(${GL_DEFINITIONS})

blender_add_lib(ge_phys_jolt "${SRC}" "${INC}" "${INC_SYS}")

# This build system collects libraries into BLENDER_SORTED_LIBS instead of linking
# them to each other, so nothing propagates between directories on its own.  The
# INTERFACE target carries the Jolt compile settings across.
if(TARGET bf_intern_jolt_config)
	target_link_libraries(ge_phys_jolt PRIVATE bf_intern_jolt_config)
else()
	message(WARNING "bf_intern_jolt_config is missing: ge_phys_jolt is being built without Jolt's compile settings")
endif()
