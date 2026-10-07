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
# The Original Code is Copyright (C) 2006 Blender Foundation
# All rights reserved.
#
# The Original Code is: all of this file.
#
# ***** END GPL LICENSE BLOCK *****

# intern/rigidbody_jolt/CMakeLists.txt
#
# Vendored Jolt Physics 5.5.x (thirdparty/Jolt), built as the bf_intern_jolt
# library.  The Blender Game Engine backend that consumes it - and the only part
# of this integration that contains hand-written Blender code - lives in
# source/gameengine/Physics/Jolt/ge_phys_jolt, which links the
# bf_intern_jolt_config interface target defined at the bottom of this file.
# The intern/ directory below is kept as the conventional place for glue code
# that vendored libraries usually need; it is currently empty.
#
# ---------------------------------------------------------------------------
# Read this before changing anything below.
# ---------------------------------------------------------------------------
#
# 1. There is deliberately no project()/cmake_minimum_required() in this file.
#    This directory is a subdirectory of the Blender build; a nested project()
#    re-runs compiler detection and resets variables the surrounding build
#    depends on.
#
# 2. Jolt's ABI depends on compile settings, not only on types.
#      * thirdparty/Jolt/Core/Core.h takes its SIMD level from the compiler's
#        own predefined macros - JPH_USE_AVX2 from __AVX2__, JPH_USE_AVX from
#        __AVX__, JPH_USE_FMADD from __AVX2__ on MSVC, and so on.  There is no
#        JPH_USE_* knob to set here; /arch:AVX2 is what selects it.
#      * JPH_ENABLE_ASSERTS, JPH_DEBUG_RENDERER, JPH_DOUBLE_PRECISION and
#        JPH_OBJECT_STREAM change class layouts and inline code paths.
#    Therefore every translation unit that includes a Jolt header - including
#    the Game Engine glue - must be compiled with exactly the same settings, or
#    the two halves disagree about object layout.
#
# 3. In this build system libraries are NOT linked against each other: they are
#    collected into BLENDER_SORTED_LIBS and linked into one flat list by
#    blender/blenderplayer.  PUBLIC properties on bf_intern_jolt would
#    therefore propagate to nobody.  The settings that consumers must share live
#    in the bf_intern_jolt_config INTERFACE target below, and a consumer opts in
#    with a single target_link_libraries() call - see
#    source/gameengine/Physics/Jolt/CMakeLists.txt.

set(INC
	.
	intern
	thirdparty
)

set(INC_SYS
)

# Jolt is vendored third-party code: do not let the surrounding build's strict
# warning flags turn into build failures.
remove_strict_flags()

# Jolt requires C++17.  Visual Studio 2017 (v141) is enough once /std:c++17 and
# /Zc:__cplusplus are in place.  Scoped to this directory: the rest of Blender
# 2.79 is C++11/14 code.  Consumers get the standard through
# bf_intern_jolt_config (cxx_std_17) rather than by repeating this.
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

file(GLOB_RECURSE JOLT_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/Jolt/*.cpp")

set(SRC
	${JOLT_SOURCES}

	# Reserved for Blender-to-Jolt glue code (currently none, see the header
	# comment).  Sources are listed explicitly rather than globbed, so that
	# adding a file always requires re-running CMake instead of silently doing
	# nothing.
)

blender_add_lib(bf_intern_jolt "${SRC}" "${INC}" "${INC_SYS}")

# ---------------------------------------------------------------------------
# Options
# ---------------------------------------------------------------------------

option(WITH_JOLT_AVX2
	"Build Jolt with AVX2/FMA code paths. The resulting binaries will fault on CPUs without AVX2."
	OFF
)

option(WITH_JOLT_ASSERTS
	"Force Jolt's internal asserts into every configuration. Debug builds always get them."
	OFF
)

option(WITH_JOLT_DEBUG_RENDERER
	"Force Jolt's debug renderer into every configuration. Debug builds always get it."
	OFF
)

# ---------------------------------------------------------------------------
# Settings shared with every consumer of Jolt headers
# ---------------------------------------------------------------------------

# Interface target: carries usage requirements only, contributes nothing to the
# link line and is intentionally not registered as a Blender library.
add_library(bf_intern_jolt_config INTERFACE)

# Makes #include <Jolt/...> resolve.  blender_add_lib() only sets the
# directory-scoped include_directories(), which does not reach other
# directories, so this has to be carried explicitly.
target_include_directories(bf_intern_jolt_config INTERFACE
	"${CMAKE_CURRENT_SOURCE_DIR}/thirdparty"
)

# Consumers include Jolt headers and so must compile as C++17 as well.
target_compile_features(bf_intern_jolt_config INTERFACE cxx_std_17)

if(WITH_JOLT_AVX2)
	if(MSVC)
		target_compile_options(bf_intern_jolt_config INTERFACE /arch:AVX2)
	else()
		# Mirrors Jolt's own Jolt.cmake.
		target_compile_options(bf_intern_jolt_config INTERFACE -mavx2 -mbmi -mpopcnt -mlzcnt -mf16c)
	endif()
elseif(NOT MSVC AND NOT CMAKE_SIZEOF_VOID_P EQUAL 8)
	# On 32-bit builds the x87 FPU would be used otherwise, and it has a higher
	# intermediate precision than Jolt expects.
	target_compile_options(bf_intern_jolt_config INTERFACE -msse2)
endif()

if(WITH_JOLT_ASSERTS)
	target_compile_definitions(bf_intern_jolt_config INTERFACE JPH_ENABLE_ASSERTS)
else()
	# JPH_ENABLE_ASSERTS is part of the ABI (it changes class layouts and inline
	# code paths), so it is set on the interface target: Jolt and every consumer
	# of its headers must agree.  Debug builds get the asserts, Release builds do
	# not - the option above is the escape hatch for a Release assert-hunting run.
	target_compile_definitions(bf_intern_jolt_config INTERFACE $<$<CONFIG:Debug>:JPH_ENABLE_ASSERTS>)
endif()

if(WITH_JOLT_DEBUG_RENDERER)
	target_compile_definitions(bf_intern_jolt_config INTERFACE JPH_DEBUG_RENDERER)
else()
	# JPH_DEBUG_RENDERER adds members to Jolt's body and shape classes, so it
	# has to be all-or-nothing per configuration.
	target_compile_definitions(bf_intern_jolt_config INTERFACE $<$<CONFIG:Debug>:JPH_DEBUG_RENDERER>)
endif()

if(MSVC)
	# /Zc:__cplusplus makes __cplusplus report the real language standard, which
	# Jolt's C++17 detection depends on.
	target_compile_options(bf_intern_jolt_config INTERFACE /Zc:__cplusplus)

	target_compile_options(bf_intern_jolt PRIVATE
		/permissive-  # Jolt's templates are written against a conforming compiler
		/wd4577       # exception handling warnings caused by /EHsc
	)
endif()

# Note: RTTI is intentionally left enabled (/GR), matching the rest of the
# build.  The Game Engine glue derives from Jolt's abstract interfaces
# (ContactListener, JobSystem, BroadPhaseLayerInterface, ...), so the whole
# process has to agree on RTTI; Jolt itself does not need /GR-.
target_link_libraries(bf_intern_jolt PRIVATE bf_intern_jolt_config)
