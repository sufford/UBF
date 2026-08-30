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
# The Original Code is Copyright (C) 2006, Blender Foundation
# All rights reserved.
#
# The Original Code is: all of this file.
#
# ***** END GPL LICENSE BLOCK *****

# intern/rigidbody_box3d/CMakeLists.txt
# Optimized specifically for Blender commit e045fe53f1b0

set(INC
	.
	include
	../../source/blender/makesdna
)

set(INC_SYS
	# Keeps include directories isolated
)

# 1. Automatically grab ALL .c files from Box3D's source subdirectory
file(GLOB BOX3D_SRC "src/*.c")

# 2. Bundle the gathered C source files along with your custom bridge file
set(SRC
	${BOX3D_SRC}
	rb_box3d_api.cpp
)

# 3. MSVC compatibility for C11/C17 features
if(MSVC)
    add_definitions(
        -D_Static_assert=static_assert
        -D_Alignof=alignof
        -D_Alignas=alignas
    )
endif()

# Enforce uniform compile behaviors for hybrid 2.79/2.80 environments
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

blender_add_lib(bf_intern_rigidbody_box3d "${SRC}" "${INC}" "${INC_SYS}")