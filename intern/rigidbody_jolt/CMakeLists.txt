cmake_minimum_required(VERSION 3.14)
project(bf_intern_jolt CXX) # Follows Blender's "bf_intern_" naming convention

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 1. Gather the Jolt Engine source files
file(GLOB_RECURSE JOLT_SOURCES 
    "${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/Jolt/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/Jolt/*.h"
)

# 2. Gather your custom Blender-to-Jolt connection code
file(GLOB_RECURSE GLUE_SOURCES 
    "${CMAKE_CURRENT_SOURCE_DIR}/intern/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/intern/*.h"
)

# 3. Combine them into a static Blender module library
add_library(bf_intern_jolt STATIC ${JOLT_SOURCES} ${GLUE_SOURCES})

# 4. Include paths so Blender can find Jolt headers cleanly
target_include_directories(bf_intern_jolt PUBLIC 
    "${CMAKE_CURRENT_SOURCE_DIR}/thirdparty"
    "${CMAKE_CURRENT_SOURCE_DIR}/extern" # Adjust if your glue headers reference external/
)

# 5. Visual Studio 2017 & Blender-Specific Rigging
if(MSVC)
    target_compile_options(bf_intern_jolt PRIVATE 
        /arch:AVX2          # Tells VS2017 to use SIMD math
        /GR-                # Disables RTTI (Mandatory for Blender 2.79)
        /Zc:__cplusplus     # Fixes MSVC's modern C++ reporting bug
        /permissive-        # Enforces standard compliance for Jolt's templates
        /wd4577             # Ignores exception warnings
    )
    
    target_compile_definitions(bf_intern_jolt PUBLIC JPH_DEBUG_RENDERER)
endif()
