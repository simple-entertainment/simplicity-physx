cmake_minimum_required(VERSION 2.8)

include(../../simplicity/simplicity/simplicity-common.cmake)

# Project
project(simplicity-physx)

# Policies
cmake_policy(SET CMP0015 NEW)

# Paths
file(GLOB_RECURSE SRC_FILES src/main/c++/*.cpp src/main/c++/*.h)
include_directories(../../simplicity/simplicity/src/main/c++)
include_directories(dep/physx/include)
include_directories(src/main/c++)

# Output
add_library(simplicity-physx STATIC ${SRC_FILES})

# TODO!!!
target_link_libraries(simplicity-physx debug PhysXVisualDebuggerSDKDEBUG)
target_link_libraries(simplicity-physx debug PhysXProfileSDKDEBUG)
#IF(CMAKE_SIZEOF_VOID_P EQUAL 8)
	target_link_libraries(simplicity-physx debug PhysX3CommonDEBUG_x64)
	target_link_libraries(simplicity-physx debug PhysX3DEBUG_x64)
#ELSE()
#ENDIF(CMAKE_SIZEOF_VOID_P EQUAL 8)

