cmake_minimum_required(VERSION 2.8.3)
project(nmpc_pc_rover)
set(PROJECT_NAME nmpc_pc_rover)
set(ROS_BUILD_TYPE Debug)

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++14" COMPILER_SUPPORTS_CXX14)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX14)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
elseif(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 ${CMAKE_CXX_FLAGS_DEBUG} -O1")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
        message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++14 or C++11 support. Please use a different C++ compiler.")
endif()

find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs roslib nav_msgs tf)

find_package(Eigen3 3.0.1 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp tf geometry_msgs
)

#############
# LIBRARIES #
#############

add_library(${PROJECT_NAME}_LIBRARY
  solver/qpoases/SRC/Bounds.cpp
  solver/qpoases/SRC/Constraints.cpp
  solver/qpoases/SRC/CyclingManager.cpp
  solver/qpoases/SRC/Indexlist.cpp
  solver/qpoases/SRC/MessageHandling.cpp
  solver/qpoases/SRC/QProblem.cpp
  solver/qpoases/SRC/QProblemB.cpp
  solver/qpoases/SRC/SubjectTo.cpp
  solver/qpoases/SRC/Utils.cpp
  solver/qpoases/SRC/EXTRAS/SolutionAnalysis.cpp
  solver/nmpc_qpoases_interface.cpp
  solver/nmpc_integrator.c
  solver/nmpc_solver.c
  solver/nmpc_auxiliary_functions.c
)

include_directories(${PROJECT_NAME}_LIBRARY PRIVATE
  ${catkin_INCLUDE_DIRS}

  ${PROJECT_SOURCE_DIR}
  ${PROJECT_SOURCE_DIR}/include

  ${PROJECT_SOURCE_DIR}/solver
  ${PROJECT_SOURCE_DIR}/solver/qpoases
  ${PROJECT_SOURCE_DIR}/solver/qpoases/INCLUDE
  ${PROJECT_SOURCE_DIR}/solver/qpoases/INCLUDE/EXTRAS
  ${PROJECT_SOURCE_DIR}/solver/qpoases/SRC

)

############
# BINARIES #
############

add_executable(nmpc_pc_rover src/nmpc_pc_rover_main.cpp src/nmpc_pc_rover.cpp)
target_link_libraries(nmpc_pc_rover ${PROJECT_NAME}_LIBRARY ${catkin_LIBRARIES})

