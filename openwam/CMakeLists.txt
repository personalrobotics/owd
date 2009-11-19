cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

add_subdirectory(positionInterface)
link_directories (positionInterface)

include(../detect_cpu.cmake)

if( DEFINED ENV{XENOMAI_PATH} )
   message(STATUS "Using $ENV{XENOMAI_PATH} for xenomai build")
   include_directories( "$ENV{XENOMAI_PATH}/include" )
else()
   message(STATUS "Using /usr/xenomai for xenomai build")
   include_directories(/usr/xenomai/include)
endif()

if( DEFINED ENV{OWD_MARCH_FLAGS} )
    message(STATUS "Using mtune flags set in environment: $ENV{OWD_MARCH_FLAGS}")
    add_definitions( "$ENV{OWD_MARCH_FLAGS}" )
elseif(VENDOR_ID STREQUAL "GenuineIntel" AND CPU_FAMILY EQUAL 6 AND MODEL EQUAL 28)
    message(STATUS "Building for Intel Atom")
    add_definitions("-march=core2 -mtune=native -mmmx -msse2 -msse3 -mfpmath=sse")
elseif(VENDOR_ID STREQUAL "CentaurHauls")
    message(STATUS "Building for VIA - Original Barrett WAM PC")
    add_definitions("-march=c3-2")
endif()

add_definitions("-ggdb3 -DWRIST -DBH8")
add_library(openwam STATIC  Joint.cc CANbus.cc Motor.cc Puck.cc Group.cc ControlLoop.cc Sigmoid.cc WAM.cc Kinematics.cc Dynamics.cc TrajType.cc PulseTraj.cc screenbuf.cc Trajectory.cc ParabolicSegment.cc ParaJointTraj.cc MacJointTraj.cc MacQuinticBlend.cc MacQuinticSegment.cc ServoTraj.cc StepTraj.cc JointCtrlSea.cc DataRecorder.cc ConstrainedForceTrajectory.cc)

target_link_libraries(openwam positionInterface)
# include_directories (../openmath ../esdcan/lib32 ../rtai)
include_directories (../openmath ../esdcan-pci200/lib32 positionInterface)
