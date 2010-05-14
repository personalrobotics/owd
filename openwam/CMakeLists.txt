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
remove_definitions("-O2")
#set (WRIST 1)
#set (BH8 1)
#configure_file(${PROJECT_SOURCE_DIR}/openwam/Configure.h.in
#  ${PROJECT_BINARY_DIR}/openwamConfigure.h)
add_library(openwam STATIC  Joint.cc CANbus.cc Motor.cc Puck.cc Group.cc ControlLoop.cc Sigmoid.cc WAM.cc Kinematics.cc Dynamics.cc TrajType.cc PulseTraj.cc screenbuf.cc Trajectory.cc ParabolicSegment.cc ParaJointTraj.cc MacJointTraj.cc MacQuinticBlend.cc MacQuinticSegment.cc ServoTraj.cc StepTraj.cc JointCtrlSea.cc DataRecorder.cc ConstrainedForceTrajectory.cc)

# set (WRIST 1)
# set (BH8 1)
# set (OWDSIM 1)
# configure_file(${PROJECT_SOURCE_DIR}/openwam/Configure.h.in
#   ${PROJECT_BINARY_DIR}/openwamsimConfigure.h)
add_library(openwamsim STATIC  Joint.cc CANbus_sim.cc Motor.cc Puck.cc Group.cc ControlLoop_sim.cc Sigmoid.cc WAM.cc Kinematics.cc Dynamics.cc TrajType.cc PulseTraj.cc screenbuf.cc Trajectory.cc ParabolicSegment.cc ParaJointTraj.cc MacJointTraj.cc MacQuinticBlend.cc MacQuinticSegment.cc ServoTraj.cc StepTraj.cc JointCtrlSea.cc DataRecorder.cc ConstrainedForceTrajectory.cc)
set_property(TARGET openwamsim PROPERTY COMPILE_FLAGS "-DOWDSIM")

target_link_libraries(openwam positionInterface)
include_directories (../openmath ../esdcan-pci200/lib32 positionInterface)
