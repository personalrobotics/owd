cmake_minimum_required(VERSION 2.4.6)
set (ROS_BUILD_STATIC_LIBS true)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
include(../find_xenomai.cmake)
include(../detect_cpu.cmake)

#set (CANBUS_TYPE "ESD")
set (CANBUS_TYPE "PEAK")

if (CANBUS_TYPE STREQUAL "ESD")
message(STATUS "using ESD CANbus driver")
set (CANBUS_DEFS "-DESD_CAN -I../../esdcan-pci200/lib32")
set (CANBUS_LIBS "ntcan")
set (CANBUS_LDFLAGS "-L../../esdcan-pci200/lib32")

elseif (CANBUS_TYPE  STREQUAL "PEAK")
message(STATUS "using PEAK CANbus driver")
set (CANBUS_DEFS "-DPEAK_CAN")
set (CANBUS_LIBS "pcan")

else (CANBUS_TYPE STREQUAL "ESD")
message(SEND_ERROR "No CANbus type recognized")
set (CANBUS_DEFS "")
set (CANBUS_LIBS "")
endif (CANBUS_TYPE STREQUAL "ESD")

# add_subdirectory(positionInterface)
# link_directories (positionInterface)

if (DEFINED ENV{OWD_MARCH_FLAGS})
    message(STATUS "Using mtune flags set in environment: $ENV{OWD_MARCH_FLAGS}")
    add_definitions( "$ENV{OWD_MARCH_FLAGS}" )
elseif (VENDOR_ID STREQUAL "GenuineIntel" AND CPU_FAMILY EQUAL 6 AND MODEL EQUAL 28)
    message(STATUS "Building for Intel Atom")
    add_definitions("-march=core2 -mtune=native -mmmx -msse2 -msse3 -mfpmath=sse")
elseif (VENDOR_ID STREQUAL "CentaurHauls")
    message(STATUS "Building for VIA - Original Barrett WAM PC")
    add_definitions("-march=c3-2")
endif (DEFINED ENV{OWD_MARCH_FLAGS})

add_definitions("-ggdb3 -DWRIST -DBH8 -DRT_STATS -DCAN_RECORD")

add_library(openwam STATIC  Joint.cc Motor.cc Puck.cc Group.cc
                            Sigmoid.cc Kinematics.cc Dynamics.cc
			    TrajType.cc PulseTraj.cc Trajectory.cc ParabolicSegment.cc
                            ParaJointTraj.cc MacJointTraj.cc MacQuinticBlend.cc MacQuinticSegment.cc
                            ServoTraj.cc StepTraj.cc ConstrainedForceTrajectory.cc)

add_library(openwamsim STATIC  Joint.cc CANbus_sim.cc Motor.cc Puck.cc Group.cc
                               ControlLoop.cc Sigmoid.cc WAM.cc Kinematics.cc Dynamics.cc
                               TrajType.cc PulseTraj.cc Trajectory.cc ParabolicSegment.cc
                               ParaJointTraj.cc MacJointTraj.cc MacQuinticBlend.cc MacQuinticSegment.cc
                               ServoTraj.cc StepTraj.cc ConstrainedForceTrajectory.cc)
rosbuild_add_compile_flags(openwamsim "-DOWDSIM")

add_library(wamcan STATIC      ControlLoop.cc CANbus.cc WAM.cc)
rosbuild_add_compile_flags(wamcan "${CANBUS_DEFS}")
rosbuild_add_link_flags(wamcan "${CANBUS_LIBS}")

add_library(wamcanrt STATIC      ControlLoop.cc CANbus.cc WAM.cc)
rosbuild_add_compile_flags(wamcanrt "${CANBUS_DEFS} ${RT_DEFS}")
rosbuild_add_link_flags(wamcanrt "${CANBUS_LIBS} ${RT_LIBS}")

add_library(bhdcan STATIC      ControlLoop.cc CANbus.cc WAM.cc)
rosbuild_add_compile_flags(bhdcan "${CANBUS_DEFS} -DBH280_ONLY")
rosbuild_add_link_flags(bhdcan "${CANBUS_LIBS}")

add_library(bhdcanrt STATIC      ControlLoop.cc CANbus.cc WAM.cc)
rosbuild_add_compile_flags(bhdcanrt "${CANBUS_DEFS} ${RT_DEFS} -DBH280_ONLY")
rosbuild_add_link_flags(bhdcanrt "${CANBUS_LIBS} ${RT_LIBS}")

# target_link_libraries(openwam positionInterface)
# include_directories (../openmath positionInterface)
include_directories (../openmath)
