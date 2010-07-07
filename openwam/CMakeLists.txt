cmake_minimum_required(VERSION 2.4.6)
set (ROS_BUILD_STATIC_LIBS true)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

set (RT on)
#set (CANBUS_TYPE "ESD")
set (CANBUS_TYPE "PEAK")

# add_subdirectory(positionInterface)
# link_directories (positionInterface)

include(../detect_cpu.cmake)

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

add_definitions("-ggdb3 -DWRIST -DBH8 -DRT_STATS")

add_library(openwam STATIC  Joint.cc CANbus.cc Motor.cc Puck.cc Group.cc ControlLoop.cc Sigmoid.cc WAM.cc Kinematics.cc Dynamics.cc TrajType.cc PulseTraj.cc screenbuf.cc Trajectory.cc ParabolicSegment.cc ParaJointTraj.cc MacJointTraj.cc MacQuinticBlend.cc MacQuinticSegment.cc ServoTraj.cc StepTraj.cc JointCtrlSea.cc DataRecorder.cc ConstrainedForceTrajectory.cc)

add_library(openwambhd STATIC  Joint.cc CANbus.cc Motor.cc Puck.cc Group.cc ControlLoop.cc Sigmoid.cc WAM.cc Kinematics.cc Dynamics.cc TrajType.cc PulseTraj.cc screenbuf.cc Trajectory.cc ParabolicSegment.cc ParaJointTraj.cc MacJointTraj.cc MacQuinticBlend.cc MacQuinticSegment.cc ServoTraj.cc StepTraj.cc JointCtrlSea.cc DataRecorder.cc ConstrainedForceTrajectory.cc)

add_library(openwam_canbhd STATIC  Joint.cc CANbus.cc Motor.cc Puck.cc Group.cc ControlLoop.cc Sigmoid.cc WAM.cc Kinematics.cc Dynamics.cc TrajType.cc PulseTraj.cc screenbuf.cc Trajectory.cc ParabolicSegment.cc ParaJointTraj.cc MacJointTraj.cc MacQuinticBlend.cc MacQuinticSegment.cc ServoTraj.cc StepTraj.cc JointCtrlSea.cc DataRecorder.cc ConstrainedForceTrajectory.cc)

add_library(openwamsim STATIC  Joint.cc CANbus_sim.cc Motor.cc Puck.cc Group.cc ControlLoop_sim.cc Sigmoid.cc WAM.cc Kinematics.cc Dynamics.cc TrajType.cc PulseTraj.cc screenbuf.cc Trajectory.cc ParabolicSegment.cc ParaJointTraj.cc MacJointTraj.cc MacQuinticBlend.cc MacQuinticSegment.cc ServoTraj.cc StepTraj.cc JointCtrlSea.cc DataRecorder.cc ConstrainedForceTrajectory.cc)

# target_link_libraries(openwam positionInterface)
#include_directories (../openmath positionInterface)
include_directories (../openmath)

if (RT)
# locate Xenomai
find_program(XENO-CONFIG NAMES xeno-config PATHS $ENV{XENOMAI_PATH}/bin /usr/bin /usr/xenomai/bin)
if (XENO-CONFIG)
   execute_process(COMMAND ${XENO-CONFIG} --xeno-cflags
                   RESULT_VARIABLE XENOMAI_NOTFOUND
                   OUTPUT_VARIABLE RT_DEFS
                   OUTPUT_STRIP_TRAILING_WHITESPACE)
   if (XENOMAI_NOTFOUND)
       message (SEND_ERROR "Could not find Xenomai include files (command xeno-config --xeno-cflags failed)\n")
   endif(XENOMAI_NOTFOUND)
   execute_process(COMMAND ${XENO-CONFIG} --xeno-ldflags
                   RESULT_VARIABLE XENOMAI_NOTFOUND
                   OUTPUT_VARIABLE RT_LIBS
                   OUTPUT_STRIP_TRAILING_WHITESPACE)
   if (XENOMAI_NOTFOUND)
      message (SEND_ERROR "Could not find Xenomai libs (command xeno-config --xeno-ldflags failed)\n")
   endif (XENOMAI_NOTFOUND)
   
elseif ("$ENV{XENOMAI_PATH}" STRGREATER "")
message(STATUS "Using XENOMAI_PATH=$ENV{XENOMAI_PATH}")
   set (RT_DEFS "-I$ENV{XENOMAI_PATH}/include")
   set (RT_LIBS "-L$ENV{XENOMAI_PATH}/lib -lnative")
endif (XENO-CONFIG)

else (RT)
set (RT_DEFS "-DNO_RT")
set (RT_LIBS "")
endif(RT)


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


rosbuild_add_compile_flags(openwam "${RT_DEFS} ${CANBUS_DEFS}")
rosbuild_add_compile_flags(openwambhd "-DBH280 ${RT_DEFS} ${CANBUS_DEFS}")
rosbuild_add_compile_flags(openwam_canbhd "-DBH280 -DBH280_ONLY ${RT_DEFS} ${CANBUS_DEFS}")
rosbuild_add_compile_flags(openwamsim "-DOWDSIM ${RT_DEFS}")

rosbuild_add_link_flags(openwam "${RT_LIBS} ${CANBUS_LIBS}")
rosbuild_add_link_flags(openwambhd "${RT_LIBS} ${CANBUS_LIBS}")
rosbuild_add_link_flags(openwam_canbhd "${RT_LIBS} ${CANBUS_LIBS}")
