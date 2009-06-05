cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

add_definitions("-march=c3-2 -ggdb3 -DWRIST -DBH8")
#add_definitions("-march=c3-2 -ggdb3 -DFAKE7 -DBH8")
add_library(openwam STATIC  Joint.cc CANbus.cc Motor.cc Puck.cc Group.cc ControlLoop.cc Sigmoid.cc WAM.cc Kinematics.cc Dynamics.cc TrajType.cc PulseTraj.cc screenbuf.cc Trajectory.cc ParabolicSegment.cc ParaJointTraj.cc MacJointTraj.cc MacQuinticBlend.cc MacQuinticSegment.cc ServoTraj.cc)

# include_directories (../openmath ../esdcan/lib32 ../rtai)
include_directories (../openmath ../esdcan-pci200/lib32 ../xenomai/include)
