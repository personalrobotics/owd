cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

set(ROS_BUILD_TYPE Debug)
rosbuild_init()
rosbuild_genmsg()
rosbuild_gensrv()

rosbuild_add_library(gfe_owd_plugin GfePlugin.cpp ApplyForceTraj.cpp DoorTraj.cpp JacobianTest.cpp WSTraj.cpp ForceController.cpp Follow.cpp HelixPlugin.cpp MoveDirection.cpp Vibration.cpp)

