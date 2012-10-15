cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

set(ROS_BUILD_TYPE Debug)
rosbuild_init()
rosbuild_genmsg()
rosbuild_gensrv()

rosbuild_add_library(owd_hybrid_plugin HybridPlugin.cpp ApplyForceTraj.cpp DoorTraj.cpp WSTraj.cpp ForceController.cpp Vibration.cpp InsertKeyTraj.cpp FTCheck.cpp Servo2Traj.cpp Teleop.cpp)

