cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

add_library(openmath STATIC Inertia.cc SE3.cc SO3.cc ZYX.cc)

add_definitions("-march=c3-2 -ggdb3")

