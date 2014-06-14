cmake_minimum_required(VERSION 2.8.3)
project(owd)

if (DEFINED CATKIN_TOPLEVEL OR (NOT ("$ENV{ROS_DISTRO}" STREQUAL "fuerte")))
    include(catkin.cmake)
else ()
    include(rosbuild.cmake)
endif ()
