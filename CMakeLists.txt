cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rospack(owd)
genmsg()
gensrv()

set (CMAKE_VERBOSE_MAKEFILE on)
add_subdirectory(openwam)
add_subdirectory(openmath)

link_directories (../lapack ../esdcan-pci200/lib32 ../xenomai/lib)
include_directories (openwam openmath esdcan-pci200/lib32 xenomai/include)

rospack_add_executable(owd owd.cpp openwamdriver.cpp)
target_link_libraries(owd openwam openmath lapack blas ntcan native gfortran)
add_definitions("-march=c3-2 -ggdb3 -DWRIST -DBH8")
#add_definitions("-march=c3-2 -ggdb3 -DFAKE7 -DBH8")

# rospack_add_executable(cantest cantest.cc)


