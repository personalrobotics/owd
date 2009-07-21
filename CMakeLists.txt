cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rospack(owd)
genmsg()
gensrv()

set (CMAKE_VERBOSE_MAKEFILE on)
add_subdirectory(openwam)
add_subdirectory(openmath)

if( DEFINED ENV{XENOMAI_PATH} )
   message(STATUS "Using $ENV{XENOMAI_PATH} for xenomai build")
   link_directories( "$ENV{XENOMAI_PATH}/lib" )
   include_directories( "$ENV{XENOMAI_PATH}/include" )
else()
   message(STATUS "Using /usr/xenomai for xenomai build")
   link_directories(/usr/xenomai/lib)
   include_directories(/usr/xenomai/include)
endif()

link_directories (../lapack ../esdcan-pci200/lib32)
include_directories (openwam openmath esdcan-pci200/lib32)

rospack_add_executable(owd owd.cpp openwamdriver.cpp)
target_link_libraries(owd openwam openmath lapack blas ntcan native gfortran)
add_definitions("-ggdb3 -DWRIST -DBH8")

include(detect_cpu.cmake)

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

# rospack_add_executable(cantest cantest.cc)


