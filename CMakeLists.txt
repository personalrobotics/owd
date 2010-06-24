cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rosbuild_init()
rosbuild_gensrv()

set (CMAKE_VERBOSE_MAKEFILE on)
add_subdirectory(openwam)
add_subdirectory(openmath)

include_directories (openwam openmath)
add_definitions("-ggdb3 -DWRIST -DBH8")

rosbuild_add_executable(owd owd.cpp openwamdriver.cpp)
# target_link_libraries(owd openwam openmath lapack blas native gfortran pcan)
target_link_libraries(owd openwam openmath lapack blas native gfortran pcan)
rosbuild_add_compile_flags(owd "-DPEAK_CAN")

rosbuild_add_executable(owdbhd owd.cpp openwamdriver.cpp bhd280.cc)
target_link_libraries(owdbhd openwambhd openmath lapack blas native gfortran pcan)
rosbuild_add_compile_flags(owdbhd "-DBH280 -DPEAK_CAN")

rosbuild_add_executable(owdsim owd.cpp openwamdriver.cpp)
target_link_libraries(owdsim openwamsim openmath lapack blas gfortran)
rosbuild_add_compile_flags(owdsim "-DOWDSIM")

rosbuild_add_executable(owd_test test.cpp)

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

# locate Xenomai
find_program(XENO-CONFIG NAMES xeno-config PATHS $ENV{XENOMAI_PATH} /usr/bin /usr/xenomai/bin)
if (XENO-CONFIG)
   execute_process(COMMAND ${XENO-CONFIG} --skin native --cflags
                   RESULT_VARIABLE XENOMAI_NOTFOUND
                   OUTPUT_VARIABLE XENOMAI_CFLAGS
                   OUTPUT_STRIP_TRAILING_WHITESPACE)
   if (XENOMAI_NOTFOUND)
       message (SEND_ERROR "Could not find Xenomai include files (command xeno-config --skin native --cflags failed)\n")
   endif(XENOMAI_NOTFOUND)
   execute_process(COMMAND ${XENO-CONFIG} --skin native --ldflags
                   RESULT_VARIABLE XENOMAI_NOTFOUND
                   OUTPUT_VARIABLE XENOMAI_LDFLAGS
                   OUTPUT_STRIP_TRAILING_WHITESPACE)
   if (XENOMAI_NOTFOUND)
      message (SEND_ERROR "Could not find Xenomai libs (command xeno-config --skin native --ldflags failed)\n")
   endif (XENOMAI_NOTFOUND)
   execute_process(COMMAND ${XENO-CONFIG} --skin native --libdir
                   RESULT_VARIABLE XENOMAI_NOTFOUND
                   OUTPUT_VARIABLE XENOMAI_LIBDIR
                   OUTPUT_STRIP_TRAILING_WHITESPACE)
   if (XENOMAI_NOTFOUND)
      message (SEND_ERROR "Could not find Xenomai lib dir (command xeno-config --skin native --libdir failed)\n")
   endif (XENOMAI_NOTFOUND)
   
   rosbuild_add_compile_flags(owd ${XENOMAI_CFLAGS})
   rosbuild_add_compile_flags(owdsim ${XENOMAI_CFLAGS})
   rosbuild_add_compile_flags(owdbhd ${XENOMAI_CFLAGS})
   rosbuild_add_link_flags(owd ${XENOMAI_LDFLAGS} -Wl,-rpath,${XENOMAI_LIBDIR})
   rosbuild_add_link_flags(owdbhd ${XENOMAI_LDFLAGS} -Wl,-rpath,${XENOMAI_LIBDIR})
endif(XENO-CONFIG)
