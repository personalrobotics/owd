cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
include(find_xenomai.cmake)

#set (CANBUS_TYPE "ESD")
set (CANBUS_TYPE "PEAK")
set (CMAKE_VERBOSE_MAKEFILE on)

rosbuild_init()
rosbuild_gensrv()

add_subdirectory(openwam)
add_subdirectory(openmath)

include_directories (openwam openmath)
add_definitions("-ggdb3 -DWRIST -DBH8 -DCAN_RECORD")

if (CANBUS_TYPE STREQUAL "ESD")
message(STATUS "using ESD CANbus driver")
set (CANBUS_DEFS "-DESD_CAN -I../esdcan-pci200/lib32")
set (CANBUS_LIBS "ntcan")
set (CANBUS_LDFLAGS "-L../esdcan-pci200/lib32")
elseif (CANBUS_TYPE  STREQUAL "PEAK")
message(STATUS "using PEAK CANbus driver")
set (CANBUS_DEFS "-DPEAK_CAN")
set (CANBUS_LIBS "pcan")
else (CANBUS_TYPE STREQUAL "ESD")
message(SEND_ERROR "No CANbus type recognized")
set (CANBUS_DEFS "")
set (CANBUS_LIBS "")
endif (CANBUS_TYPE STREQUAL "ESD")

rosbuild_add_executable(owd owd.cpp openwamdriver.cpp)
target_link_libraries(owd wamcan openwam openmath lapack blas
			  gfortran ${CANBUS_LIBS})
rosbuild_add_compile_flags(owd "${CANBUS_DEFS}")
rosbuild_add_link_flags(owd "${CANBUS_LDFLAGS}")

rosbuild_add_executable(owdrt owd.cpp openwamdriver.cpp)
target_link_libraries(owdrt wamcanrt openwam openmath lapack blas
			  gfortran ${CANBUS_LIBS} ${RT_LIBS})
rosbuild_add_compile_flags(owdrt "${CANBUS_DEFS} ${RT_DEFS}")
rosbuild_add_link_flags(owdrt "${CANBUS_LDFLAGS}")

rosbuild_add_executable(owdbhd owd.cpp openwamdriver.cpp bhd280.cc)
target_link_libraries(owdbhd wambhdcan openwam openmath lapack blas
			     gfortran ${CANBUS_LIBS})
rosbuild_add_compile_flags(owdbhd "-DBH280 ${CANBUS_DEFS}")
rosbuild_add_link_flags(owdbhd "${CANBUS_LDFLAGS}")

rosbuild_add_executable(owdbhdrt owd.cpp openwamdriver.cpp bhd280.cc)
target_link_libraries(owdbhdrt wambhdcanrt openwam openmath lapack blas
			     gfortran ${CANBUS_LIBS} ${RT_LIBS})
rosbuild_add_compile_flags(owdbhdrt "-DBH280 ${CANBUS_DEFS} ${RT_DEFS}")
rosbuild_add_link_flags(owdbhdrt "${CANBUS_LDFLAGS}")

rosbuild_add_executable(canbhd owd.cpp openwamdriver.cpp bhd280.cc)
target_link_libraries(canbhd bhdcan openwam openmath lapack blas
			     gfortran ${CANBUS_LIBS})
rosbuild_add_compile_flags(canbhd "-DBH280 -DBH280_ONLY ${CANBUS_DEFS}")
rosbuild_add_link_flags(canbhd "${CANBUS_LDFLAGS}")

rosbuild_add_executable(canbhdrt owd.cpp openwamdriver.cpp bhd280.cc)
target_link_libraries(canbhdrt bhdcanrt openwam openmath lapack blas
			     gfortran ${CANBUS_LIBS} ${RT_LIBS})
rosbuild_add_compile_flags(canbhdrt "-DBH280 -DBH280_ONLY ${CANBUS_DEFS} ${RT_DEFS}")
rosbuild_add_link_flags(canbhdrt "${CANBUS_LDFLAGS}")

rosbuild_add_executable(owdsim owd.cpp openwamdriver.cpp)
target_link_libraries(owdsim openwamsim openmath lapack blas gfortran)
rosbuild_add_compile_flags(owdsim "-DOWDSIM")

rosbuild_add_executable(owd_test test.cpp)

include(detect_cpu.cmake)

if (DEFINED ENV{OWD_MARCH_FLAGS})
    message(STATUS "Using mtune flags set in environment: $ENV{OWD_MARCH_FLAGS}")
    add_definitions( "$ENV{OWD_MARCH_FLAGS}" )
elseif (VENDOR_ID STREQUAL "GenuineIntel" AND CPU_FAMILY EQUAL 6 AND MODEL EQUAL 28)
    message(STATUS "Building for Intel Atom")
    add_definitions("-march=core2 -mtune=native -mmmx -msse2 -msse3 -mfpmath=sse")
elseif (VENDOR_ID STREQUAL "CentaurHauls")
    message(STATUS "Building for VIA - Original Barrett WAM PC")
    add_definitions("-march=c3-2")
endif  (DEFINED ENV{OWD_MARCH_FLAGS})






