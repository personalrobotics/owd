cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

set (OWD_RT on)
set (CANBUS_TYPE "ESD")
#set (CANBUS_TYPE "PEAK")
set (CMAKE_VERBOSE_MAKEFILE on)

rosbuild_init()
rosbuild_gensrv()

add_subdirectory(openwam)
add_subdirectory(openmath)

include_directories (openwam openmath)
add_definitions("-ggdb3 -DWRIST -DBH8 -DCAN_RECORD")

if (OWD_RT)
set (RT_DEFS "-DOWD_RT")
set (RT_LIBS "native")
else (OWD_RT)
set (RT_DEFS "")
set (RT_LIBS "")
endif (OWD_RT)

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
target_link_libraries(owd openwam openmath lapack blas
			  gfortran ${CANBUS_LIBS} ${RT_LIBS})
rosbuild_add_compile_flags(owd "${CANBUS_DEFS} ${RT_DEFS}")
rosbuild_add_link_flags(owd "${CANBUS_LDFLAGS}")

rosbuild_add_executable(owdbhd owd.cpp openwamdriver.cpp bhd280.cc)
target_link_libraries(owdbhd openwambhd openmath lapack blas
			     gfortran ${CANBUS_LIBS} ${RT_LIBS})
rosbuild_add_compile_flags(owdbhd "-DBH280 ${CANBUS_DEFS} ${RT_DEFS}")
rosbuild_add_link_flags(owdbhd "${CANBUS_LDFLAGS}")

rosbuild_add_executable(canbhd owd.cpp openwamdriver.cpp bhd280.cc)
target_link_libraries(canbhd openwam_canbhd openmath lapack blas
			     gfortran ${CANBUS_LIBS} ${RT_LIBS})
rosbuild_add_compile_flags(canbhd "-DBH280 -DBH280_ONLY ${CANBUS_DEFS} ${RT_DEFS}")
rosbuild_add_link_flags(canbhd "${CANBUS_LDFLAGS}")

rosbuild_add_executable(owdsim owd.cpp openwamdriver.cpp)
target_link_libraries(owdsim openwamsim openmath lapack blas gfortran)
rosbuild_add_compile_flags(owdsim "-DOWDSIM ${RT_DEFS}")

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

# locate Xenomai
if (OWD_RT)
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
   
elseif ("$ENV{XENOMAI_PATH}" STRGREATER "")
   set (XENOMAI_CFLAGS "-I$ENV{XENOMAI_PATH}/include")
   set (XENOMAI_LDFLAGS "-L$ENV{XENOMAI_PATH}/lib -lnative")
   set (XENOMAI_LIBDIR "$ENV{XENOMAI_PATH}/lib")
endif (XENO-CONFIG)

   rosbuild_add_compile_flags(owd ${XENOMAI_CFLAGS})
   rosbuild_add_compile_flags(owdsim ${XENOMAI_CFLAGS})
   rosbuild_add_compile_flags(owdbhd ${XENOMAI_CFLAGS})
   rosbuild_add_compile_flags(canbhd ${XENOMAI_CFLAGS})
   rosbuild_add_link_flags(owd ${XENOMAI_LDFLAGS} -Wl,-rpath,${XENOMAI_LIBDIR})
   rosbuild_add_link_flags(owdbhd ${XENOMAI_LDFLAGS} -Wl,-rpath,${XENOMAI_LIBDIR})
   rosbuild_add_link_flags(canbhd ${XENOMAI_LDFLAGS} -Wl,-rpath,${XENOMAI_LIBDIR})

endif (RT)








