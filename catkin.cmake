cmake_minimum_required(VERSION 2.8.3)

# Depends on the pcan kernel driver, which depends on libpopt-dev

# NOTE: These must be Catkin packages. This exports ${catkin_INCLUDE_DIRS}, etc.
find_package(catkin REQUIRED COMPONENTS
    owd_msgs
    std_msgs
    geometry_msgs
    roscpp
    tf
)

# NOTE: This is used by packages that depend on you. List of dependencies that
# dependencies might need. For Catkin and non-Catkin packages. INCLUDE_DIRS and
# LIBRARIES are exported from this package.
catkin_package(
    #INCLUDE_DIRS include
    #LIBRARIES ${PROJECT_NAME}
    #CATKIN_DEPENDS std_msgs owd_msgs geometry_msgs roscpp tf
    #DEPENDS libblas-dev liblapack-dev
)

include(${PROJECT_SOURCE_DIR}/find_xenomai.cmake)
include(${PROJECT_SOURCE_DIR}/detect_cpu.cmake)

set(RT_BUILD false CACHE BOOL "Real-time scheduler support.")
set(CANBUS_TYPE "PEAK" CACHE STRING "Type of CanBUS driver.")
set_property(CACHE CANBUS_TYPE PROPERTY STRINGS SIMULATED ESD PEAK)

# Detect the CANbus interface.
if (CANBUS_TYPE STREQUAL "ESD")
    message(STATUS "using ESD CANbus driver")
    set(CANBUS_DEFS "-DESD_CAN -I../../esdcan-pci200/lib32")
    set(CANBUS_LIBS "ntcan")
    set(CANBUS_LDFLAGS "-L../../esdcan-pci200/lib32")
elseif (CANBUS_TYPE  STREQUAL "PEAK")
    message(STATUS "using PEAK CANbus driver")
    set(CANBUS_DEFS "-DPEAK_CAN")
    set(CANBUS_LIBS "pcan")
else ()
    message(STATUS "No CANbus type recognized; only building owdsim.")
    set(CANBUS_DEFS "")
    set(CANBUS_LIBS "")
endif ()

include_directories(
    openwam
    openmath
    ${catkin_INCLUDE_DIRS}
)

add_subdirectory(openmath)
add_subdirectory(openwam)

set(OWD_SOURCE 
    owd.cpp
    openwamdriver.cpp
    bhd280.cc
    ft.cc
    tactile.cc
)
set(OWD_LIBS
    wamcan
    openwam
    openmath
    lapack
    blas
    gfortran
    ${CANBUS_LIBS}
)

if (CANBUS_TYPE STREQUAL "ESD" OR CANBUS_TYPE STREQUAL "PEAK")
    add_executable(owd ${OWD_SOURCE})
    target_link_libraries(owd ${OWD_LIBS})
    set_target_properties(owd PROPERTIES
        COMPILE_FLAGS "${CANBUS_DEFS}"
        LINK_FLAGS "${CANBUS_LDFLAGS}"
    )

    add_executable(canbhd ${OWD_SOURCE})
    target_link_libraries(canbhd ${OWD_LIBS})
    set_target_properties(canbhd PROPERTIES
        COMPILE_FLAGS "${CANBUS_DEFS} -DBH280 -DBH280_ONLY"
        LINK_FLAGS "${CANBUS_LDFLAGS}"
    )
endif ()

#rosbuild_add_executable(owdsim owd.cpp openwamdriver.cpp)
#target_link_libraries(owdsim openwamsim openmath lapack blas gfortran)
#rosbuild_add_compile_flags(owdsim "-DOWDSIM")
#
#if (CANBUS_TYPE STREQUAL "ESD" OR CANBUS_TYPE STREQUAL "PEAK")
#rosbuild_add_executable(owd_test test.cpp)
#rosbuild_add_executable(synctest synctest.cpp)
#
#rosbuild_add_executable(puck_update puck_update.cpp)
#target_link_libraries(puck_update wamcan openwam ${CANBUS_LIBS})
#rosbuild_add_compile_flags(puck_update "${CANBUS_DEFS}")
#rosbuild_add_link_flags(puck_update "${CANBUS_LDFLAGS}")
#
#rosbuild_add_executable(puck_defaults puck_defaults.cpp)
#target_link_libraries(puck_defaults wamcan openwam ${CANBUS_LIBS})
#rosbuild_add_compile_flags(puck_defaults "${CANBUS_DEFS}")
#rosbuild_add_link_flags(puck_defaults "${CANBUS_LDFLAGS}")
#
#rosbuild_add_executable(puck_getprops puck_getprops.cpp)
#target_link_libraries(puck_getprops wamcan openwam ${CANBUS_LIBS})
#rosbuild_add_compile_flags(puck_getprops "${CANBUS_DEFS}")
#rosbuild_add_link_flags(puck_getprops "${CANBUS_LDFLAGS}")
#
#rosbuild_add_executable(puck_find_mofst puck_find_mofst.cpp)
#target_link_libraries(puck_find_mofst wamcan openwam ${CANBUS_LIBS})
#rosbuild_add_compile_flags(puck_find_mofst "${CANBUS_DEFS}")
#rosbuild_add_link_flags(puck_find_mofst "${CANBUS_LDFLAGS}")
#
#rosbuild_add_executable(puck_setprop puck_setprop.cpp)
#target_link_libraries(puck_setprop wamcan openwam ${CANBUS_LIBS})
#rosbuild_add_compile_flags(puck_setprop "${CANBUS_DEFS}")
#rosbuild_add_link_flags(puck_setprop "${CANBUS_LDFLAGS}")
#endif (CANBUS_TYPE STREQUAL "ESD" OR CANBUS_TYPE STREQUAL "PEAK")
#
#rosbuild_add_executable(owd_traj_timer owd_traj_timer.cpp)
#if (CANBUS_TYPE STREQUAL "ESD" OR CANBUS_TYPE STREQUAL "PEAK")
#  target_link_libraries(owd_traj_timer openwam)
#else (CANBUS_TYPE STREQUAL "ESD" OR CANBUS_TYPE STREQUAL "PEAK")
#  target_link_libraries(owd_traj_timer openwamsim)
#endif (CANBUS_TYPE STREQUAL "ESD" OR CANBUS_TYPE STREQUAL "PEAK")
#
#if (CANBUS_TYPE STREQUAL "ESD" OR CANBUS_TYPE STREQUAL "PEAK")
#if (RT_BUILD)
#  rosbuild_add_executable(owdrt owd.cpp openwamdriver.cpp bhd280.cc
#                                ft.cc tactile.cc)
#  target_link_libraries(owdrt wamcanrt openwam openmath lapack blas
#                              gfortran ${CANBUS_LIBS} ${RT_LIBS})
#  rosbuild_add_compile_flags(owdrt "${CANBUS_DEFS} ${RT_DEFS}")
#  rosbuild_add_link_flags(owdrt "${CANBUS_LDFLAGS}")
#
#  rosbuild_add_executable(canbhdrt owd.cpp openwamdriver.cpp bhd280.cc
#                                   ft.cc tactile.cc)
#  target_link_libraries(canbhdrt bhdcanrt openwam openmath lapack blas
#      	                         gfortran ${CANBUS_LIBS} ${RT_LIBS})
#  rosbuild_add_compile_flags(canbhdrt "-DBH280 -DBH280_ONLY ${CANBUS_DEFS} ${RT_DEFS}")
#  rosbuild_add_link_flags(canbhdrt "${CANBUS_LDFLAGS}")
#endif (RT_BUILD)
#endif (CANBUS_TYPE STREQUAL "ESD" OR CANBUS_TYPE STREQUAL "PEAK")

#include_directories(openwam openmath ${catkin_INCLUDE_DIRS})
#link_libraries(${catkin_LIBRARIES})
