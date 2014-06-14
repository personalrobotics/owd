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

include_directories(${catkin_INCLUDE_DIRS})

add_subdirectory(openmath)
add_subdirectory(openwam)
#add_subdirectory(${PROJECT_SOURCE_DIR}/openmath)

#include_directories(openwam openmath ${catkin_INCLUDE_DIRS})
#link_libraries(${catkin_LIBRARIES})
