cmake_minimum_required(VERSION 2.8.3)

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

include_directories(openwam openmath ${catkin_INCLUDE_DIRS})
link_libraries(${catkin_LIBRARIES})

# TODO: Why are we compiling with -O0?
add_definitions("-O0 -ggdb3 -DWRIST -DBH8 -DRT_STATS")

# Choose the CAN driver.
set(CANBUS_TYPE "PEAK")

if (CANBUS_TYPE STREQUAL "ESD")
    message(STATUS "using ESD CANbus driver")
    set (CANBUS_DEFS "-DESD_CAN -I../esdcan-pci200/lib32")
    set (CANBUS_LIBS "ntcan")
    set (CANBUS_LDFLAGS "-L../esdcan-pci200/lib32")
elseif (CANBUS_TYPE  STREQUAL "PEAK")
    message(STATUS "using PEAK CANbus driver")
    set (CANBUS_DEFS "-DPEAK_CAN")
    set (CANBUS_LIBS "pcan")
else ()
    message(STATUS "No CANbus type recognized; only building owdsim.")
    set (CANBUS_DEFS "")
    set (CANBUS_LIBS "")
endif ()

# Enable CPU-specific optimizations.
if (DEFINED ENV{OWD_MARCH_FLAGS})
    message(STATUS "Using mtune flags set in environment: $ENV{OWD_MARCH_FLAGS}")
    add_definitions( "$ENV{OWD_MARCH_FLAGS}" )
elseif (DEFINED ENV{ROS_MARCH_FLAGS})
    message(STATUS "Using mtune flags set in environment: $ENV{ROS_MARCH_FLAGS}")
    add_definitions( "$ENV{ROS_MARCH_FLAGS}" )
elseif (VENDOR_ID STREQUAL "GenuineIntel" AND CPU_FAMILY EQUAL 6 AND MODEL EQUAL 28)
    message(STATUS "Building for Intel Atom")
    add_definitions("-march=core2 -mtune=native -mmmx -msse2 -msse3 -mfpmath=sse")
elseif (VENDOR_ID STREQUAL "CentaurHauls")
    message(STATUS "Building for VIA - Original Barrett WAM PC")
    add_definitions("-march=c3-2")
endif ()

# Build the OpenMath library.
add_library(openmath STATIC
    openmath/Inertia.cc
    openmath/SE3.cc
    openmath/SO3.cc
)

# Build the OpenWAM library.
set(OPENWAM_SOURCE
    openwam/Joint.cc
    openwam/Motor.cc
    openwam/Puck.cc
    openwam/Group.cc
    openwam/Sigmoid.cc
    openwam/Kinematics.cc
    openwam/Dynamics.cc
    openwam/TrajType.cc
    openwam/PulseTraj.cc
    openwam/Trajectory.cc
    openwam/ParabolicSegment.cc
    openwam/ParaJointTraj.cc
    openwam/MacJointTraj.cc
    openwam/MacQuinticBlend.cc
    openwam/MacQuinticSegment.cc
    openwam/ServoTraj.cc
    openwam/StepTraj.cc
    openwam/ConstrainedForceTrajectory.cc
    openwam/butterworth_solver.c
    openwam/MultiSync.cc
    openwam/BinaryData.cc
    openwam/JSController.cc
)
set(OPENWAM_CAN_SOURCE 
    openwam/ControlLoop.cc
    openwam/CANbus.cc
    openwam/WAM.cc
    openwam/Plugin.cc
)

if (CANBUS_TYPE STREQUAL "ESD" OR CANBUS_TYPE STREQUAL "PEAK")
    add_library(openwam ${OPENWAM_SOURCE})
    add_dependencies(openwam ${catkin_EXPORTED_TARGETS})
endif ()

# Build OpenWAM utility programs.
if (CANBUS_TYPE STREQUAL "ESD" OR CANBUS_TYPE STREQUAL "PEAK")
    add_library(wamcan ${OPENWAM_CAN_SOURCE})
    add_dependencies(wamcan ${catkin_EXPORTED_TARGETS})
    set_target_properties(wamcan PROPERTIES
        COMPILE_FLAGS "${CANBUS_DEFS}"
        LINK_FLAGS "${CANBUS_LIBS}"
    )

    add_library(bhdcan ${OPENWAM_CAN_SOURCE})
    add_dependencies(bhdcan ${catkin_EXPORTED_TARGETS})
    set_target_properties(bhdcan PROPERTIES
        COMPILE_FLAGS "${CANBUS_DEFS} -DBH280_ONLY"
        LINK_FLAGS "${CANBUS_LIBS}"
    )

    if (RT_BUILD)
        add_library(wamcanrt ${OPENWAM_CAN_SOURCE})
        add_dependencies(wamcanrt ${catkin_EXPORTED_TARGETS})
        set_target_properties(wamcanrt PROPERTIES
            COMPILE_FLAGS "${CANBUS_DEFS} ${RT_DEFS}"
            LINK_FLAGS "${CANBUS_LIBS} ${RT_LIBS}"
        )

        add_library(bhdcanrt ${OPENWAM_CAN_SOURCE})
        add_dependencies(bhdcanrt ${catkin_EXPORTED_TARGETS})
        set_target_properties(bhdcanrt PROPERTIES
            COMPILE_FLAGS "${CANBUS_DEFS} ${RT_DEFS} -DBH280_ONLY"
            LINK_FLAGS "${CANBUS_LIBS} ${RT_LIBS}"
        )
    endif ()
endif ()

add_library(openwamsim
    ${OPENWAM_SOURCE}
    openwam/CANbus_sim.cc
    openwam/ControlLoop.cc
    openwam/WAM.cc
    openwam/Plugin.cc
)
add_dependencies(openwamsim ${catkin_EXPORTED_TARGETS})
set_target_properties(openwamsim PROPERTIES COMPILE_FLAGS "-DOWDSIM")

add_executable(MacTrajTest openwam/MacTrajTest.cc)
add_dependencies(MacTrajTest ${catkin_EXPORTED_TARGETS})
target_link_libraries(MacTrajTest openwamsim)
