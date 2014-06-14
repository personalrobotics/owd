cmake_minimum_required(VERSION 2.8.3)

find_package(catkin REQUIRED COMPONENTS
    owd_msgs
    std_msgs
    geometry_msgs
    roscpp
    tf
)

# TODO: Populate this.
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
link_libraries(${catkin_LIBRARIES})

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
    openwam
    openmath
    lapack
    blas
    gfortran
)
set(OWD_TARGETS owdsim)

add_executable(owdsim owd.cpp openwamdriver.cpp)
target_link_libraries(owdsim openwamsim ${OWD_LIBS})
set_target_properties(owdsim PROPERTIES COMPILE_FLAGS "-DOWDSIM")

if (CANBUS_TYPE STREQUAL "ESD" OR CANBUS_TYPE STREQUAL "PEAK")
    list(APPEND OWD_TARGETS owd canbhd)

    add_executable(owd ${OWD_SOURCE})
    target_link_libraries(owd wamcan ${OWD_LIBS} ${CANBUS_LIBS})
    set_target_properties(owd PROPERTIES
        COMPILE_FLAGS "${CANBUS_DEFS}"
        LINK_FLAGS "${CANBUS_LDFLAGS}"
    )

    add_executable(canbhd ${OWD_SOURCE})
    target_link_libraries(canbhd wamcan ${OWD_LIBS} ${CANBUS_LIBS})
    set_target_properties(canbhd PROPERTIES
        COMPILE_FLAGS "${CANBUS_DEFS} -DBH280 -DBH280_ONLY"
        LINK_FLAGS "${CANBUS_LDFLAGS}"
    )
endif ()

# TODO: Also build the large collection of utilities.

install(TARGETS ${OWD_TARGETS}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

