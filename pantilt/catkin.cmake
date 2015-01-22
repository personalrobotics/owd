cmake_minimum_required(VERSION 2.8.3)
find_package(catkin REQUIRED COMPONENTS
    owd_msgs
    std_msgs
    geometry_msgs
    rosconsole
    roscpp
    tf
)

# TODO: Populate this.
catkin_package(
    #INCLUDE_DIRS include
    #LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS std_msgs owd_msgs geometry_msgs roscpp tf
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

add_definitions("-O0 -ggdb3 -DHEAD -DBH8")

set(OWD_SOURCE 
    owd.cpp
    openwamdriver.cpp
    bhd280.cc
    ft.cc
    tactile.cc
)
set(OWD_LIBS
    ${PROJECT_NAME}_openwam
    ${PROJECT_NAME}_openmath
    lapack
    blas
    # NOTE: I have no idea why this depends on gfortran. Maybe we can remove
    # this dependency?
    gfortran
    # NOTE: Linking fails with the error message "undefined reference to
    # dlclose@@GLIBC_2.2.5" if this is omitted. I have no idea why this is
    # necessary, but I found the solution here:
    #   http://askubuntu.com/q/334884
    dl
)
set(OWD_TARGETS ${PROJECT_NAME}_owdsim)

add_executable(${PROJECT_NAME}_owdsim owd.cpp openwamdriver.cpp)
add_dependencies(${PROJECT_NAME}_owdsim owd_msgs_generate_messages_cpp)
target_link_libraries(${PROJECT_NAME}_owdsim ${PROJECT_NAME}_openwamsim ${OWD_LIBS})
set_target_properties(${PROJECT_NAME}_owdsim PROPERTIES
    OUTPUT_NAME owdsim
    COMPILE_FLAGS "-DOWDSIM"
)

if (CANBUS_TYPE STREQUAL "ESD" OR CANBUS_TYPE STREQUAL "PEAK")
    list(APPEND OWD_TARGETS ${PROJECT_NAME}_owd)

    add_executable(${PROJECT_NAME}_owd ${OWD_SOURCE})
    add_dependencies(${PROJECT_NAME}_owd owd_msgs_generate_messages_cpp)
    target_link_libraries(${PROJECT_NAME}_owd ${PROJECT_NAME}_wamcan ${OWD_LIBS} ${CANBUS_LIBS})
    set_target_properties(${PROJECT_NAME}_owd PROPERTIES
        OUTPUT_NAME owd
        COMPILE_FLAGS "${CANBUS_DEFS}"
        LINK_FLAGS "${CANBUS_LDFLAGS}"
    )

    if (RT_BUILD)
        list(APPEND OWD_TARGETS ${PROJECT_NAME}_owdrt)

        add_executable(${PROJECT_NAME}_owdrt ${OWD_SOURCE})
        add_dependencies(${PROJECT_NAME}_owdrt owd_msgs_generate_messages_cpp)
        target_link_libraries(${PROJECT_NAME}_owdrt ${PROJECT_NAME}_wamcanrt ${OWD_LIBS} ${CANBUS_LIBS} ${RT_LIBS})
        set_target_properties(${PROJECT_NAME}_owdrt PROPERTIES
            OUTPUT_NAME owdrt
            COMPILE_FLAGS "${CANBUS_DEFS} ${RT_DEFS}"
            LINK_FLAGS "${CANBUS_LDFLAGS}"
        )
    endif ()
endif()

# TODO: Also build the large collection of utilities.

install(TARGETS ${OWD_TARGETS}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

