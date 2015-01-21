cmake_minimum_required(VERSION 2.8.3)

set(OPENWAM_SOURCE
    Joint.cc
    Motor.cc
    Puck.cc
    Group.cc
    Sigmoid.cc
    Kinematics.cc
    Dynamics.cc
    TrajType.cc
    PulseTraj.cc
    Trajectory.cc
    ParabolicSegment.cc
    ParaJointTraj.cc
    MacJointTraj.cc
    MacQuinticBlend.cc
    MacQuinticSegment.cc
    ServoTraj.cc
    StepTraj.cc
    ConstrainedForceTrajectory.cc 
    butterworth_solver.c
    MultiSync.cc
    BinaryData.cc
    JSController.cc
)
set(OPENWAM_IMPL_SOURCE
    Plugin.cc
    WAM.cc
    ControlLoop.cc
)
set(OPENWAM_TARGETS openwamsim)

add_definitions("-O0 -ggdb3 -DHEAD -DRT_STATS")

if (CANBUS_TYPE STREQUAL "ESD" OR CANBUS_TYPE STREQUAL "PEAK")
    list(APPEND OPENWAM_TARGETS openwam wamcan bhdcan)

    add_library(openwam STATIC ${OPENWAM_SOURCE})
    add_dependencies(openwam owd_msgs_generate_messages_cpp)

    add_library(wamcan STATIC ${OPENWAM_IMPL_SOURCE} CANbus.cc)
    add_dependencies(wamcan owd_msgs_generate_messages_cpp)
    target_link_libraries(wamcan ${CANBUS_LIBS})
    set_target_properties(wamcan PROPERTIES
        COMPILE_FLAGS "${CANBUS_DEFS}"
        LINK_FLAGS "${CANBUS_LDFLAGS}"
    )

    add_library(bhdcan STATIC ${OPENWAM_IMPL_SOURCE} CANbus.cc)
    add_dependencies(bhdcan owd_msgs_generate_messages_cpp)
    target_link_libraries(bhdcan ${CANBUS_LIBS})
    set_target_properties(bhdcan PROPERTIES
        COMPILE_FLAGS "${CANBUS_DEFS} -DBH280_ONLY"
        LINK_FLAGS "${CANBUS_LDFLAGS}"
    )

    if (RT_BUILD)
        list(APPEND OPENWAM_TARGETS wamcanrt bhdcanrt)

        add_library(wamcanrt STATIC ${OPENWAM_IMPL_SOURCE} CANbus.cc)
        add_dependencies(wamcanrt owd_msgs_generate_messages_cpp)
        target_link_libraries(wamcanrt ${CANBUS_LIBS} ${RT_LIBS})
        set_target_properties(wamcanrt PROPERTIES
            COMPILE_FLAGS "${CANBUS_DEFS} ${RT_DEFS}"
            LINK_FLAGS "${CANBUS_LDFLAGS}"
        )

        add_library(bhdcanrt STATIC ${OPENWAM_IMPL_SOURCE} CANbus.cc)
        add_dependencies(bhdcanrt owd_msgs_generate_messages_cpp)
        target_link_libraries(bhdcanrt ${CANBUS_LIBS} ${RT_LIBS})
        set_target_properties(bhdcanrt PROPERTIES
            COMPILE_FLAGS "${CANBUS_DEFS} ${RT_DEFS} -DBH280_ONLY"
            LINK_FLAGS "${CANBUS_LDFLAGS}"
        )
    endif ()
endif ()

add_library(openwamsim STATIC ${OPENWAM_SOURCE} ${OPENWAM_IMPL_SOURCE}
                              CANbus_sim.cc)
add_dependencies(openwamsim owd_msgs_generate_messages_cpp)
target_link_libraries(openwamsim ${CANBUS_LIBS})
set_target_properties(openwamsim PROPERTIES
    COMPILE_FLAGS "${CANBUS_DEFS} -DOWDSIM"
    LINK_FLAGS "${CANBUS_LDFLAGS}"
)

# TODO: Something needs to link with openmath.
#add_executable(MacTrajTest MacTrajTest.cc)
#target_link_libraries(MacTrajTest openwamsim)

install(TARGETS ${OPENWAM_TARGETS}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

