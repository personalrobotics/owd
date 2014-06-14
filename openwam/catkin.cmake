cmake_minimum_required(VERSION 2.8.3)

if (CANBUS_TYPE STREQUAL "ESD" OR CANBUS_TYPE STREQUAL "PEAK")
    add_library(openwam
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
endif ()

add_library(openwamsim
    CANbus_sim.cc
    Joint.cc
    Motor.cc
    Puck.cc
    Group.cc
    ControlLoop.cc
    Sigmoid.cc
    WAM.cc
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
    Plugin.cc
    butterworth_solver.c
    MultiSync.cc
    BinaryData.cc
    JSController.cc
)
target_link_libraries(openwamsim ${CANBUS_LIBS})
set_target_properties(openwamsim PROPERTIES
    COMPILE_FLAGS "${CANBUS_DEFS} -DOWDSIM"
)

if (CANBUS_TYPE STREQUAL "ESD" OR CANBUS_TYPE STREQUAL "PEAK")
    add_library(wamcan
        ControlLoop.cc
        CANbus.cc
        WAM.cc
        Plugin.cc
    )
    target_link_libraries(wamcan ${CANBUS_LIBS})
    set_target_properties(wamcan PROPERTIES COMPILE_FLAGS ${CANBUS_DEFS})

    add_library(bhdcan
        ControlLoop.cc
        CANbus.cc
        WAM.cc
        Plugin.cc
    )
    target_link_libraries(bhdcan ${CANBUS_LIBS})
    set_target_properties(bhdcan PROPERTIES
        COMPILE_FLAGS "${CANBUS_DEFS} -DBH280_ONLY"
    )

    if (RT_BUILD)
        add_library(wamcanrt
            ControlLoop.cc
            CANbus.cc
            WAM.cc
            Plugin.cc
        )
        target_link_libraries(wamcanrt ${CANBUS_LIBS} ${RT_LIBS})
        set_target_properties(wamcanrt PROPERTIES
            COMPILE_FLAGS "${CANBUS_DEFS} ${RT_DEFS}"
        )

        add_library(bhdcanrt
            ControlLoop.cc
            CANbus.cc
            WAM.cc
            Plugin.cc
        )
        target_link_libraries(bhdcanrt ${CANBUS_LIBS} ${RT_LIBS})
        set_target_properties(bhdcanrt PROPERTIES
            COMPILE_FLAGS "${CANBUS_DEFS} ${RT_DEFS} -DBH280_ONLY"
        )
    endif ()
endif ()

#add_executable(MacTrajTest MacTrajTest.cc)
#target_link_libraries(MacTrajTest openwamsim)
