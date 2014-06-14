cmake_minimum_required(VERSION 2.8.3)

add_definitions("-ggdb3")
add_library(openmath STATIC
    Inertia.cc
    SE3.cc
    SO3.cc
)

install(TARGETS openmath
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

