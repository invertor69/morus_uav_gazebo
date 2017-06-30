cmake_minimum_required(VERSION 2.8.3)
project(morus_control)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    geometry_msgs
    rosgraph_msgs
    genmsg
    control_msgs
    sensor_msgs
    morus_msgs
)

# check c++11 / c++0x
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "-std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "-std=c++0x")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

find_package(cmake_modules REQUIRED)

# enable the use of Eigen
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS} )

catkin_package(
    INCLUDE_DIRS include lib src config
    LIBRARIES morus_control
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  lib
)

#link_directories(lib)

install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

#############
# LIBRARIES #
#############

# convex solver - CVXGEN
add_library(cvxgen_solver_moving_masses
        lib/ldl.c
        lib/solver.c
        lib/matrix_support.c
        lib/util.c
        lib/variables_definition.c
)

# general algorithm for executing
add_library(mass_ctl_attitude_mpc
        src/mass_ctl_attitude_mpc.cpp
        src/steady_state_calculation.cpp
        src/KFDisturbanceObserver.cpp
        lib/solver.c
)

add_dependencies(mass_ctl_attitude_mpc
        cvxgen_solver_moving_masses
)

target_link_libraries(mass_ctl_attitude_mpc
        cvxgen_solver_moving_masses
        ${catkin_LIBRARIES}
)

# target_include_directories(cvxgen_solver_moving_masses PUBLIC lib)

#set(mass_ctl_attitude_mpc_node_SRCS
#        src/nodes/mass_ctl_attitude_mpc_node.cpp
#        # mass_ctl_attitude_mpc depends on steady_state_calculation
#        src/mass_ctl_attitude_mpc.cpp
#        src/steady_state_calculation.cpp src/KFDisturbanceObserver.cpp include/morus_control/KFDisturbanceObserver.h
#        include/morus_control/mass_ctl_attitude_mpc.h
#        include/morus_control/mass_ctl_attitude_mpc_node.h
#        include/morus_control/steady_state_calculation.h
#        )

add_executable(mass_ctl_attitude_mpc_node
        src/nodes/mass_ctl_attitude_mpc_node.cpp)

target_link_libraries(mass_ctl_attitude_mpc_node
        ${catkin_LIBRARIES}
        mass_ctl_attitude_mpc
        cvxgen_solver_moving_masses)
