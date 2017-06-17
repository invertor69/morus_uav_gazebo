cmake_minimum_required(VERSION 2.8.3)
project(morus_control)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    rosgraph_msgs
    genmsg
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

catkin_package(
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

set(mass_ctl_attitude_mpc_node_SRCS
        src/nodes/mass_ctl_attitude_mpc_node.cpp
        # mass_ctl_attitude_mpc depends on steady_state_calculation
        src/mass_ctl_attitude_mpc.cpp
        src/steady_state_calculation.cpp)

add_executable(mass_ctl_attitude_mpc_node ${mass_ctl_attitude_mpc_node_SRCS})
target_link_libraries(mass_ctl_attitude_mpc_node ${catkin_LIBRARIES})
