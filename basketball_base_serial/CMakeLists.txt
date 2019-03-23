cmake_minimum_required(VERSION 2.8.3)
project(basketball_base_serial)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
set(CXXFLAGS -DEBUG)
find_package(catkin REQUIRED COMPONENTS
  basketball_msgs
  roscpp
  rospy
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES basketball_base_serial
  CATKIN_DEPENDS roscpp rospy std_msgs basketball_msgs
  DEPENDS system_lib
)

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(robot_base_serial src/mainNode.cpp src/SerialNode.cpp src/SerialPort.cpp)
target_link_libraries(robot_base_serial ${catkin_LIBRARIES})
add_dependencies(robot_base_serial basketball_msgs_gencpp)
