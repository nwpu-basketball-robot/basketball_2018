cmake_minimum_required(VERSION 2.8.3)
project(basketball_shoot_srv)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  basketball_msgs
  roscpp
  rospy
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES basketball_shoot_srv
  CATKIN_DEPENDS roscpp rospy std_msgs
  DEPENDS system_lib
)

include_directories(
    include
  ${catkin_INCLUDE_DIRS}
)

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
add_executable(robot_shoot_srv src/basketball_shoot_service.cpp)
target_link_libraries(robot_shoot_srv ${catkin_LIBRARIES})
 add_dependencies(robot_shoot_srv basketball_msgs_gencpp)
