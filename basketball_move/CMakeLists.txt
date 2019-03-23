cmake_minimum_required(VERSION 2.8.3)
project(basketball_move)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  basketball_msgs
  std_msgs
  nav_msgs
  tf
)
find_package(OpenCV REQUIRED)
catkin_package(
  INCLUDE_DIRS include
  #LIBRARIES robot_move_pkg
  CATKIN_DEPENDS roscpp rospy geometry_msgs basketball_msgs nav_msgs
  DEPENDS system_lib
)
include_directories(
    #  include/robot_move
    include
  ${catkin_INCLUDE_DIRS}
)
#add_executable(linear_move src/linear_move.cpp)
add_executable(move_srv src/move_srv.cpp)
target_link_libraries(move_srv ${catkin_LIBRARIES})
add_executable(basketball_controler src/Controler.cpp)
target_link_libraries(basketball_controler ${catkin_LIBRARIES} ${OpenCV_LIBS})
#target_link_libraries(linear_move ${catkin_LIBRARIES})
add_dependencies(move_srv basketball_msgs_gencpp)
#
