cmake_minimum_required(VERSION 2.8.3)
project(basketball_msgs)

find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs
  geometry_msgs
  message_generation
  nav_msgs
  std_msgs
)
add_message_files(
   FILES
   basketball_position.msg
   mark_post_position.msg
   robot_message.msg
   robot_order.msg
   robot_state.msg
   ballElem.msg
   cylinderElem.msg
 )

 add_service_files(
   FILES
   basketball_nav_srv.srv
   basketball_position_srv.srv
   basketball_shoot_srv.srv
   basketball_shovel_srv.srv
   basketball_shengine_srv.srv
   mark_post_position_srv.srv
   ControlState.srv 
   camVisionDate.srv
   visionDate.srv
   move_to_point.srv
   robot_rotate.srv
   focus_target.srv
   ballDetect.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   actionlib_msgs
   geometry_msgs
   nav_msgs
   std_msgs
 )

catkin_package(
  CATKIN_DEPENDS actionlib_msgs geometry_msgs nav_msgs  std_msgs message_runtime
)



