#!/usr/bin/python
#-*- coding: UTF-8 -*-

import rospy
import geometry_msgs.msg as g_msgs
import config
import sys
sys.path.append(config.robot_state_pkg_path)
import robot_state_pkg.get_robot_position as robot_state_pkg_path
class world_locate(object):
    def __init__(self):
        self.robot_state  = robot_state_pkg_path.robot_position_state()
        self.cmd_move_pub = rospy.Publisher('/world_locate', g_msgs.Twist, queue_size = 100)
        self.rate = rospy.Rate(10)
    def move_to(self, x = 0.0, y = 0.0, yaw = 0.0):
        rospy.logwarn("Move to x = %s y = %s and turn an angular = %s"%(x, y, yaw))
        dest_point = g_msgs.Twist()
        dest_point.linear.x = x
        dest_point.linear.y = y
        dest_point.angular.z = yaw
        current_x, current_y, current_yaw = self.robot_state.get_robot_current_x_y_w()
        if abs(current_x - x) > 0.038 and abs(current_y - y) > 0.038 and abs(current_yaw - yaw) > 0.01:
            print('sending move message x = %f ' % dest_point.linear.x)
            self.cmd_move_pub.publish(dest_point)
            print('sending move message y = %f ' % dest_point.linear.y)
            self.rate.sleep()

if __name__ == "__main__":
    rospy.init_node("move_to_point")
    move_cmd = world_locate()
    print 'I\'m trying to move, don\'t bibi'
    move_cmd.move_to(0.5, 0.5, 0.5)

sys.path.remove(config.robot_state_pkg_path)

