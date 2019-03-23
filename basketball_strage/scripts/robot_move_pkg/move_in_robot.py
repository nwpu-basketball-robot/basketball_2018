#!/usr/bin/env python
# coding=utf-8

import rospy
import config
import sys
import math
sys.path.append(config.robot_state_pkg_path)
sys.path.append(config.robot_msg_pkg_path)
import world_move
import robot_state_pkg.get_robot_position as robot_state

class move_in_robot(object):
    def __init__(self):
        self.robot_location = robot_state.robot_position_state()
        self.move_cmd = world_move.world_move()


    ##
    # @brief 将机器人坐标系的移动乘一个旋转矩阵解算至全局坐标系，使用全局移动来完成机器人坐标移动
    #
    # @param x_distance
    # @param y_distance
    # @param z_angle
    #
    # @return 
    def move_to(self,x_distance, y_distance, z_angle):
        start_x, start_y, start_yaw = self.robot_location.get_robot_current_x_y_w()
        x_goal = start_x + x_distance*math.cos(start_yaw) - y_distance*math.sin(start_yaw)
        y_goal = start_y + x_distance*math.sin(start_yaw) + y_distance*math.cos(start_yaw)
        z_goal = start_yaw + z_angle
        if z_goal < 0.0:
            z_goal = z_goal + 2*math.pi
        if z_goal > 2*math.pi:
            z_goal = z_goal % 2*math.pi
        self.move_cmd.move_to(x_goal, y_goal, z_goal)

if __name__ == '__main__':
    rospy.init_node("move_in_robot")
    move_cmd = move_in_robot()
    move_cmd.move_to(0.6, 0.6, math.pi/2)

sys.path.remove(config.robot_msg_pkg_path)
sys.path.remove(config.robot_state_pkg_path)



