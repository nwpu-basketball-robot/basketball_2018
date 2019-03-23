#!/usr/bin/env python
# coding=utf-8

import rospy
import config
import sys
import math
sys.path.append(config.robot_msg_pkg_path)
sys.path.append(config.robot_state_pkg_path)
import move_in_robot
import robot_state_pkg.get_robot_position as robot_state

class turn_an_angular(object):
    def __init__(self):
        self.move_cmd = move_in_robot.move_in_robot()

    def turn_to(self,angular):
        self.move_cmd.move_to(0, 0, angular)

if __name__ == '__main__':
    rospy.init_node("turn_an_angular")
    turn_cmd = turn_an_angular()
    turn_cmd.turn_to(-math.pi/2)
