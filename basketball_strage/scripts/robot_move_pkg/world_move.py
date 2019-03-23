#!/usr/bin/env python
# coding=utf-8

import rospy
import config
import sys
import math
from time import sleep
sys.path.append(config.robot_state_pkg_path)
sys.path.append(config.robot_msg_pkg_path)
import geometry_msgs.msg as g_msgs
from basketball_msgs.srv import *

class world_move(object):
    def __init__(self):
        try:
            self.world_move_client = rospy.ServiceProxy("/robot_move/world_locate_srv", move_to_point)
        except rospy.ServiceException, e:
            print 'Service call failed: %s'%e

    ##
    # @brief 调用move_srv，完成机器人的定点移动
    #
    # @param x_goal
    # @param y_goal
    # @param z_goal
    #
    # @return 
    def move_to(self, x_goal, y_goal, z_goal):
        rospy.loginfo('running\n')
        self.world_move_client.wait_for_service()
        if z_goal < 0:
            z_goal = z_goal + 2*math.pi
        rospy.loginfo('running\n')
        resp = self.world_move_client( x_goal, y_goal, z_goal )
        while resp.ok != True:
            resp = self.world_move_client( x_goal, y_goal, z_goal )
            #rospy.loginfo(resp)
            rospy.sleep(0.1)
        return True

if __name__ == '__main__':
    rospy.init_node('world_move')
    move_cmd = world_move()
    move_cmd.move_to(0.0, 0.0, 0.0)

sys.path.remove(config.robot_state_pkg_path)
sys.path.remove(config.robot_msg_pkg_path)


