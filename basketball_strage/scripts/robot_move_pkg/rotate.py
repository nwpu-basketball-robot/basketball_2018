# coding=utf-8
#!usr/bin/env python

import rospy
import config
import sys

#sys.path.append(config.robot_msg_pkg_path)
from basketball_msgs.srv import *

class rotat_cmd(object):
    def __init__(self):
        try:
            self.rotate_client = rospy.ServiceProxy("/robot_move/world_locate_srv", robot_rotate)
        except rospy.ServiceException, e:
            print 'Service call failed: %s' %e
    def rotate(self, radius, center_x, center_y, speed):
        rospy.loginfo("Waiting for service")
        self.rotate_client.wait_for_service()
        rospy.loginfo("Service ok")
        resp = self.rotate_client(radius, radius, center_x, center_y, speed, speed)
        return True

if __name__ == '__main__':
    rospy.init_node('robot_rotate')
    rotate = rotate_cmd()
    rotate.rotate(1,1,1,1)
