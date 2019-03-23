#!/usr/bin/env python
#coding=utf-8

import tf
import actionlib
import math
import rospy
import roslib
from basketball_msgs.srv import *

#铲子控制接口
class shovelControlSrv(object):
    def __init__(self):
        self.__shovel_control_client = rospy.ServiceProxy('cmd_shovel',basketball_shovel_srv)
        rospy.loginfo('waiting for the shovel srv...')
        self.__shovel_control_client.wait_for_service()
        rospy.loginfo('connect to the shovel srv!!!')

    def control_shovel(self, control_type):
        self.__shovel_control_client.wait_for_service()
        self.__shovel_control_client(control_type)
        return True
        #type代表铲子指令类型
        #详细可见后面的文档

#弹射控制接口
class shootControlSrv(object):
    def __init__(self):
        self.__shoot_control_client = rospy.ServiceProxy('cmd_shoot',basketball_shoot_srv)
        rospy.loginfo('waiting for the shooting srv...')
        self.__shoot_control_client.wait_for_service()
        rospy.loginfo('connect to the shooting srv')

    def shoot_ball(self):
        self.__shoot_control_client.wait_for_service()
        self.__shoot_control_client()
        return True

#持球确认接口
class BallDetectSrv(object):
    def __init__(self):
        self.__shovel_control_client = rospy.ServiceProxy('cmd_shovel', basketball_shovel_srv)
        self.ball_detect_client = rospy.ServiceProxy('ball_detect', ballDetect)
        rospy.loginfo('waiting for the shovel service...')
        self.__shovel_control_client.wait_for_service()
        self.ball_detect_client.wait_for_service()
        rospy.loginfo('connect to the shovel service')
    
    def detect(self):
        self.__shovel_control_client(0x0d)
        rospy.sleep(0.1)
        has_ball = self.ball_detect_client(1)
        print has_ball
        while has_ball.ok == 2:
            self.__shovel_control_client(0x0d)
            rospy.sleep(0.1)
            has_ball = self.ball_detect_client(1)
            print has_ball
        if has_ball.ok == 1:
            return True
        else:
            return False

if __name__ == '__main__':
    det = BallDetectSrv()
    has_ball = det.detect()
    print has_ball
