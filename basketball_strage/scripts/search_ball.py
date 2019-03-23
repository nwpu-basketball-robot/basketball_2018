#!/usr/bin/env python
# coding=utf-8
import rospy
import smach
import math
import smach_ros
from robot_state_class.pass_ball_first_state import *

def pass_ball():
    pass_ball_ = smach.StateMachine(outcomes=['successed', 'failed'])
    rospy.logwarn("testing")

    with pass_ball_:

        smach.StateMachine.add('FindBall', Search_Ball(), transitions={'successed':'successed', 'failed':'failed'})

    pass_ball_.execute()

if __name__ == '__main__':
    rospy.init_node('getball')
    pass_ball()
