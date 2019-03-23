#!/usr/bin/python
# coding=utf-8
import rospy
import smach
import math
import smach_ros
from robot_state_class.robot_state_class import *

def TestShoot():
    ShootBall = smach.StateMachine(outcomes=['succeed','failed'])
    rospy.logwarn('Prepare to shoot ball')
    with ShootBall:
        smach.StateMachine.add('SHOVEL_TOP', ShovelTop(),
                                transitions={'succeed':'FINDCYLINDER',
                                             'failed':'failed'})
        
        smach.StateMachine.add('FINDCYLINDER', LockCylinder(1),
                                transitions={'succeed':'SHOVEL_DOWN',
                                             'failed':'failed'})

        smach.StateMachine.add('SHOVEL_DOWN', ShovelDown(),
                                transitions={'succeed':'SHOOT1',
                                             'failed':'failed'})
                                                                                        
        smach.StateMachine.add('SHOOT1',Shoot(),
                                transitions={'succeed':'succeed',
                                            'failed':'failed'})
    ShootBall.execute()

if __name__ == '__main__':
    rospy.init_node("TestShoot")
    TestShoot()
