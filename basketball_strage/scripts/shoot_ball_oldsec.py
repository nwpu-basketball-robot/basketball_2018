#!/usr/bin/env python
# -*- coding: UTF-8 -*-

#投篮项目2的状态机
#流程： 前进到三分线附近 -> 自转并检测篮球 -> 检测到球后接近球 -> 二次检测球并调整角度（记录球所在位置，世界坐标系）-> 铲球 -> 前进到定位柱附近 -> 检测定位柱 -> 调整角度对准定位柱  -> 投篮
#      前进到三分线捡球处（防止撞到排球）-> 移动到中间置球区附近 ->在自转并检测球 -> 检测到球后接近球 -> 二次检测并调整 -> 铲球 -> 前进到三分线捡球处（防止撞到排球） -> 前进到定位柱附近
#      -> 检测定位柱 -> 调整角度对准定位柱  -> 投篮

import rospy
import smach
import math
import smach_ros
from robot_state_class.robot_state_class import *


def shoot_second():
    shoot_ball_second = smach.StateMachine(outcomes=['succeed', 'failed'])
    rospy.logwarn("pass_ball_second STARTING!!!")
    with shoot_ball_second:
        smach.StateMachine.add('OutHome',OutHome(),
                                transitions={'succeed':'MoveToMidline',
                                             'failed':'failed'})
        
        MoveAndRaise = smach.Concurrence(outcomes=['succeed','failed'],
                                default_outcome = 'failed',
                                outcome_map={'succeed':{'MoveToMidlineSearch':'succeed',
                                                        'ShovelUp1':'succeed'}})
        with MoveAndRaise:
            smach.Concurrence.add('MoveToMidlineSearch',MoveToMidlineSearch())
            smach.Concurrence.add('ShovelUp1', ShovelUp())
        
        smach.StateMachine.add('MoveToMidline', MoveAndRaise,
                                transitions={'succeed':'FindBall1',
                                             'failed':'failed'})
        
        smach.StateMachine.add('FindBall1', MidlineSearchVolleyball(),
                                transitions={'succeed':'GoThroughThreePtline',
                                             'failed':'ShovelTop1'})

        smach.StateMachine.add('ShovelTop1', ShovelTop(),
                                transitions={'succeed':'GoThroughThreePtline',
                                             'failed':'failed'})
        
        smach.StateMachine.add('GoThroughThreePtline', GoThroughThreePtline(),
                                transitions={'succeed':'FINDCYLINDER1',
                                             'failed':'failed'})
        
        smach.StateMachine.add('FINDCYLINDER1', LockCylinder(2),
                                transitions={'succeed':'SHOVEL_DOWN1',
                                             'failed':'failed'})
        
        smach.StateMachine.add('SHOVEL_DOWN1', ShovelUp(),
                                transitions={'succeed':'SHOOT1',
                                             'failed':'failed'})
        
        smach.StateMachine.add('SHOOT1',Shoot(),
                                transitions={'succeed':'InnerThreePtlineSearch',
                                            'failed':'failed'})

        smach.StateMachine.add('InnerThreePtlineSearch', InnerThreePtLineSearchVolleyball(),
                                transitions={'succeed':'MoveToShoot',
                                             'failed':'GoOut'})
        
        smach.StateMachine.add('MoveToShoot', MoveToShootPoint(),
                                transitions={'succeed':'FINDCYLINDER2',
                                             'failed':'failed'})

        smach.StateMachine.add('FINDCYLINDER2', LockCylinder(2),
                                transitions={'succeed':'SHOVEL_DOWN2',
                                             'failed':'failed'})
        
        smach.StateMachine.add('SHOVEL_DOWN2', ShovelUp(),
                                transitions={'succeed':'SHOOT2',
                                             'failed':'failed'})
        
        smach.StateMachine.add('SHOOT2',Shoot(),
                                transitions={'succeed':'GoOut',
                                            'failed':'failed'})

        smach.StateMachine.add('GoOut', GoOut(),
                                transitions={'succeed':'Return',
                                             'failed':'failed'})

        smach.StateMachine.add('Return',ReturnHome(),transitions={'succeed':'succeed',
                                                                    'failed':'failed'})
        
    shoot_ball_second.execute()
    rospy.logerr('Shoot_Ball_Second has finished!!!!')


if __name__ == '__main__':
    rospy.init_node('shootBall_second')
    rospy.sleep(19.5)
    start_time = rospy.get_time()
    shoot_second()
    end_time = rospy.get_time()
    print start_time - end_time
