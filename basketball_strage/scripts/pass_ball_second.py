#!/usr/bin/env python
# -*- coding: UTF-8 -*-

#传球项目2状态机
#流程： 前进到三分线附近 -> 自转并检测篮球 -> 检测到球后接近球 -> 二次检测球并调整角度 -> 铲球 -> 调整传球方向 -> 传球
#      -> 移动到中间置球区附近 ->在自转并检测球（为检测到时移动到另一个点并再次执行该步骤） -> 检测到球后接近球 -> 二次检测并调整 -> 铲球 -> 调整传球方向 -> 传球 —> 回家
import rospy
import smach
import math
import smach_ros
from robot_state_class.robot_state_class import *


def pass_second():
    pass_ball_second = smach.StateMachine(outcomes=['succeed', 'failed'])
    rospy.logwarn("pass_ball_second STARTING!!!")
    with pass_ball_second:
        smach.StateMachine.add('OutHome', OutHome(),
                                transitions={'succeed':'MOVE_TO_THREE_POINT_LINE',
                                             'failed':'failed'})
        MoveAndRaise = smach.Concurrence(outcomes=['succeed','failed'],
                                default_outcome = 'failed',
                                outcome_map={'succeed':{'MoveToThreePtLine':'succeed',
                                                        'ShovelUp1':'succeed'}})
        with MoveAndRaise:
            smach.Concurrence.add('MoveToThreePtLine',MoveToThreePtlineSearch())
            smach.Concurrence.add('ShovelUp1', ShovelUp())
        
        smach.StateMachine.add('MOVE_TO_THREE_POINT_LINE',MoveAndRaise,
                                transitions={'succeed':'FindBall1',
                                             'failed':'failed'})

        smach.StateMachine.add('FindBall1', ThreePtLineSearchBasketball(),
                               transitions={'succeed':'SHOOT_ADJUST1',
                                            'failed':'FIND_ANOTHER_BALL'})

        MoveAndDown  = smach.Concurrence(outcomes=['succeed','failed'],
                                default_outcome='failed',
                                outcome_map={'succeed':{'MoveToMidlinePass':'succeed',
                                                        'ShovelDown1':'succeed'}})
        with MoveAndDown:
            smach.Concurrence.add('MoveToMidlinePass',MoveToMidlinePassFromGap())
            smach.Concurrence.add('ShovelDown1',ShovelUp())
        
        smach.StateMachine.add('SHOOT_ADJUST1', MoveAndDown,
                                transitions={'succeed':'SHOOT1',
                                             'failed':'failed'})

        smach.StateMachine.add('SHOOT1',Shoot(),
                                transitions={'succeed':'FIND_ANOTHER_BALL',
                                            'failed':'failed'})

        MoveToMidline  = smach.Concurrence(outcomes=['succeed','failed'],
                                default_outcome='failed',
                                outcome_map={'succeed':{'MoveToMidlineSearch':'succeed',
                                                        'ShovelUp2':'succeed'}})
        with MoveToMidline:
            smach.Concurrence.add('MoveToMidlineSearch',MoveToMidlineSearch())
            smach.Concurrence.add('ShovelUp2',ShovelUp())
        
        smach.StateMachine.add('FIND_ANOTHER_BALL', MoveToMidline,
                                transitions={'succeed':'FindBall2',
                                             'failed':'failed'})

        smach.StateMachine.add('FindBall2', MidlineSearchBasketball(),
                               transitions={'succeed': 'SHOOT_ADJUST2',
                                            'failed': 'failed'})
        
        MoveToMidline2  = smach.Concurrence(outcomes=['succeed','failed'],
                                default_outcome='failed',
                                outcome_map={'succeed':{'MoveToMidlinePass2':'succeed',
                                                        'ShovelDown2':'succeed'}})
        with MoveToMidline2:
            smach.Concurrence.add('MoveToMidlinePass2',MoveToMidlinePass())
            smach.Concurrence.add('ShovelDown2',ShovelUp())
        
        smach.StateMachine.add('SHOOT_ADJUST2', MoveToMidline2,
                                transitions={'succeed':'SHOOT2',
                                             'failed':'failed'})

        smach.StateMachine.add('SHOOT2',Shoot(),
                               transitions={'succeed':'Return',
                                            'failed':'failed'})

        smach.StateMachine.add('Return',ReturnHome(),
                                transitions={'succeed':'succeed',
                                             'failed':'failed'})
    pass_ball_second.execute()
    rospy.logerr('Pass_Ball_Second has finished!!!!')

if __name__ == '__main__':
    #start_time  =  rospy.get_time()
    rospy.init_node('PassBall_second')
    rospy.sleep(19.5)
    start_time  =  rospy.get_time()
    pass_second()
    end_time    = rospy.get_time()
    print end_time - start_time
