#!/usr/bin/env python
# -*- coding: UTF-8 -*-
#test的状态机
#流程： 前进到三分线附近 -> 自转并检测篮球 -> 检测到球后接近球 -> 二次检测球并调整角度 -> 铲球 -> 调整传球方向 -> 传球
#      -> 移动到底脚置球区附近 ->在自转并检测球 -> 检测到球后接近球 -> 二次检测并调整 -> 铲球 -> 调整传球位置和方向 -> 传球 —> 回家

import rospy
import smach
import math
import smach_ros
from robot_state_class.robot_state_class import *


def pass_third():
    pass_ball_third = smach.StateMachine(outcomes=['succeed', 'failed'])

    rospy.logwarn("pass_ball_third STARTING!!!")
    with pass_ball_third:
        smach.StateMachine.add('OutHome',OutHome(),
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
                                                    
        smach.StateMachine.add('FindBall1', ThreePtLineSearchVolleyball(),
                               transitions={'succeed':'GoThroughLine',
                                            'failed':'failed'})

        smach.StateMachine.add('GoThroughLine', GoThroughBalls(),
                                transitions={'succeed':'FINDCYLINDER1',
                                             'failed':'failed'})
        
        smach.StateMachine.add('FINDCYLINDER1', LockCylinder(3),
                                transitions={'succeed':'SHOVEL_DOWN',
                                             'failed':'failed'})
        
        smach.StateMachine.add('SHOVEL_DOWN', ShovelUp(),
                                transitions={'succeed':'SHOOT1',
                                             'failed':'failed'})
        
        smach.StateMachine.add('SHOOT1',Shoot(),
                                transitions={'succeed':'FIND_ANOTHER',
                                            'failed':'failed'})

        smach.StateMachine.add('FIND_ANOTHER',InnerLineSearchVolleyball(),
                                transitions={'succeed':'FINDCYLINDER2',
                                            'failed':'failed'})

        smach.StateMachine.add('FINDCYLINDER2', LockCylinder(3),
                                transitions={'succeed':'SHOVEL_DOWN2',
                                             'failed':'failed'})
        
        smach.StateMachine.add('SHOVEL_DOWN2', ShovelUp(),
                                transitions={'succeed':'SHOOT2',
                                             'failed':'failed'})
        
        smach.StateMachine.add('SHOOT2',Shoot(),
                                transitions={'succeed':'succeed',
                                            'failed':'failed'})

        # smach.StateMachine.add('GO_BACK',GoBack(),
        #                         transitions={'succeed':'MOVE_TO_GAP',
        #                                     'failed':'failed'})

        # MoveToGapPoint = smach.Concurrence(outcomes=['succeed','failed'],
        #                         default_outcome='failed',
        #                         outcome_map={'succeed':{'ShovelUp':'succeed',
        #                                                 'MoveToGap':'succeed'}})   
        # with MoveToGapPoint:
        #     smach.Concurrence.add('ShovelUp', ShovelTop())
        #     smach.Concurrence.add('MoveToGap', MoveToThreePtlineGap())
        
        # smach.StateMachine.add('MOVE_TO_GAP', MoveToGapPoint,
        #                         transitions={'succeed':'RETURN',
        #                                      'failed':'failed'})
        
        # smach.StateMachine.add('RETURN', ReturnHome(),
        #                         transitions={'succeed':'succeed',
        #                                      'failed':'failed'})

    pass_ball_third.execute()
    rospy.logerr('Pass_Ball_Third has finished!!!!')

if __name__ == '__main__':
    rospy.init_node('PassBall_third')
    rospy.sleep(19.5)
    start_time = rospy.get_time()
    pass_third()
    end_time   = rospy.get_time()
    print end_time - start_time
