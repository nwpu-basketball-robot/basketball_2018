#!/usr/bin/env python
# -*- coding: UTF-8 -*-

#传球项目1状态机：
#流程：机器前往传球区并放下铲子 -> 传球 -> 前进到找球的位置 -> 开始检测球（以原地自转的方式） -> 检测到球后接近球到球前方0.8米处
#      -> 再次检测球并调整铲子方向后前进  -> 铲球 -> 调整机器角度朝向传球区 -> 投球 -> 升起铲子 -> 回家

import rospy
import smach
import math
import smach_ros
from robot_state_class.robot_state_class import *

def pass_first():
    pass_ball_first = smach.StateMachine(outcomes=['succeed', 'failed'])
    rospy.logwarn("pass_ball_first STARTING!!!")
    
    with pass_ball_first:
        
        smach.StateMachine.add('Start', OutHome(),
                                transitions={'succeed':'MoveToPass',
                                             'failed':'failed'})

        MoveToPass = smach.Concurrence(outcomes=['succeed','failed'],
                                        default_outcome='failed',
                                        outcome_map={'succeed':{'ShovelUp1':'succeed',
                                                                'MoveToMidline1':'succeed'}})
        with MoveToPass:
            smach.Concurrence.add('ShovelUp1', ShovelUp())
            smach.Concurrence.add('MoveToMidline1', MoveToMidlinePass())
        
        smach.StateMachine.add('MoveToPass', MoveToPass,
                                transitions={'succeed':'SHOOT1',
                                             'failed':'failed'})
        
        smach.StateMachine.add('SHOOT1',Shoot(),
                                transitions={'succeed':'MoveToSearch',
                                            'failed':'failed'})

        smach.StateMachine.add('MoveToSearch', MoveToMidlineSearch(), 
                                transitions={'succeed':'FindBall',
                                             'failed':'failed'})

        smach.StateMachine.add('FindBall', MidlineSearchBasketball(),
                               transitions={'succeed': 'Shovel_Down',
                                            'failed': 'Return'})

        smach.StateMachine.add('Shovel_Down', ShovelUp(),
                                transitions={'succeed': 'ADJUST2',
                                'failed':'failed'})
        
        smach.StateMachine.add('ADJUST2',MoveToMidlinePass(),
                               transitions={'succeed':'SHOOT2',
                                            'failed':'failed'})
        
        smach.StateMachine.add('SHOOT2',Shoot(),
                               transitions={'succeed':'Return',
                                            'failed':'failed'})

        smach.StateMachine.add('Return', ReturnHome(),
                                transitions={'succeed':'succeed',
                                             'failed':'failed'})
        
                
    pass_ball_first.execute()
    rospy.logerr('Pass_Ball_First has finished!!!!')

if __name__ == '__main__':

    rospy.init_node('passBall_first')
    rospy.sleep(19.9)
    start_time = rospy.get_time()
    pass_first()
    end_time   = rospy.get_time()
    print (end_time - start_time)
