#!/usr/bin/env python
# -*- coding: UTF-8 -*-


#投篮项目1状态机：
#流程：机器前往投篮区域（并放下铲子）-> 检测定位柱 -> 对准定位柱 -> 投篮 -> 前进到中间置球区附近 -> 开始检测球（以原地自转的方式） -> 检测到球后接近球到球前方1米处
#      -> 再次检测球并调整铲子方向后前进  -> 铲球 -> 前进到投篮区域（定位柱附近） -> 检测定位柱 -> 对准定位柱 -> 投篮


import rospy
import smach
import math
import smach_ros
from robot_state_class.robot_state_class import *


def Shoot_First():
    Shoot_Ball_First = smach.StateMachine(outcomes=['succeed', 'failed'])
    rospy.logwarn("shoot_ball_first STARTING!!!")
    with Shoot_Ball_First:
        smach.StateMachine.add('OutHome', OutHome(),
                                transitions={'succeed':'MoveToShoot1',
                                             'failed':'failed'})
        
        MoveToShoot = smach.Concurrence(outcomes=['succeed','failed'],
                                        default_outcome='failed',
                                        outcome_map={'succeed':{'MoveToShootPoint1':'succeed',
                                                                'ShovelTop1':'succeed'}})
        with MoveToShoot:
            smach.Concurrence.add('MoveToShootPoint1', MoveToShootPoint())
            smach.Concurrence.add('ShovelTop1', ShovelTop())

        smach.StateMachine.add('MoveToShoot1', MoveToShoot,
                                transitions={'succeed':'FINDCYLINDER',
                                             'failed':'failed'})
        
        smach.StateMachine.add('FINDCYLINDER', LockCylinder(1),
                                transitions={'succeed':'SHOVEL_DOWN',
                                             'failed':'failed'})

        smach.StateMachine.add('SHOVEL_DOWN', ShovelUp(),
                                transitions={'succeed':'SHOOT1',
                                             'failed':'failed'})
                                                                                        
        smach.StateMachine.add('SHOOT1',Shoot(),
                                transitions={'succeed':'MoveToSearch',
                                            'failed':'failed'})
        
        MoveToSearch = smach.Concurrence(outcomes=['succeed','failed'],
                                        default_outcome='failed',
                                        outcome_map={'succeed':{'MoveToMidline':'succeed',
                                                                'ShovelUp1':'succeed'}})
        with MoveToSearch:
            smach.Concurrence.add('MoveToMidline', MoveToMidlineSearch())
            smach.Concurrence.add('ShovelUp1', ShovelUp())

        smach.StateMachine.add('MoveToSearch', MoveToSearch,
                                transitions={'succeed':'FindBall',
                                             'failed':'failed'})

        smach.StateMachine.add('FindBall', MidlineSearchVolleyball(),
                               transitions={'succeed': 'ADJUST2',
                                            'failed': 'failed'})
        
        smach.StateMachine.add('ADJUST2',MoveToShootPoint(),
                               transitions={'succeed':'FINDCYLINDER2',
                                            'failed':'failed'})
        
        smach.StateMachine.add('FINDCYLINDER2', LockCylinder(1),
                                transitions={'succeed':'SHOVEL_DOWN2'})
        
        smach.StateMachine.add('SHOVEL_DOWN2', ShovelUp(),
                                transitions={'succeed':'SHOOT2',
                                             'failed':'failed'})
                                                                                        
        smach.StateMachine.add('SHOOT2',Shoot(),
                               transitions={'succeed':'succeed',
                                            'failed':'failed'})

        # smach.StateMachine.add('Shovel_Control_Up2',ShovelUp(),
        #                        transitions={'succeed':'Return',
        #                                     'failed':'failed'})

        # smach.StateMachine.add('Return',ReturnHome(),
        #                        transitions={'succeed':'succeed',
        #                                     'failed':'failed'})
         
    Shoot_Ball_First.execute()
    rospy.logerr('Shoot_Ball_First has finished!!!!')

if __name__ == '__main__':
    rospy.init_node('ShootBall_First')
    rospy.sleep(19.9)
    Shoot_First()
