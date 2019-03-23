#!/usr/bin/env python
# -*- coding: UTF-8 -*-

#传球项目2状态机
#流程： 前进到三分线附近 -> 自转并检测篮球 -> 检测到球后接近球 -> 二次检测球并调整角度 -> 铲球 -> 调整传球方向 -> 传球
#      -> 移动到中间置球区附近 ->在自转并检测球（为检测到时移动到另一个点并再次执行该步骤） -> 检测到球后接近球 -> 二次检测并调整 -> 铲球 -> 调整传球方向 -> 传球 —> 回家
import rospy
import smach
import math
import smach_ros
from robot_state_class.pass_ball_second_state import *


def pass_second():
    pass_ball_second = smach.StateMachine(outcomes=['successed', 'failed'])
    rospy.logwarn("pass_ball_second STARTING!!!")
    with pass_ball_second:

        smach.StateMachine.add('FindBall2', Search_Ball_Ni(),
                               transitions={'successed': 'successed',
                                            'failed': 'failed'},)
        
    pass_ball_second.execute()
    rospy.logerr('Pass_Ball_Second has finished!!!!')

if __name__ == '__main__':
    rospy.init_node('PassBall_second')
    pass_second()
