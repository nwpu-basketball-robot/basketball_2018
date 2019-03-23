#!/usr/bin/python
# -*- coding: UTF-8 -*-

import math

import rospy
import smach
import smach_ros
import geometry_msgs
from basketball_msgs.srv import *

from position_parameters import *
from robot_find_pkg import find_ball, find_cylinder
from robot_move_pkg import move_in_robot, turn_an_angular, world_move
from robot_shovel_srv import control_srv
from robot_state_pkg import get_robot_position


gap_x = 0
gap_y = 0
gap2_x = 0
gap2_y = 0
gap2_z = 0
point1 = []         # Begin point when going through inner line
point2 = []         # End point after through inner line
## shoot
class Shoot(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ['succeed', 'failed'])
        self.cmd_shoot = control_srv.shootControlSrv()
        rospy.loginfo('Shoot initialize ok!\n')

    def execute(self, ud):
        rospy.logwarn("Start Shoot!")
        if self.preempt_requested():
            self.service_preempt()
        self.cmd_shoot.shoot_ball()
        rospy.sleep(0.3) #Delay to avoid moving before shoot finished
        return 'succeed'

##
# @brief 将铲子放至中间位置
class ShovelUp(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ['succeed', 'failed'])
        self.cmd_shovel     = control_srv.shovelControlSrv()
    
    def execute(self, ud):
        rospy.logwarn('Shovel is prepared to up!')
        if self.preempt_requested():
            self.service_preempt()
        self.cmd_shovel.control_shovel(control_type = 2)
        rospy.sleep(0.5)
        return 'succeed'


##
# @brief 将铲子抬到最高
class ShovelTop(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ['succeed', 'failed'])
        self.cmd_shovel     = control_srv.shovelControlSrv()
    
    def execute(self, ud):
        rospy.logwarn('Shovel is prepared to up!')
        if self.preempt_requested():
            self.service_preempt()
        self.cmd_shovel.control_shovel(control_type = 3)
        rospy.sleep(0.5)
        return 'succeed'

##
# @brief 将铲子放下
class ShovelDown(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ['succeed', 'failed'])
        self.cmd_shovel     = control_srv.shovelControlSrv()

    def execute(self, ud):
        rospy.logwarn('Shovel is prepared to down')
        if self.preempt_requested():
            self.service_preempt()
        self.cmd_shovel.control_shovel(control_type = 1)
        rospy.sleep(0.5)
        return 'succeed'

##
# @brief 走出启动区
class OutHome(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ['succeed', 'failed'])
        self.move_cmd       = world_move.world_move()

    def execute(self, ud):
        rospy.logwarn('I\'m getting out of home~')
        if self.preempt_requested():
            self.service_preempt()
        self.move_cmd.move_to(0, 2.6, 0)
        rospy.logwarn('Ok I\'m out')
        return 'succeed'

##
# @brief 从场上返回启动区
class ReturnHome(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ['succeed', 'failed'])
        self.move_cmd       = world_move.world_move()
    
    def execute(self, ud):
        rospy.logwarn('I\'m Returning Home')
        if self.preempt_requested():
            self.service_preempt()
        self.move_cmd.move_to(RobotHome_.x, 1.8, 0)
        rospy.sleep(0.5)
        self.move_cmd.move_to(RobotHome_.x, RobotHome_.y, RobotHome_.z)
        rospy.logwarn('Ok I am back')
        return 'succeed'
        

##
# @brief 移动至中线找球的位置
class MoveToMidlineSearch(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ['succeed', 'failed'])
        self.move_cmd       = world_move.world_move()
        if start_position == 0:
            self.dir = 1
        else:
            self.dir = -1
    
    def execute(self, ud):
        rospy.logwarn('Moving to midline search ball point~')
        if self.preempt_requested():
            self.service_preempt()
        self.move_cmd.move_to(MidlineSearch_.x + self.dir*midlinedirection*0.9, MidlineSearch_.y, self.dir*midlinedirection*MidlineSearch_.z)
        return 'succeed'

##
# @brief 按先x后y的方式走到中线找球的位置
class MoveToMidlineSearch1(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ['succeed', 'failed'])
        self.move_cmd       = world_move.world_move()
        self.current_position = get_robot_position.robot_position_state()
        if start_position == 0:
            self.dir = 1
        else:
            self.dir = -1
    
    def execute(self, ud):
        rospy.logwarn('Moving to midline search ball point~')
        if self.preempt_requested():
            self.service_preempt()
        current_x, current_y = self.current_position.get_robot_current_x_y()
        self.move_cmd.move_to(MidlineSearch_.x + self.dir*midlinedirection*0.9, current_y, self.dir*midlinedirection*MidlineSearch_.z)
        self.move_cmd.move_to(MidlineSearch_.x + self.dir*midlinedirection*0.9, MidlineSearch_.y, self.dir*midlinedirection*MidlineSearch_.z)
        return 'succeed'

##
# @brief 走到中线传球的位置
class MoveToMidlinePass(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ['succeed', 'failed'])
        self.move_cmd       = world_move.world_move()

    def execute(self, ud):
        rospy.logwarn('Moveing to midline pass point~, x is %f' % MidlinePass_.x)
        if self.preempt_requested():
            self.service_preempt()
        self.move_cmd.move_to(MidlinePass_.x, MidlinePass_.y, MidlinePass_.z)
        return 'succeed'

##
# @brief 从三分线铲球后留下的缺口移动至中线传球位置
class MoveToMidlinePassFromGap(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ['succeed', 'failed'])
        self.move_cmd       = world_move.world_move()
        if start_position == 0:
            self.dir = 1
        else:
            self.dir = -1

    def execute(self, ud):
        rospy.logwarn('Moveing to midline pass point~, x is %f' % MidlinePass_.x)
        if self.preempt_requested():
            self.service_preempt()
        self.move_cmd.move_to(ThreePtLineSearch_.x + self.dir*0.3, gap_y, ThreePtLineSearch_.z)
        self.move_cmd.move_to(ThreePtLineSearch_.x + self.dir*0.3, ThreePtLineSearch_.y, ThreePtLineSearch_.z)
        self.move_cmd.move_to(MidlinePass_.x, MidlinePass_.y, MidlinePass_.z)
        return 'succeed'

##
# @brief 从三分线铲球后留下的缺口移动至中线找球位置
class MoveToMidlineSearchFromGap(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ['succeed', 'failed'])
        self.move_cmd       = world_move.world_move()
        if start_position == 0:
            self.dir = 1
        else:
            self.dir = -1


    def execute(self, ud):
        rospy.logwarn('Moveing to midline pass point~, x is %f' % MidlinePass_.x)
        if self.preempt_requested():
            self.service_preempt()
        self.move_cmd.move_to(ThreePtLineSearch_.x + self.dir*0.3, gap_y, ThreePtLineSearch_.z)
        self.move_cmd.move_to(ThreePtLineSearch_.x + self.dir*0.3, ThreePtLineSearch_.y, ThreePtLineSearch_.z)
        self.move_cmd.move_to(MidlineSearch_.x + self.dir*midlinedirection*0.9, MidlineSearch_.y, self.dir*midlinedirection*MidlineSearch_.z)
        return 'succeed'

##
# @brief 在中线位置寻找篮球
class MidlineSearchBasketball(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ['succeed', 'failed'])
        self.move_cmd       = move_in_robot.move_in_robot()
        self.world_move_cmd = world_move.world_move()
        self.turn_cmd       = turn_an_angular.turn_an_angular()
        self.cmd_shovel     = control_srv.shovelControlSrv()
        self.find_ball      = find_ball.find_ball(1)
        self.balldetect     = control_srv.BallDetectSrv()           # Using the laser module on shovel to detect whether ball is on or not
        self.current_position = get_robot_position.robot_position_state()
        self.dir            = 0
        if start_position == 0:
            self.dir = 1
        else:
            self.dir = -1
    
    def search(self):
        rospy.logwarn('Midline Searching Basketball!~, ')
        if self.preempt_requested():
            self.service_preempt()
        # Search ball (anti)clockwise
        (ball_dis, ball_theta) = self.find_ball.findball_cw(self.dir*RotateSpeed*midlinedirection, MidlineMaxBallDis)
        (ball_dis, ball_theta) = self.find_ball.findball_cw(0, MidlineMaxBallDis)
        theta = -math.atan((ball_dis*math.tan(ball_theta) + KinectOffset)/(ball_dis+0.275))
        dir_dis = ball_dis/math.cos(ball_theta)
        if dir_dis > 0.80:  #if ball dis is fairly long
            # Turn to directly face to ball and Move to 0.8m in front of ball
            self.turn_cmd.turn_to(theta)
            current_w = self.current_position.get_robot_current_w()
            current_x = self.current_position.get_robot_current_x()
            current_y = self.current_position.get_robot_current_y()
            if current_w > math.pi:
                current_w = current_w - 2*math.pi
            # Take a movement to avoid collision with other balls
            if(abs(current_w) > math.pi/4):
                move_dis = dir_dis*(abs(math.sin(current_w)) - math.cos(current_w))
                self.world_move_cmd.move_to(current_x - math.copysign(move_dis, current_w), current_y, math.copysign(math.pi/4, current_w))
                self.move_cmd.move_to(0, dir_dis*math.cos(current_w)*math.sqrt(2) - 0.80, 0)
            else:
                self.move_cmd.move_to(0, dir_dis - 0.8, 0)
            rospy.sleep(0.2)
            (ball_dis, ball_theta) = self.find_ball.findball_cw(-self.dir*RotateSpeed*midlinedirection, SecondDetectDis)
            self.move_cmd.move_to(math.tan(ball_theta)*ball_dis + KinectOffset, ball_dis - 0.8, 0)
            # Take a 3-2 vote to decrease error ratio
            ball_queue = []
            for i in range(0,3):
                (ball_dis, ball_theta) = self.find_ball.findball_cw(0, SecondDetectDis)
                ball_queue.append((ball_dis, ball_theta, ball_dis*math.tan(ball_theta)))
            ball_queue.sort(lambda x,y:cmp(x[2], y[2]))
            if ball_queue[1][2] - ball_queue[0][2] < ball_queue[2][2] - ball_queue[1][2]:
                ball_dis = (ball_queue[0][0] + ball_queue[1][0]) / 2
                ball_theta = (ball_queue[0][1] + ball_queue[1][1]) / 2
            else:
                ball_dis = (ball_queue[1][0] + ball_queue[2][0]) / 2
                ball_theta = (ball_queue[1][1] + ball_queue[2][1]) / 2
            self.cmd_shovel.control_shovel(1) #down the shovel
            #self.move_cmd.move_to(math.tan(ball_theta)*ball_dis + KinectOffset + 0.06, 0, 0)
            self.move_cmd.move_to(math.tan(ball_theta)*ball_dis + KinectOffset + start_position*0.0, 0, 0)
            self.move_cmd.move_to(0, ball_dis - GetBallOffset, 0)
        elif dir_dis > 0:   #if ball is close
            self.turn_cmd.turn_to(theta)
            rospy.sleep(0.2)
            ball_queue = []
            for i in range(0,3):
                (ball_dis, ball_theta) = self.find_ball.findball_cw(0, SecondDetectDis)
                ball_queue.append((ball_dis, ball_theta, ball_dis*math.tan(ball_theta)))
            ball_queue.sort(lambda x,y:cmp(x[2], y[2]))
            if ball_queue[1][2] - ball_queue[0][2] < ball_queue[2][2] - ball_queue[1][2]:
                ball_dis = (ball_queue[0][0] + ball_queue[1][0]) / 2
                ball_theta = (ball_queue[0][1] + ball_queue[1][1]) / 2
            else:
                ball_dis = (ball_queue[1][0] + ball_queue[2][0]) / 2
                ball_theta = (ball_queue[1][1] + ball_queue[2][1]) / 2
            self.cmd_shovel.control_shovel(1) #down the shovel
            self.move_cmd.move_to(math.tan(ball_theta)*ball_dis + KinectOffset + start_position*0.0, 0, 0)
            self.move_cmd.move_to(0, ball_dis - GetBallOffset, 0)
        else:   # if no ball detected
            (ball_dis, ball_theta) = self.find_ball.findball_cw(self.dir*RotateSpeed*midlinedirection, MidlineMaxBallDis)
            theta = -math.atan((ball_dis*math.tan(ball_theta) + KinectOffset)/(ball_dis+0.275))
            dir_dis = ball_dis/math.cos(ball_theta)
            if dir_dis > 0.80:
                #Turn to directly face to the ball and Move to 0.8m in front of ball
                self.turn_cmd.turn_to(theta)
                current_w = self.current_position.get_robot_current_w()
                current_x = self.current_position.get_robot_current_x()
                current_y = self.current_position.get_robot_current_y()
                if current_w > math.pi:
                    current_w = current_w - 2*math.pi
                if(abs(current_w) > math.pi/4):
                    move_dis = dir_dis*(abs(math.sin(current_w)) - math.cos(current_w))
                    self.world_move_cmd.move_to(current_x - math.copysign(move_dis, current_w), current_y, math.copysign(math.pi/4, current_w))
                    self.move_cmd.move_to(0, dir_dis*math.cos(current_w)*math.sqrt(2) - 0.80, 0)
                else:
                    self.move_cmd.move_to(0, dir_dis - 0.8, 0)
                rospy.sleep(0.2)
                (ball_dis, ball_theta) = self.find_ball.findball_cw(-self.dir*RotateSpeed*midlinedirection, SecondDetectDis)
                self.move_cmd.move_to(math.tan(ball_theta)*ball_dis, ball_dis - 0.8, 0)
                # Take a 3-2 vote to decrease error ratio
                ball_queue = []
                for i in range(0,3):
                    (ball_dis, ball_theta) = self.find_ball.findball_cw(0, SecondDetectDis)
                    ball_queue.append((ball_dis, ball_theta, ball_dis*math.tan(ball_theta)))
                ball_queue.sort(lambda x,y:cmp(x[2], y[2]))
                if ball_queue[1][2] - ball_queue[0][2] < ball_queue[2][2] - ball_queue[1][2]:
                    ball_dis = (ball_queue[0][0] + ball_queue[1][0]) / 2
                    ball_theta = (ball_queue[0][1] + ball_queue[1][1]) / 2
                else:
                    ball_dis = (ball_queue[1][0] + ball_queue[2][0]) / 2
                    ball_theta = (ball_queue[1][1] + ball_queue[2][1]) / 2
                self.cmd_shovel.control_shovel(1) #down the shovel
                self.move_cmd.move_to(math.tan(ball_theta)*ball_dis + KinectOffset + start_position*0.0, 0, 0)
                self.move_cmd.move_to(0, ball_dis - GetBallOffset, 0)
            elif dir_dis > 0:
                self.turn_cmd.turn_to(theta)
                rospy.sleep(0.2)
                # Take a 3-2 vote to decrease error ratio
                ball_queue = []
                for i in range(0,3):
                    (ball_dis, ball_theta) = self.find_ball.findball_cw(0, SecondDetectDis)
                    ball_queue.append((ball_dis, ball_theta, ball_dis*math.tan(ball_theta)))
                ball_queue.sort(lambda x,y:cmp(x[2], y[2]))
                if ball_queue[1][2] - ball_queue[0][2] < ball_queue[2][2] - ball_queue[1][2]:
                    ball_dis = (ball_queue[0][0] + ball_queue[1][0]) / 2
                    ball_theta = (ball_queue[0][1] + ball_queue[1][1]) / 2
                else:
                    ball_dis = (ball_queue[1][0] + ball_queue[2][0]) / 2
                    ball_theta = (ball_queue[1][1] + ball_queue[2][1]) / 2
                self.cmd_shovel.control_shovel(1) #down the shovel
                self.move_cmd.move_to(math.tan(ball_theta)*ball_dis + KinectOffset + start_position*0.0, 0, 0)
                self.move_cmd.move_to(0, ball_dis - GetBallOffset, 0)
            else:
                return

    def execute(self, ud):
        # Take one search
        self.search()
        # If no ball on shovel, backforward and re-search
        if self.balldetect.detect() == False:
            self.move_cmd.move_to(0, -1.4, 0)
            self.cmd_shovel.control_shovel(2)
            self.search()
        rospy.sleep(0.1)
        self.cmd_shovel.control_shovel(3)
        rospy.sleep(0.5)
        self.move_cmd.move_to(0, -1.0, 0)
        return 'succeed'

##
# @brief 在中线位置寻找排球
# The same process as SearchBasketball. Make a copy for easy distinguish
class MidlineSearchVolleyball(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ['succeed', 'failed'])
        self.move_cmd       = move_in_robot.move_in_robot()
        self.world_move_cmd = world_move.world_move()
        self.turn_cmd       = turn_an_angular.turn_an_angular()
        self.cmd_shovel     = control_srv.shovelControlSrv()
        self.find_ball      = find_ball.find_ball(2)
        self.balldetect     = control_srv.BallDetectSrv()
        self.current_position = get_robot_position.robot_position_state()
        self.dir            = 0
        if start_position == 0:
            self.dir = 1
        else:
            self.dir = -1
    
    def search(self):
        rospy.logwarn('Midline Searching Volleyball!~')
        if self.preempt_requested():
            self.service_preempt()
        (ball_dis, ball_theta) = self.find_ball.findball_cw(self.dir*RotateSpeed*midlinedirection, MidlineMaxBallDis)
        theta = -math.atan((ball_dis*math.tan(ball_theta) + KinectOffset)/(ball_dis+0.275))
        dir_dis = ball_dis/math.cos(ball_theta)
        if dir_dis > 0.80:
            #Move to 0.8m in front of ball
            self.turn_cmd.turn_to(theta)
            current_w = self.current_position.get_robot_current_w()
            current_x = self.current_position.get_robot_current_x()
            current_y = self.current_position.get_robot_current_y()
            if current_w > math.pi:
                current_w = current_w - 2*math.pi
            if(abs(current_w) > math.pi/4):
                move_dis = dir_dis*(abs(math.sin(current_w)) - math.cos(current_w))
                self.world_move_cmd.move_to(current_x - math.copysign(move_dis, current_w), current_y, math.copysign(math.pi/4, current_w))
                self.move_cmd.move_to(0, dir_dis*math.cos(current_w)*math.sqrt(2) - 0.80, 0)
            else:
                self.move_cmd.move_to(0, dir_dis - 0.8, 0)
            rospy.sleep(0.2)
            (ball_dis, ball_theta) = self.find_ball.findball_cw(-self.dir*RotateSpeed*midlinedirection, SecondDetectDis)
            ball_queue = []
            for i in range(0,3):
                (ball_dis, ball_theta) = self.find_ball.findball_cw(0, SecondDetectDis)
                ball_queue.append((ball_dis, ball_theta, ball_dis*math.tan(ball_theta)))
            ball_queue.sort(lambda x,y:cmp(x[2], y[2]))
            if ball_queue[1][2] - ball_queue[0][2] < ball_queue[2][2] - ball_queue[1][2]:
                ball_dis = (ball_queue[0][0] + ball_queue[1][0]) / 2
                ball_theta = (ball_queue[0][1] + ball_queue[1][1]) / 2
            else:
                ball_dis = (ball_queue[1][0] + ball_queue[2][0]) / 2
                ball_theta = (ball_queue[1][1] + ball_queue[2][1]) / 2
            self.cmd_shovel.control_shovel(1) #down the shovel
            #self.move_cmd.move_to(math.tan(ball_theta)*ball_dis + KinectOffset + 0.06, 0, 0)
            self.move_cmd.move_to(math.tan(ball_theta)*ball_dis + KinectOffset + start_position*0.0, 0, 0)
            self.move_cmd.move_to(0, ball_dis - GetBallOffset, 0)
        elif dir_dis > 0:
            self.turn_cmd.turn_to(theta)
            rospy.sleep(0.2)
            ball_queue = []
            for i in range(0,3):
                (ball_dis, ball_theta) = self.find_ball.findball_cw(0, SecondDetectDis)
                ball_queue.append((ball_dis, ball_theta, ball_dis*math.tan(ball_theta)))
            ball_queue.sort(lambda x,y:cmp(x[2], y[2]))
            if ball_queue[1][2] - ball_queue[0][2] < ball_queue[2][2] - ball_queue[1][2]:
                ball_dis = (ball_queue[0][0] + ball_queue[1][0]) / 2
                ball_theta = (ball_queue[0][1] + ball_queue[1][1]) / 2
            else:
                ball_dis = (ball_queue[1][0] + ball_queue[2][0]) / 2
                ball_theta = (ball_queue[1][1] + ball_queue[2][1]) / 2
            self.cmd_shovel.control_shovel(1) #down the shovel
            self.move_cmd.move_to(math.tan(ball_theta)*ball_dis + KinectOffset + start_position*0.0, 0, 0)
            self.move_cmd.move_to(0, ball_dis - GetBallOffset, 0)
        else:
            (ball_dis, ball_theta) = self.find_ball.findball_cw(self.dir*RotateSpeed*midlinedirection, MidlineMaxBallDis)
            theta = -math.atan((ball_dis*math.tan(ball_theta) + KinectOffset)/(ball_dis+0.275))
            dir_dis = ball_dis/math.cos(ball_theta)
            if dir_dis > 0.80:
                #Move to 0.8m in front of ball
                self.turn_cmd.turn_to(theta)
                current_w = self.current_position.get_robot_current_w()
                current_x = self.current_position.get_robot_current_x()
                current_y = self.current_position.get_robot_current_y()
                if current_w > math.pi:
                    current_w = current_w - 2*math.pi
                if(abs(current_w) > math.pi/4):
                    move_dis = dir_dis*(abs(math.sin(current_w)) - math.cos(current_w))
                    self.world_move_cmd.move_to(current_x - math.copysign(move_dis, current_w), current_y, math.copysign(math.pi/4, current_w))
                    self.move_cmd.move_to(0, dir_dis*math.cos(current_w)*math.sqrt(2) - 0.80, 0)
                else:
                    self.move_cmd.move_to(0, dir_dis - 0.8, 0)
                rospy.sleep(0.2)
                ball_queue = []
                for i in range(0,3):
                    (ball_dis, ball_theta) = self.find_ball.findball_cw(0, SecondDetectDis)
                    ball_queue.append((ball_dis, ball_theta, ball_dis*math.tan(ball_theta)))
                ball_queue.sort(lambda x,y:cmp(x[2], y[2]))
                if ball_queue[1][2] - ball_queue[0][2] < ball_queue[2][2] - ball_queue[1][2]:
                    ball_dis = (ball_queue[0][0] + ball_queue[1][0]) / 2
                    ball_theta = (ball_queue[0][1] + ball_queue[1][1]) / 2
                else:
                    ball_dis = (ball_queue[1][0] + ball_queue[2][0]) / 2
                    ball_theta = (ball_queue[1][1] + ball_queue[2][1]) / 2
                self.cmd_shovel.control_shovel(1) #down the shovel
                self.move_cmd.move_to(math.tan(ball_theta)*ball_dis + KinectOffset + start_position*0.0, 0, 0)
                self.move_cmd.move_to(0, ball_dis - GetBallOffset, 0)
            elif dir_dis > 0:
                self.turn_cmd.turn_to(theta)
                rospy.sleep(0.2)
                ball_queue = []
                for i in range(0,3):
                    (ball_dis, ball_theta) = self.find_ball.findball_cw(0, SecondDetectDis)
                    ball_queue.append((ball_dis, ball_theta, ball_dis*math.tan(ball_theta)))
                ball_queue.sort(lambda x,y:cmp(x[2], y[2]))
                if ball_queue[1][2] - ball_queue[0][2] < ball_queue[2][2] - ball_queue[1][2]:
                    ball_dis = (ball_queue[0][0] + ball_queue[1][0]) / 2
                    ball_theta = (ball_queue[0][1] + ball_queue[1][1]) / 2
                else:
                    ball_dis = (ball_queue[1][0] + ball_queue[2][0]) / 2
                    ball_theta = (ball_queue[1][1] + ball_queue[2][1]) / 2
                self.cmd_shovel.control_shovel(1) #down the shovel
                self.move_cmd.move_to(math.tan(ball_theta)*ball_dis + KinectOffset + start_position*0.0, 0, 0)
                self.move_cmd.move_to(0, ball_dis - GetBallOffset, 0)
            else:
                return

    def execute(self, ud):
        self.search()
        if self.balldetect.detect() == False:
            self.move_cmd.move_to(0, -1.4, 0)
            self.cmd_shovel.control_shovel(2)
            self.search()
        rospy.sleep(0.1)
        self.cmd_shovel.control_shovel(3)
        rospy.sleep(0.5)
        self.move_cmd.move_to(0, -1.0, 0)
        return 'succeed'


##
# @brief 移动至三分线找球的开始位置
class MoveToThreePtlineSearch(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ['succeed', 'failed'])
        self.move_cmd       = world_move.world_move()
    
    def execute(self, ud):
        rospy.logwarn('Moving to midline search ball point~')
        if self.preempt_requested():
            self.service_preempt()
        self.move_cmd.move_to(ThreePtLineSearch_.x, ThreePtLineSearch_.y, ThreePtLineSearch_.z)
        return 'succeed'

##
# @brief 按先x后y的顺序移动至三分线找球的开始位置
class MoveToThreePtlineSearch1(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ['succeed', 'failed'])
        self.move_cmd       = world_move.world_move()
        self.current_position= get_robot_position.robot_position_state()
    
    def execute(self, ud):
        rospy.logwarn('Moving to midline search ball point~')
        if self.preempt_requested():
            self.service_preempt()
        current_x, current_y, current_w = self.current_position.get_robot_current_x_y_w()
        self.move_cmd.move_to(ThreePtLineSearch_.x, current_y, ThreePtLineSearch_.z)
        self.move_cmd.move_to(ThreePtLineSearch_.x, ThreePtLineSearch_.y, ThreePtLineSearch_.z)
        return 'succeed'

##
# @brief 移动至三分线传球位置
# This will risk confronting against rival
class MoveToThreePtlinePass(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ['succeed', 'failed'])
        self.move_cmd       = world_move.world_move()
        self.current_position = get_robot_position.robot_position_state()

    def execute(self, ud):
        rospy.logwarn('Moveing to midline pass point~')
        if self.preempt_requested():
            self.service_preempt()
        current_y = self.current_position.get_robot_current_y()
        current_w = self.current_position.get_robot_current_w()
        self.move_cmd.move_to(ThreePtlinePass_.x, current_y, current_w)
        self.move_cmd.move_to(ThreePtlinePass_.x, ThreePtlinePass_.y, ThreePtlinePass_.z)
        return 'succeed'

##
# @brief 沿着三分线公转寻找篮球
class ThreePtLineSearchBasketball(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ['succeed', 'failed'])
        self.move_cmd       = move_in_robot.move_in_robot()
        self.world_move_cmd = world_move.world_move()
        self.turn_cmd       = turn_an_angular.turn_an_angular()
        self.cmd_shovel     = control_srv.shovelControlSrv()
        self.balldetect     = control_srv.BallDetectSrv()
        self.find_ball      = find_ball.find_ball(1)
        self.current_position = get_robot_position.robot_position_state()
        self.dir            = 0
        self.deviation      = 0
        if start_position == 0:
            self.dir = 1
            self.deviation = 0.05
        else:
            self.dir = -1
            self.deviation = 0


    def search(self):
        rospy.logwarn('Three Pt Line Searching Basketball!~')
        if self.preempt_requested():
            self.service_preempt()
        radius = math.sqrt(math.pow(math.fabs(ThreePtLineSearch_.x - ThreePtLineSearch_.center_x), 2 ) + math.pow(math.fabs(ThreePtLineSearch_.y - ThreePtLineSearch_.center_y), 2))
        #(ball_dis, ball_theta) = self.find_ball.threePtLinefind(radius, ThreePtLineSearch_.center_x, ThreePtLineSearch_.center_y, -self.dir*0.15, 1.6)
        (ball_dis, ball_theta) = self.find_ball.threePtLinefind(radius, ThreePtLineSearch_.center_x, ThreePtLineSearch_.center_y, self.dir*0.15, 1.6)
        theta = -math.atan((ball_dis*math.tan(ball_theta) + KinectOffset)/(ball_dis+0.275))
        dir_dis = ball_dis/math.cos(ball_theta)
        rospy.logwarn('ball dis is now %f' % ball_dis)

        if dir_dis > 0.80:     #If ball detected and its distance is fairly long
            rospy.logwarn('ball dis longer than 0.8, which is now %f' % ball_dis)
            self.turn_cmd.turn_to(theta)
            self.move_cmd.move_to(0, dir_dis - 0.80,  0)
            rospy.logwarn('I\'m moving closer! %f' % ball_dis)
            rospy.sleep(0.2)
            if ball_theta > 0:
                (ball_dis, ball_theta) = self.find_ball.findball_cw(-0.3, SecondDetectDis)
            else:
                (ball_dis, ball_theta) = self.find_ball.findball_cw(0.3, SecondDetectDis)
            rospy.sleep(0.2)
            ball_queue = []
            for i in range(0,3):
                (ball_dis, ball_theta) = self.find_ball.findball_cw(0, SecondDetectDis)
                ball_queue.append((ball_dis, ball_theta, ball_dis*math.tan(ball_theta)))
            ball_queue.sort(lambda x,y:cmp(x[2], y[2]))
            if ball_queue[1][2] - ball_queue[0][2] < ball_queue[2][2] - ball_queue[1][2]:
                ball_dis = (ball_queue[0][0] + ball_queue[1][0]) / 2
                ball_theta = (ball_queue[0][1] + ball_queue[1][1]) / 2
            else:
                ball_dis = (ball_queue[1][0] + ball_queue[2][0]) / 2
                ball_theta = (ball_queue[1][1] + ball_queue[2][1]) / 2
            self.cmd_shovel.control_shovel(1)
            self.move_cmd.move_to(math.tan(ball_theta)*ball_dis + KinectOffset + self.deviation, 0, 0)
            self.move_cmd.move_to(0, ball_dis - GetBallOffset, 0)
            rospy.logwarn('I get the ball~ ')

        elif ball_dis > 0:      #If ball detected and its close
            rospy.logwarn('ball dis shorter than 0.8, which is now %f' % ball_dis)
            rospy.sleep(0.2)
            self.turn_cmd.turn_to(theta)
            #(ball_dis, ball_theta) = self.find_ball.findball_cw(0.0, SecondDetectDis)
            ball_queue = []
            for i in range(0,3):
                (ball_dis, ball_theta) = self.find_ball.findball_cw(0, SecondDetectDis)
                ball_queue.append((ball_dis, ball_theta, ball_dis*math.tan(ball_theta)))
            ball_queue.sort(lambda x,y:cmp(x[2], y[2]))
            if ball_queue[1][2] - ball_queue[0][2] < ball_queue[2][2] - ball_queue[1][2]:
                ball_dis = (ball_queue[0][0] + ball_queue[1][0]) / 2
                ball_theta = (ball_queue[0][1] + ball_queue[1][1]) / 2
            else:
                ball_dis = (ball_queue[1][0] + ball_queue[2][0]) / 2
                ball_theta = (ball_queue[1][1] + ball_queue[2][1]) / 2
            self.cmd_shovel.control_shovel(1)
            self.move_cmd.move_to(math.tan(ball_theta)*ball_dis + KinectOffset + self.deviation, 0, 0)
            self.move_cmd.move_to(0, ball_dis - GetBallOffset, 0)
            rospy.logwarn('I get the ball~ ')

        else:                   #If No ball detected, search ball in a reverse direction
            rospy.logwarn('Shit, Now ball found. Try again ')
            #(ball_dis, ball_theta) = self.find_ball.threePtLinefind(radius, ThreePtLineSearch_.center_x, ThreePtLineSearch_.center_y, self.dir*0.15, 1.6)
            (ball_dis, ball_theta) = self.find_ball.threePtLinefind(radius, ThreePtLineSearch_.center_x, ThreePtLineSearch_.center_y, -self.dir*0.15, 1.6)
            theta = -math.atan((ball_dis*math.tan(ball_theta) + KinectOffset)/(ball_dis+0.275))
            dir_dis = ball_dis/math.cos(ball_theta)
            if dir_dis > 0.80:
                rospy.logwarn('ball dis longer than 0.8, which is now %f' % ball_dis)
                self.turn_cmd.turn_to(theta)
                self.move_cmd.move_to(0, dir_dis - 0.80,  0)
                rospy.sleep(0.2)
                if ball_theta > 0:
                    (ball_dis, ball_theta) = self.find_ball.findball_cw(-0.3, SecondDetectDis)
                else:
                    (ball_dis, ball_theta) = self.find_ball.findball_cw(0.3, SecondDetectDis)
                rospy.sleep(0.2)
                ball_queue = []
                for i in range(0,3):
                    (ball_dis, ball_theta) = self.find_ball.findball_cw(0, SecondDetectDis)
                    ball_queue.append((ball_dis, ball_theta, ball_dis*math.tan(ball_theta)))
                ball_queue.sort(lambda x,y:cmp(x[2], y[2]))
                if ball_queue[1][2] - ball_queue[0][2] < ball_queue[2][2] - ball_queue[1][2]:
                    ball_dis = (ball_queue[0][0] + ball_queue[1][0]) / 2
                    ball_theta = (ball_queue[0][1] + ball_queue[1][1]) / 2
                else:
                    ball_dis = (ball_queue[1][0] + ball_queue[2][0]) / 2
                    ball_theta = (ball_queue[1][1] + ball_queue[2][1]) / 2
                self.cmd_shovel.control_shovel(1)
                self.move_cmd.move_to(math.tan(ball_theta)*ball_dis + KinectOffset + self.deviation, 0, 0)
                self.move_cmd.move_to(0, ball_dis - GetBallOffset, 0)

            elif ball_dis > 0:
                rospy.logwarn('ball dis shorter than 0.8, which is now %f' % ball_dis)
                rospy.sleep(0.2)
                self.turn_cmd.turn_to(theta)
                ball_queue = []
                for i in range(0,3):
                    (ball_dis, ball_theta) = self.find_ball.findball_cw(0, SecondDetectDis)
                    ball_queue.append((ball_dis, ball_theta, ball_dis*math.tan(ball_theta)))
                ball_queue.sort(lambda x,y:cmp(x[2], y[2]))
                if ball_queue[1][2] - ball_queue[0][2] < ball_queue[2][2] - ball_queue[1][2]:
                    ball_dis = (ball_queue[0][0] + ball_queue[1][0]) / 2
                    ball_theta = (ball_queue[0][1] + ball_queue[1][1]) / 2
                else:
                    ball_dis = (ball_queue[1][0] + ball_queue[2][0]) / 2
                    ball_theta = (ball_queue[1][1] + ball_queue[2][1]) / 2
                self.cmd_shovel.control_shovel(1)
                self.move_cmd.move_to(math.tan(ball_theta)*ball_dis + KinectOffset + self.deviation, 0, 0)
                self.move_cmd.move_to(0, ball_dis - GetBallOffset, 0)

            else:
                rospy.logwarn('Holly Shit, I failed')
                return 

    def execute(self, ud):
        self.search()

        if self.balldetect.detect() == False:
            self.move_cmd.move_to(0, -1.4, 0)
            self.cmd_shovel.control_shovel(2)
            self.search()
        rospy.sleep(0.1)
        self.cmd_shovel.control_shovel(3)

        current_x, current_y = self.current_position.get_robot_current_x_y()
        global gap_x
        gap_x = current_x
        global gap_y
        gap_y = current_y
        return 'succeed'

##
# @brief 沿着三分线公转寻找排球
class ThreePtLineSearchVolleyball(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ['succeed', 'failed'])
        self.move_cmd       = move_in_robot.move_in_robot()
        self.world_move_cmd = world_move.world_move()
        self.turn_cmd       = turn_an_angular.turn_an_angular()
        self.cmd_shovel     = control_srv.shovelControlSrv()
        self.balldetect     = control_srv.BallDetectSrv()
        self.find_ball      = find_ball.find_ball(2)
        self.current_position = get_robot_position.robot_position_state()
        self.dir            = 0
        self.deviation      = 0
        if start_position == 0:
            self.dir = 1
            self.deviation = 0.05
        else:
            self.dir = -1
            self.deviation = 0


    def search(self):
        rospy.logwarn('Three Pt Line Searching Volleyball!~')
        if self.preempt_requested():
            self.service_preempt()
        radius = math.sqrt(math.pow(math.fabs(ThreePtLineSearch_.x - ThreePtLineSearch_.center_x), 2 ) + math.pow(math.fabs(ThreePtLineSearch_.y - ThreePtLineSearch_.center_y), 2))
        #(ball_dis, ball_theta) = self.find_ball.threePtLinefind(radius, ThreePtLineSearch_.center_x, ThreePtLineSearch_.center_y, -self.dir*0.15, 1.6)
        (ball_dis, ball_theta) = self.find_ball.threePtLinefind(radius, ThreePtLineSearch_.center_x, ThreePtLineSearch_.center_y, self.dir*0.15, 1.6)
        theta = -math.atan((ball_dis*math.tan(ball_theta) + KinectOffset)/(ball_dis+0.275))
        dir_dis = ball_dis/math.cos(ball_theta)
        rospy.logwarn('ball dis is now %f' % ball_dis)

        if dir_dis > 0.80:     #If ball detected and its distance is fairly long
            rospy.logwarn('ball dis longer than 0.8, which is now %f' % ball_dis)
            self.turn_cmd.turn_to(theta)
            self.move_cmd.move_to(0, dir_dis - 0.80,  0)
            rospy.logwarn('I\'m moving closer! %f' % ball_dis)
            rospy.sleep(0.2)
            if ball_theta > 0:
                (ball_dis, ball_theta) = self.find_ball.findball_cw(-0.3, SecondDetectDis)
            else:
                (ball_dis, ball_theta) = self.find_ball.findball_cw(0.3, SecondDetectDis)
            rospy.sleep(0.2)
            ball_queue = []
            for i in range(0,3):
                (ball_dis, ball_theta) = self.find_ball.findball_cw(0, SecondDetectDis)
                ball_queue.append((ball_dis, ball_theta, ball_dis*math.tan(ball_theta)))
            ball_queue.sort(lambda x,y:cmp(x[2], y[2]))
            if ball_queue[1][2] - ball_queue[0][2] < ball_queue[2][2] - ball_queue[1][2]:
                ball_dis = (ball_queue[0][0] + ball_queue[1][0]) / 2
                ball_theta = (ball_queue[0][1] + ball_queue[1][1]) / 2
            else:
                ball_dis = (ball_queue[1][0] + ball_queue[2][0]) / 2
                ball_theta = (ball_queue[1][1] + ball_queue[2][1]) / 2
            self.cmd_shovel.control_shovel(1)
            self.move_cmd.move_to(math.tan(ball_theta)*ball_dis + KinectOffset + self.deviation, 0, 0)
            self.move_cmd.move_to(0, ball_dis - GetBallOffset, 0)
            rospy.logwarn('I get the ball~ ')

        elif ball_dis > 0:      #If ball detected and its close
            rospy.logwarn('ball dis shorter than 0.8, which is now %f' % ball_dis)
            rospy.sleep(0.2)
            self.turn_cmd.turn_to(theta)
            #(ball_dis, ball_theta) = self.find_ball.findball_cw(0.0, SecondDetectDis)
            ball_queue = []
            for i in range(0,3):
                (ball_dis, ball_theta) = self.find_ball.findball_cw(0, SecondDetectDis)
                ball_queue.append((ball_dis, ball_theta, ball_dis*math.tan(ball_theta)))
            ball_queue.sort(lambda x,y:cmp(x[2], y[2]))
            if ball_queue[1][2] - ball_queue[0][2] < ball_queue[2][2] - ball_queue[1][2]:
                ball_dis = (ball_queue[0][0] + ball_queue[1][0]) / 2
                ball_theta = (ball_queue[0][1] + ball_queue[1][1]) / 2
            else:
                ball_dis = (ball_queue[1][0] + ball_queue[2][0]) / 2
                ball_theta = (ball_queue[1][1] + ball_queue[2][1]) / 2
            self.cmd_shovel.control_shovel(1)
            self.move_cmd.move_to(math.tan(ball_theta)*ball_dis + KinectOffset + self.deviation, 0, 0)
            self.move_cmd.move_to(0, ball_dis - GetBallOffset, 0)
            rospy.logwarn('I get the ball~ ')

        else:                   #If No ball detected, search ball in a reverse direction
            rospy.logwarn('Shit, Now ball found. Try again ')
            #(ball_dis, ball_theta) = self.find_ball.threePtLinefind(radius, ThreePtLineSearch_.center_x, ThreePtLineSearch_.center_y, self.dir*0.15, 1.6)
            (ball_dis, ball_theta) = self.find_ball.threePtLinefind(radius, ThreePtLineSearch_.center_x, ThreePtLineSearch_.center_y, -self.dir*0.15, 1.6)
            theta = -math.atan((ball_dis*math.tan(ball_theta) + KinectOffset)/(ball_dis+0.275))
            dir_dis = ball_dis/math.cos(ball_theta)
            if dir_dis > 0.80:
                rospy.logwarn('ball dis longer than 0.8, which is now %f' % ball_dis)
                self.turn_cmd.turn_to(theta)
                self.move_cmd.move_to(0, dir_dis - 0.80,  0)
                rospy.sleep(0.2)
                if ball_theta > 0:
                    (ball_dis, ball_theta) = self.find_ball.findball_cw(-0.3, SecondDetectDis)
                else:
                    (ball_dis, ball_theta) = self.find_ball.findball_cw(0.3, SecondDetectDis)
                rospy.sleep(0.2)
                ball_queue = []
                for i in range(0,3):
                    (ball_dis, ball_theta) = self.find_ball.findball_cw(0, SecondDetectDis)
                    ball_queue.append((ball_dis, ball_theta, ball_dis*math.tan(ball_theta)))
                ball_queue.sort(lambda x,y:cmp(x[2], y[2]))
                if ball_queue[1][2] - ball_queue[0][2] < ball_queue[2][2] - ball_queue[1][2]:
                    ball_dis = (ball_queue[0][0] + ball_queue[1][0]) / 2
                    ball_theta = (ball_queue[0][1] + ball_queue[1][1]) / 2
                else:
                    ball_dis = (ball_queue[1][0] + ball_queue[2][0]) / 2
                    ball_theta = (ball_queue[1][1] + ball_queue[2][1]) / 2
                self.cmd_shovel.control_shovel(1)
                self.move_cmd.move_to(math.tan(ball_theta)*ball_dis + KinectOffset + self.deviation, 0, 0)
                self.move_cmd.move_to(0, ball_dis - GetBallOffset, 0)

            elif ball_dis > 0:
                rospy.logwarn('ball dis shorter than 0.8, which is now %f' % ball_dis)
                rospy.sleep(0.2)
                self.turn_cmd.turn_to(theta)
                ball_queue = []
                for i in range(0,3):
                    (ball_dis, ball_theta) = self.find_ball.findball_cw(0, SecondDetectDis)
                    ball_queue.append((ball_dis, ball_theta, ball_dis*math.tan(ball_theta)))
                ball_queue.sort(lambda x,y:cmp(x[2], y[2]))
                if ball_queue[1][2] - ball_queue[0][2] < ball_queue[2][2] - ball_queue[1][2]:
                    ball_dis = (ball_queue[0][0] + ball_queue[1][0]) / 2
                    ball_theta = (ball_queue[0][1] + ball_queue[1][1]) / 2
                else:
                    ball_dis = (ball_queue[1][0] + ball_queue[2][0]) / 2
                    ball_theta = (ball_queue[1][1] + ball_queue[2][1]) / 2
                self.cmd_shovel.control_shovel(1)
                self.move_cmd.move_to(math.tan(ball_theta)*ball_dis + KinectOffset + self.deviation, 0, 0)
                self.move_cmd.move_to(0, ball_dis - GetBallOffset, 0)

            else:
                rospy.logwarn('Holly Shit, I failed')
                return 

    def execute(self, ud):
        self.search()

        if self.balldetect.detect() == False:
            self.move_cmd.move_to(0, -1.4, 0)
            self.cmd_shovel.control_shovel(2)
            self.search()
        rospy.sleep(0.1)
        self.cmd_shovel.control_shovel(3)

        current_x, current_y = self.current_position.get_robot_current_x_y()
        self.world_move_cmd.move_to(current_x, current_y, -self.dir*math.pi/2)
        global gap_x
        gap_x = current_x
        global gap_y
        gap_y = current_y
        return 'succeed'

##
# @brief 在三分线内置球区寻找排球
class InnerThreePtLineSearchVolleyball(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ['succeed', 'failed'])
        self.world_move_cmd = world_move.world_move()
        self.move_cmd       = move_in_robot.move_in_robot()
        self.turn_cmd       = turn_an_angular.turn_an_angular()
        self.cmd_shovel     = control_srv.shovelControlSrv()
        self.find_ball      = find_ball.find_ball(2)
        self.current_position = get_robot_position.robot_position_state()
        self.balldetect     = control_srv.BallDetectSrv()
        self.dir            = 0
        if start_position == 0:
            self.dir = 1
        else:
            self.dir = -1
    
    def search(self):
        (ball_dis, ball_theta) = self.find_ball.findball_cw(- self.dir*RotateSpeed, 3.0)
        theta = -math.atan((ball_dis*math.tan(ball_theta) + KinectOffset)/(ball_dis+0.275))
        dir_dis = ball_dis/math.cos(ball_theta)

        if dir_dis > 0.80:
            self.turn_cmd.turn_to(theta)
            self.move_cmd.move_to(0, ball_dis - 0.80,  0) 
            if ball_theta > 0:
                (ball_dis, ball_theta) = self.find_ball.findball_cw(-0.3, SecondDetectDis)
            else:
                (ball_dis, ball_theta) = self.find_ball.findball_cw(0.3, SecondDetectDis)
            ball_queue = []
            for i in range(0,3):
                (ball_dis, ball_theta) = self.find_ball.findball_cw(0, SecondDetectDis)
                ball_queue.append((ball_dis, ball_theta, ball_dis*math.tan(ball_theta)))
            ball_queue.sort(lambda x,y:cmp(x[2], y[2]))
            if ball_queue[1][2] - ball_queue[0][2] < ball_queue[2][2] - ball_queue[1][2]:
                ball_dis = (ball_queue[0][0] + ball_queue[1][0]) / 2
                ball_theta = (ball_queue[0][1] + ball_queue[1][1]) / 2
            else:
                ball_dis = (ball_queue[1][0] + ball_queue[2][0]) / 2
                ball_theta = (ball_queue[1][1] + ball_queue[2][1]) / 2
            self.cmd_shovel.control_shovel(1)
            #self.move_cmd.move_to(math.tan(ball_theta)*ball_dis + KinectOffset + 0.06, 0, 0)
            self.move_cmd.move_to(math.tan(ball_theta)*ball_dis + KinectOffset, 0, 0)
            self.move_cmd.move_to(0, ball_dis - GetBallOffset, 0)
        elif dir_dis > 0:
            if ball_theta > 0:
                (ball_dis, ball_theta) = self.find_ball.findball_cw(-0.3, SecondDetectDis)
            else:
                (ball_dis, ball_theta) = self.find_ball.findball_cw(0.3, SecondDetectDis)
            ball_queue = []
            for i in range(0,3):
                (ball_dis, ball_theta) = self.find_ball.findball_cw(0, SecondDetectDis)
                ball_queue.append((ball_dis, ball_theta, ball_dis*math.tan(ball_theta)))
            ball_queue.sort(lambda x,y:cmp(x[2], y[2]))
            if ball_queue[1][2] - ball_queue[0][2] < ball_queue[2][2] - ball_queue[1][2]:
                ball_dis = (ball_queue[0][0] + ball_queue[1][0]) / 2
                ball_theta = (ball_queue[0][1] + ball_queue[1][1]) / 2
            else:
                ball_dis = (ball_queue[1][0] + ball_queue[2][0]) / 2
                ball_theta = (ball_queue[1][1] + ball_queue[2][1]) / 2
            self.cmd_shovel.control_shovel(1)
            self.move_cmd.move_to(math.tan(ball_theta)*ball_dis + KinectOffset, 0, 0)
            self.move_cmd.move_to(0, ball_dis - GetBallOffset, 0)
        else:
            (ball_dis, ball_theta) = self.find_ball.findball_cw(- self.dir*RotateSpeed, 3.0)
            theta = -math.atan((ball_dis*math.tan(ball_theta) + KinectOffset)/(ball_dis+0.275))
            dir_dis = ball_dis/math.cos(ball_theta)
            if dir_dis > 0.80:
                self.turn_cmd.turn_to(theta)
                self.move_cmd.move_to(0, ball_dis - 0.80,  0) 
                if ball_theta > 0:
                    (ball_dis, ball_theta) = self.find_ball.findball_cw(-0.3, SecondDetectDis)
                else:
                    (ball_dis, ball_theta) = self.find_ball.findball_cw(0.3, SecondDetectDis)
                ball_queue = []
                for i in range(0,3):
                    (ball_dis, ball_theta) = self.find_ball.findball_cw(0, SecondDetectDis)
                    ball_queue.append((ball_dis, ball_theta, ball_dis*math.tan(ball_theta)))
                ball_queue.sort(lambda x,y:cmp(x[2], y[2]))
                if ball_queue[1][2] - ball_queue[0][2] < ball_queue[2][2] - ball_queue[1][2]:
                    ball_dis = (ball_queue[0][0] + ball_queue[1][0]) / 2
                    ball_theta = (ball_queue[0][1] + ball_queue[1][1]) / 2
                else:
                    ball_dis = (ball_queue[1][0] + ball_queue[2][0]) / 2
                    ball_theta = (ball_queue[1][1] + ball_queue[2][1]) / 2
                self.cmd_shovel.control_shovel(1)
                self.move_cmd.move_to(math.tan(ball_theta)*ball_dis + KinectOffset, 0, 0)
                self.move_cmd.move_to(0, ball_dis - GetBallOffset, 0)
            elif dir_dis > 0:
                if ball_theta > 0:
                    (ball_dis, ball_theta) = self.find_ball.findball_cw(-0.3, SecondDetectDis)
                else:
                    (ball_dis, ball_theta) = self.find_ball.findball_cw(0.3, SecondDetectDis)
                ball_queue = []
                for i in range(0,3):
                    (ball_dis, ball_theta) = self.find_ball.findball_cw(0, SecondDetectDis)
                    ball_queue.append((ball_dis, ball_theta, ball_dis*math.tan(ball_theta)))
                ball_queue.sort(lambda x,y:cmp(x[2], y[2]))
                if ball_queue[1][2] - ball_queue[0][2] < ball_queue[2][2] - ball_queue[1][2]:
                    ball_dis = (ball_queue[0][0] + ball_queue[1][0]) / 2
                    ball_theta = (ball_queue[0][1] + ball_queue[1][1]) / 2
                else:
                    ball_dis = (ball_queue[1][0] + ball_queue[2][0]) / 2
                    ball_theta = (ball_queue[1][1] + ball_queue[2][1]) / 2
                self.cmd_shovel.control_shovel(1)
                self.move_cmd.move_to(math.tan(ball_theta)*ball_dis + KinectOffset, 0, 0)
                self.move_cmd.move_to(0, ball_dis - GetBallOffset, 0)
            else:
                return
    def execute(self, ud):
        rospy.logerr('Prepare to Move~')
        self.world_move_cmd.move_to(ThreePtInnerSearch_.x, ThreePtInnerSearch_.y, ThreePtInnerSearch_.z)
        rospy.logerr('Prepare to Search ball~')
        self.search()
        if self.balldetect.detect() == False:
            self.move_cmd.move_to(0, -1.4, 0)
            self.cmd_shovel.control_shovel(2)
            self.search()
        rospy.sleep(0.1)
        if self.balldetect.detect():
            self.cmd_shovel.control_shovel(3)
        else:
            return 'failed'
        rospy.sleep(0.5)
        #global gap2_x
        #global gap2_y
        #gap2_x = self.current_position.get_robot_current_x()
        #gap2_y = self.current_position.get_robot_current_y()
        return 'succeed'

# Return by the way NewBee come
class GoOut(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeed', 'failed'])
        self.move_cmd = world_move.world_move()
        self.robot_move = move_in_robot.move_in_robot()
        self.dir                = 0
        if start_position == 0:
            self.dir = 1
        else:
            self.dir = -1
    def execute(self, ud):
        self.move_cmd.move_to(gap2_x, gap2_y, gap2_z)
        self.robot_move.move_to(0, -2.4, 0)
        return 'succeed'

# Move to where we got ball last time
class MoveToThreePtlineGap(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ['succeed', 'failed'])
        self.move_cmd           = world_move.world_move()
        self.turn_cmd           = turn_an_angular.turn_an_angular()
        self.current_position   = get_robot_position.robot_position_state()
        self.dir                = 0
        if start_position == 0:
            self.dir = 1
        else:
            self.dir = -1
    
    def execute(self, ud):
        rospy.logwarn('Ready to move to the Three Pt Line Gap!~')
        current_x           = self.current_position.get_robot_current_x()
        self.move_cmd.move_to(current_x, gap_y, - self.dir*math.pi/2)
        self.move_cmd.move_to(gap_x + self.dir*0.1, gap_y, - self.dir*math.pi/2)
        return 'succeed'

##
# @brief 从中线位置移动到三分线铲球留下的缺口
class MoveToThreePtlineGapFromMidline(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ['succeed', 'failed'])
        self.move_cmd           = world_move.world_move()
        self.turn_cmd           = turn_an_angular.turn_an_angular()
        self.current_position   = get_robot_position.robot_position_state()
        self.dir                = 0
        if start_position == 0:
            self.dir = 1
        else:
            self.dir = -1
    
    def execute(self, ud):
        rospy.logwarn('Ready to move to the Three Pt Line Gap!~')
        self.move_cmd.move_to(ThreePtLineSearch_.x, ThreePtLineSearch_.y, - self.dir*math.pi/2)
        current_x           = self.current_position.get_robot_current_x()
        self.move_cmd.move_to(current_x, gap_y, - self.dir*math.pi/2)
        self.move_cmd.move_to(gap_x + self.dir*0.1, gap_y, - self.dir*math.pi/2)
        return 'succeed'

##
# @brief 锁定定位柱
class LockCylinder(smach.State):
    def __init__(self, Round):
        smach.State.__init__(self, outcomes = ['succeed', 'failed'])
        self.move_cmd           = move_in_robot.move_in_robot()
        self.round              = Round
        self.world_move_cmd     = world_move.world_move()
        self.turn_cmd           = turn_an_angular.turn_an_angular()
        self.find_cylinder      = find_cylinder.find_cylinder()
        self.current_position   = get_robot_position.robot_position_state()
        self.dir                = 0
        self.CameraOffsetX      = 0.100
        self.CameraOffsetY      = 0.275
        self.CylinderOffset     = 2.78
        if start_position == 0:
            self.dir = 1
        else:
            self.dir = -1
    
    def execute(self, ud):
        rospy.logwarn('Start Shoot~')
        if self.preempt_requested():
            self.service_preempt()
        (cylinder_dis, cylinder_theta) = self.find_cylinder.findcylinder_cw(self.dir * 0.3)
        rospy.logwarn('Now speed is %d' % self.dir)
        theta = -math.atan((cylinder_dis*math.tan(cylinder_theta) + self.CameraOffsetX)/(cylinder_dis+self.CameraOffsetY))
        #self.turn_cmd.turn_to(theta)
        rospy.sleep(0.3)
        (cylinder_dis, cylinder_theta) = self.find_cylinder.findcylinder_cw(0)
        theta = -math.atan((cylinder_dis*math.tan(cylinder_theta) + self.CameraOffsetX)/(cylinder_dis+self.CameraOffsetY))
        self.turn_cmd.turn_to(theta)
        (cylinder_dis, cylinder_theta) = self.find_cylinder.findcylinder_cw(0)
        current_x, current_y, current_w = self.current_position.get_robot_current_x_y_w()
        if current_w > math.pi:
            current_w -= 2*math.pi
        # 解算出以当前角度后退至投球距离后的全局坐标，以判定是否会碰到三分线内置球线的球或是出界
        next_x = current_x + (self.CylinderOffset - cylinder_dis)*math.sin(current_w)
        next_y = current_y - (self.CylinderOffset - cylinder_dis)*math.cos(current_w)
        rospy.logwarn('Next x and y are %f, %f' % (next_x, next_y))
        if self.round == 3:
            # 如果有可能碰到球, 向边上靠靠再对准柱子
            if abs(next_x) < 8.0:
                cylinder_x = current_x - (cylinder_dis + self.CameraOffsetY)*math.sin(current_w)
                cylinder_y = current_y + (cylinder_dis + self.CameraOffsetY)*math.cos(current_w)
                rospy.logwarn('Cylinder x and y are %f, %f' % (cylinder_x, cylinder_y))
                if abs(cylinder_x - self.dir*8.0)/(self.CylinderOffset + self.CameraOffsetY) <= 1.0:
                    rospy.logwarn('Adjusting~~~~')
                    dest_x     = self.dir*8.0
                    dest_y     = cylinder_y - (self.CylinderOffset + self.CameraOffsetY)*math.cos(math.asin(abs(cylinder_x - self.dir*8.0)/(self.CylinderOffset + self.CameraOffsetY)))
                    dest_w     = -math.asin((cylinder_x - self.dir*8.0)/(self.CylinderOffset + self.CameraOffsetY))
                    self.world_move_cmd.move_to(dest_x, dest_y, dest_w)
                    (cylinder_dis, cylinder_theta) = self.find_cylinder.findcylinder_cw(0)
                    theta = -math.atan((cylinder_dis*math.tan(cylinder_theta) + self.CameraOffsetX)/(cylinder_dis+self.CameraOffsetY))
                    self.turn_cmd.turn_to(theta)
            # 如果可能会出界，向里靠靠再对准柱子
            if next_y < 0.5:
                cylinder_x = current_x - (cylinder_dis + self.CameraOffsetY)*math.sin(current_w)
                cylinder_y = current_y + (cylinder_dis + self.CameraOffsetY)*math.cos(current_w)
                if abs(cylinder_x - self.dir*8.0)/(self.CylinderOffset + self.CameraOffsetY) <= 1.0:
                    dest_x     = self.dir*8.0
                    dest_y     = cylinder_y - (self.CylinderOffset + self.CameraOffsetY)*math.cos(math.asin(abs(cylinder_x - self.dir*8.0)/(self.CylinderOffset + self.CameraOffsetY)))
                    dest_w     = -math.asin((cylinder_x - self.dir*8.0)/(self.CylinderOffset + self.CameraOffsetY))
                    self.world_move_cmd.move_to(dest_x, dest_y, dest_w)
                    (cylinder_dis, cylinder_theta) = self.find_cylinder.findcylinder_cw(0)
                    theta = -math.atan((cylinder_dis*math.tan(cylinder_theta) + self.CameraOffsetX)/(cylinder_dis+self.CameraOffsetY))
                    self.turn_cmd.turn_to(theta)
        self.move_cmd.move_to(-0.04, -(self.CylinderOffset - cylinder_dis), 0.0)
        return 'succeed'

# Move to a point to start search cylinder(x first)
class MoveToShootPoint(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ['succeed', 'failed'])
        self.move_cmd           = world_move.world_move()
        self.current_position   = get_robot_position.robot_position_state()
        
    def execute(self, ud):
        rospy.logwarn('Moving to shoot point~')
        if self.preempt_requested():
            self.service_preempt()
        current_y = self.current_position.get_robot_current_y()
        self.move_cmd.move_to(ShootPoint_.x, current_y, ShootPoint_.z)
        rospy.logwarn('Shootpoing is %f', ShootPoint_.x)
        self.move_cmd.move_to(ShootPoint_.x, ShootPoint_.y, ShootPoint_.z)
        return 'succeed'

# Move to a point to start search cylinder(y first)
class MoveToShootPoint2(smach.State): 
    def __init__(self):
        smach.State.__init__(self, outcomes = ['succeed', 'failed'])
        self.move_cmd           = world_move.world_move()
        self.current_position   = get_robot_position.robot_position_state()
        
    def execute(self, ud):
        rospy.logwarn('Moving to shoot point~')
        if self.preempt_requested():
            self.service_preempt()
        current_x = self.current_position.get_robot_current_y()
        self.move_cmd.move_to(current_x, ShootPoint_.y, ShootPoint_.z)
        self.move_cmd.move_to(ShootPoint_.x, ShootPoint_.y, ShootPoint_.z)
        return 'succeed'

# cross the balls placed on inner line
class GoThroughBalls(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ['succeed', 'failed'])
        self.move_cmd           = world_move.world_move()
        self.robot_move         = move_in_robot.move_in_robot()
        self.find_ball          = find_ball.find_ball(0)
        self.turn_cmd           = turn_an_angular.turn_an_angular()
        self.current_position   = get_robot_position.robot_position_state()
        self.cmd_shovel         = control_srv.shovelControlSrv()
        self.dir                = 0
        self.ball_gap_x         = 0
        self.ball_gap_y         = 0
        self.robot_speed_puber  = rospy.Publisher('/robot_move/robot_velocity', geometry_msgs.msg.Twist, queue_size=100)
        if start_position == 0:
            self.dir = 1
        else:
            self.dir = -1

    def execute(self, ud):
        rospy.sleep(0.2)
        rospy.logwarn('Moving through Inner line balls!~')
        obs = self.find_ball.find_obs_ball(0.6, 2.0)
        if len(obs) == 0:
            obs = self.find_ball.find_obs_ball(0.6, 2.0)
        if len(obs) == 0:
            self.ball_gap_y = self.current_position.get_robot_current_y()
        elif len(obs) == 1:
            obs_y = obs[0][0] * math.tan(obs[0][1]) * self.dir + self.current_position.get_robot_current_y() #- 0.125
            print 'obs_y is %f' % obs_y
            if abs(obs_y - self.current_position.get_robot_current_y()) > 0.45:
                self.ball_gap_y = self.current_position.get_robot_current_y()
            elif obs_y > 2.3:
                self.ball_gap_y = obs_y - 0.5
            else:
                self.ball_gap_y = obs_y + 0.5
        elif len(obs) == 2:
            obs_y1 = obs[0][0] * math.tan(obs[0][1]) * self.dir + self.current_position.get_robot_current_y() #- 0.125
            obs_y2 = obs[1][0] * math.tan(obs[1][1]) * self.dir + self.current_position.get_robot_current_y() #- 0.125
            self.ball_gap_y     = (obs_y1 + obs_y2) / 2.0 
            print 'obs_y1, obs_y2 are %f, %f' % (obs_y1, obs_y2)
        else:
            obs_y1 = obs[0][0] * math.tan(obs[0][1]) * self.dir + self.current_position.get_robot_current_y() #- 0.125
            obs_y2 = obs[1][0] * math.tan(obs[1][1]) * self.dir + self.current_position.get_robot_current_y() #- 0.125
            obs_y3 = obs[2][0] * math.tan(obs[2][1]) * self.dir + self.current_position.get_robot_current_y() #- 0.125
            print 'obs_y1, obs_y2, obs_y3 are %f, %f, %f' % (obs_y1, obs_y2, obs_y3)
            if obs_y1 > obs_y2:
                temp = obs_y1
                obs_y1 = obs_y2
                obs_y2 = temp
            if obs_y1 > obs_y3:
                temp = obs_y1
                obs_y1 = obs_y3
                obs_y3 = temp
            if obs_y2 > obs_y3:
                temp = obs_y2
                obs_y2 = obs_y3
                obs_y3 = temp
            if math.fabs(obs_y1 - obs_y2) < 0.8:
                self.ball_gap_y     = (obs_y1 + obs_y2) / 2.0
            else:
                self.ball_gap_y     = (obs_y2 + obs_y3) / 2.0
        global point1
        global point2
        current_y = self.current_position.get_robot_current_y()
        rospy.logwarn('Ball Gap y is %f ' % self.ball_gap_y)
        self.move_cmd.move_to(self.dir * 6.90, current_y, -self.dir * math.pi/2)
        self.move_cmd.move_to(self.dir * 6.90, self.ball_gap_y, -self.dir * math.pi/2)
        point1.append(self.dir*6.90)
        point1.append(self.ball_gap_y)
        point1.append(- self.dir*math.pi/2)
        move_velocity = geometry_msgs.msg.Twist()
        move_velocity.linear.y = 0.6
        start_x = self.current_position.get_robot_current_x()
        self.robot_speed_puber.publish(move_velocity)
        while not rospy.is_shutdown():
            current_x = self.current_position.get_robot_current_x()
            if abs(current_x - start_x) > 1.6:
                break
        current_x = self.current_position.get_robot_current_x()
        point2.append(current_x)
        point2.append(self.ball_gap_y)
        point2.append(- self.dir*math.pi/2)
        self.turn_cmd.turn_to(math.pi/6)
        self.cmd_shovel.control_shovel(2)
        return 'succeed'

# cross the balls placed on three pt line
class GoThroughThreePtline(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeed', 'failed'])
        self.move_cmd           = move_in_robot.move_in_robot()
        self.world_move_cmd     = world_move.world_move()
        self.find_obs_ball      = find_ball.find_ball(0)
        self.rotate_client      = rospy.ServiceProxy('/robot_move/rotate_cmd_srv', robot_rotate)
        self.current_position   = get_robot_position.robot_position_state()
        self.dir                = 0
        self.robot_speed_puber  = rospy.Publisher('/robot_move/robot_velocity', geometry_msgs.msg.Twist, queue_size=100)
        if start_position == 0:
            self.dir = 1
        else:
            self.dir = -1

    def execute(self, ud):
        angle = - math.atan((ThreePtLineSearch_.center_x - ThreePtLineSearch_.x) / (ThreePtLineSearch_.center_y - ThreePtLineSearch_.y))
        required_angle = math.asin(0.5/3.75)
        radius = math.sqrt(math.pow(math.fabs(ThreePtLineSearch_.x - ThreePtLineSearch_.center_x), 2 ) + math.pow(math.fabs(ThreePtLineSearch_.y - ThreePtLineSearch_.center_y), 2))
        self.world_move_cmd.move_to(ThreePtLineSearch_.x, ThreePtLineSearch_.y, angle)
        obs_balls = self.find_obs_ball.find_obs_ball(0.6, 1.6)
        if len(obs_balls) == 0:
            pass
        elif len(obs_balls) == 1:
            if abs(math.tan(obs_balls[0][1]) * obs_balls[0][0]) > 0.45:
                pass
            else:
                start_w = self.current_position.get_robot_current_w()
                AngleToTurn = required_angle - math.asin(abs(math.tan(obs_balls[0][1]) * obs_balls[0][0]) / 3.75)
                speed = math.copysign(0.15, obs_balls[0][1])
                self.rotate_client(radius, radius, ThreePtLineSearch_.center_x, ThreePtLineSearch_.center_y, speed, speed*1.0)
                current_w = self.current_position.get_robot_current_w()
                while abs(current_w - start_w) < AngleToTurn:
                    current_w = self.current_position.get_robot_current_w()
                    rospy.sleep(0.05)
                self.rotate_client(radius, radius, ThreePtLineSearch_.center_x, ThreePtLineSearch_.center_y, 0, 0)
                
        else:
            obs_balls.sort(lambda x,y:cmp(x[0], y[0]))
            if abs(math.tan(obs_balls[0][1]) * obs_balls[0][0]) > 0.4:
                pass
            else:
                start_w = self.current_position.get_robot_current_w()
                AngleToTurn = required_angle - math.asin(abs(math.tan(obs_balls[0][1]) * obs_balls[0][0]) / 3.75)
                speed = math.copysign(0.15, obs_balls[0][1])
                self.rotate_client(radius, radius, ThreePtLineSearch_.center_x, ThreePtLineSearch_.center_y, speed, speed*0.85)
                current_w = self.current_position.get_robot_current_w()
                while abs(current_w - start_w) < AngleToTurn:
                    current_w = self.current_position.get_robot_current_w()
                    rospy.sleep(0.05)
                self.rotate_client(radius, radius, ThreePtLineSearch_.center_x, ThreePtLineSearch_.center_y, 0, 0)
        move_velocity = geometry_msgs.msg.Twist()
        move_velocity.linear.y = 0.6
        rospy.sleep(0.5)
        start_x = self.current_position.get_robot_current_x()
        start_w = self.current_position.get_robot_current_w()
        self.robot_speed_puber.publish(move_velocity)
        while not rospy.is_shutdown():
            current_x = self.current_position.get_robot_current_x()
            if abs(current_x - start_x)/abs(math.cos(current_w)) > 1.6:
                break
        move_velocity.linear.y = 0
        self.robot_speed_puber.publish(move_velocity)
        self.move_cmd.move_to(0, 0.6, 0)
        global gap2_x
        global gap2_y
        global gap2_z
        gap2_x, gap2_y, gap2_z = self.current_position.get_robot_current_x_y_w()
        return 'succeed'


class GoBack(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeed', 'failed'])
        self.move_cmd       = world_move.world_move()
        self.robot_speed_puber = rospy.Publisher('/robot_move/robot_velocity', geometry_msgs.msg.Twist, queue_size=100)
        self.current_position  = get_robot_position.robot_position_state()
        self.dir = 0
        if start_position == 0:
            self.dir = 1
        else:
            self.dir = -1
    
    def execute(self, ud):
        self.move_cmd.move_to(point2[0], point2[1], point2[2])
        move_velocity = geometry_msgs.msg.Twist()
        move_velocity.linear.y = -0.6
        start_x = self.current_position.get_robot_current_x()
        self.robot_speed_puber.publish(move_velocity)
        while not rospy.is_shutdown():
            current_x = self.current_position.get_robot_current_x()
            if abs(current_x - start_x) > 1.6:
                break
            rospy.sleep(0.1)
        #self.move_cmd.move_to(point1[0], point1[1], point1[2])
        self.move_cmd.move_to(gap_x, gap_y, point2[2])
        return 'succeed'

# grab the balls on the inner line(similar to MidlineSearch)
class InnerLineSearchBasketball(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeed', 'failed'])
        self.move_cmd       = move_in_robot.move_in_robot()
        self.world_move_cmd     = world_move.world_move()
        self.turn_cmd       = turn_an_angular.turn_an_angular()
        self.cmd_shovel     = control_srv.shovelControlSrv()
        self.find_ball      = find_ball.find_ball(1)
        self.balldetect     = control_srv.BallDetectSrv()
        self.dir            = 0
        if start_position == 0:
            self.dir = 1
            self.deviation = 0.07
        else:
            self.dir = -1
            self.deviation = 0.05
    
    def search(self):
        self.world_move_cmd.move_to(point2[0], point2[1], point2[2] - self.dir*math.pi/1.1)
        rospy.logwarn('Inner line Searching Basketball!~')
        if self.preempt_requested():
            self.service_preempt()
        (ball_dis, ball_theta) = self.find_ball.findball_cw(-self.dir*RotateSpeed, InnerLineMaxBallDis)
        theta = -math.atan((ball_dis*math.tan(ball_theta) + KinectOffset)/(ball_dis+0.275))
        dir_dis = ball_dis/math.cos(ball_theta)
        if dir_dis > 0.80:
            #Move to 0.8m in front of ball
            self.turn_cmd.turn_to(theta)
            self.move_cmd.move_to(0, dir_dis - 0.80, 0)
            ball_queue = []
            for i in range(0,3):
                (ball_dis, ball_theta) = self.find_ball.findball_cw(0, SecondDetectDis)
                ball_queue.append((ball_dis, ball_theta, ball_dis*math.tan(ball_theta)))
            ball_queue.sort(lambda x,y:cmp(x[2], y[2]))
            if ball_queue[1][2] - ball_queue[0][2] < ball_queue[2][2] - ball_queue[1][2]:
                ball_dis = (ball_queue[0][0] + ball_queue[1][0]) / 2
                ball_theta = (ball_queue[0][1] + ball_queue[1][1]) / 2
            else:
                ball_dis = (ball_queue[1][0] + ball_queue[2][0]) / 2
                ball_theta = (ball_queue[1][1] + ball_queue[2][1]) / 2
            self.cmd_shovel.control_shovel(1) #down the shovel
            rospy.sleep(0.5)
            #self.move_cmd.move_to(math.tan(ball_theta)*ball_dis + KinectOffset + (1 - start_position)*0.05, 0, 0)
            self.move_cmd.move_to(math.tan(ball_theta)*ball_dis + KinectOffset + self.deviation, 0, 0)
            self.move_cmd.move_to(0, ball_dis - GetBallOffset, 0)
        elif dir_dis > 0:
            self.turn_cmd.turn_to(theta)
            ball_queue = []
            for i in range(0,3):
                (ball_dis, ball_theta) = self.find_ball.findball_cw(0, SecondDetectDis)
                ball_queue.append((ball_dis, ball_theta, ball_dis*math.tan(ball_theta)))
            ball_queue.sort(lambda x,y:cmp(x[2], y[2]))
            if ball_queue[1][2] - ball_queue[0][2] < ball_queue[2][2] - ball_queue[1][2]:
                ball_dis = (ball_queue[0][0] + ball_queue[1][0]) / 2
                ball_theta = (ball_queue[0][1] + ball_queue[1][1]) / 2
            else:
                ball_dis = (ball_queue[1][0] + ball_queue[2][0]) / 2
                ball_theta = (ball_queue[1][1] + ball_queue[2][1]) / 2
            self.cmd_shovel.control_shovel(1) #down the shovel
            rospy.sleep(0.5)
            self.move_cmd.move_to(math.tan(ball_theta)*ball_dis + KinectOffset + self.deviation, 0, 0)
            self.move_cmd.move_to(0, ball_dis - GetBallOffset, 0)
        else:
            (ball_dis, ball_theta) = self.find_ball.findball_cw(self.dir*RotateSpeed, InnerLineMaxBallDis)
            theta = -math.atan((ball_dis*math.tan(ball_theta) + KinectOffset)/(ball_dis+0.275))
            dir_dis = ball_dis/math.cos(ball_theta)
            if dir_dis > 0.80:
                #Move to 0.8m in front of ball
                self.turn_cmd.turn_to(theta)
                self.move_cmd.move_to(0, dir_dis - 0.80, 0)
                ball_queue = []
                for i in range(0,3):
                    (ball_dis, ball_theta) = self.find_ball.findball_cw(0, SecondDetectDis)
                    ball_queue.append((ball_dis, ball_theta, ball_dis*math.tan(ball_theta)))
                ball_queue.sort(lambda x,y:cmp(x[2], y[2]))
                if ball_queue[1][2] - ball_queue[0][2] < ball_queue[2][2] - ball_queue[1][2]:
                    ball_dis = (ball_queue[0][0] + ball_queue[1][0]) / 2
                    ball_theta = (ball_queue[0][1] + ball_queue[1][1]) / 2
                else:
                    ball_dis = (ball_queue[1][0] + ball_queue[2][0]) / 2
                    ball_theta = (ball_queue[1][1] + ball_queue[2][1]) / 2
                self.cmd_shovel.control_shovel(1) #down the shovel
                rospy.sleep(0.5)
                self.move_cmd.move_to(math.tan(ball_theta)*ball_dis + KinectOffset + self.deviation, 0, 0)
                self.move_cmd.move_to(0, ball_dis - GetBallOffset, 0)
            elif dir_dis > 0:
                self.turn_cmd.turn_to(theta)
                ball_queue = []
                for i in range(0,3):
                    (ball_dis, ball_theta) = self.find_ball.findball_cw(0, SecondDetectDis)
                    ball_queue.append((ball_dis, ball_theta, ball_dis*math.tan(ball_theta)))
                ball_queue.sort(lambda x,y:cmp(x[2], y[2]))
                if ball_queue[1][2] - ball_queue[0][2] < ball_queue[2][2] - ball_queue[1][2]:
                    ball_dis = (ball_queue[0][0] + ball_queue[1][0]) / 2
                    ball_theta = (ball_queue[0][1] + ball_queue[1][1]) / 2
                else:
                    ball_dis = (ball_queue[1][0] + ball_queue[2][0]) / 2
                    ball_theta = (ball_queue[1][1] + ball_queue[2][1]) / 2
                self.cmd_shovel.control_shovel(1) #down the shovel
                rospy.sleep(0.5)
                self.move_cmd.move_to(math.tan(ball_theta)*ball_dis + KinectOffset + self.deviation, 0, 0)
                self.move_cmd.move_to(0, ball_dis - GetBallOffset, 0)
            else:
                return
    def execute(self, ud):
        self.search()
        if self.balldetect.detect() == False:
            self.move_cmd.move_to(0, -1.4, 0)
            self.cmd_shovel.control_shovel(2)
            self.search()
        rospy.sleep(0.1)
        #if self.balldetect.detect():
        self.cmd_shovel.control_shovel(3)
        self.move_cmd.move_to(0, -1.0, 0)
        return 'succeed'

class InnerLineSearchVolleyball(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeed', 'failed'])
        self.move_cmd           = move_in_robot.move_in_robot()
        self.world_move_cmd     = world_move.world_move()
        self.current_position   = get_robot_position.robot_position_state()
        self.turn_cmd           = turn_an_angular.turn_an_angular()
        self.cmd_shovel         = control_srv.shovelControlSrv()
        self.find_ball          = find_ball.find_ball(2)
        self.balldetect         = control_srv.BallDetectSrv()
        self.dir                = 0
        if start_position == 0:
            self.dir = 1
            self.deviation = 0.07
        else:
            self.deviation = 0.05
            self.dir = -1
    
    def search(self):
        self.world_move_cmd.move_to(point2[0], point2[1] + self.dir*0.4, point2[2] - self.dir*math.pi/1.2)
        rospy.logwarn('Inner line Searching Volleyball!~')
        if self.preempt_requested():
            self.service_preempt()
        (ball_dis, ball_theta) = self.find_ball.findball_cw(-self.dir*RotateSpeed, InnerLineMaxBallDis)
        theta = -math.atan((ball_dis*math.tan(ball_theta) + KinectOffset)/(ball_dis+0.275))
        dir_dis = ball_dis/math.cos(ball_theta)
        if dir_dis > 0.80:
            #Move to 0.8m in front of ball
            self.turn_cmd.turn_to(theta)
            self.move_cmd.move_to(0, dir_dis - 0.80, 0)
            ball_queue = []
            for i in range(0,3):
                (ball_dis, ball_theta) = self.find_ball.findball_cw(0, SecondDetectDis)
                ball_queue.append((ball_dis, ball_theta, ball_dis*math.tan(ball_theta)))
            ball_queue.sort(lambda x,y:cmp(x[2], y[2]))
            if ball_queue[1][2] - ball_queue[0][2] < ball_queue[2][2] - ball_queue[1][2]:
                ball_dis = (ball_queue[0][0] + ball_queue[1][0]) / 2
                ball_theta = (ball_queue[0][1] + ball_queue[1][1]) / 2
            else:
                ball_dis = (ball_queue[1][0] + ball_queue[2][0]) / 2
                ball_theta = (ball_queue[1][1] + ball_queue[2][1]) / 2
            self.cmd_shovel.control_shovel(1) #down the shovel
            rospy.sleep(0.1)
            self.move_cmd.move_to(math.tan(ball_theta)*ball_dis + KinectOffset + self.deviation, 0, 0)
            self.move_cmd.move_to(0, ball_dis - GetBallOffset, 0)
        elif dir_dis > 0:
            self.turn_cmd.turn_to(theta)
            ball_queue = []
            for i in range(0,3):
                (ball_dis, ball_theta) = self.find_ball.findball_cw(0, SecondDetectDis)
                ball_queue.append((ball_dis, ball_theta, ball_dis*math.tan(ball_theta)))
            ball_queue.sort(lambda x,y:cmp(x[2], y[2]))
            if ball_queue[1][2] - ball_queue[0][2] < ball_queue[2][2] - ball_queue[1][2]:
                ball_dis = (ball_queue[0][0] + ball_queue[1][0]) / 2
                ball_theta = (ball_queue[0][1] + ball_queue[1][1]) / 2
            else:
                ball_dis = (ball_queue[1][0] + ball_queue[2][0]) / 2
                ball_theta = (ball_queue[1][1] + ball_queue[2][1]) / 2
            self.cmd_shovel.control_shovel(1) #down the shovel
            rospy.sleep(0.5)
            self.move_cmd.move_to(math.tan(ball_theta)*ball_dis + KinectOffset + self.deviation, 0, 0)
            self.move_cmd.move_to(0, ball_dis - GetBallOffset, 0)
        else:
            (ball_dis, ball_theta) = self.find_ball.findball_cw(self.dir*RotateSpeed, InnerLineMaxBallDis)
            theta = -math.atan((ball_dis*math.tan(ball_theta) + KinectOffset)/(ball_dis+0.275))
            dir_dis = ball_dis/math.cos(ball_theta)
            if dir_dis > 0.80:
                #Move to 0.8m in front of ball
                self.turn_cmd.turn_to(theta)
                self.move_cmd.move_to(0, dir_dis - 0.80, 0)
                ball_queue = []
                for i in range(0,3):
                    (ball_dis, ball_theta) = self.find_ball.findball_cw(0, SecondDetectDis)
                    ball_queue.append((ball_dis, ball_theta, ball_dis*math.tan(ball_theta)))
                ball_queue.sort(lambda x,y:cmp(x[2], y[2]))
                if ball_queue[1][2] - ball_queue[0][2] < ball_queue[2][2] - ball_queue[1][2]:
                    ball_dis = (ball_queue[0][0] + ball_queue[1][0]) / 2
                    ball_theta = (ball_queue[0][1] + ball_queue[1][1]) / 2
                else:
                    ball_dis = (ball_queue[1][0] + ball_queue[2][0]) / 2
                    ball_theta = (ball_queue[1][1] + ball_queue[2][1]) / 2
                self.cmd_shovel.control_shovel(1) #down the shovel
                rospy.sleep(0.5)
                self.move_cmd.move_to(math.tan(ball_theta)*ball_dis + KinectOffset + self.deviation, 0, 0)
                self.move_cmd.move_to(0, ball_dis - GetBallOffset, 0)
            elif dir_dis > 0:
                self.turn_cmd.turn_to(theta)
                ball_queue = []
                for i in range(0,3):
                    (ball_dis, ball_theta) = self.find_ball.findball_cw(0, SecondDetectDis)
                    ball_queue.append((ball_dis, ball_theta, ball_dis*math.tan(ball_theta)))
                ball_queue.sort(lambda x,y:cmp(x[2], y[2]))
                if ball_queue[1][2] - ball_queue[0][2] < ball_queue[2][2] - ball_queue[1][2]:
                    ball_dis = (ball_queue[0][0] + ball_queue[1][0]) / 2
                    ball_theta = (ball_queue[0][1] + ball_queue[1][1]) / 2
                else:
                    ball_dis = (ball_queue[1][0] + ball_queue[2][0]) / 2
                    ball_theta = (ball_queue[1][1] + ball_queue[2][1]) / 2
                self.cmd_shovel.control_shovel(1) #down the shovel
                rospy.sleep(0.5)
                self.move_cmd.move_to(math.tan(ball_theta)*ball_dis + KinectOffset + self.deviation, 0, 0)
                self.move_cmd.move_to(0, ball_dis - GetBallOffset, 0)
            else:
                return
    def execute(self, ud):
        self.search()
        #if self.balldetect.detect() == False:
            #self.move_cmd.move_to(0, -1.4, 0)
            #self.cmd_shovel.control_shovel(2)
            #self.search()
        #rospy.sleep(0.1)
        #if self.balldetect.detect():
        self.cmd_shovel.control_shovel(3)
        #else:
            #return 'failed'
        rospy.sleep(0.3)
        self.move_cmd.move_to(0, -1.0, 0)
        current_x, current_y = self.current_position.get_robot_current_x_y()
        self.world_move_cmd.move_to(current_x, current_y, -self.dir*math.pi/2.1)
        return 'succeed'

# class InnerLineSearchVolleyball(smach.State):
#     def __init__(self):
#         smach.State.__init__(self, outcomes=['succeed', 'failed'])
#         self.move_cmd       = move_in_robot.move_in_robot()
#         self.world_move_cmd = world_move.world_move()
#         self.turn_cmd       = turn_an_angular.turn_an_angular()
#         self.cmd_shovel     = control_srv.shovelControlSrv()
#         self.find_ball      = find_ball.find_ball(2)
#         self.dir            = 0
#         if start_position == 0:
#             self.dir = 1
#         else:
#             self.dir = -1
    
#     def execute(self, ud):
#         self.world_move_cmd.move_to(point2[0], point2[1], point2[2] - self.dir*math.pi/1.5)
#         rospy.logwarn('Inner line Searching Basketball!~')
#         if self.preempt_requested():
#             self.service_preempt()
#         (ball_dis, ball_theta) = self.find_ball.findball_cw(-self.dir*RotateSpeed)
#         theta = -math.atan((ball_dis*math.tan(ball_theta) + KinectOffset)/(ball_dis+0.275))
#         dir_dis = ball_dis/math.cos(ball_theta)
#         if abs(dir_dis) > 0.80:
#             #Move to 0.8m in front of ball
#             self.turn_cmd.turn_to(theta)
#             self.move_cmd.move_to(0, dir_dis - 0.80, 0)
#             (ball_dis, ball_theta) = self.find_ball.findball_cw(self.dir*RotateSpeed)
#             (ball_dis, ball_theta) = self.find_ball.findball_cw(0)
#             self.cmd_shovel.control_shovel(1) #down the shovel
#             rospy.sleep(0.5)
#             self.move_cmd.move_to(math.tan(ball_theta)*ball_dis + KinectOffset, 0, 0)
#             self.move_cmd.move_to(0, ball_dis - GetBallOffset, 0)
#         else:
#             self.turn_cmd.turn_to(theta)
#             (ball_dis, ball_theta) = self.find_ball.findball_cw(0)
#             self.cmd_shovel.control_shovel(1) #down the shovel
#             rospy.sleep(0.5)
#             self.move_cmd.move_to(math.tan(ball_theta)*ball_dis + KinectOffset, 0, 0)
#             self.move_cmd.move_to(0, ball_dis - GetBallOffset, 0)
#         self.cmd_shovel.control_shovel(3)
#         rospy.sleep(0.5)
#         return 'succeed'

class MoveForwardY(smach.State):
    def __init__(self, distance):
        smach.State.__init__(self, outcomes=['succeed','failed'])
        self.move_cmd           = move_in_robot.move_in_robot()
        self.distance           = distance
    def execute(self, ud):
        self.move_cmd.move_to(0, self.distance, 0)
        return 'succeed'

class MoveForwardX(smach.State):
    def __init__(self, distance):
        smach.State.__init__(self, outcomes=['succeed','failed'])
        self.move_cmd           = move_in_robot.move_in_robot()
        self.distance           = distance
    def execute(self, ud):
        self.move_cmd.move_to(self.distance, 0, 0)
        return 'succeed'
