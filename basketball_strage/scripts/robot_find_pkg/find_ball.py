#!/usr/bin/env python
#coding:utf-8

# 检测球的相关接口
import math
import rospy
import geometry_msgs.msg as g_msgs
from robot_state_pkg import get_robot_position
from basketball_msgs.srv import robot_rotate, visionDate
from robot_state_class.start_position import *

class find_ball(object):
    def __init__(self, Type):
        self.cmd_angular_pub    = rospy.Publisher('/robot_move/world_velocity',g_msgs.Twist,queue_size=100)
        self.find_ball_client   = rospy.ServiceProxy('visionDate',visionDate)
        self.rotate_client      = rospy.ServiceProxy('/robot_move/rotate_cmd_srv', robot_rotate)
        self.current_position   = get_robot_position.robot_position_state()
        self.requiredType       = Type
    
    #发送急停速度，使机器人转动停止
    #停止时的回调函数
    #是机器人当前的所有速度为0
    def brake(self):
        move_velocity = g_msgs.Twist()
        move_velocity.linear.x = 0
        move_velocity.linear.y = 0
        move_velocity.angular.z = 0
        self.cmd_angular_pub.publish(move_velocity)
    
    def findBallAlongLine(self, speed, max_dis):
        rospy.logwarn('[visionDate]->waiting BallDate service')
        self.find_ball_client.wait_for_service()
        rospy.logwarn('[visionDate] -> connected to ball service')
        move_velocity = g_msgs.Twist()
        move_velocity.angular.z = speed
        index = 0
        has_required_ball = False
        ball_dis = -1
        ball_theta = 0
        move = True
        min_dis = 100
        min_dis_index = -1
        start_w = self.current_position.get_robot_current_w()
        if start_w > math.pi:
            start_w = start_w - 2*math.pi
        while not rospy.is_shutdown() :
            current_w = self.current_position.get_robot_current_w()
            rospy.logwarn('Looking for balls')
            if current_w > math.pi:
                current_w = current_w - 2*math.pi
            
            ##
            # @brief 如果此次搜寻的范围到达阈值而未寻找到目标，返回错误
            if math.fabs(current_w - start_w) > 150.0/180.0*math.pi:
                min_dis_index = -1
                ball_dis = -1
                break
            res = self.find_ball_client(1)
            if len(res.balls) >= 1:
                if res.balls[0].distance>0:
                    for index in range(0, len(res.balls)):
                        if res.balls[index].type == self.requiredType and res.balls[index].distance > 0:
                            has_required_ball = True
                            if res.balls[index].distance < min_dis:
                                min_dis = res.balls[index].distance
                                min_dis_index = index
            if min_dis_index < 0:
                has_required_ball = False
            if has_required_ball == True:
                if res.balls[min_dis_index].distance > max_dis:
                    rospy.logwarn('No ball in the accepted distance, the min distance is %f' % res.balls[min_dis_index].distance)
                    min_dis_index = -1
                elif res.balls[min_dis_index].distance > 0:
                    current_x, current_y, current_w = self.current_position.get_robot_current_x_y_w()
                    x_distance = res.balls[min_dis_index].distance*res.balls[min_dis_index].theta
                    y_distance = res.balls[min_dis_index].distance + 0.275
                    x_location = current_x + x_distance*math.cos(current_w) - y_distance*math.sin(current_w)
                    y_location = current_y + x_distance*math.sin(current_w) + y_distance*math.cos(current_w)
                    
                    ##
                    # @brief 如果球在场外，放弃
                    if y_location > 0.5 and y_location < 4.5 and abs(x_location) < 12:
                        break
                else:
                    pass
            if min_dis_index == -1:
                if move == True:
                    self.cmd_angular_pub.publish(move_velocity)
                    move = False
        if min_dis_index != -1:
            ball_dis = res.balls[min_dis_index].distance
            ball_theta = -res.balls[min_dis_index].theta
        self.brake()
        return (ball_dis, ball_theta)

    def threePtLinefind(self, radius, center_x, center_y, speed, max_dis):
        rospy.logwarn('[visionDate]->waiting BallDate service')
        self.find_ball_client.wait_for_service()
        self.rotate_client.wait_for_service()
        rospy.logwarn('[visionDate] -> connected to ball service')
        has_required_ball = False
        index       = 0
        ball_dis    = -1
        ball_theta  = 0
        rotate      = True
        min_dis     = 100
        flag        = False
        start_y     = self.current_position.get_robot_current_y()
        start_time  = rospy.get_time()
        min_dis_index = -1
        while not rospy.is_shutdown():
            has_required_ball = False
            current_y = self.current_position.get_robot_current_y()
            current_time = rospy.get_time()
            if speed == 0 and current_time - start_time > 1.2:
                if start_position == 0:
                    speed = -0.3
                else:
                    speed = 0.3
            if abs(current_y - start_y) > 2.3:            
                min_dis_index = -1
                ball_dis      = -1
                break                                       ## If beyond range, stop and return failed(ball_dis = -1)
            res = self.find_ball_client(1)
            if len(res.balls) >= 1:                         ## If balls detected
                if res.balls[0].distance > 0:
                    for index in range(0, len(res.balls)):
                        ## If required balls are detected
                        if res.balls[index].type == self.requiredType and res.balls[index].distance > 0:
                            has_required_ball = True
                            ## Get the info of closest ball 
                            if res.balls[index].distance < min_dis:
                                min_dis = res.balls[index].distance
                                min_dis_index = index
            ## Judge whether the closest ball is in the expected area
            if min_dis_index < 0:
                has_required_ball = False
            if has_required_ball == True:
                if res.balls[min_dis_index].distance > max_dis:
                    rospy.logwarn('the min distance is %f' % res.balls[min_dis_index].distance)
                    flag = False
                    min_dis_index = -1
                elif res.balls[min_dis_index].distance > 0:
                    current_x, current_y, current_w = self.current_position.get_robot_current_x_y_w()
                    x_distance = res.balls[min_dis_index].distance*math.tan(res.balls[min_dis_index].theta)
                    y_distance = res.balls[min_dis_index].distance + 0.275
                    x_location = current_x + x_distance*math.cos(current_w) - y_distance*math.sin(current_w)
                    y_location = current_y + x_distance*math.sin(current_w) + y_distance*math.cos(current_w)
                    if y_location > 0.5 and y_location < 4.5 and abs(x_location) < 12:
                        break
                else:
                    pass
            ## If no right ball found, move
            if flag == False:
                if rotate == True:
                    self.rotate_client(radius, radius, center_x, center_y, speed, 0.0)
                    rotate = False
        if min_dis_index != -1:
            ball_dis = res.balls[min_dis_index].distance
            ball_theta = -res.balls[min_dis_index].theta
        self.brake()
        return (ball_dis, ball_theta)

    def findball_cw(self, speed, max_dis):
        rospy.logwarn('[visionDate]->waiting BallDate service')
        self.find_ball_client.wait_for_service()
        rospy.logwarn('[visionDate] -> connected to ball service')
        move_velocity = g_msgs.Twist()
        move_velocity.angular.z = speed
        index = 0
        has_required_ball = False
        ball_dis = -1
        ball_theta = 0
        move = True
        min_dis = 100
        min_dis_index = -1
        start_w = self.current_position.get_robot_current_w()
        start_time = rospy.get_time()
        if start_w > math.pi:
            start_w = start_w - 2*math.pi
        while not rospy.is_shutdown() :
            current_w = self.current_position.get_robot_current_w()
            current_time = rospy.get_time()
            if speed == 0 and current_time - start_time > 1.2:
                speed = -0.3
            rospy.logwarn('Looking for balls')
            if current_w > math.pi:
                current_w = current_w - 2*math.pi
            if math.fabs(current_w - start_w) > 150.0/180.0*math.pi:
                min_dis_index = -1
                ball_dis = -1
                break
            res = self.find_ball_client(1)
            if len(res.balls) >= 1:
                if res.balls[0].distance>0:
                    for index in range(0, len(res.balls)):
                        if res.balls[index].type == self.requiredType and res.balls[index].distance > 0:
                            has_required_ball = True
                            if res.balls[index].distance < min_dis:
                                min_dis = res.balls[index].distance
                                min_dis_index = index
            if min_dis_index < 0:
                has_required_ball = False
            if has_required_ball == True:
                if res.balls[min_dis_index].distance > max_dis:
                    rospy.logwarn('No ball in the accepted distance, the min distance is %f' % res.balls[min_dis_index].distance)
                    min_dis_index = -1
                elif res.balls[min_dis_index].distance > 0:
                    current_x, current_y, current_w = self.current_position.get_robot_current_x_y_w()
                    x_distance = res.balls[min_dis_index].distance*math.tan(res.balls[min_dis_index].theta)
                    y_distance = res.balls[min_dis_index].distance + 0.275
                    x_location = current_x + x_distance*math.cos(current_w) - y_distance*math.sin(current_w)
                    y_location = current_y + x_distance*math.sin(current_w) + y_distance*math.cos(current_w)
                    if y_location > 0.5 and y_location < 4.5 and abs(x_location) < 12:
                        break
                    break
                else:
                    pass
            if min_dis_index == -1:
                if move == True:
                    self.cmd_angular_pub.publish(move_velocity)
                    move = False
        if min_dis_index != -1:
            ball_dis = res.balls[min_dis_index].distance
            ball_theta = -res.balls[min_dis_index].theta
        self.brake()
        return (ball_dis, ball_theta)

    ## Detect the obstacle balls in front of NewBee in a specific range 
    def find_obs_ball(self, min_dis, max_dis):
        obs_ball = []       
        rospy.logwarn('[visionDate]->waiting BallDate service')
        self.find_ball_client.wait_for_service()
        rospy.logwarn('[visionDate] -> connected to ball service')
        index=0
        res = self.find_ball_client(1)
        if len(res.balls) >= 1:
            if res.balls[0].distance>0:
                for index in range(0, len(res.balls)):
                    if res.balls[index].type != self.requiredType and res.balls[index].distance >= min_dis and res.balls[index].distance <= max_dis:
                        obs_ball.append((res.balls[index].distance, res.balls[index].theta))
        else:
            pass
        return obs_ball

if __name__ == '__main__':
    rospy.init_node('find_basketball')

