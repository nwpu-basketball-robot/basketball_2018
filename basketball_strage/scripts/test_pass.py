#!/usr/bin/python
# -*- coding: UTF-8 -*-
import rospy
import math
from robot_move_pkg import world_move
from robot_shovel_srv import control_srv

def AdjustShoot(x, y, z):
    move_cmd = world_move.world_move()
    shoot_cmd = control_srv.shootControlSrv()
    move_cmd.move_to(x, y, z)
    rospy.sleep(0.5)
    shoot_cmd.shoot_ball()


if __name__ == '__main__':
    x = input()
    y = input()
    z = input()
    AdjustShoot(x, y, z/180.0*math.pi)
    rospy.logwarn('shoot over')

