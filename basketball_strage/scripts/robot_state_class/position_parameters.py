import math
from start_position import *

class RobotHome(object):
    def __init__(self):
        self.x = -0.0
        self.y = 0.0
        self.z = 0.0
    def reverse(self):
        self.x = - self.x

class MidlinePass(object):
    def __init__(self):
        self.x = 0.8
        self.y = 2.6
        self.z = -math.pi/1.5
    def reverse(self):
        self.x = - self.x
        self.z = - self.z
    
class MidlineSearch(object):
    def __init__(self):
        self.x = 0.9
        self.y = 2.8
        self.z = -math.pi/4.0
    def reverse(self):
        self.x = - self.x
        self.z = - self.z

class ShootPoint(object):
    def __init__(self):
        self.x = 8.1
        self.y = 3.75
        self.z = -math.pi/2.2
    def reverse(self):
        self.x = - self.x
        self.z = - self.z

class ThreePtLineSearch(object):
    def __init__(self):
        self.x = 4.5
        #self.x = 5.1
        #self.y = 1.6
        self.y = 3.7
        self.z = -math.pi/2
        self.center_x = 9.5
        self.center_y = 4.25
    def reverse(self):
        self.x = - self.x
        self.z = - self.z
        self.center_x = - self.center_x

class ThreePtlinePass(object):
    def __init__(self):
        self.x = 4.8
        self.y = 4.05
        self.z = 160.0/180.0*math.pi
    def reverse(self):
        self.x = - self.x
        self.z = - self.z

class ThreePtInnerSearch(object):
    def __init__(self):
        self.x = 8.0
        self.y = 2.4
        self.z = 160.0/180.0*math.pi
    def reverse(self):
        self.x = - self.x
        self.z = - self.z


RobotHome_ = RobotHome()
MidlinePass_ = MidlinePass()
MidlineSearch_ = MidlineSearch()
ShootPoint_ = ShootPoint()
ThreePtLineSearch_ = ThreePtLineSearch()
ThreePtInnerSearch_ = ThreePtInnerSearch()
ThreePtlinePass_ = ThreePtlinePass()
KinectOffset    = 0.01
MidlineMaxBallDis = 3.5
GetBallOffset   = 0.14
InnerLineMaxBallDis = 2.5
SecondDetectDis   = 1.2
RotateSpeed       = 0.3

if start_position == 1:
    RobotHome_.reverse()
    MidlinePass_.reverse()
    MidlineSearch_.reverse()
    ShootPoint_.reverse()
    ThreePtInnerSearch_.reverse()
    ThreePtlinePass_.reverse()
    ThreePtLineSearch_.reverse()
