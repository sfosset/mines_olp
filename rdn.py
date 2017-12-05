#!/usr/bin/env python
# -*- encoding: UTF-8 -*-
import os
import sys
import time
import rospy

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from math import *

class Pioneer:
    def __init__(self, rospy):

        rospy.init_node('RDN')

        self.r = 0.096
        self.R = 0.27

        self.vp_msg = Twist()
        self.cmd_vp_pub = rospy.Publisher('/RosAria/cmd_vel', Twist, queue_size=1)
        self.sub = rospy.Subscriber("/RosAria/pose", Odometry, self.CB_pos)

        self.mySin =0
        self.myCos =0
        self.myPX = 0
        self.myPY = 0
        self.myPTheta = 0


    def kill(self):
        self.exit()

    def get_position(self):
        """Get the position (x,y,theta) of the robot

        Return:
            position (list): the position [x,y,theta]
        """
        position = []
        position.append(self.myPX)
        position.append(self.myPY)
        position.append(self.myPTheta)
        return position

    def CB_pos(self, msg):
        self.mySin = msg.pose.pose.orientation.z
        self.myCos = msg.pose.pose.orientation.w
        self.myPX = msg.pose.pose.position.x
        self.myPY = msg.pose.pose.position.y
        self.myPTheta = 2 * atan2(self.mySin, self.myCos)

        if (self.myPTheta > 3.1415):
            self.myPTheta = self.myPTheta - 6.2830

        #self.upd([mySin,myCos,myPX,myPY, myPTheta])

    # def upd(self,list):
    #     self.mySin = list[0]
    #     self.myCos = list[1]
    #     self.myPX = list[2]
    #     self.myPY = list[3]
    #     self.myPTheta = list[4]

    def set_motor_velocity(self, control):
        """Set a target velocity on the pioneer motors, multiplied by the gain
        defined in self.gain

        Args:
            control(list): the control [left_motor, right_motor]
        """
        vg = control[0]*self.r
        vd = control[1]*self.r
        self.vp_msg.linear.x = 10*0.5*(vg+vd)
        self.vp_msg.angular.z= 0.5*(vd-vg)/self.R
        #print ("pubishing velocity" , str(self.vp_msg.linear.x) ," , ",self.vp_msg.angular.z)
        self.cmd_vp_pub.publish(self.vp_msg)
