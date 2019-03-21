#!/usr/bin/env python
"""
Starter code for EE106B Turtlebot Lab
Author: Valmik Prabhu, Chris Correa
"""
import rospy
from lab3_pkg.msg import BicycleCommandMsg, BicycleStateMsg
import numpy as np
import tf2_ros
import tf


class BangBang(object):
    """docstring for BangBang"""
    def __init__(self):
        self.pub = rospy.Publisher('/bicycle/cmd_vel', BicycleCommandMsg, queue_size=10)
        self.rate = rospy.Rate(10)

    def run(self):
        for i in range(10):
            # self.turn(1, -1)
            if rospy.is_shutdown():
                self.cmd(0,0)
                break
            self.strafe()

    def strafe(self):
        self.turn(1, 1)
        self.rate.sleep()
        self.cmd(2, 0)
        self.rate.sleep()
        self.rate.sleep()
        self.turn(1, -1)
        self.rate.sleep()
        self.cmd(-2, 0)
        self.rate.sleep()
        self.rate.sleep()
        self.cmd(0,0) #important since safety function in bicycle converter as each command should only last 1/10 of a second.
        # Remember to set to 0,0 when you don't want to move.


    def turn(self, mag, d):
        self.cmd(mag, d*mag)
        self.rate.sleep()
        self.cmd(-mag, d*mag)
        self.rate.sleep()
        self.cmd(-mag, -d*mag)
        self.rate.sleep()
        self.cmd(mag, -d*mag)
        self.rate.sleep()
        self.cmd(0,0) #important since safety function in bicycle converter as each command should only last 1/10 of a second

    def cmd(self, u1, u2):
        self.pub.publish(BicycleCommandMsg(u1, u2))

if __name__ == '__main__':
    rospy.init_node('bangbang', anonymous=False)

    b = BangBang()
    b.run()
    #while (the distance is quite large)
        # update d and magnitude according to the position error of turtle
        # execute the strafe movement
        # update distance between current and desired after the strafe


        #rosservice list
            #/converter/reset
