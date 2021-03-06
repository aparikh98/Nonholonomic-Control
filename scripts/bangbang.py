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
import matplotlib.pyplot as plt

def callback(data):
    return data.x, data.y, data.theta, data.phi
class BangBang(object):
    """docstring for BangBang"""

    def __init__(self):
        self.pub = rospy.Publisher('/bicycle/cmd_vel', BicycleCommandMsg, queue_size=10)
        self.pose_subscriber = rospy.Subscriber('/bicycle/state', BicycleStateMsg, self.update_pose)
        self.rate = rospy.Rate(1)
        self.state = np.array([0.0,0.0,0.0,0.0])
        self.state_record = []
        self.goal = np.array([0.0,0.0,0.0,0.0])

    def update_pose(self, data):
        self.state = np.array([data.x, data.y, data.theta, data.phi])

    def run(self):
        #self.translate_theta()
        for i in range(2):
            self.turn(4, 1)
            if rospy.is_shutdown():
               self.cmd(0,0)
               break
            # self.strafe()

    def translate_x(self, dist):
        goal = np.array([dist,0.0,0.0,0.0])
        self.goal = goal
        distance = goal - self.state
        while (distance[0] > 0.01):
            self.cmd(distance[0], distance[3])
            if rospy.is_shutdown():
                self.cmd(0,0)
                break
            self.rate.sleep()
            distance = goal - self.state
            print(self.state, distance)
        self.cmd(0,0)
        self.rate.sleep()

    def move_forward(self, dist):
        initial_pos = np.array([self.state[0], self.state[1]])
        self.goal = np.array([self.state[0] + np.cos(self.state[2]) * dist, self.state[1] + np.sin(self.state[2]) * dist, self.state[2], self.state[3]])
        current_pos = np.array([self.state[0], self.state[1]])
        distance = np.linalg.norm(current_pos-initial_pos)
        while (distance < dist ):
            self.cmd(dist/2, -1 * self.state[3])
            if rospy.is_shutdown():
                self.cmd(0,0)
                break
            self.rate.sleep()
            current_pos = np.array([self.state[0], self.state[1]])
            distance = np.linalg.norm(current_pos-initial_pos)
        self.cmd(0,0)
        self.rate.sleep()


    def translate_theta(self, angle):
        goal = np.array([0.0,0.0,angle,0.0])
        self.goal = goal
        distance = goal - self.state
        print(distance)
        while ((distance[2]) > 0.1 and distance[2] < 6):


            self.cmd( 0.3, 1)
            self.rate.sleep()
            self.cmd(-0.3,-1)
            if rospy.is_shutdown():
                print("SHUT DOWN")
                self.cmd(0,0)
                break
            self.rate.sleep()
            distance = goal - self.state
            print(self.state, distance)
        self.cmd(0,0)
        self.rate.sleep()

    # def rotate_theta2(self, angle):
    #     goal = np.array([0.0,0.0,angle,0.0])
    #     self.goal = goal
    #     distance = goal - self.state
    #     #sign = distance > 0 ? -1 : 1
    #     while (abs(distance[2]) > 0.05):
    #         if distance[2] > 0:
    #             sign = 1
    #         else:
    #             sign = -1

    #         self.cmd(0.3 ,sign * 2)
    #         if rospy.is_shutdown():
    #             print("SHUT DOWN")
    #             self.cmd(0,0)
    #             break
    #         self.rate.sleep()
    #         distance = goal - self.state
    #         print(self.state, distance)
    #     self.rate.sleep()
    #     self.rate.sleep()
    #     self.cmd(0,0)
    #     self.rate.sleep()

    def translate_y(self, dist):
        goal = np.array([0.0,dist,0.0,0.0])
        self.goal = goal
        distance = goal - self.state
        self.cmd(1, 1)
        self.rate.sleep()
        while (distance[1] > 0.1):

            self.cmd(0.5, -1)
            self.rate.sleep()
            self.cmd(-0.5, 1)
            self.rate.sleep()
            self.cmd(-0.5, -1)
            self.rate.sleep()
            self.cmd(0.5, 1)
            self.rate.sleep()
            self.cmd(0,0)

            if rospy.is_shutdown():
                self.cmd(0,0.)
                break
            self.rate.sleep()
            distance = goal - self.state
            print(self.state)
        self.cmd(0,0)
        self.rate.sleep()

    def translate_y_and_x(self):
        self.goal = np.array([0.5,0.5,0.0, 0.0])
        self.move_forward(0.2)
        self.translate_theta(np.pi/2)
        self.move_forward(0.6)
        self.translate_theta(np.pi*2)
        self.move_forward(0.2)

        self.cmd(0,0)
        print(self.state)

    def strafe_pos(self):
        pass

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
        self.state_record.append(self.state)
        self.pub.publish(BicycleCommandMsg(u1, u2))

    def plot(self):
        #this plotting is pretty bad since it assumes time steps are equal, but ehh

        f, axarr = plt.subplots(4, sharex=True)
        time = [i for i in range(len(self.state_record))]
        des = [self.goal[0]/(len(self.state_record)-1)* i  for i in range(len(self.state_record))]
        real = [record[0] for record in self.state_record]
        axarr[0].plot(time, des, color = 'r')
        axarr[0].plot(time, real, color = 'g')
        des = [self.goal[1] /(len(self.state_record)-1) * i  for i in range(len(self.state_record))]
        real = [record[1] for record in self.state_record]
        axarr[1].plot(time, des, color = 'r')
        axarr[1].plot(time, real, color = 'g')
        des = [self.goal[2] /(len(self.state_record)-1) * i  for i in range(len(self.state_record))]
        real = [record[2] for record in self.state_record]
        axarr[2].plot(time, des, color = 'r')
        axarr[2].plot(time, real, color = 'g')
        des = [self.goal[3] /(len(self.state_record)-1) * i  for i in range(len(self.state_record))]
        real = [record[3] for record in self.state_record]
        axarr[3].plot(time, des, color = 'r')
        axarr[3].plot(time, real, color = 'g')
        plt.show()

        f, ax = plt.subplots()
        x_des = [self.goal[0] /(len(self.state_record)-1) * i  for i in range(len(self.state_record))]
        y_des = [self.goal[1] /(len(self.state_record)-1) * i  for i in range(len(self.state_record))]
        x_real = [record[0] for record in self.state_record]
        y_real = [record[1] for record in self.state_record]
        ax.plot(x_des, y_des,  color='r')
        ax.plot(x_real, y_real,  color='b')
        plt.show()

if __name__ == '__main__':
    rospy.init_node('bangbang', anonymous=False)

    b = BangBang()
    b.translate_y_and_x()
    # b.translate_x(1.0)
    # b.translate_y(1)
    # b.translate_y(0.5)
    # b.translate_theta(3.14)
    b.plot()
    #while (the distance is quite large)
        # update d and magnitude according to the position error of turtle
        # execute the strafe movement
        # update distance between current and desired after the strafe


        #rosservice list
            #/converter/reset


