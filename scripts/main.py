#!/usr/bin/env python
"""
Starter code for EE106B Turtlebot Lab
Author: Valmik Prabhu, Chris Correa
"""
import numpy as np
import sys
import argparse

import tf2_ros
import tf
from std_srvs.srv import Empty as EmptySrv
import rospy
import matplotlib.pyplot as plt

from lab3_pkg.msg import BicycleCommandMsg, BicycleStateMsg

from lab3.planners import SinusoidPlanner

class Exectutor(object):

        # We should make our implementations in here...

    def __init__(self):
        """
        Executes a plan made by the planner
        """
        self.pub = rospy.Publisher('/bicycle/cmd_vel', BicycleCommandMsg, queue_size=10)
        self.sub = rospy.Subscriber('/bicycle/state', BicycleStateMsg, self.subscribe )
        self.rate = rospy.Rate(100)
        self.state = BicycleStateMsg()
        rospy.on_shutdown(self.shutdown)
        self.state_record = []

    def calculateGain(self, plan):
        gains = []
        # plan[i] = 
        #  x0(t) state (state.x, state.y, state.theta, state.phi)
        #  u0(t) cmd  (cmd.linear_velocity, cmd.steering_rate)
        for (t, cmd, state) in plan:
            #gain should be an np.array
            
            gains.append(gain)

        return gains

    def executeGain(self, plan, gains):
        for i in range(len(plan)):
            newcmd = plan[i][1] - np.matmul(gains[i], (self.state - plan[i][2]))
            self.cmd(newcmd)
            self.rate.sleep()
            self.state_record.append((state,self.state))
            if rospy.is_shutdown():
                break
        self.cmd(BicycleCommandMsg())

    def execute(self, plan):
        """
        Executes a plan made by the planner

        Parameters
        ----------
        plan : :obj:`list` of (time, BicycleCommandMsg, BicycleStateMsg)
        self.state -> x(t)
        state -> x0(t)
        cmd -> u0(t)
        """
        if len(plan) == 0:
            return

        for (t, cmd, state) in plan:
            self.cmd(cmd)
            self.rate.sleep()
            self.state_record.append((state,self.state))
            if rospy.is_shutdown():
                break
        self.cmd(BicycleCommandMsg())

    def cmd(self, msg):
        """
        Sends a command to the turtlebot / turtlesim

        Parameters
        ----------
        msg : :obj:`BicycleCommandMsg`
        """
        self.pub.publish(msg)

    def subscribe(self, msg):
        """
        callback fn for state listener.  Don't call me...

        Parameters
        ----------
        msg : :obj:`BicycleStateMsg`
        """
        self.state = msg

    def shutdown(self):
        rospy.loginfo("Shutting Down")
        self.cmd(BicycleCommandMsg())

    def plot(self):
        f, axarr = plt.subplots(4, sharex=True)
        time = [i for i in range(len(self.state_record))]
        des = [self.state_record[i][0].x for i in range(len(self.state_record))]
        real = [self.state_record[i][1].x for i in range(len(self.state_record))]
        axarr[0].plot(time, des, color = 'r')
        axarr[0].plot(time, real, color = 'g')
        des = [self.state_record[i][0].y for i in range(len(self.state_record))]
        real = [self.state_record[i][1].y for i in range(len(self.state_record))]
        axarr[1].plot(time, des, color = 'r')
        axarr[1].plot(time, real, color = 'g')
        des = [self.state_record[i][0].theta for i in range(len(self.state_record))]
        real = [self.state_record[i][1].theta for i in range(len(self.state_record))]
        axarr[2].plot(time, des, color = 'r')
        axarr[2].plot(time, real, color = 'g')
        des = [self.state_record[i][0].phi for i in range(len(self.state_record))]
        real = [self.state_record[i][1].phi for i in range(len(self.state_record))]
        axarr[3].plot(time, des, color = 'r')
        axarr[3].plot(time, real, color = 'g')
        plt.show()

        f, ax = plt.subplots()
        x_des = [self.state_record[i][0].x for i in range(len(self.state_record))]
        y_des = [self.state_record[i][0].y for i in range(len(self.state_record))]
        x_real = [self.state_record[i][1].x for i in range(len(self.state_record))]
        y_real = [self.state_record[i][1].y for i in range(len(self.state_record))]
        ax.plot(x_des, y_des,  color='r')
        ax.plot(x_real, y_real,  color='b')
        plt.show()




def parse_args():
    """
    Pretty self explanatory tbh
    """
    parser = argparse.ArgumentParser()
    parser.add_argument('-x', type=float, default=0.0, help='Desired position in x')
    parser.add_argument('-y', type=float, default=0.0, help='Desired position in y')
    parser.add_argument('-theta', type=float, default=0.0, help='Desired turtlebot angle')
    parser.add_argument('-phi', type=float, default=0.0, help='Desired angle of the (imaginary) steering wheel')
    return parser.parse_args()

if __name__ == '__main__':
    rospy.init_node('sinusoid', anonymous=False)
    args = parse_args()

    # reset turtlesim state
    print 'Waiting for converter/reset service ...',
    rospy.wait_for_service('/converter/reset')
    print 'found!'
    reset = rospy.ServiceProxy('/converter/reset', EmptySrv)
    reset()

    ex = Exectutor()

    print "Initial State"
    print ex.state

    p = SinusoidPlanner(0.3, 0.3, 2, 3)
    goalState = BicycleStateMsg(args.x, args.y, args.theta, args.phi)
    plan = p.plan_to_pose(ex.state, goalState, 0.01, 6)

    print "Predicted Initial State"
    print plan[0][2]
    print "Predicted Final State"
    print plan[-1][2]

    gains = ex.calculateGain(plan)
    ex.executeGain(plan, gains)

    ex.execute(plan)
    print "Final State"
    print ex.state
    ex.plot()

    #reset turtle in terminal

    #rosservice call /converter/reset


    #we should make our own plotting code
    # They want us to share plotting code

#videos:
# 1: bang bang x(1)
# 2: bang bang y(0.5)
# 3: bang bang y(1)
# 4: bang bang theta(pi)


# 5: sin x(1)