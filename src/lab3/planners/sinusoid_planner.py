#!/usr/bin/env python
"""
Starter code for EE106B Turtlebot Lab
Author: Valmik Prabhu, Chris Correa
"""
import numpy as np
from scipy.integrate import quad
import sys
from copy import copy

import rospy
from lab3_pkg.msg import BicycleCommandMsg, BicycleStateMsg
import tf2_ros
import tf

# This is what we should edit
class SinusoidPlanner():
    def __init__(self, l, max_phi, max_u1, max_u2):
        """
        Turtlebot planner that uses sequential sinusoids to steer to a goal pose

        Parameters
        ----------
        l : float
            length of car
        """
        self.l = l
        self.max_phi = max_phi
        self.max_u1 = max_u1
        self.max_u2 = max_u2

    # We made deside to make 4 plans instead of 1 plan. This would be good as we will get drift in phi. And this way we can deal with the drift.
    def plan_to_pose(self, start_state, goal_state, dt = 0.01, delta_t=2):
        """
        Plans to a specific pose in (x,y,theta,phi) coordinates.  You
        may or may not have to convert the state to a v state with state2v()
        This is a very optional function.  You may want to plan each component separately
        so that you can reset phi in case there's drift in phi

        Parameters
        ----------
        start_state: :obj:`BicycleStateMsg`
        goal_state: :obj:`BicycleStateMsg`
        dt : float
            how many seconds between trajectory timesteps
        delta_t : float
            how many seconds each trajectory segment should run for

        Returns
        -------
        :obj:`list` of (float, BicycleCommandMsg, BicycleStateMsg)
            This is a list of timesteps, the command to be sent at that time, and the predicted state at that time
        """

        # This bit has print "Predicted Initial State"n't been exhaustively tested, so you might hit a singularity anyways
        max_abs_angle = max(abs(goal_state.theta), abs(start_state.theta))
        min_abs_angle = min(abs(goal_state.theta), abs(start_state.theta))
        if (max_abs_angle > np.pi/2) and (min_abs_angle < np.pi/2):
            raise ValueError("You'll cause a singularity here. You should add something to this function to fix it", max_abs_angle, min_abs_angle)

        if abs(start_state.phi) > self.max_phi or abs(goal_state.phi) > self.max_phi:
            raise ValueError("Either your start state or goal state exceeds steering angle bounds", goal_state.phi)

        # We can only change phi up to some threshold
        self.phi_dist = min(
            abs(goal_state.phi - self.max_phi),
            abs(goal_state.phi + self.max_phi)
        )

        x_path =        self.steer_x(
                            start_state,
                            goal_state,
                            0,
                            dt,
                            delta_t
                        )
        phi_path =      self.steer_phi(
                            x_path[-1][2],
                            goal_state,
                            x_path[-1][0] + dt,
                            dt,
                            delta_t
                        )
        alpha_path =    self.steer_alpha(
                            phi_path[-1][2],
                            goal_state,
                            phi_path[-1][0] + dt,
                            dt,
                            delta_t
                        )
        y_path =        self.steer_y(
                            alpha_path[-1][2],
                            goal_state,
                            alpha_path[-1][0] + dt,
                            dt,
                            delta_t
                        )

        path = []
        for p in [x_path, phi_path, alpha_path, y_path]:
            path.extend(p)
        return path

    def plan_to_pose3(self, start_state, goal_state, dt = 0.01, delta_t=2):
        """
        Plans to a specific pose in (x,y,theta,phi) coordinates.  You
        may or may not have to convert the state to a v state with state2v()
        This is a very optional function.  You may want to plan each component separately
        so that you can reset phi in case there's drift in phi

        Parameters
        ----------
        start_state: :obj:`BicycleStateMsg`
        goal_state: :obj:`BicycleStateMsg`
        dt : float
            how many seconds between trajectory timesteps
        delta_t : float
            how many seconds each trajectory segment should run for

        Returns
        -------
        :obj:`list` of (float, BicycleCommandMsg, BicycleStateMsg)
            This is a list of timesteps, the command to be sent at that time, and the predicted state at that time
        """

        # This bit has print "Predicted Initial State"n't been exhaustively tested, so you might hit a singularity anyways
            # We can only change phi up to some threshold
        self.phi_dist = min(
            abs(goal_state.phi - self.max_phi),
            abs(goal_state.phi + self.max_phi)
        )
        theta = goal_state.theta
        goal_state.theta = theta/3 
        x_path =        self.steer_x(
                            start_state,
                            goal_state,
                            0,
                            dt,
                            delta_t
                        )
        phi_path =      self.steer_phi(
                            x_path[-1][2],
                            goal_state,
                            x_path[-1][0] + dt,
                            dt,
                            delta_t
                        )
        alpha_path =    self.steer_alpha(
                            phi_path[-1][2],
                            goal_state,
                            phi_path[-1][0] + dt,
                            dt,
                            delta_t
                        )
        y_path =        self.steer_y(
                            alpha_path[-1][2],
                            goal_state,
                            alpha_path[-1][0] + dt,
                            dt,
                            delta_t
                        )
        y_path[-1][2].theta = 0
        alpha_path1 =    self.steer_alpha(
                            y_path[-1][2],
                            goal_state,
                            y_path[-1][0] + dt,
                            dt,
                            delta_t
                        )
        y_path1 =        self.steer_y(
                            alpha_path1[-1][2],
                            goal_state,
                            alpha_path1[-1][0] + dt,
                            dt,
                            delta_t
                        )

        y_path1[-1][2].theta = 0
        alpha_path2 =    self.steer_alpha(
                            y_path1[-1][2],
                            goal_state,
                            y_path1[-1][0] + dt,
                            dt,
                            delta_t
                        )
        y_path2 =        self.steer_y(
                            alpha_path2[-1][2],
                            goal_state,
                            alpha_path2[-1][0] + dt,
                            dt,
                            delta_t
                        )

        path = []
        for i in range (len(alpha_path1)):
            alpha_path1[i][2].theta += theta/3.0
            y_path1[i][2].theta += theta/3.0
            alpha_path2[i][2].theta += 2*theta/3.0
            y_path2[i][2].theta += 2*theta/3.0

        for p in [x_path, phi_path, alpha_path, y_path, alpha_path1, y_path1, alpha_path2, y_path2]:
            path.extend(p)
        return path

    def steer_theta(self, start_state, goal_state, t0 = 0, dt = 0.001, delta_t = 2):
        return

    # if we want to stitch paths together we can use t0.
    def steer_x(self, start_state, goal_state, t0 = 0, dt = 0.001, delta_t = 2):
        """
        Create a trajectory to move the turtlebot in the x direction

        Parameters
        ----------
        start_state : :obj:`BicycleStateMsg`
            current state of the turtlebot
        start_state : :obj:`BicycleStateMsg`
            desired state of the turtlebot
        t0 : float
            what timestep this trajectory starts at
        dt : float
            how many seconds between each trajectory point
        delta_t : float
            how many seconds the trajectory should run for

        Returns
        -------
        :obj:`list` of (float, BicycleCommandMsg, BicycleStateMsg)
            This is a list of timesteps, the command to be sent at that time, and the predicted state at that time
        """

        start_state_v = self.state2v(start_state)
        goal_state_v = self.state2v(goal_state)
        delta_x = goal_state_v[0] - start_state_v[0]

        v1 = delta_x/delta_t
        v2 = 0

        path, t = [], t0
        while t < t0 + delta_t:
            path.append([t, v1, v2])
            t = t + dt
        return self.v_path_to_u_path(path, start_state, dt)

    def steer_phi(self, start_state, goal_state, t0 = 0, dt = 0.001, delta_t = 2):
        """
        Create a trajectory to move the turtlebot in the phi direction

        Parameters
        ----------
        start_state : :obj:`BicycleStateMsg`
            current state of the turtlebot
        goal_state : :obj:`BicycleStateMsg`
            desired state of the turtlebot
        t0 : float
            what timestep this trajectory starts at
        dt : float
            how many seconds between each trajectory point
        delta_t : float
            how many seconds the trajectory should run for

        Returns
        -------
        :obj:`list` of (float, BicycleCommandMsg, BicycleStateMsg)
            This is a list of timesteps, the command to be sent at that time, and the predicted state at that time
        """
        # v1 = 0
        # v2 = delta_phi/delta_t
        #
        # path, t = [], t0
        # while t < t0 + delta_t:
        #     path.append([t, v1, v2])
        #     t = t + dt
        # return self.v_path_to_u_path(path, start_state, dt)

        # ************* IMPLEMENT THIS
        # return []

        start_state_v = self.state2v(start_state)
        goal_state_v = self.state2v(goal_state)
        delta_phi = goal_state_v[1] - start_state_v[1]

        v1 = 0
        v2 = delta_phi/delta_t
        # we can have phi only changed with v1 = 0 and v2 = delta_phi/delta_t (linear function)  
        
        path, t = [], t0
        while t < t0 + delta_t:
            path.append([t, v1, v2])
            t = t + dt
        return self.v_path_to_u_path(path, start_state, dt)
        

    def steer_alpha(self, start_state, goal_state, t0 = 0, dt = 0.001, delta_t = 2):
        """
        Create a trajectory to move the turtlebot in the alpha direction.
        Remember dot{alpha} = f(phi(t))*u_1(t) = f(frac{a_2}{omega}*sin(omega*t))*a_1*sin(omega*t)
        also, f(phi) = frac{1}{l}tan(phi)
        See the doc for more math details

        Parameters
        ----------
        start_state : :obj:`BicycleStateMsg`
            current state of the turtlebot
        goal_state : :obj:`BicycleStateMsg`
            desired state of the turtlebot
        t0 : float
            what timestep this trajectory starts at
        dt : float
            how many seconds between each trajectory point
        delta_t : float
            how many seconds the trajectory should run for

        Returns
        -------
        :obj:`list` of (float, BicycleCommandMsg, BicycleStateMsg)
            This is a list of timesteps, the command to be sent at that time, and the predicted state at that time
        """

        start_state_v = self.state2v(start_state)
        goal_state_v = self.state2v(goal_state)
        delta_alpha = goal_state_v[2] - start_state_v[2]

        omega = 2*np.pi / delta_t

        a2 = min(2, self.phi_dist*omega)
        f = lambda phi: (1/self.l)*np.tan(phi) # This is from the car model
        phi_fn = lambda t: (a2/omega)*np.sin(omega*t) + start_state_v[1]

        integrand = lambda t: f(phi_fn(t))*np.sin(omega*t) # The integrand to find beta
        beta1 = (omega/np.pi) * quad(integrand, 0, delta_t)[0]

        a1 = (delta_alpha*omega)/(np.pi*beta1)


        v1 = lambda t: a1*np.sin(omega*(t))
        v2 = lambda t: a2*np.cos(omega*(t))

        path, t = [], t0
        while t < t0 + delta_t:
            path.append([t, v1(t-t0), v2(t-t0)])
            t = t + dt
        return self.v_path_to_u_path(path, start_state, dt)



    def steer_y(self, start_state, goal_state, t0 = 0, dt = 0.001, delta_t = 2):
        """
        Create a trajectory to move the turtlebot in the y direction.
        Remember, dot{y} = g(alpha(t))*v1 = frac{alpha(t)}{sqrt{1-alpha(t)^2}}*a_1*sin(omega*t)
        See the doc for more math details

        Parameters
        ----------
        start_state : :obj:`BicycleStateMsg`
            current state of the turtlebot
        goal_state : :obj:`BicycleStateMsg`
            desired state of the turtlebot
        t0 : float
            what timestep this trajectory starts at
        dt : float
            how many seconds between each trajectory point
        delta_t : float
            how many seconds the trajectory should run for

        Returns
        -------
        :obj:`list` of (float, BicycleCommandMsg, BicycleStateMsg)
            This is a list of timesteps, the command to be sent at that time, and the predicted state at that time
        """

        # ************* IMPLEMENT THIS
        # return []
        start_state_v = self.state2v(start_state)
        goal_state_v = self.state2v(goal_state)
        delta_y = goal_state_v[3] - start_state_v[3]

        omega = 2*np.pi / delta_t
        a2 = min(1, self.phi_dist*omega) # default value for a2
        a1_min = 0.0
        a1_max = 5.0
        error = 1.0 # i.nitialization
        error_tol = 0.01 # Let the binary search find the approrpiate value with given tolerance
        print(delta_y)  
        while abs(error) > error_tol*delta_y:
            a1_mid = (a1_min+a1_max)/2.0
            a1 = a1_mid
            f = lambda phi: (1/self.l)*np.tan(phi) # This is from the car model
            g = lambda alpha: alpha/np.sqrt(1-alpha*alpha) # This is also from the car model

            phi_fn = lambda tau: (a2/(2*omega))*np.sin(2*omega*tau)

            integrand1 = lambda tau: f(phi_fn(tau))*a1*np.sin(omega*tau) # The integrand to find beta
            integrand2 = lambda t: g(quad(integrand1, 0, t)[0])*np.sin(omega*t) 
            beta1 = (omega/np.pi) * quad(integrand2, 0, 2*np.pi/omega)[0]
            error = delta_y-np.pi*a1*beta1/omega
            print(error,a1_mid)
            if error >= 0:
                a1_min = a1_mid # we should increase a1
            else:
                a1_max = a1_mid
            if a1_max - a1_min <= 0.0001:
                break

        print(a1,a2,beta1)

        v1 = lambda t: a1*np.sin(omega*(t))
        v2 = lambda t: a2*np.cos(2*omega*(t))

        path, t = [], t0
        while t < t0 + delta_t:
            path.append([t, v1(t-t0), v2(t-t0)])
            t = t + dt
        return self.v_path_to_u_path(path, start_state, dt)

    def state2v(self, state):
        """
        Takes a state in (x,y,theta,phi) coordinates and returns a state of (x,phi,alpha,y)

        Parameters
        ----------
        state : :obj:`BicycleStateMsg`
            some state

        Returns
        -------
        4x1 :obj:`numpy.ndarray`
            x, phi, alpha, y
        """
        return np.array([state.x, state.phi, np.sin(state.theta), state.y])

    def v_path_to_u_path_singular(self, path, start_state, dt):
        def v2cmd(v1, v2, state):
            u1 = v1
            u2 = v2
            return BicycleCommandMsg(u1, u2)

        curr_state = copy(start_state)
        for i, (t, v1, v2) in enumerate(path):
            cmd_u = v2cmd(v1, v2, curr_state)
            path[i] = [t, cmd_u, curr_state]

            curr_state = BicycleStateMsg(
                curr_state.x     + np.cos(curr_state.theta)               * cmd_u.linear_velocity*dt,
                curr_state.y     + np.sin(curr_state.theta)               * cmd_u.linear_velocity*dt,
                curr_state.theta + np.tan(curr_state.phi) / float(self.l) * cmd_u.linear_velocity*dt,
                curr_state.phi   + cmd_u.steering_rate*dt
            )

        return path

    def v_path_to_u_path(self, path, start_state, dt):
        """
        convert a trajectory in v commands to u commands

        Parameters
        ----------
        path : :obj:`list` of (float, float, float)
            list of (time, v1, v2) commands
        start_state : :obj:`BicycleStateMsg`
            starting state of this trajectory
        dt : float
            how many seconds between timesteps in the trajectory

        Returns
        -------
        :obj:`list` of (time, BicycleCommandMsg, BicycleStateMsg)
            This is a list of timesteps, the command to be sent at that time, and the predicted state at that time
        """
        def v2cmd(v1, v2, state):
            u1 = v1/np.cos(state.theta)
            u2 = v2
            return BicycleCommandMsg(u1, u2)

        curr_state = copy(start_state)
        for i, (t, v1, v2) in enumerate(path):
            cmd_u = v2cmd(v1, v2, curr_state)
            path[i] = [t, cmd_u, curr_state]

            curr_state = BicycleStateMsg(
                curr_state.x     + np.cos(curr_state.theta)               * cmd_u.linear_velocity*dt,
                curr_state.y     + np.sin(curr_state.theta)               * cmd_u.linear_velocity*dt,
                curr_state.theta + np.tan(curr_state.phi) / float(self.l) * cmd_u.linear_velocity*dt,
                curr_state.phi   + cmd_u.steering_rate*dt
            )

        return path
