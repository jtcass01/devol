''' ####################
EN605.613 - Introduction to Robotics
Assignment 6
Extended Kalman Filter
-------

This file is used to test the EKF and dLQR functions. 
When it is executed it will attempt to track a trajectory using an LQR controller and an EKF state estimator

==================================
Copyright 2020,
The Johns Hopkins University Applied Physics Laboratory LLC (JHU/APL).
All Rights Reserved.

####################

Adapted from code authored by Atsushi Sakai (@Atsushi_twi)

'''

import sys
import numpy as np
import math
import matplotlib.pyplot as plt
import scipy.linalg as la
import cubic_spline_planner

from pid_steer_control import calc_speed_profile, calc_nearest_index, pi_2_pi
from kinematics import *
from sensors import *
from utils import *


show_animation = True
save_video = True

def closed_loop_prediction(desired_traj, landmarks):

    ## Simulation Parameters
    T = desired_traj.shape[0]  # max simulation time
    goal_dis = 0.3
    goal = desired_traj[-1,:]
    dt = 0.1
    time = 0.0


    ## Initial States 
    state = np.array([0,0,0])
    state_est = state.copy()
    sigma = 0.1*np.ones(3)
    
    ## Initialize the Sensor and Kinematics objects 
    DiffDrive = DifferentialDrive()
    LandSens = LandmarkDetector(landmarks)
    V = DiffDrive.get_V()
    W = LandSens.get_W()


    ## Create objects for storing states and estiamtes
    t = [time]
    traj = np.array([state])
    traj_est = np.array([state_est])

    ind = 0
    while T >= time:
        ## Point to track
        ind = int(np.floor(time))
        goal_i = desired_traj[ind,:]

        ## Generate noise
        # v = process noise, w = measurement noise
        v = np.random.multivariate_normal(np.zeros(V.shape[0]),V)
        w = np.random.multivariate_normal(np.zeros(W.shape[0]),W)

        ## Generaet optimal control
        u_pid= DiffDrive.PID(state_est,goal_i[0:3],dt)

        ## Move forwad in time
        state = DiffDrive.forward(state, u_pid, v, dt)
        # Take a measurement for the new state
        y = LandSens.measure(state,w)

        #Update the estimate of the state
        state_est, sigma = EKF(DiffDrive,LandSens,y,state_est,sigma,u_pid,dt,V=V,W=W)


        # Increment time
        time = time + dt

        #Store the trajectory and estimated trajectory
        t.append(time)
        traj = np.concatenate((traj,[state]),axis=0)
        traj_est = np.concatenate((traj_est,[state_est]),axis=0)

        # check to see if the robot reached goal
        if np.linalg.norm(state[0:2]-goal[0:2]) <= goal_dis:
            print("Goal reached")
            break

        ## Plot the vehicles trajectory
        if time % 1 < 0.1 and show_animation:
            plt.cla()
            plt.plot(desired_traj[:,0], desired_traj[:,1], "-r", label="course")
            plt.plot(traj[:,0], traj[:,1], "ob", label="trajectory")
            plt.plot(traj_est[:,0], traj_est[:,1], "sk", label="estimated trajectory")

            plt.plot(goal_i[0], goal_i[1], "xg", label="target")
            plt.axis("equal")
            plt.grid(True)
            plt.title("speed[m/s]:" + str(round(np.mean(u_pid), 2)) +
                      ",target index:" + str(ind))
            plt.pause(0.0001)

        #input()

    return t, traj


def main():
    print("LQR + EKF steering control tracking start!!")
    ax = [0.0, 6.0, 12.5, 10.0, 7.5, 3.0, -1.0]
    ay = [0.0, -3.0, -5.0, -6.5, 3.0, 5.0, -2.0]
    landmarks = [[10,6],[4,-2]]
    desired_traj = compute_traj(ax,ay)

    t, traj = closed_loop_prediction(desired_traj,landmarks)

    #print(traj)
    if show_animation:
        plt.close()
        flg, _ = plt.subplots(1)
        plt.plot(ax, ay, "xb", label="input")
        plt.plot(desired_traj[:,0], desired_traj[:,1], "-r", label="spline")
        plt.plot(traj[:,0], traj[:,1], "-g", label="tracking")
        plt.grid(True)
        plt.axis("equal")
        plt.xlabel("x[m]")
        plt.ylabel("y[m]")
        plt.legend()

        plt.show()


if __name__ == '__main__':
    main()
