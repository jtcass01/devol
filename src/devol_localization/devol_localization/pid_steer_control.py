''' ####################
EN605.613 - Introduction to Robotics
Assignment 5
Linear Quadratice Regulator
-------

This file is used to test the DifferentialDrive class and dLQR function.
When it is executed it will attempt to track a trajectory using an LQR controller.

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
from kinematics import *
from utils import *
from time import sleep

show_animation = True

def closed_loop_prediction(desired_traj, landmarks):

    ## Simulation Parameters
    T = desired_traj.shape[0]  # max simulation time
    goal_dis = 0.3
    goal = desired_traj[-1,:]
    dt = 0.1
    time = 0.0


    ## Initial States for the system
    state = np.array([0,0,0])

    ## Get the Cost-to-go and input cost matrices for LQR
    Q = get_Q() # Defined in kinematics.py
    R = get_R() # Defined in kinematics.py


    ## Initialize the Sensor and Kinematics objects 
    DiffDrive = DifferentialDrive()
    V = DiffDrive.get_V()

    ## Create objects for storing states and estiamtes
    t = [time]
    traj = np.array([state])

    ind = 0
    dx_dt = np.array([0,0,0])
    
    while T >= time:
        ## Point to track
        ind = int(np.floor(time))
        goal_i = desired_traj[ind,:]

        ## Generate noise
        # v = process noise
        v = np.random.multivariate_normal(np.zeros(V.shape[0]),V)

        ## Generaet optimal control
        u_pid = DiffDrive.PID(state,goal_i[0:3],dt)

        ## Move forward in time
        state = DiffDrive.forward(state,u_pid,v,dt)


        # Increment time
        time = time + dt

        #Store the trajectory and estimated trajectory
        t.append(time)
        traj = np.concatenate((traj,[state]),axis=0)

        # check to see if the robot reached goal
        if np.linalg.norm(state[0:2]-goal[0:2]) <= goal_dis:
            print("Goal reached")
            break

        ## Plot the vehicles trajectory
        if time % 1 < 0.1 and show_animation:
            plt.cla()
            plt.plot(desired_traj[:,0], desired_traj[:,1], "-r", label="course")
            plt.plot(traj[:,0], traj[:,1], "ob", label="trajectory")

            plt.plot(goal_i[0], goal_i[1], "xg", label="target")
            plt.axis("equal")
            plt.grid(True)
            plt.title("speed[m/s]:" + str(round(np.mean(u_pid), 2)) +
                      ",target index:" + str(ind))
            plt.pause(0.0001)

        #input()

    return t, traj


def main():
    print("PID steering control tracking start!!")
    ax = [0.0, 6.0, 12.5, 10.0, 7.5, 3.0, -1.0]
    ay = [0.0, -3.0, -5.0, 6.5, 3.0, 5.0, -2.0]
    landmarks = [[10,4],[5,1]]
    desired_traj = compute_traj(ax,ay)

    t, traj = closed_loop_prediction(desired_traj,landmarks)

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
