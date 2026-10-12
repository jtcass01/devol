"""
This DifferentialDrive model comes from Introduction of Robotics course notes.
"""

# Import Libraries
from numpy import (array, cos, sin, linspace, arange, 
                   pi, ndarray, exp, sqrt, eye, inf, 
                   ones, searchsorted, cumsum, zeros, 
                   argmax, diag, deg2rad, average)
from numpy.random import uniform, multivariate_normal
from typing import List, Tuple
from math import ceil
from typing import Iterable
from ipywidgets import interact, IntSlider

import matplotlib.pyplot as plt

class DifferentialDrive(object):
  """
  Implementation of Differential Drive kinematics.
  This represents a two-wheeled vehicle defined by the following states
  state = [x,y,theta]
  and accepts the following control inputs
  input = [left_wheel_rotation_rate,right_wheel_rotation_rate]
  """
  def __init__(self,L=1,R=1):
    """
    Initializes the class
    Input
      :param L: The distance between the wheels
      :param R: The radius of the wheels
    """
    self.L = L
    self.R = R
    self.u_limit = 1 / R
    self.V = None
    self.k_p = 1

  def get_state_size(self):
    return 3

  def get_input_size(self):
    return 2

  def get_V(self):
    '''
    This function provides the covariance matrix V which describes the noise that can be applied to the forward kinematics.
    
    Feel free to experiment with different levels of noise.

      Output
        :return: V: input cost matrix
    '''
    if self.V is None:
        self.V = eye(3)
        self.V[0,0] = 0.01
        self.V[1,1] = 0.01
        self.V[2,2] = 0.1
    return 1e-5*self.V

  def forward(self, x, u, v, dt):
    """
    Computes the forward kinematics for the system.

    Input
      :param x0: The starting state (position) of the system (units:[m,m,rad])
      :param u: The control input to the system (e.g. wheel rotation rates) (units: [rad/s,rad/s])
      :param v: The noise applied to the system (units:[m/s, m/s])

    Output
      :return: x1: The new state of the system
    """
    x_t, y_t, theta_t = x
    wL, wR = u
    R: float = self.R
    L: float = self.L

    v_t = R/2 * (wR + wL)
    omega_t: float = R/L * (wR - wL)

    x_next: float = x_t + dt*v_t*cos(theta_t)
    y_next: float = y_t + dt*v_t*sin(theta_t)
    theta_next: float = theta_t+dt*omega_t
    theta_next: float = (theta_next + pi) % (2*pi) - pi

    return array([x_next, y_next, theta_next]) + v

  def linearize(self,x,u,dt):
    """
    Computes the first order jacobian for A and B around the state x and input u

    Input
      :param x: The state of the system 
      :param u: The control input to the system (e.g. wheel rotation rates)

    Output
      :return: A: The Jacobian of the kinematics with respect to x
      :return: B: The Jacobian of the kinematics with respect to u

    """
    x_t, y_t, theta_t = x
    wL, wR = u
    R: float = self.R
    L: float = self.L
    
    A: ndarray = array([
      [1., 0., -dt * R/2 * (wL + wR) * sin(theta_t)],
      [0., 1.,  dt * R/2 * (wL + wR) *cos(theta_t)],
      [0., 0., 1.]
    ])

    B: ndarray = dt * array([
      [R/2*cos(theta_t), R/2*cos(theta_t)],
      [R/2*sin(theta_t), R/2*sin(theta_t)],
      [-R/L, R/L]
    ])

    return A, B

  def PID(self,x, x_goal, dt):
    """
    Computes the Proportional control for the diff drive system given
    the current state and velocity and the desired state. Do not worry about the derivative
    or integral term for this controller. 

    Input
      :param x: The state of the system 
      :param v: The current forward velocity and turn rate
      :param x_goal: The control input to the system (e.g. wheel rotation rates)
      :param dt: The timestep of the simulation
      
    Output
      :return u: The control input to the system (e.g. wheel rotation rates)
    """
    x_r = x[0]
    y_r = x[1]
    th_r = x[2]
    pid_length_offset = 0.1
    
    x_p  =  x_r + pid_length_offset * cos(th_r)
    y_p  =  y_r + pid_length_offset * sin(th_r)

    x_g = x_goal[0]
    y_g = x_goal[1]

    delta_x = x_p - x_g 
    delta_y = y_p - y_g 

    v_x = 0
    v_y = 0 

    vf =  - self.k_p * ( cos(th_r) * delta_x + sin(th_r) * delta_y) - self.k_p * ( cos(th_r) * v_x + sin(th_r) * v_y)

    wl = - self.k_p * ( - sin(th_r) * delta_x + cos(th_r) * delta_y) - self.k_p * ( - sin(th_r) * v_x + cos(th_r) * v_y)
 
    v_new = array([vf,wl])
    
    return self.inverse(x, v_new)
  
  def inverse(self, x, v):
    """
    Computes the inverse kinematics for the DiffDrive system (vehicle motion --> wheel motion).

    Input
      :param x: The starting state (position) of the system. This is [x,y,theta].
      :param v: The desired velocity vector for the system (in robots frame of reference).
                This is [speed, turn_rate] where Vtheta is the yaw rate (rotation rate around the z-axis)

    Output
      :return: u: The necessary control inputs to achieve the desired velocity vector. This is the rotation rates for each wheel [psi_dot_left,psi_dot_right] (e.g. radians per second)

    """
    psi_left  = 1 / self.R * (v[0] - self.L/2 * v[1])
    psi_right =  1 / self.R * (v[0] + self.L/2 * v[1])

    return array([psi_left,psi_right])
