"""
This LaserModel comes from Introduction of Robotics course notes.
"""

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

class LaserModel:
  @staticmethod
  def distance_ray(x, theta, map, max_range=4, step=0.1) -> float:
      n, m = map.shape
      ray_theta = x[2] + theta
      ray = array([cos(ray_theta), sin(ray_theta)])
      for d in linspace(0,max_range, int(max_range // step)):
        point = x[:2] + d * ray
        cell = (round(point[0]), round(point[1]))
        if cell[0] < 0 or cell[0] >= n or cell[1] < 0 or cell[1] >= m:
          return d - step
        if map[cell[0], cell[1]] == 0:
          return d
      return max_range

  @staticmethod
  def scan_map(x, map, max_range = 4.0, theta_res=0.2) -> Tuple[ndarray, ndarray]:
      theta_range = arange(-pi,pi,theta_res)
      dists = []
      for theta in theta_range:
        dist = LaserModel.distance_ray(x, theta, map, max_range=max_range)
        dists.append(dist)
      dists = array(dists)
      return dists, theta_range

  @staticmethod
  def ray_prob(point: ndarray, map: ndarray, std = 0.5):
      i, j = round(point[0]), round(point[1])
      n, m = map.shape
      out_of_bounds_dist = 0
      # Handle rays that go out of bounds
      if i < 0:
        i = 0
        out_of_bounds_dist += abs(i-point[0])
      elif i >= n:
        i = n-1
        out_of_bounds_dist += abs(i-point[0])
      if j < 0:
        j = 0
        out_of_bounds_dist += abs(j-point[1])
      elif j >= m:
        j = m-1
        out_of_bounds_dist += abs(j-point[1])

      dist = map[i, j] + out_of_bounds_dist
      return exp(-dist**2/(2*std**2))/sqrt(2*pi*std**2)

  @staticmethod
  def likelihood_field_range_finder(z_dists, z_thetas, x, map, z_max = 4.0, std=0.5):
    q = 1
    eps = 0.02
    n, m = map.shape
    for z, theta in zip(z_dists, z_thetas):
      if z > z_max - eps:
        continue
      ray_theta = x[2] + theta
      ray = array([cos(ray_theta), sin(ray_theta)])
      point = x[:2] + z * ray
      q *= LaserModel.ray_prob(point, map, std=std)
    return q