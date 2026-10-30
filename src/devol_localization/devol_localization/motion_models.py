#!/usr/bin/env python3

from typing import Union
from numpy import cos, sin, eye, atleast_2d, ndarray


__author__ = 'Jacob Taylor Cassady'
__email__ = 'jcassad1@jh.edu'


class UnicycleMotionModel:
    @staticmethod
    def forward(
        state: Union[float, ndarray], u: ndarray, noise: float, dt: float
    ) -> Union[float, ndarray]:
        v = u[0] + noise[0]
        omega = u[1] + noise[1]

        # Support both 1D arrays (single state) and 2D arrays (particle cloud)
        is_1d = state.ndim == 1
        state_arr = atleast_2d(state).copy()

        theta = state_arr[:, 2]
        state_arr[:, 0] += v * cos(theta) * dt
        state_arr[:, 1] += v * sin(theta) * dt
        state_arr[:, 2] += omega * dt + noise[2]

        return state_arr[0] if is_1d else state_arr

    @staticmethod
    def linearize(state: ndarray, u: ndarray, dt: float) -> ndarray:
        v = u[0]
        theta = state[2]

        A = eye(3)
        A[0, 2] = -v * sin(theta) * dt
        A[1, 2] = v * cos(theta) * dt

        return A
