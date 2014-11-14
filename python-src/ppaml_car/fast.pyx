# cython: profile=True

cimport fast_lasers

import numpy as np
cimport numpy as np


def readings_for_obstacles(
        double laser_x, double laser_y, double laser_theta,
        np.ndarray[np.double_t, ndim=1] laser_angles, double laser_max_range,
        np.ndarray[np.double_t, ndim=2] obstacles):
    cdef np.ndarray[np.double_t, ndim=1] readings = \
        np.empty(len(laser_angles), dtype=np.double)
    fast_lasers.readings_for_obstacles(
        laser_x, laser_y, laser_theta,
        len(laser_angles), <double*> laser_angles.data, laser_max_range,
        len(obstacles), <double(*)[3]> obstacles.data, <double*> readings.data)
    return readings
