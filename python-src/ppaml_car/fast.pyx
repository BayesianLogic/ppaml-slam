# cython: profile=True

cimport fast_lasers

from libc.math cimport sqrt
from libcpp cimport bool
import bisect
import math
import numpy as np


cdef void solve_quadratic_equation(
        double a, double b, double c,
        bool &success, double &x1, double &x2):
    """
    Solve A * x^2 + B * x + C = 0 and fill in (x1, x2) s.t. x1 <= x2.

    If there are no solutions, sets success to false.
    """
    cdef double delta = b ** 2 - 4.0 * a * c
    if delta < 0:
        # This is how you assign to references in Cython...
        (&success)[0] = False
    else:
        (&success)[0] = True
        (&x1)[0] = (-b - sqrt(delta)) / (2.0 * a)
        (&x2)[0] = (-b + sqrt(delta)) / (2.0 * a)


def readings_for_obstacles(
        laser_x, laser_y, laser_theta,
        laser_angles, laser_max_range,
        obstacles):
    """
    Like readings_for_obstacle, but accepts multiple obstacles.

    `obstacles` is a list of (x, y, r) tuples.

    `laser_angles` is assumed to be in sorted order.
    """
    readings = np.ones_like(laser_angles) * laser_max_range

    def _update_ray(i, obstacle_x, obstacle_y, obstacle_r):
        # Return true iff the given ray hits the obstacle.
        angle = laser_angles[i]
        a = 1.0
        b = (2.0 * (laser_x - obstacle_x) * np.cos(laser_theta + angle) +
             2.0 * (laser_y - obstacle_y) * np.sin(laser_theta + angle))
        c = ((laser_x - obstacle_x) ** 2 +
             (laser_y - obstacle_y) ** 2 -
             obstacle_r ** 2)
        cdef bool success = False
        cdef double k1 = 0, k2 = 0
        solve_quadratic_equation(a, b, c, success, k1, k2)
        if not success:
            return False  # does not intersect ray
        assert 0 <= k1 <= laser_max_range
        readings[i] = np.minimum(readings[i], k1)
        return True

    for x, y, r in obstacles:
        dist = np.sqrt((x - laser_x) ** 2 + (y - laser_y) ** 2)
        if dist - r <= 0:
            # Ignore obstacle which overlaps the laser location.
            continue
        if dist - r >= laser_max_range:
            # Ignore obstacle which is too far.
            continue
        angle_to_obst = math.atan2(y - laser_y, x - laser_x) - laser_theta
        # Find a ray that hits the obstacle.
        if angle_to_obst <= laser_angles[0]:
            index = 0
        elif angle_to_obst >= laser_angles[-1]:
            index = len(laser_angles) - 1
        else:
            index = bisect.bisect_right(laser_angles, angle_to_obst)
        # Update rays to the left of center.
        for i in xrange(index - 1, -1, -1):
            hit = _update_ray(i, x, y, r)
            if not hit:
                break
        # Update rays to the right of center:
        for i in xrange(index, len(laser_angles)):
            hit = _update_ray(i, x, y, r)
            if not hit:
                break

    return readings
