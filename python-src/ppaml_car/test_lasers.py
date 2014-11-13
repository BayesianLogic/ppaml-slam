import nose
import numpy as np

from ppaml_car import lasers


def test_readings_for_obstacle_vectorized():
    lx, ly, ltheta = -7.0, -7.0, 1.7
    laser_angles = lasers.default_laser_angles()
    laser_max_range = lasers.default_laser_max_range()
    ox, oy, orad = 7.0, 9.0, 1.0
    readings_1 = lasers.readings_for_obstacle(
        lx, ly, ltheta, laser_angles, laser_max_range, ox, oy, orad)
    readings_2 = lasers.readings_for_obstacle_vectorized(
        lx, ly, ltheta, laser_angles, laser_max_range, ox, oy, orad)
    nose.tools.assert_true(np.sum(np.abs(readings_1 - readings_2)) < 1e-6)


def test_readings_for_obstacles_old_new():
    laser_angles = lasers.default_laser_angles()
    laser_max_range = lasers.default_laser_max_range()
    obstacles = np.array([
        [-4.475, 1.45, 0.4],
        [-1.3, 1.025, 0.4],
        [-3.0, -1.55, 0.4],
        [0.65, -1.95, 0.4],
        [-1.95, -3.8, 0.4],
        [0.15, -5.625, 0.4]
    ])
    lx = -6.1039
    ly = -0.0499926
    for ltheta in np.arange(-np.pi, np.pi, 0.1):
        print ltheta
        readings_1 = lasers.readings_for_obstacles_old(
            lx, ly, ltheta, laser_angles, laser_max_range, obstacles)
        readings_2 = lasers.readings_for_obstacles(
            lx, ly, ltheta, laser_angles, laser_max_range, obstacles)
        nose.tools.assert_true(np.sum(np.abs(readings_1 - readings_2)) < 1e-6)
