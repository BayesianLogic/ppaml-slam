# from ppaml_car import data
from ppaml_car import lasers

from timeit import timeit
import matplotlib.pyplot as plt
import numpy as np


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
true_x = -6.1039
true_y = -0.0499926
true_theta = -0.14
# true_theta = -2.34159265359


def visual_test():
    readings_old = readings_for_obstacles_old()
    plt.figure('old')
    lasers.plot_lasers(
        true_x, true_y, true_theta,
        laser_angles, laser_max_range,
        obstacles, readings_old, plt.gca())

    readings_new = readings_for_obstacles_new()
    plt.figure('new')
    lasers.plot_lasers(
        true_x, true_y, true_theta,
        laser_angles, laser_max_range,
        obstacles, readings_new, plt.gca())

    plt.show()


def readings_for_obstacles_old():
    return lasers.readings_for_obstacles_old(
        true_x, true_y, true_theta,
        laser_angles, laser_max_range,
        obstacles)


def readings_for_obstacles_new():
    return lasers.readings_for_obstacles(
        true_x, true_y, true_theta,
        laser_angles, laser_max_range,
        obstacles)


if __name__ == "__main__":
    visual_test()

    num_trials = 1000
    t_old = timeit(
        "readings_for_obstacles_old()",
        setup="from __main__ import readings_for_obstacles_old",
        number=num_trials) / num_trials
    t_new = timeit(
        "readings_for_obstacles_new()",
        setup="from __main__ import readings_for_obstacles_new",
        number=num_trials) / num_trials

    print "old: {} sec per call".format(t_old)
    print "new: {} sec per call ({} times faster)".format(t_new, t_old / t_new)
