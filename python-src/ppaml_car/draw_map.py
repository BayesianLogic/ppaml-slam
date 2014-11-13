#!/usr/bin/env python

"""
Draw map of laser readings and intensity data, using ground-truth GPS location.
"""

from ppaml_car.data import Dataset
from ppaml_car.data import INTENSITY_COLS
from ppaml_car.data import INTENSITY_MAX
from ppaml_car.data import LASER_COLS
from ppaml_car.data import LASER_MAX
from ppaml_car.data import X_MIN
from ppaml_car.data import X_MAX
from ppaml_car.data import Y_MIN
from ppaml_car.data import Y_MAX
from ppaml_car.lasers import car_loc_to_laser_loc

import numpy as np
import matplotlib.pyplot as plt
import sys


def draw_map(ax, dataset):
    """
    Draw map based on laser, intensity, and GPS data.
    """
    assert LASER_COLS == INTENSITY_COLS == 361

    obst_xs = []
    obst_ys = []
    obst_cs = []
    agent_xs = []
    agent_ys = []
    prev_gps_reading = None
    for ts, time in enumerate(dataset.timestamps):
        if dataset.ts2sensor[ts] == 'gps':
            prev_gps_reading = dataset.ground_ts2gps[ts]
        if dataset.ts2sensor[ts] == 'laser':
            agent_x, agent_y, agent_phi = prev_gps_reading
            laser_x, laser_y, _ = car_loc_to_laser_loc(
                agent_x, agent_y, agent_phi, dataset.a, dataset.b)
            agent_xs.append(agent_x)
            agent_ys.append(agent_y)
            for i in xrange(LASER_COLS):
                distance = dataset.ts2laser[ts][i]
                if 0.9 < distance / LASER_MAX < 1.1:
                    continue  # no obstacles within laser range
                intensity = dataset.ts2intensity[ts][i]
                radians = agent_phi + (-90 + 0.5 * i) * np.pi / 180
                obst_x = laser_x + distance * np.cos(radians)
                obst_y = laser_y + distance * np.sin(radians)
                obst_c = intensity / INTENSITY_MAX
                obst_xs.append(obst_x)
                obst_ys.append(obst_y)
                obst_cs.append(obst_c)

    print "Have {} agent locations and {} obstacle locations.".format(
        len(agent_xs), len(obst_xs))
    ax.scatter(agent_xs, agent_ys, c='red', linewidths=0)
    # ax.scatter(obst_xs, obst_ys, c=obst_cs, linewidths=0, alpha=0.01)
    ax.scatter(obst_xs, obst_ys, c='blue', linewidths=0, alpha=0.01)
    true_obst_xs = []
    true_obst_ys = []
    if dataset.ground_obstacles:
        true_obst_xs, true_obst_ys = zip(*dataset.ground_obstacles)
    ax.scatter(true_obst_xs, true_obst_ys, c='yellow')
    ax.set_xlim(X_MIN - 1, X_MAX + 1)
    ax.set_ylim(Y_MIN - 1, Y_MAX + 1)
    plt.draw()


def demo(dataset_name):
    """
    Read data and show map of laser data for the entire run.
    """
    dataset = Dataset.read(dataset_name)
    fig = plt.figure()
    ax = fig.add_subplot(111)
    draw_map(ax, dataset)
    plt.show()


if __name__ == "__main__":
    if len(sys.argv) != 2:
        raise RuntimeError(
            "Usage example: {} 1_straight".format(sys.argv[0]))
    demo(sys.argv[1])
