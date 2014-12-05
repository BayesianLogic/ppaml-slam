from ppaml_car import data
from ppaml_car import lasers
from ppaml_car import ransac

import numpy as np


def extract_obstacles_py():
    ransac.extract_obstacles(
        lx, ly, lt, laser_angles, laser_max_range, obs_lasers)


if __name__ == "__main__":
    dataset = data.Dataset.read("2_bend")
    cx, cy, ct = dataset.init_x, dataset.init_y, dataset.init_angle
    lx, ly, lt = lasers.car_loc_to_laser_loc(cx, cy, ct, dataset.a, dataset.b)
    obs_lasers = None
    for ts in xrange(len(dataset.timestamps)):
        if dataset.ts2sensor[ts] == 'laser':
            obs_lasers = np.array(dataset.ts2laser[ts])
            break
    assert obs_lasers is not None
    laser_angles = lasers.default_laser_angles()
    laser_max_range = lasers.default_laser_max_range()

    import time
    timer = time.time()
    num_trials = 1000
    for t in xrange(num_trials):
        extract_obstacles_py()
    timer = time.time() - timer
    print "{} seconds per trial".format(timer / num_trials)

    # from timeit import timeit
    # t_old = timeit(
    #     "extract_obstacles_py()",
    #     setup="from __main__ import extract_obstacles_py",
    #     number=num_trials) / num_trials
    # t_new = timeit(
    #     "readings_for_obstacles_new()",
    #     setup="from __main__ import readings_for_obstacles_new",
    #     number=num_trials) / num_trials

    # print "old: {} sec per call".format(t_old)
    # print "new: {} sec per call ({} times faster)".format(
    #   t_new, t_old / t_new)
