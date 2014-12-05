from numutil import norm_log_pdf_id_cov
from ppaml_car import fast

import numpy as np


def ray_end(laser_x, laser_y, laser_theta, angle, distance):
    """
    Return the point (x, y) at the end of the given laser ray.
    """
    dx = distance * np.cos(laser_theta + angle)
    dy = distance * np.sin(laser_theta + angle)
    return (laser_x + dx, laser_y + dy)


def extract_obstacles(
        laser_x, laser_y, laser_theta, laser_angles, laser_max_range,
        obs_lasers):
    """
    Given pose and laser readings, guess what the obstacles are.

    Returns the obstacles as an array with rows of (x, y, r).
    """
    # We assume the obstacle radius and noise params are known.
    known_radius = 0.37
    true_laser_std = 0.1
    scoring_laser_std = 2.0

    def calc_score(obstacles_arr):
        lasers = fast.readings_for_obstacles(
            laser_x, laser_y, laser_theta,
            laser_angles, laser_max_range, obstacles_arr)
        return norm_log_pdf_id_cov(obs_lasers, lasers, scoring_laser_std)

    def overlap(o1, o2):
        dist2 = (o1[0] - o2[0]) ** 2 + (o1[1] - o2[1]) ** 2
        return dist2 <= known_radius ** 2

    # Start with an empty set of obstacles.
    empty_score = calc_score(np.empty((0, 3)))
    # print "empty_score={}".format(empty_score)

    # Collect a bunch of obstacles that individually help explain the readings.
    all_obstacles = []
    min_improvement = 2.0
    tmp_obstacles = np.empty((1, 3))
    for i in xrange(1000):
        index = np.random.randint(len(laser_angles))
        noise = np.random.normal(loc=0.0, scale=true_laser_std)
        x, y = ray_end(
            laser_x, laser_y, laser_theta,
            laser_angles[index], obs_lasers[index] + known_radius + noise)
        tmp_obstacles[0, :] = x, y, known_radius
        score = calc_score(tmp_obstacles)
        improvement = score - empty_score
        # print "Considering {}; improvement {}".format((x, y), improvement)
        if improvement >= min_improvement:
            all_obstacles.append((x, y, known_radius))
    # print "Have {} candidates".format(len(all_obstacles))

    # Add obstacles greedily from the list of candidates.
    all_candidates = all_obstacles
    all_obstacles = np.empty((0, 3))
    while all_candidates:
        all_score = calc_score(all_obstacles)
        # print "Have {} obstacles; all_score={}".format(
        #     len(all_obstacles), all_score)
        best_i = None
        best_improvement = -np.inf
        tmp_obstacles = np.empty((len(all_obstacles) + 1, 3))
        tmp_obstacles[:-1, :] = all_obstacles
        for i in xrange(len(all_candidates)):
            tmp_obstacles[-1, :] = all_candidates[i]
            score = calc_score(tmp_obstacles)
            improvement = score - all_score
            if improvement > best_improvement:
                best_i = i
                best_improvement = improvement
        assert best_i is not None
        if best_improvement < min_improvement:
            break
        # print "Added {} for an improvement of {}".format(
        #     candidate, best_improvement)
        tmp_obstacles[-1, :] = all_candidates[best_i]
        all_obstacles = tmp_obstacles

    # print "Have {} obstacles; all_score={}".format(
    #     len(all_obstacles), all_score)
    return all_obstacles
