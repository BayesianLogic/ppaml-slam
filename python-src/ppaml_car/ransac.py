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
    min_improvement = 2.0

    def calc_score(obstacles):
        if obstacles:
            obstacles_arr = np.array(obstacles)
        else:
            obstacles_arr = np.empty((0, 3))
        lasers = fast.readings_for_obstacles(
            laser_x, laser_y, laser_theta,
            laser_angles, laser_max_range, obstacles_arr)
        return norm_log_pdf_id_cov(obs_lasers, lasers, scoring_laser_std)

    # Find segments where the laser readings are less than laser_max_range.
    segments = []
    last_start = None
    if obs_lasers[0] < 0.9 * laser_max_range:
        last_start = 0
    for i, reading in enumerate(obs_lasers):
        if last_start and reading >= 0.9 * laser_max_range:
            segments.append((last_start, i - 1))
            last_start = None
        elif not last_start and reading < 0.9 * laser_max_range:
            last_start = i
    if last_start:
        segments.append((last_start, len(obs_lasers) - 1))
    # print segments

    # Place an obstacle in the center of each of those segments.
    all_obstacles = []
    empty_score = calc_score([])
    for start, stop in segments:
        mid = (start + stop + 1) / 2
        best_obstacle = None
        best_improvement = -np.inf
        for trial in xrange(10):
            noise = np.random.normal(loc=0.0, scale=true_laser_std)
            x, y = ray_end(
                laser_x, laser_y, laser_theta,
                laser_angles[mid], obs_lasers[mid] + known_radius + noise)
            obstacle = (x, y, known_radius)
            score = calc_score([obstacle])
            improvement = score - empty_score
            # print "Considering {}; improvement {}".format(
            #     (x, y), improvement)
            if improvement >= best_improvement:
                best_obstacle = obstacle
                best_improvement = improvement
        if best_improvement > min_improvement:
            all_obstacles.append(best_obstacle)

    if all_obstacles:
        return np.array(all_obstacles)
    else:
        return np.empty((0, 3))
