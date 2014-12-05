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

    def get_all_candidates():
        # List of indices where the laser readings are not laser_max_range.
        indices = []
        for i, reading in enumerate(obs_lasers):
            if reading < 0.9 * laser_max_range:
                indices.append(i)
        print len(indices)

        all_candidates = []
        for i in xrange(1000):
            index = np.random.randint(len(laser_angles))
            noise = np.random.normal(loc=0.0, scale=true_laser_std)
            x, y = ray_end(
                laser_x, laser_y, laser_theta,
                laser_angles[index], obs_lasers[index] + known_radius + noise)
            score = calc_score([(x, y, known_radius)])
            improvement = score - empty_score
            # print "Considering {}; improvement {}".format(
            #     (x, y), improvement)
            if improvement >= min_improvement:
                all_candidates.append((x, y, known_radius))
        return all_candidates

    def greedy_get_all_obstacles(all_candidates):
        all_obstacles = []
        while all_candidates:
            all_score = calc_score(all_obstacles)
            # print "Have {} obstacles; all_score={}".format(
            #     len(all_obstacles), all_score)
            best_candidate = None
            best_obstacles = None
            best_improvement = -np.inf
            for candidate in all_candidates:
                obstacles = all_obstacles + [candidate]
                score = calc_score(obstacles)
                improvement = score - all_score
                if improvement > best_improvement:
                    best_candidate = candidate
                    best_obstacles = obstacles
                    best_improvement = improvement
            assert best_candidate is not None
            if best_improvement < min_improvement:
                break
            # print "Added {} for an improvement of {}".format(
            #     candidate, best_improvement)
            all_obstacles = best_obstacles
        return all_obstacles

    # Start with an empty set of obstacles.
    empty_score = calc_score([])
    # print "empty_score={}".format(empty_score)

    # Collect a bunch of obstacles that individually help explain the readings.
    all_candidates = get_all_candidates()
    print "Have {} candidates".format(len(all_candidates))

    # Add obstacles greedily from the list of candidates.
    all_obstacles = greedy_get_all_obstacles(all_candidates)
    # print "Have {} obstacles; all_score={}".format(
    #     len(all_obstacles), all_score)

    return np.array(all_obstacles)
