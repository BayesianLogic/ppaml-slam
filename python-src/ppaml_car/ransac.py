from ppaml_car import fast

from numutil import norm_log_pdf_id_cov
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
    # For now, we assume the obstacle radius is known.
    known_radius = 0.37
    true_laser_std = 0.1
    scoring_laser_std = 30.0

    # Start with an empty set of obstacles.
    cur_obstacles = np.empty((0, 3))
    cur_lasers = fast.readings_for_obstacles(
        laser_x, laser_y, laser_theta,
        laser_angles, laser_max_range, cur_obstacles)
    cur_score = norm_log_pdf_id_cov(
        obs_lasers, cur_lasers, scoring_laser_std)

    # Try adding obstacles that explain the readings well.
    for i in xrange(100):
        index = np.random.randint(len(laser_angles))
        noise = np.random.normal(loc=0.0, scale=true_laser_std)
        x, y = ray_end(
            laser_x, laser_y, laser_theta,
            laser_angles[index], obs_lasers[index] + known_radius + noise)
        new_obstacles = np.empty((len(cur_obstacles) + 1, 3))
        new_obstacles[:-1, :] = cur_obstacles
        new_obstacles[-1] = x, y, known_radius
        new_lasers = fast.readings_for_obstacles(
            laser_x, laser_y, laser_theta,
            laser_angles, laser_max_range, new_obstacles)
        new_score = norm_log_pdf_id_cov(
            obs_lasers, new_lasers, scoring_laser_std)
        if new_score > cur_score:
            cur_obstacles = new_obstacles
            cur_score = new_score

    return cur_obstacles


        # # Pick three points near one another.
        # index = np.random.randint(len(laser_angles))
        # min_index = np.max(0, index - 10)
        # max_index = np.min(len(laser_angles) - 1, index + 10)
        # indices = range(min_index, max_index + 1)
        # np.shuffle(indices)
        # indices = indices[:3]
        # points = [
        #     ray_end(
        #         laser_x, laser_y, laser_theta,
        #         laser_angles[index], obs_lasers[index])
        #     for index in indices]

        # # Fit a circle through those three points.
        # assert len(indices) == 3
        # x, y, r = fit_circle(*points)

        # # Fix the radius, and randomly perturb the circle a bit. If we get a
        # # good increase in likelihood, save the circle as a new obstacle.
        # r = known_radius
        # for j in xrange(20):
