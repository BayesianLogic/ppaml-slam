from numutil import norm_log_pdf_id_cov
from ppaml_car import fast

# import networkx as nx
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
    scoring_laser_std = 30.0

    def calc_score(obstacles):
        if obstacles:
            obstacles_arr = np.array(obstacles)
        else:
            obstacles_arr = np.empty((0, 3))
        lasers = fast.readings_for_obstacles(
            laser_x, laser_y, laser_theta,
            laser_angles, laser_max_range, obstacles_arr)
        return norm_log_pdf_id_cov(obs_lasers, lasers, scoring_laser_std)

    def overlap(o1, o2):
        dist2 = (o1[0] - o2[0]) ** 2 + (o1[1] - o2[1]) ** 2
        return dist2 <= known_radius ** 2

    # Start with an empty set of obstacles.
    empty_score = calc_score([])
    print "empty_score={}".format(empty_score)

    # Collect a bunch of obstacles that individually help explain the readings.
    all_obstacles = []
    min_improvement = 2.0
    for i in xrange(1000):
        index = np.random.randint(len(laser_angles))
        noise = np.random.normal(loc=0.0, scale=true_laser_std)
        x, y = ray_end(
            laser_x, laser_y, laser_theta,
            laser_angles[index], obs_lasers[index] + known_radius + noise)
        score = calc_score([(x, y, known_radius)])
        improvement = score - empty_score
        print "Considering {}; improvement {}".format((x, y), improvement)
        if improvement >= min_improvement:
            all_obstacles.append((x, y, known_radius))

    # Add obstacles greedily from the list of candidates.
    all_candidates = all_obstacles
    all_obstacles = []
    while all_candidates:
        all_score = calc_score(all_obstacles)
        print "Have {} obstacles; all_score={}".format(
            len(all_obstacles), all_score)
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
        print "Added {} for an improvement of {}".format(
            candidate, best_improvement)
        all_obstacles = best_obstacles

    # # Find the connected components in a graph, where each obstacle is a
    # # node, and there is an edge between two obstacles if they overlap.
    # graph = nx.Graph()
    # for i in xrange(len(all_obstacles)):
    #     graph.add_node(i)
    # for i in xrange(len(all_obstacles)):
    #     for j in xrange(i + 1, len(all_obstacles)):
    #         if overlap(all_obstacles[i], all_obstacles[j]):
    #             graph.add_edge(i, j)
    # components = nx.connected_components(graph)
    # for c in components:
    #     print c

    # # Prune obstacles greedily.
    # max_loss = 0.5
    # while True:
    #     all_score = calc_score(all_obstacles)
    #     print "Have {} obstacles; all_score={}".format(
    #         len(all_obstacles), all_score)
    #     best_i = None
    #     best_obstacles = None
    #     best_loss = np.inf
    #     for i in xrange(len(all_obstacles)):
    #         obstacles = all_obstacles[:i] + all_obstacles[i + 1:]
    #         score = calc_score(obstacles)
    #         loss = all_score - score
    #         if loss < best_loss:
    #             best_i = i
    #             best_obstacles = obstacles
    #             best_loss = loss
    #     assert best_i is not None
    #     if best_loss > max_loss:
    #         break
    #     print "Removing {} for a loss of {}".format(
    #         all_obstacles[best_i], best_loss)
    #     all_obstacles = best_obstacles

    return np.array(all_obstacles)
