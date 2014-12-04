#!/usr/bin/env python

"""
Evaluate the trajectory output by BLOG against the ground-truth GPS trajectory.

Example usage:
    python -m ppaml_car.evaluate eval_data_dir out_dir --plot
"""


import argparse
import csv
import numpy as np
import os

from ppaml_car.lasers import plot_obstacles
from slam_eval.slam_eval import obsticle_compare


def read_csv(path):
    """
    Read CSV of real values, discard header, and return np.array.
    """
    rows = []
    with open(path) as csv_file:
        reader = csv.reader(csv_file)
        header = reader.next()
        if header[0].isdigit():
            print "Warning: Discarding header that looks like numbers."
        for line in reader:
            rows.append(map(float, line))
    return np.array(rows)


def compute_error(ground_traj, my_traj):
    """
    Compute error between trajectories, in the same way as the evaluator does.
    """
    # Times should match.
    if ground_traj.shape != my_traj.shape:
        raise ValueError("ground_traj and my_traj must have the same shape")
    if np.max(np.abs(ground_traj[:, 0] - my_traj[:, 0])) > 1e-2:
        raise ValueError("ground_traj and my_traj must have the same times")

    d = ground_traj[:, 1:3] - my_traj[:, 1:3]
    norm2 = np.sqrt(np.sum(d * d, axis=1))
    return np.sum(norm2)


def evaluate_trajectories(args):
    # Array with columns (time, lat, lon).
    # We get rid of the 4th orientation column.
    ground_traj = read_csv(os.path.join(args.eval_data_dir, 'eval_gps.csv'))
    ground_traj = ground_traj[:, :3]

    # Array with columns (time, lat, lon).
    out_traj = read_csv(os.path.join(args.out_dir, 'slam_out_path.csv'))

    # Evaluate error between the trajectories.
    if len(out_traj) == len(ground_traj):
        print "Traj error: {}".format(compute_error(ground_traj, out_traj))
    else:
        print "Traj not the same length; can't evaluate."

    # Optionally plot trajectories.
    if args.plot:
        plt.plot(
            ground_traj[:, 2], ground_traj[:, 1], label='ground', color='g')
        plt.plot(
            out_traj[:, 2], out_traj[:, 1], label='out', color='r')


def evaluate_obstacles(args):
    # Array with columns (x, y).
    ground_obst = read_csv(os.path.join(
        args.eval_data_dir, 'eval_obstacles.csv'))

    # Array with columns (x, y).
    out_obst = read_csv(os.path.join(args.out_dir, 'slam_out_landmarks.csv'))

    # Use evaluation metric written by Galois.
    n_extras, score = obsticle_compare(out_obst, ground_obst)
    print "{} ground obstacles; {} out obstacles".format(
        len(ground_obst), len(out_obst))
    print "Map error: {}".format(score)

    # Optionally plot obstacles.
    if args.plot:
        r = 0.37
        ground_obst_3 = [(x, y, r) for x, y in ground_obst]
        out_obst_3 = [(x, y, r) for x, y in out_obst]
        plot_obstacles(ground_obst_3, plt.gca(), color=(0, 1, 0, 0.6))
        plot_obstacles(out_obst_3, plt.gca(), color=(1, 0, 0, 0.6))


if __name__ == "__main__":
    # Parse command-line args.
    parser = argparse.ArgumentParser(
        description='Evaluate trajectory output by BLOG.')
    parser.add_argument('eval_data_dir')
    parser.add_argument('out_dir')
    parser.add_argument('--plot', action='store_true')
    args = parser.parse_args()

    if args.plot:
        import matplotlib.pyplot as plt
        plt.figure(figsize=(8, 8))

    evaluate_trajectories(args)
    evaluate_obstacles(args)

    if args.plot:
        plt.plot([-7, -7, 7, 7, -7], [-7, 7, 7, -7, -7], 'k')
        plt.xlim(-8, 8)
        plt.ylim(-8, 8)
        plt.legend()
        plt.show()
