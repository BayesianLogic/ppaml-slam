#!/usr/bin/env python

"""
Evaluate the trajectory output by BLOG against the ground-truth GPS trajectory.

Example usage:
    python -m ppaml_car.evaluate ground_gps.csv slam_out_path.csv --plot
"""


import argparse
import csv
import numpy as np


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


def plot_traj(ax, label, traj):
    """
    Plot trajectory as a line in 2D.
    """
    # Traj has columns (time, lat, lon). Note x=lon, y=lat.
    ax.plot(traj[:, 2], traj[:, 1], label=label)


def compute_error(ground_traj, my_traj):
    """
    Compute error between trajectories, in the same way as the evaluator does.
    """
    # Times should match.
    if ground_traj.shape != my_traj.shape:
        raise ValueError("ground_traj and my_traj must have the same shape")
    if np.sum(np.abs(ground_traj[:, 0] - my_traj[:, 0])) > 1e-10:
        raise ValueError("ground_traj and my_traj must have the same times")

    d = ground_traj[:, 1:3] - my_traj[:, 1:3]
    norm2 = np.sqrt(np.sum(d * d, axis=1))
    return np.sum(norm2)


if __name__ == "__main__":
    # Parse command-line args.
    parser = argparse.ArgumentParser(
        description='Evaluate trajectory output by BLOG.')
    parser.add_argument('ground_gps_csv')
    parser.add_argument('out_csv')
    parser.add_argument('--plot', action='store_true')
    args = parser.parse_args()

    # Array with columns (time, lat, lon).
    # We get rid of the 4th orientation column.
    ground_traj = read_csv(args.ground_gps_csv)
    ground_traj = ground_traj[:, :3]

    # Array with columns (time, lat, lon).
    blog_traj = read_csv(args.out_csv)

    # Evaluate error between the trajectories.
    if len(blog_traj) == len(ground_traj):
        print "Traj error: {}".format(compute_error(ground_traj, blog_traj))
    else:
        print "Traj not the same length; can't evaluate."

    # Optionally plot trajectories.
    if args.plot:
        import matplotlib.pyplot as plt
        plt.figure(figsize=(8, 8))
        plot_traj(plt.gca(), 'ground', ground_traj)
        plot_traj(plt.gca(), 'blog', blog_traj)

        plt.plot([-7, -7, 7, 7, -7], [-7, 7, 7, -7, -7], 'k')
        plt.xlim(-8, 8)
        plt.ylim(-8, 8)
        plt.legend()
        plt.show()
