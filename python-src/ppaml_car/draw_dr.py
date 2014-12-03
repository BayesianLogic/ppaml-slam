#!/usr/bin/env python

"""
Draw GPS trajectory and dead-reckoning trajectory computed from the controls.
"""

from numutil import normalize_radians
from ppaml_car.data import Dataset
from ppaml_car.data import X_MIN
from ppaml_car.data import X_MAX
from ppaml_car.data import Y_MIN
from ppaml_car.data import Y_MAX

import numpy as np
import matplotlib.pyplot as plt
import sys


def dynamics(dataset, old_state, encoder_velocity, steering_angle, delta_t):
    """
    Apply dynamics model and return new state.

    The model is taken from guivant_2006, equations 5 and 6.
    The model is non-linear and noise-free.

    state is assumed to be a column vector of [x, y, theta].

    steering_angle is in radians.
    """
    a, b, h, L = dataset.a, dataset.b, dataset.h, dataset.L
    # print
    # print "controls:", encoder_velocity, steering_angle
    # print "old_state:", old_state
    x, y, theta = old_state

    # Translate velocity from encoder to center of back axle:
    velocity = encoder_velocity / (1 - np.tan(steering_angle) * h / L)

    # Compute new xdot, ydot, thetadot:
    xdot = (
        velocity * np.cos(theta) -
        (velocity / L) *
        (a * np.sin(theta) + b * np.cos(theta)) *
        np.tan(steering_angle))
    ydot = (
        velocity * np.sin(theta) +
        (velocity / L) *
        (a * np.cos(theta) - b * np.sin(theta)) *
        np.tan(steering_angle))
    thetadot = (velocity / L) * np.tan(steering_angle)

    # Compute new x, y, theta:
    new_x = x + delta_t * xdot
    new_y = y + delta_t * ydot
    new_theta = theta + delta_t * thetadot
    new_theta = normalize_radians(new_theta)

    new_state = [new_x, new_y, new_theta]
    # print "new_state:", new_state
    return new_state


def plot_components(fig, label, gps_traj, dr_traj, controls):
    """
    Plot x, y, theta, velocity, steering individually.
    """
    gps_ts = gps_traj[:, 0]
    gps_xs = gps_traj[:, 1]
    gps_ys = gps_traj[:, 2]
    gps_thetas = gps_traj[:, 3]

    dr_ts = dr_traj[:, 0]
    dr_xs = dr_traj[:, 1]
    dr_ys = dr_traj[:, 2]
    dr_thetas = dr_traj[:, 3]

    control_ts, velocities, steerings = controls

    x_ax = fig.add_subplot(511)
    x_ax.plot(gps_ts, gps_xs, label='ground')
    x_ax.plot(dr_ts, dr_xs, label='mine')
    x_ax.set_ylabel('x')
    x_ax.set_ylim(X_MIN - 1, X_MAX + 1)
    x_ax.legend()

    y_ax = fig.add_subplot(512)
    y_ax.plot(gps_ts, gps_ys, label='ground')
    y_ax.plot(dr_ts, dr_ys, label='mine')
    y_ax.set_ylabel('y')
    y_ax.set_ylim(Y_MIN - 1, Y_MAX + 1)
    y_ax.legend()

    theta_ax = fig.add_subplot(513)
    theta_ax.plot(gps_ts, gps_thetas, label='ground')
    theta_ax.plot(dr_ts, dr_thetas, label='mine')
    theta_ax.set_ylabel('theta')
    theta_ax.set_ylim(-np.pi, np.pi)
    theta_ax.legend()

    velocity_ax = fig.add_subplot(514)
    velocity_ax.plot(control_ts, velocities)
    velocity_ax.set_ylabel('velocity')

    steering_ax = fig.add_subplot(515)
    steering_ax.plot(control_ts, steerings)
    steering_ax.set_ylabel('steering')


def get_controls(dataset):
    """
    Return control_ts, velocities, steerings from control readings.
    """
    control_ts = []
    velocities = []
    steerings = []
    for ts, time in enumerate(dataset.timestamps):
        if dataset.ts2sensor[ts] != 'control':
            continue
        control_ts.append(time)
        velocities.append(dataset.ts2control[ts][0])
        steerings.append(dataset.ts2control[ts][1])
    return control_ts, velocities, steerings


def get_ground_truth_traj(dataset):
    """
    Return ground-truth trajectory from the GPS readings.

    The trajectory is an array with (time, x, y, theta) rows.
    """
    traj = []
    for ts, time in enumerate(dataset.timestamps):
        if dataset.ts2sensor[ts] != 'gps':
            continue
        reading = dataset.ground_ts2gps[ts]
        traj.append((time, reading[0], reading[1], reading[2]))
    return np.array(traj)


def get_dead_reckoning_traj(dataset, dynamics):
    """
    Return trajectory given by dynamics model.

    The trajectory is an array with (time, x, y, theta) rows.
    """
    control_ts, velocities, steerings = get_controls(dataset)
    traj = np.empty((1 + len(control_ts), 4))
    traj[0] = (0.0, dataset.init_x, dataset.init_y, dataset.init_angle)
    prev_time = 0.0
    prev_state = [dataset.init_x, dataset.init_y, dataset.init_angle]
    for i in xrange(len(control_ts)):
        delta_t = control_ts[i] - prev_time
        assert delta_t > 0
        new_state = dynamics(
            dataset, prev_state, velocities[i],
            steerings[i], delta_t)
        traj[i + 1][0] = control_ts[i]
        traj[i + 1][1:] = new_state[:]
        prev_time = control_ts[i]
        prev_state = new_state
    return traj


def get_alt_dead_reckoning_traj(dataset, dynamics):
    """
    Return trajectory given by dynamics model, computed in an alternative way.

    Instead of only updating the pose on control timesteps, update it at every
    timestep.

    The trajectory is an array with (time, x, y, theta) rows.
    """
    traj = np.empty((1 + len(dataset.timestamps), 4))
    traj[0] = (0.0, dataset.init_x, dataset.init_y, dataset.init_angle)
    prev_velocity = 0.0
    prev_steering = 0.0
    prev_time = 0.0
    prev_state = [dataset.init_x, dataset.init_y, dataset.init_angle]
    for ts in xrange(len(dataset.timestamps)):
        delta_t = dataset.timestamps[ts] - prev_time
        assert delta_t > 0
        new_state = dynamics(
            dataset, prev_state, prev_velocity, prev_steering, delta_t)
        traj[ts + 1][0] = dataset.timestamps[ts]
        traj[ts + 1][1:] = new_state[:]
        prev_time = dataset.timestamps[ts]
        prev_state = new_state
        if dataset.ts2sensor[ts] == 'control':
            prev_velocity, prev_steering = dataset.ts2control[ts]
    return traj


def demo(dataset_name):
    """
    Read data and show true trajectory and trajectory given by dynamics model.
    """
    dataset = Dataset.read(dataset_name)

    # Ground-truth trajectory vs trajectory from dynamics model:
    fig1 = plt.figure()
    ax1 = fig1.add_subplot(111)
    gps_traj = get_ground_truth_traj(dataset)
    ax1.plot(gps_traj[:, 1], gps_traj[:, 2], label='ground')
    dr_traj = get_dead_reckoning_traj(dataset, dynamics)
    ax1.plot(dr_traj[:, 1], dr_traj[:, 2], label='dead-reckoning')
    alt_dr_traj = get_alt_dead_reckoning_traj(dataset, dynamics)
    ax1.plot(alt_dr_traj[:, 1], alt_dr_traj[:, 2], label='alt-dead-reckoning')
    ax1.set_xlim(X_MIN - 1, X_MAX + 1)
    ax1.set_ylim(Y_MIN - 1, Y_MAX + 1)
    ax1.legend()

    # Components of ground-truth trajectory vs my trajectory:
    controls = get_controls(dataset)
    fig2 = plt.figure()
    plot_components(fig2, 'ground', gps_traj, dr_traj, controls)

    plt.show()


if __name__ == "__main__":
    if len(sys.argv) != 2:
        raise RuntimeError(
            "Usage example: {} 1_straight".format(sys.argv[0]))
    demo(sys.argv[1])
