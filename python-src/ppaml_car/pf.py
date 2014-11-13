from ppaml_car.data import Dataset
from ppaml_car.data import X_MIN
from ppaml_car.data import X_MAX
from ppaml_car.data import Y_MIN
from ppaml_car.data import Y_MAX
from ppaml_car.draw_dr import dynamics
from ppaml_car.draw_dr import get_dead_reckoning_poses
from ppaml_car.draw_dr import get_ground_truth_poses
from ppaml_car.draw_dr import plot_traj
from ppaml_car.lasers import car_loc_to_laser_loc
from ppaml_car.lasers import default_laser_angles
from ppaml_car.lasers import default_laser_max_range
from ppaml_car.lasers import plot_lasers
from ppaml_car.lasers import readings_for_obstacles

from numutil import logaddexp_many
from numutil import norm_log_pdf_id_cov
import matplotlib.pyplot as plt
import numpy as np
import sys


class PF(object):
    """
    Abstract particle filter.
    """
    def __init__(self, num_particles):
        self.num_particles = num_particles
        self.current_ts = None

    def initialize(self):
        self.particles = []
        for i in xrange(self.num_particles):
            self.particles.append(self.new_particle_from_prior())
        self.current_ts = 0
        self.hook_after_initialize()

    def step(self):
        new_particles = []
        logweights = []

        # Advance particles.
        for old_particle in self.particles:
            new_particle = self.advance_particle(old_particle)
            new_particles.append(new_particle)
            logweights.append(self.logweigh_particle(new_particle))

        # Normalize weights.
        logweights = np.array(logweights)
        logweights -= logaddexp_many(logweights)
        weights = np.exp(logweights)
        assert np.abs(np.sum(weights) - 1.0) < 1e-6
        self.hook_after_advance(new_particles, logweights)

        # Resample particles by weight.
        assert len(new_particles) == self.num_particles
        indices = np.random.choice(
            len(new_particles),
            size=self.num_particles,
            p=weights)
        self.particles = []
        for index in indices:
            self.particles.append(new_particles[index].clone())
        self.hook_after_resample(indices)

        # Advance timestep.
        self.hook_after_step()
        self.current_ts += 1

    def run(self):
        self.initialize()
        while self.have_more_data():
            self.step()

    def new_particle_from_prior(self):
        raise NotImplementedError()

    def advance_particle(self, particle):
        raise NotImplementedError()

    def logweigh_particle(self, particle):
        raise NotImplementedError()

    def have_more_data(self):
        raise NotImplementedError()

    def hook_after_initialize(self):
        pass

    def hook_after_advance(self, new_particles, norm_logweights):
        pass

    def hook_after_resample(self, indices):
        pass

    def hook_after_step(self):
        pass


class LocPFParticle(object):
    """
    Particle for LocPF.
    """
    def __init__(self, x, y, theta, xdot, ydot, thetadot, obstacles):
        self.x = x
        self.y = y
        self.theta = theta
        self.xdot = xdot
        self.ydot = ydot
        self.thetadot = thetadot
        self.obstacles = obstacles

    def clone(self):
        return LocPFParticle(
            self.x, self.y, self.theta,
            self.xdot, self.ydot, self.thetadot,
            self.obstacles)


class LocPF(PF):
    """
    PF for localization on a known map.
    """
    def __init__(self, dataset, num_particles):
        super(LocPF, self).__init__(num_particles)
        self.dataset = dataset
        self.last_control_ts = 0
        self.dyn_noise_stdev = 0.003
        self.obs_cov_scale = 0.1
        self.obstacle_radius = 0.5
        self.laser_angles = default_laser_angles()
        self.laser_max_range = default_laser_max_range()

        # Plot ground-truth and dead-reckoning trajectories.
        self.fig1 = plt.figure()
        self.ax1 = self.fig1.add_subplot(111)
        gps_poses = get_ground_truth_poses(dataset)
        plot_traj(self.ax1, 'ground', *gps_poses)
        dr_poses = get_dead_reckoning_poses(dataset, dynamics)
        plot_traj(self.ax1, 'dead-reckoning', *dr_poses)
        self.ax1.set_xlim(X_MIN - 1, X_MAX + 1)
        self.ax1.set_ylim(Y_MIN - 1, Y_MAX + 1)
        self.ax1.legend()
        plt.ion()  # needed for running through cProfile
        plt.show()

    def new_particle_from_prior(self):
        return LocPFParticle(
            self.dataset.init_x, self.dataset.init_y, self.dataset.init_angle,
            0.0, 0.0, 0.0,
            self.dataset.ground_obstacles)

    def advance_particle(self, particle):
        if self.dataset.ts2sensor[self.current_ts] == 'control':
            old_state = [
                particle.x, particle.y, particle.theta,
                particle.xdot, particle.ydot, particle.thetadot,
                None, None]
            velocity, steering = self.dataset.ts2control[self.current_ts]
            delta_t = (
                self.dataset.timestamps[self.current_ts] -
                self.dataset.timestamps[self.last_control_ts])
            new_state = dynamics(
                self.dataset, old_state, velocity, steering, delta_t)
            noise = self.sample_dynamics_noise()
            new_state[0] += noise[0]
            new_state[1] += noise[1]
            new_state[2] += noise[2]
            self.last_control_ts = self.current_ts
            return LocPFParticle(
                new_state[0], new_state[1], new_state[2],
                new_state[3], new_state[4], new_state[5],
                particle.obstacles)
        else:
            return particle.clone()

    def sample_dynamics_noise(self):
        return np.random.normal(loc=0.0, scale=self.dyn_noise_stdev, size=3)

    def logweigh_particle(self, particle):
        logweight = 0.0

        if self.dataset.ts2sensor[self.current_ts] == 'laser':
            obstacles = []
            for x, y in particle.obstacles:
                obstacles.append((x, y, self.obstacle_radius))
            laser_x, laser_y, laser_theta = car_loc_to_laser_loc(
                particle.x, particle.y, particle.theta,
                self.dataset.a, self.dataset.b)
            true_lasers = readings_for_obstacles(
                laser_x, laser_y, laser_theta,
                self.laser_angles, self.laser_max_range,
                obstacles)
            obs_lasers = np.array(self.dataset.ts2laser[self.current_ts])
            logweight = norm_log_pdf_id_cov(
                obs_lasers, true_lasers, self.obs_cov_scale)

            if False:
            # if self.particles[0].x != self.particles[1].x:
                plt.figure('true_lasers')
                plt.gca().clear()
                plot_lasers(
                    laser_x, laser_y, laser_theta,
                    self.laser_angles, self.laser_max_range,
                    obstacles, true_lasers, plt.gca())

                plt.figure('obs_lasers')
                plt.gca().clear()
                plot_lasers(
                    laser_x, laser_y, laser_theta,
                    self.laser_angles, self.laser_max_range,
                    obstacles, obs_lasers, plt.gca())

                plt.figure('difference')
                plt.gca().clear()
                plt.plot(true_lasers, 'g')
                plt.plot(obs_lasers, 'r')

                print "logweight {}".format(logweight)
                raw_input()

        return logweight

    def have_more_data(self):
        # return self.current_ts < 500
        return self.current_ts < len(self.dataset.timestamps)

    def hook_after_advance(self, new_particles, norm_logweights):
        self.debug_logweights = norm_logweights

    def hook_after_resample(self, indices):
        print "{} particles survived".format(len(set(indices)))

    def hook_after_step(self):
        max_logweight = np.max(self.debug_logweights)
        print (
            "finished timestep {} of {} (sensor={})"
            " max_logweight={} max_weight={}"
            .format(
                self.current_ts,
                len(self.dataset.timestamps),
                self.dataset.ts2sensor[self.current_ts],
                max_logweight,
                np.exp(max_logweight)))

        # Plotting is slow, so don't do it at every time step.
        if self.current_ts % 10 == 0:
            all_particles = []
            for particle in self.particles:
                all_particles.append((particle.x, particle.y))
            all_particles = np.array(all_particles)
            # self.ax1.clear()
            self.ax1.set_xlim(X_MIN - 1, X_MAX + 1)
            self.ax1.set_ylim(Y_MIN - 1, Y_MAX + 1)
            self.ax1.scatter(
                all_particles[:, 0],
                all_particles[:, 1],
                s=1,
                label='all particles')
            plt.draw()


# FIXME:
#
# - statistically inefficient because resamples even on timesteps where it's
# not necessary (where advance_particle returns the particles unchanged, and
# logweigh_particle returns uniform weights).
#
# - java code uses new_xdot etc and keeps only (x, y, theta) as state; python
# code uses the old xdot etc and keeps (x, y, theta, xdot, ydot, thetadot) as
# state.
#
# - when considering a laser reading in logweigh_particle(), does not cause
# particles to advance to that time, so particles are a little bit in the past,
# when the last control reading was processed.


def demo(dataset_name, num_particles):
    """
    Run LocPF on the given dataset.
    """
    dataset = Dataset.read(dataset_name)
    pf = LocPF(dataset, num_particles)
    pf.run()


if __name__ == "__main__":
    if len(sys.argv) != 3:
        raise RuntimeError(
            "Usage example: {} 1_straight num_particles".format(sys.argv[0]))
    demo(sys.argv[1], int(sys.argv[2]))
