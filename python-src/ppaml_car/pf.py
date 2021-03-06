from ppaml_car.data import Dataset
from ppaml_car.data import X_MIN
from ppaml_car.data import X_MAX
from ppaml_car.data import Y_MIN
from ppaml_car.data import Y_MAX
from ppaml_car import draw_dr
from ppaml_car import lasers
from ppaml_car import ransac
from ppaml_car.fast import readings_for_obstacles

from numutil import logaddexp_many
from numutil import norm_log_pdf_id_cov
from numutil import normalize_radians
import matplotlib.pyplot as plt
import numpy as np
import random
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
    def __init__(self, x, y, theta, obstacles):
        self.x = x
        self.y = y
        self.theta = theta
        self.obstacles = obstacles

    def clone(self):
        return LocPFParticle(
            self.x, self.y, self.theta,
            self.obstacles)


class LocPF(PF):
    """
    PF for localization on a known map.
    """
    def __init__(self, dataset, num_particles):
        super(LocPF, self).__init__(num_particles)
        self.dataset = dataset
        self.current_velocity = 0.0
        self.current_steering = 0.0

        self.add_controls_noise = False
        self.velocity_noise_stdev = 0.1
        self.steering_noise_stdev = 0.3

        self.add_dynamics_noise = True
        self.x_noise_stdev = 0.003
        self.y_noise_stdev = 0.003
        self.theta_noise_stdev = 0.0015

        self.obs_cov_scale = 30.0
        self.laser_angles = lasers.default_laser_angles()
        self.laser_max_range = lasers.default_laser_max_range()

        # Convert obstacles from list of (x, y) to array of (x, y, r).
        obstacle_radius = 0.37
        self.ground_obstacles = []
        for x, y in dataset.ground_obstacles:
            self.ground_obstacles.append((x, y, obstacle_radius))
        self.ground_obstacles = np.array(self.ground_obstacles)

        self.fig1 = plt.figure(figsize=(8, 12))
        self.ax1 = self.fig1.add_subplot(211)
        self.ax2 = self.fig1.add_subplot(413)
        self.ax3 = self.fig1.add_subplot(414)

        # Hack to display colorbar for angle.
        plt.sca(self.ax1)
        a = np.array([[-np.pi, np.pi]])
        x = plt.pcolor(a)
        self.ax1.clear()
        plt.colorbar(x, ax=self.ax1)

        # Hack to display legends for the bottom plots.
        dummy_lines = []
        dummy_lines += self.ax2.plot(
            [0, 0], [0, 0], 'g', label='ground_gps_llik')
        dummy_lines += self.ax2.plot(
            [0, 0], [0, 0], 'r', label='best_particle_llik')
        self.ax2.legend()
        dummy_lines += self.ax3.plot([0, 0], [0, 0], label='error^2')
        self.ax3.legend()
        for line in dummy_lines:
            line.remove()

        # Plot ground-truth and dead-reckoning trajectories.
        gps_traj = draw_dr.get_ground_truth_traj(dataset)
        self.ax1.plot(gps_traj[:, 1], gps_traj[:, 2], label='ground')
        dr_traj = draw_dr.get_dead_reckoning_traj(dataset, draw_dr.dynamics)
        self.ax1.plot(dr_traj[:, 1], dr_traj[:, 2], label='dead-reckoning')
        lasers.plot_obstacles(self.ground_obstacles, self.ax1)

        self.ax1.set_xlim(X_MIN - 1, X_MAX + 1)
        self.ax1.set_ylim(Y_MIN - 1, Y_MAX + 1)
        self.ax1.legend()
        plt.ion()  # needed for running through cProfile
        plt.show()

    def new_particle_from_prior(self):
        return LocPFParticle(
            self.dataset.init_x, self.dataset.init_y, self.dataset.init_angle,
            np.empty((0, 3)))

    def advance_particle(self, particle):
        old_state = [particle.x, particle.y, particle.theta]

        velocity = self.current_velocity
        steering = self.current_steering

        # Add controls noise:
        if self.add_controls_noise:
            velocity += np.random.normal(
                loc=0.0, scale=self.velocity_noise_stdev)
            steering += np.random.normal(
                loc=0.0, scale=self.steering_noise_stdev)
            steering = normalize_radians(steering)

        # Dynamics update:
        prev_timestamp = 0.0
        if self.current_ts > 0:
            prev_timestamp = self.dataset.timestamps[self.current_ts - 1]
        delta_t = self.dataset.timestamps[self.current_ts] - prev_timestamp
        assert delta_t > 0
        new_state = draw_dr.dynamics(
            self.dataset, old_state, velocity, steering, delta_t)

        # Add dynamics noise:
        if self.add_dynamics_noise:
            new_state[0] += np.random.normal(
                loc=0.0, scale=self.x_noise_stdev)
            new_state[1] += np.random.normal(
                loc=0.0, scale=self.y_noise_stdev)
            new_state[2] += np.random.normal(
                loc=0.0, scale=self.theta_noise_stdev)
            new_state[2] = normalize_radians(new_state[2])

        # Extract obstacles.
        obstacles = particle.obstacles
        if (len(obstacles) == 0 and
                self.dataset.ts2sensor[self.current_ts] == 'laser'):
            laser_x, laser_y, laser_theta = lasers.car_loc_to_laser_loc(
                particle.x, particle.y, particle.theta,
                self.dataset.a, self.dataset.b)
            obstacles = ransac.extract_obstacles(
                laser_x, laser_y, laser_theta,
                self.laser_angles, self.laser_max_range,
                np.array(self.dataset.ts2laser[self.current_ts]))
            # print "old obstacles:", particle.obstacles
            # print "new obstacles:", obstacles

        return LocPFParticle(
            new_state[0], new_state[1], new_state[2], obstacles)

    def logweigh_particle(self, particle):
        logweight = 0.0

        if self.dataset.ts2sensor[self.current_ts] == 'laser':
            laser_x, laser_y, laser_theta = lasers.car_loc_to_laser_loc(
                particle.x, particle.y, particle.theta,
                self.dataset.a, self.dataset.b)
            true_lasers = readings_for_obstacles(
                laser_x, laser_y, laser_theta,
                self.laser_angles, self.laser_max_range,
                particle.obstacles)
            obs_lasers = np.array(self.dataset.ts2laser[self.current_ts])
            logweight = norm_log_pdf_id_cov(
                obs_lasers, true_lasers, self.obs_cov_scale)

        return logweight

    def have_more_data(self):
        # return self.current_ts < 30
        return self.current_ts < len(self.dataset.timestamps)

    def hook_after_initialize(self):
        self.old_scatter = None
        self.old_obstacle_scatter = None
        self.old_gps = None
        self.gps_timesteps = []
        self.ground_gps_traj = []
        self.ground_gps_lliks = []
        self.ground_gps_lliks_line = None
        self.map_traj = []
        self.map_lliks = []
        self.map_lliks_line = None
        self.err2s = []
        self.err2_line = None
        self.old_map_traj_line = None
        self.plot_xs = []
        self.plot_counter = 0

    def hook_after_advance(self, new_particles, norm_logweights):
        self.debug_new_particles = new_particles
        self.debug_logweights = norm_logweights

    def hook_after_resample(self, indices):
        # dists = [
        #     (p.x - self.dataset.init_x) ** 2 +
        #     (p.y - self.dataset.init_y) ** 2
        #     for p in self.debug_new_particles]
        # sorted_is = np.argsort(dists)
        # for i in sorted_is:
        #     p = self.debug_new_particles[i]
        #     lw = self.debug_logweights[i]
        #     print "particle at {}: logweight {} weight {}".format(
        #         (p.x, p.y, p.theta), lw, np.exp(lw))

        print "-> {} particles survived:".format(len(set(indices)))

        # for i in set(indices):
        #     p = self.debug_new_particles[i]
        #     print "particle at {}".format((p.x, p.y, p.theta))

    def hook_after_step(self):
        # This is needed for advance_particle() to work correctly.
        if self.dataset.ts2sensor[self.current_ts] == 'control':
            self.current_velocity, self.current_steering = \
                self.dataset.ts2control[self.current_ts]

        # Everything below is plotting and output; not needed for correctness.

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

        if self.dataset.ts2sensor[self.current_ts] == 'gps':
            self.gps_timesteps.append(self.current_ts)
            self.ground_gps_traj.append(
                self.dataset.ground_ts2gps[self.current_ts])
            best_i = np.argmax(self.debug_logweights)
            best_particle = self.debug_new_particles[best_i]
            self.map_traj.append(
                (best_particle.x, best_particle.y, best_particle.theta))
            # TODO avg_traj

        # Plotting is slow, so don't do it at every time step.
        if self.dataset.ts2sensor[self.current_ts] == 'laser':
            self.plot_counter += 1
            do_draw = (self.plot_counter % 100 == 0)

            # Scatterplot particles.
            if do_draw:
                all_particles = []
                for particle in self.particles:
                    assert -np.pi <= particle.theta <= np.pi
                    all_particles.append((
                        particle.x, particle.y, particle.theta))
                all_particles = np.array(all_particles)
                if self.old_scatter:
                    self.old_scatter.remove()
                self.old_scatter = self.ax1.scatter(
                    all_particles[:, 0],
                    all_particles[:, 1],
                    c=all_particles[:, 2],
                    s=10,
                    linewidths=0,
                    vmin=-np.pi,
                    vmax=np.pi,
                    label='all particles')

            best_i = np.argmax(self.debug_logweights)
            best_particle = self.debug_new_particles[best_i]
            print "best_particle = {}".format(
                [best_particle.x, best_particle.y, best_particle.theta])

            # Plot obstacles of the best particle.
            if do_draw:
                if self.old_obstacle_scatter:
                    self.old_obstacle_scatter.remove()
                self.old_obstacle_scatter = self.ax1.scatter(
                    best_particle.obstacles[:, 0],
                    best_particle.obstacles[:, 1],
                    c='k',
                    s=50,
                    marker='x')

            # Plot the last known ground GPS location.
            ground_gps = self.ground_gps_traj[-1]
            ground_gps_ts = self.gps_timesteps[-1]
            if do_draw:
                print "ground_gps = {}".format(ground_gps)
                print "dx = {};  dy = {};  dtheta = {}".format(
                    best_particle.x - ground_gps[0],
                    best_particle.y - ground_gps[1],
                    best_particle.theta - ground_gps[2])
                if self.old_gps:
                    self.old_gps.remove()
                self.old_gps = self.ax1.scatter(
                    [ground_gps[0]], [ground_gps[1]], marker='x')

            # Update MAP trajectory plot.
            if do_draw:
                if self.old_map_traj_line:
                    self.old_map_traj_line.remove()
                self.old_map_traj_line = self.ax1.plot(
                    [pose[0] for pose in self.map_traj],
                    [pose[1] for pose in self.map_traj], 'r')[0]

            # Update plot of ground_gps lliks.
            self.plot_xs.append(ground_gps_ts)
            ground_gps_llik = self.logweigh_particle(LocPFParticle(
                ground_gps[0], ground_gps[1], ground_gps[2],
                self.ground_obstacles))
            self.ground_gps_lliks.append(ground_gps_llik)
            if do_draw:
                if self.ground_gps_lliks_line:
                    self.ground_gps_lliks_line.remove()
                self.ground_gps_lliks_line = self.ax2.plot(
                    self.plot_xs, self.ground_gps_lliks, 'g')[0]

            # Update plot of best_particle lliks.
            # Note: have to recompute llik, because logweights are scaled.
            map_llik = self.logweigh_particle(best_particle)
            self.map_lliks.append(map_llik)
            if do_draw:
                if self.map_lliks_line:
                    self.map_lliks_line.remove()
                self.map_lliks_line = self.ax2.plot(
                    self.plot_xs, self.map_lliks, 'r')[0]

            # Update plot of pose error.
            dx = ground_gps[0] - best_particle.x
            dy = ground_gps[1] - best_particle.y
            err2 = dx * dx + dy * dy
            self.err2s.append(err2)
            if do_draw:
                if self.err2_line:
                    self.err2_line.remove()
                self.err2_line = self.ax3.plot(
                    self.plot_xs, self.err2s, 'b')[0]

            if do_draw:
                plt.draw()
                # raw_input()


# FIXME:
#
# - statistically inefficient because resamples even on timesteps where it's
# not necessary (where advance_particle returns the particles unchanged, and
# logweigh_particle returns uniform weights).
#
# - when considering a laser reading in logweigh_particle(), does not cause
# particles to advance to that time, so particles are a little bit in the past,
# when the last control reading was processed.
#
# - laser readings are not independent of one another... nearby readings are
# highly correlated, which means we might give too much weight to a
# misalignment that spans multiple rays?


def demo(dataset_name, num_particles):
    """
    Run LocPF on the given dataset.
    """
    seed = 666
    np.random.seed(seed)
    random.seed(seed)

    dataset = Dataset.read(dataset_name)
    pf = LocPF(dataset, num_particles)
    pf.run()
    return pf


if __name__ == "__main__":
    if len(sys.argv) != 3:
        raise RuntimeError(
            "Usage example: {} 1_straight num_particles".format(sys.argv[0]))
    pf = demo(sys.argv[1], int(sys.argv[2]))
