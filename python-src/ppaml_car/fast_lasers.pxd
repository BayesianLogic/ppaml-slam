cdef extern from "fast_lasers.h":
    void readings_for_obstacles(
            double laser_x, double laser_y, double laser_theta,
            int num_angles, double laser_angles[], double laser_max_range,
            int num_obstacles, double obstacles[][3], double readings[])
