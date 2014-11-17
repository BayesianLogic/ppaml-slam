#include <assert.h>
#include <math.h>
#include <stdbool.h>

#define M_PI 3.14159265358979323846264338327

void solve_quadratic_equation(
        double a, double b, double c, bool *success, double *x1, double *x2) {
    double delta = b * b - 4 * a * c;
    if (delta < 0) {
        *success = false;
    } else {
        *success = true;
        *x1 = (-b - sqrt(delta)) / (2 * a);
        *x2 = (-b + sqrt(delta)) / (2 * a);
    }
}

double square(double x) {
    return x * x;
}

double euclidean_dist(double x, double y) {
    return sqrt(square(x) + square(y));
}

/*
Return the index where to insert `key` into (sorted) `elems`.
*/
int bisect(int num_elems, double elems[], double key) {
    int imin = 0;
    int imax = num_elems;
    while (imin < imax) {
        int imid = (imin + imax) / 2;
        assert(imid < imax);
        if (elems[imid] < key) {
            imin = imid + 1;
        } else {
            imax = imid;
        }
    }
    return imin;
}

// Return true iff the given ray hits the obstacle.
bool update_ray(
        double laser_x, double laser_y, double laser_theta,
        double obstacle_x, double obstacle_y, double obstacle_r,
        double laser_max_range,
        int i, double angle, double readings[]) {
    double a = 1.0;
    double b = (2.0 * (laser_x - obstacle_x) * cos(laser_theta + angle) +
                2.0 * (laser_y - obstacle_y) * sin(laser_theta + angle));
    double c = (square(laser_x - obstacle_x) +
                square(laser_y - obstacle_y) -
                square(obstacle_r));
    bool success;
    double x1, x2;
    solve_quadratic_equation(a, b, c, &success, &x1, &x2);
    if (!success) {
        return false;
    }
    assert(x1 >= 0 && x1 <= laser_max_range);
    if (x1 < readings[i]) {
        readings[i] = x1;
    }
    return true;
}


// Bring angle within [-PI, PI).
// Currently works only for angles that are at most 2*PI off.
double normalize_radians(double theta) {
    if (theta < -M_PI) {
        theta += 2 * M_PI;
    } else if (theta > M_PI) {
        theta -= 2 * M_PI;
    }
    assert (theta >= -M_PI && theta < M_PI);
    return theta;
}

void readings_for_obstacles(
        double laser_x, double laser_y, double laser_theta,
        int num_angles, double laser_angles[], double laser_max_range,
        int num_obstacles, double obstacles[][3], double readings[]) {
    for (int i = 0; i < num_angles; i++) {
        readings[i] = laser_max_range;
    }

    for (int o = 0; o < num_obstacles; o++) {
        double x = obstacles[o][0];
        double y = obstacles[o][1];
        double r = obstacles[o][2];
        double dist = euclidean_dist(x - laser_x, y - laser_y);
        if (dist - r <= 0) {
            continue; // obstacle overlaps with laser location
        }
        if (dist - r >= laser_max_range) {
            continue; // obstacle is too far
        }

        // Find a ray that hits the obstacle.
        double angle_to_obst = normalize_radians(atan2(y - laser_y, x - laser_x) - laser_theta);
        int index;
        if (angle_to_obst <= laser_angles[0]) {
            index = 0;
        } else if (angle_to_obst >= laser_angles[num_angles - 1]) {
            index = num_angles - 1;
        } else {
            index = bisect(num_angles, laser_angles, angle_to_obst);
        }

        // Update rays to the left of that index.
        for (int i = index - 1; i >= 0; i--) {
            bool hit = update_ray(
                laser_x, laser_y, laser_theta,
                x, y, r, laser_max_range,
                i, laser_angles[i], readings);
            if (!hit) {
                break;
            }
        }

        // Update rays to the right of that index.
        for (int i = index; i < num_angles; i++) {
            bool hit = update_ray(
                laser_x, laser_y, laser_theta,
                x, y, r, laser_max_range,
                i, laser_angles[i], readings);
            if (!hit) {
                break;
            }
        }
    }
}
