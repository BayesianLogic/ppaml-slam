package ppaml_slam;

import java.util.ArrayList;

// NOTE: This file is directly ported from fast_lasers.c, so the style not great.

/**
 * Computes ground-truth laser readings given known pose and obstacles.
 * 
 * This class is deliberately decoupled from BLOG, so that we can test it
 * independently. We plug it into BLOG using the LaserInterp class.
 */
public class LaserLogic {

  /**
   * Coordinates of a single obstacle.
   */
  public static class Obstacle {
    public double x;
    public double y;
    public double r;

    public Obstacle() {
      this(0, 0, 0);
    }

    public Obstacle(double x, double y, double r) {
      this.x = x;
      this.y = y;
      this.r = r;
    }
  };

  private static Double solveQuadraticEquation(double a, double b, double c) {
    double delta = b * b - 4 * a * c;
    if (delta < 0) {
      return null;
    } else {
      return (-b - Math.sqrt(delta)) / (2 * a);
    }
  }

  private static double square(double x) {
    return x * x;
  }

  private static double euclideanDist(double x, double y) {
    return Math.sqrt(square(x) + square(y));
  }

  // Return the index where to insert `key` into (sorted) `elems`.
  private static int bisect(double[] elems, double key) {
    int imin = 0;
    int imax = elems.length;
    while (imin < imax) {
      int imid = (imin + imax) / 2;
      assert (imid < imax);
      if (elems[imid] < key) {
        imin = imid + 1;
      } else {
        imax = imid;
      }
    }
    return imin;
  }

  // Return true iff the given ray hits the obstacle.
  private static boolean updateRay(double laser_x, double laser_y, double laser_theta, double obstacle_x,
      double obstacle_y, double obstacle_r, double laser_max_range, int i, double angle, double[] readings) {
    double a = 1.0;
    double b = (2.0 * (laser_x - obstacle_x) * Math.cos(laser_theta + angle) + 2.0 * (laser_y - obstacle_y)
        * Math.sin(laser_theta + angle));
    double c = (square(laser_x - obstacle_x) + square(laser_y - obstacle_y) - square(obstacle_r));
    Double x1 = solveQuadraticEquation(a, b, c);
    if (x1 == null) {
      return false;
    }
    assert (x1 >= 0 && x1 <= laser_max_range);
    if (x1 < readings[i]) {
      readings[i] = x1;
    }
    return true;
  }

  // Bring angle within [-PI, PI).
  // Currently works only for angles that are at most 2*PI off.
  private static double normalizeRadians(double theta) {
    if (theta < -Math.PI) {
      theta += 2 * Math.PI;
    } else if (theta > Math.PI) {
      theta -= 2 * Math.PI;
    }
    assert (theta >= -Math.PI && theta < Math.PI);
    return theta;
  }

  public static double[] readingsForObstacles(double laserX, double laserY, double laserTheta, double[] laserAngles,
      double laserMaxRange, ArrayList<Obstacle> obstacles) {
    assert (laserTheta >= -Math.PI);
    assert (laserTheta < Math.PI);

    double[] readings = new double[laserAngles.length];
    for (int a = 0; a < laserAngles.length; a++) {
      readings[a] = laserMaxRange;
    }

    for (Obstacle obst : obstacles) {
      double dist = euclideanDist(obst.x - laserX, obst.y - laserY);
      if (dist - obst.r <= 0) {
        continue; // obstacle overlaps with laser location
      }
      if (dist - obst.r >= laserMaxRange) {
        continue; // obstacle is too far
      }

      // Find a ray that hits the obstacle.
      double angleToObst = normalizeRadians(Math.atan2(obst.y - laserY, obst.x - laserX) - laserTheta);
      int index;
      if (angleToObst <= laserAngles[0]) {
        index = 0;
      } else if (angleToObst >= laserAngles[laserAngles.length - 1]) {
        index = laserAngles.length - 1;
      } else {
        index = bisect(laserAngles, angleToObst);
      }

      // Update rays to the left of that index.
      for (int i = index - 1; i >= 0; i--) {
        boolean hit = updateRay(laserX, laserY, laserTheta, obst.x, obst.y, obst.r, laserMaxRange, i, laserAngles[i],
            readings);
        if (!hit) {
          break;
        }
      }

      // Update rays to the right of that index.
      for (int i = index; i < laserAngles.length; i++) {
        boolean hit = updateRay(laserX, laserY, laserTheta, obst.x, obst.y, obst.r, laserMaxRange, i, laserAngles[i],
            readings);
        if (!hit) {
          break;
        }
      }
    }

    /*-
    System.out.println("laser_x = " + laserX);
    System.out.println("laser_y = " + laserY);
    System.out.println("laser_theta = " + laserTheta);
    System.out.print("java_readings = [");
    for (int i = 0; i < laserAngles.length; i++) {
      if (i > 0) {
        System.out.print(", ");
      }
      System.out.print(readings[i]);
    }
    System.out.println("]");
     */

    return readings;
  }
};
