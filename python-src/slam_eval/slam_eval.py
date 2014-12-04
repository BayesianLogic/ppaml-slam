#!/usr/bin/python
# slam_eval -- evaluates SLAM data set in relation to the ground truth
# Copyright (C) 2014  Galois, Inc.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are
# met:
#   1. Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#   2. Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in
#      the documentation and/or other materials provided with the
#      distribution.
#   3. Neither Galois's name nor the names of other contributors may be
#      used to endorse or promote products derived from this software
#      without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY GALOIS AND OTHER CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
# PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL GALOIS OR OTHER
# CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
# EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
# PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
# PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
# LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
# NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import optparse
import sys

import numpy as np

from slam_csv import read_floats_csv_file


################################# Evaluation ##################################

def gps_euclidean_distance(result, ground):
    # presupposes that the input has identical time stamps and samples
    strip_xy  = lambda x: np.array([x[1], x[2]])
    result_xy = map(strip_xy, result)
    ground_xy = map(strip_xy, ground)

    dist = lambda (a, b): np.linalg.norm(a-b)
    distance_xy = map(dist, zip(result_xy, ground_xy))
    return sum(distance_xy)

EPSILON  = 10**-4
MIN_TIME =  1.0 - EPSILON
MAX_TIME =  10.0 + EPSILON

def gps_time_bounds(result, ground):
    # this should be applied after time_match
    # some lines come in empty for some reason
    bound = lambda vec : bool(vec) and vec[0] <= MAX_TIME and vec[0] >= MIN_TIME
    result = filter(bound, result)
    ground = filter(bound, ground)

    # assert(len(result) == len(ground)) 
    # TODO: raise exception
    # XXXPMR: force same length instead of assert
    [result, ground] = zip(*zip(result, ground))

    delta_check = lambda (a, b): np.abs(a[0]-b[0]) > EPSILON
    tst = filter(delta_check, zip(result, ground))
    # tst should be an empty list
    if tst: # TODO: raise exception
        print len(tst)
        for elm in tst:
            print elm

    return result, ground


def gps_compare(result, ground):
    result, ground = gps_time_bounds(result, ground)
    return gps_euclidean_distance(result, ground)


def gaussian2(a, bx, by, c, x, y):
    return a * np.exp(-(np.sqrt((x-bx)**2 + (y-by)**2)) / (2*c*c))

def map_pointset(points, x_range, y_range, width):
    xs, ys = np.meshgrid(x_range, y_range)
    (npts, _junk) = points.shape
    result = np.zeros(xs.shape)
    for i in xrange(0, npts):
        result = np.maximum(result,
                            gaussian2(1.0,
                                      points[i,0], points[i,1],
                                      width,
                                      xs, ys))
    return result

def map_metric(points1, points2, x_range, y_range, width):
    map1 = map_pointset(points1, x_range, y_range, width)
    map2 = map_pointset(points2, x_range, y_range, width)
    return np.sum(np.abs(map1 - map2))

def obsticle_compare(result, ground):
    # How many extra points were found?
    n_extras = np.abs(len(result) - len(ground))
    # How close were the found points?
    score = map_metric(np.array(list(result)), np.array(list(ground)),
                       np.arange(-5, 5, 0.1), np.arange(-5, 5, 0.1),
                       1.0)
    return (n_extras, score)


#################################### Main #####################################

def read_cli():
    parser = optparse.OptionParser(
        usage="Usage: %prog result ground_truth output")
    _options, args = parser.parse_args()
    if len(args) != 3:
        parser.print_usage()
        sys.exit(1)
    else:
        return args

def main():
    solution_output, ground_path, output_path = read_cli()

    try:
        objects_result = read_floats_csv_file(
            #XXXPMR :: this needs to be updated per the challenge problem description
            solution_output,
            "slam_out_landmarks.csv", 2)
        
        objects_ground = read_floats_csv_file(ground_path, "eval_obstacles.csv", 2)
        # obsicle_compaire has to be moved here because some teams create the file
        # and partially populate it
        extras, score = obsticle_compare(objects_result, objects_ground)
    except Exception as e:
        print e
        extras, score = None, None        

    try:
        paths_result = read_floats_csv_file(solution_output, "slam_out_path.csv", 3)
        paths_ground = read_floats_csv_file(ground_path, "eval_gps.csv", 4)
    except Exception as e:
        print e
        dist  = None
        diff  = None
        lines = None
    else:
        diff  = abs(len(paths_result) - len(paths_ground))
        lines = len(paths_ground)
        dist  = gps_compare(paths_result, paths_ground)

    with open(output_path, 'w') as output:
        output.writelines(str(val) + '\n' for val in (extras, score, dist, diff,lines))


if __name__ == '__main__':
    main()
