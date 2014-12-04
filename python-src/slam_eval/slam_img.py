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
import os
import numpy as np
from slam_csv import read_floats_csv_file


################################# Evaluation ##################################


import matplotlib as mpl
mpl.use('Agg')
import matplotlib.pyplot as pyplot

def get_lat_lon(dataset):
    """
      [time, lat, lon, ...] data format
    """
    # some lines come in empty, for some reason.
    lat = np.asarray([row[1] for row in dataset if row])
    lon = np.asarray([row[2] for row in dataset if row])
    return lat, lon


def gps_graphic(ax, path, label='Truth', color='k', startpoint=True):
    # gps_graphic(ax, results, 'm', False)

    p_x, p_y = get_lat_lon(path)
    PLOT = ax.plot(p_y, p_x, marker = '.', label=label, color=color)
    pyplot.setp(PLOT, linestyle='-', markersize=2)

    if startpoint:
        start_pos = ax.plot(p_y[0], p_x[0], marker='x', color='r')
        pyplot.setp(start_pos, markersize = 20, markeredgewidth=3.5)

    return ax


def gps_compare_graphic(result, ground):
    """
    """
    fig   = pyplot.figure(figsize=(8,8))
    ax    = fig.add_subplot(111)
    pyplot.title("result - compare")

    ax = gps_graphic(ax, result, "Reconstructed", 'm', False)
    ax = gps_graphic(ax, ground, "Truth")

    SIZE = 15 # originally ten
    ax.set_xlim([-SIZE, SIZE])
    ax.set_ylim([-SIZE, SIZE])

    handles, labels = ax.get_legend_handles_labels()

    ax.legend(handles, labels, loc=3)

    return fig


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
    result_path, ground_path, output_path = read_cli()
    
    paths_result = read_floats_csv_file(result_path, "slam_out_path.csv", 3)
    paths_ground = read_floats_csv_file(
      ground_path, "eval_gps.csv", 4)

    fig = gps_compare_graphic(paths_result, paths_ground)
    fig.savefig(os.path.join(output_path,"img.png"))


if __name__ == '__main__':
    main()
