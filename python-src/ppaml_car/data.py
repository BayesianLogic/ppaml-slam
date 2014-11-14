import ppaml_car

from numpy.testing import assert_almost_equal as assert_feq
import csv
import numpy as np
import os


ALL_DATASETS = [
    "1_straight",
    "2_bend",
    "3_curvy",
    "4_circle",
    "5_eight",
    "6_loop",
    "7_random",
]

LASER_COLS = 361
INTENSITY_COLS = 361
X_MIN = -7
X_MAX = 7
Y_MIN = -7
Y_MAX = 7
LASER_MIN = 0.0
LASER_MAX = 10.0
INTENSITY_MIN = 0.0
INTENSITY_MAX = 32768.0


class Dataset(object):
    """
    This stores everything as Python lists, not numpy arrays.
    """
    def __init__(self):
        # Car properties:
        self.L = None
        self.h = None
        self.a = None
        self.b = None
        self.init_angle = None
        self.init_x = None
        self.init_y = None

        # Mapping from timestep (integer) to timestamp (real).
        self.timestamps = None

        # Type of sensor ('gps', 'control', 'laser'), indexed by timestep.
        self.ts2sensor = None

        # Input data (indexed by timestep).
        self.ts2control = None  # tuples of (velocity, steering)
        self.ts2laser = None  # arrays of 361 readings
        self.ts2intensity = None  # arrays of 361 readings

        # Ground-truth data for evaluation (indexed by timestep).
        self.ground_obstacles = []  # tuples of (x, y)
        self.ground_ts2control = None  # tuples of (velocity, steering)
        self.ground_ts2gps = None  # tuples of (x, y, angle)
        self.ground_ts2laser = None  # arrays of 361 readings
        self.ground_ts2intensity = None  # arrays of 361 readings

        # We always store (x, y, theta) in that order, even though the CSV
        # files sometimes list latitude first.
        #
        # The laser and intensity values are given counterclockwise, from 0
        # degrees (to agent's right) to 180 degrees (to agent's left), in
        # 0.5-degree increments.

    @staticmethod
    def read(dirname, prefix_path=None):
        """
        Read the given dataset, e.g. "1_straight".
        """
        if prefix_path is None:
            prefix_path = os.path.join(ppaml_car.__path__[0], '../../data')
        dirpath = os.path.join(prefix_path, dirname)
        dataset = Dataset()
        dataset.read_properties(dirpath)
        dataset.read_timestamps(dirpath)
        dataset.read_input_data(dirpath)
        dataset.read_ground_data(dirpath)
        return dataset

    def read_properties(self, dirpath):
        path = os.path.join(dirpath, "input", "input_properties.csv")
        with open(path) as csv_file:
            reader = csv.reader(csv_file)
            header = reader.next()
            values = reader.next()
            data = dict(zip(header, map(float, values)))
            self.L = data['L']
            self.h = data['h']
            self.a = data['a']
            self.b = data['b']
            self.init_angle = data['InitialAngle']
            self.init_x = data['GPSLon']
            self.init_y = data['GPSLat']
            for row in reader:
                raise ValueError("did not expect more rows")

    def read_timestamps(self, dirpath):
        self.timestamps = []
        self.ts2sensor = {}
        path = os.path.join(dirpath, "input", "input_sensor.csv")
        code_to_sensor = {'1': 'gps', '2': 'control', '3': 'laser'}
        with open(path) as csv_file:
            reader = csv.reader(csv_file)
            header = reader.next()
            assert header[0] == 'Time'
            for row in reader:
                assert len(row) == 2
                timestamp = float(row[0])
                sensor = code_to_sensor[row[1]]
                self.timestamps.append(timestamp)
                self.ts2sensor[len(self.timestamps) - 1] = sensor

    def read_input_data(self, dirpath):
        control_data = Dataset.read_raw_control_data(
            os.path.join(dirpath, "input", "input_control.csv"))
        laser_intensity_data = Dataset.read_raw_laser_intensity_data(
            os.path.join(dirpath, "input", "input_laser.csv"))
        self.ts2control = {}
        self.ts2laser = {}
        self.ts2intensity = {}
        control_i = 0
        laser_intensity_i = 0
        for ts, time in enumerate(self.timestamps):
            if self.ts2sensor[ts] == 'gps':
                pass
            elif self.ts2sensor[ts] == 'control':
                assert_feq(time, control_data[control_i][0], 2)
                self.ts2control[ts] = control_data[control_i][1:]
                control_i += 1
            elif self.ts2sensor[ts] == 'laser':
                assert_feq(time, laser_intensity_data[laser_intensity_i][0], 2)
                row = laser_intensity_data[laser_intensity_i]
                self.ts2laser[ts] = row[1:1 + LASER_COLS]
                self.ts2intensity[ts] = row[1 + LASER_COLS:]
                laser_intensity_i += 1
            else:
                assert False
        assert control_i == len(control_data)
        assert laser_intensity_i == len(laser_intensity_data)

    def read_ground_data(self, dirpath):
        # Obstacles:
        self.ground_obstacles = []
        path = os.path.join(dirpath, "eval_data", "eval_obstacles.csv")
        with open(path) as csv_file:
            reader = csv.reader(csv_file)
            header = reader.next()
            assert header[0] == 'GPSLon'
            for row in reader:
                assert len(row) == 2
                self.ground_obstacles.append((float(row[0]), float(row[1])))

        # Everything else:
        gps_data = Dataset.read_raw_gps_data(
            os.path.join(dirpath, "eval_data", "eval_gps.csv"))
        control_data = Dataset.read_raw_control_data(
            os.path.join(dirpath, "eval_data", "eval_control.csv"))
        laser_intensity_data = Dataset.read_raw_laser_intensity_data(
            os.path.join(dirpath, "eval_data", "eval_laser.csv"))
        self.ground_ts2gps = {}
        self.ground_ts2control = {}
        self.ground_ts2laser = {}
        self.ground_ts2intensity = {}
        gps_i = 0
        control_i = 0
        laser_intensity_i = 0
        for ts, time in enumerate(self.timestamps):
            if self.ts2sensor[ts] == 'gps':
                assert_feq(time, gps_data[gps_i][0], 2)
                row = gps_data[gps_i]
                self.ground_ts2gps[ts] = [row[2], row[1], row[3]]
                gps_i += 1
            elif self.ts2sensor[ts] == 'control':
                assert_feq(time, control_data[control_i][0], 2)
                self.ground_ts2control[ts] = control_data[control_i][1:]
                control_i += 1
            elif self.ts2sensor[ts] == 'laser':
                assert_feq(time, laser_intensity_data[laser_intensity_i][0], 2)
                row = laser_intensity_data[laser_intensity_i]
                self.ground_ts2laser[ts] = row[1:1 + LASER_COLS]
                self.ground_ts2intensity[ts] = row[1 + LASER_COLS:]
                laser_intensity_i += 1
            else:
                assert False
        assert gps_i == len(gps_data)
        assert control_i == len(control_data)
        assert laser_intensity_i == len(laser_intensity_data)

    @staticmethod
    def read_raw_gps_data(path):
        """
        Return float array with columns
        (time, latitude, longitude, orientation).
        """
        gps = []
        with open(path) as csv_file:
            reader = csv.reader(csv_file)
            header = reader.next()
            assert header[0] == 'TimeGPS'
            for row in reader:
                assert len(row) == 4
                gps.append(map(float, row))
        return gps

    @staticmethod
    def read_raw_control_data(path):
        """
        Return float array with columns (time, velocity, steering).
        """
        controls = []
        with open(path) as csv_file:
            reader = csv.reader(csv_file)
            header = reader.next()
            assert header[0] == 'Time_VS'
            for row in reader:
                assert len(row) == 3
                controls.append(map(float, row))
        return controls

    @staticmethod
    def read_raw_laser_intensity_data(path):
        """
        Return float array with columns:
        (time, laser1, ..., laser361, intensity1, ..., intensity361).
        """
        laser_intensity = []
        with open(path) as csv_file:
            reader = csv.reader(csv_file)
            header = reader.next()
            assert header[0] == 'TimeLaser'
            for row in reader:
                assert len(row) == 1 + LASER_COLS + INTENSITY_COLS
                row = map(float, row)
                time = row[0]
                lasers = row[1:1 + LASER_COLS]
                intensity = row[1 + LASER_COLS:]
                # In the data, the laser and intensity readings are clockwise.
                # Make them counter-clockwise (trigonometric order).
                lasers.reverse()
                intensity.reverse()
                laser_intensity.append([time] + lasers + intensity)
        return laser_intensity

    def get_all_lasers(self):
        """
        Return lasers as an array with (time, laser0, ..., laser360) rows.
        """
        lasers = []
        for ts, time in enumerate(self.timestamps):
            if self.ts2sensor[ts] == 'laser':
                lasers.append([time] + self.ts2laser[ts])
        return np.array(lasers)

    def get_all_ground_lasers(self):
        """
        Return ground lasers as array with (time, laser0, ..., laser360) rows.
        """
        lasers = []
        for ts, time in enumerate(self.timestamps):
            if self.ts2sensor[ts] == 'laser':
                lasers.append([time] + self.ground_ts2laser[ts])
        return np.array(lasers)
