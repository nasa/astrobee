#!/usr/bin/env python

""" Determines if a given pose or list of poses in a file is within an Astrobee volume
    based on a database of known poses and their corresponding ML features
"""

import math
import os
import re
import subprocess
import sys
import timeit

import constants
from tf.transformations import *


class Coverage_Analyzer:
    def __init__(self):

        self.grid_size = 0.30  # meters
        self.max_pose_error = ((self.grid_size / 2.0) ** 2) * 3  # Max squared error
        self.matchCounter = 0
        print(
            "Grid size: {:.3f}, Max Pose Error: {:.3f}".format(
                self.grid_size, self.max_pose_error
            )
        )

        self.grid_params_z16 = [
            2
        ]  # Bays 1-6: Grid params for Z-axis of FWD and AFT walls
        self.grid_params_x16 = [
            2
        ]  # Bays 1-6: Grid params for X-axis of OVHD and DECK walls
        self.grid_params_z67 = [
            2
        ]  # Bays 6-8: Grid params for Z-axis of FWD and AFT walls
        self.grid_params_x67 = [
            2
        ]  # Bays 6-8: Grid params for X-axis of FWD and AFT walls
        self.x_init_bay16 = 0.0
        self.x_fin_bay16 = 0.0
        self.x_init_bay67 = 0.0
        self.x_fin_bay67 = 0.0

    # Returns the squared distance between two Points (x,y,z)
    def pose_inside_cube(self, robot_pose, cube_pose, grid_size):
        cube_center = grid_size / 2.0

        cube_squared = ((cube_center) ** 2) * 3

        error_squared = (
            (robot_pose[0] - cube_pose[0]) ** 2
            + (robot_pose[1] - cube_pose[1]) ** 2
            + (robot_pose[2] - cube_pose[2]) ** 2
        )

        # True if inside cube
        return error_squared < cube_squared

    # Calculates the relative rotation between 2 quaternions, returns angle between them
    def quat_error(self, robot_quat, test_quat):

        q1_inverse = self.quat_inverse(robot_quat)

        q_relative = quaternion_multiply(test_quat, q1_inverse)

        ang_error = 2 * math.atan2(
            math.sqrt(q_relative[0] ** 2 + q_relative[1] ** 2 + q_relative[2] ** 2),
            q_relative[3],
        )

        # To compensate for a singularity where atan2 = Pi
        tolerance = 0.008  # 0.00 rad = 0.46 deg
        if abs((math.pi * 2) - ang_error) < tolerance:
            ang_error = 0.0

        return ang_error

    # Returns the inverse (conjugate) of the quaternion given
    def quat_inverse(self, robot_quat):
        q_inv = [robot_quat[0], robot_quat[1], robot_quat[2], -robot_quat[3]]

        return q_inv

    def find_nearby_pose(self, testPt, fileIn, fileOut):
        write_features = False

        print(
            "Finding nearby poses around: {:3f} {:3f} {:3f} {:3f} {:3f} {:3f} {:3f}".format(
                testPt[1],
                testPt[2],
                testPt[3],
                testPt[4],
                testPt[5],
                testPt[6],
                testPt[7],
            )
        )  # skips ML num

        with open(fileIn) as f:
            f_out = open(fileOut, "a")
            lines = f.readlines()

            for line in lines:
                # Skip empty lines and those starting with comments
                if re.match("^\s*\n", line) or re.match("^\s*\#", line):
                    continue

                # Extract all the values separated by spaces
                vals = []
                for v in re.split("\s+", line):
                    if v == "":
                        continue
                    vals.append(v)

                if len(vals) == constants.NUM_FEAT_AND_ROBOT_COORDS_COLUMNS:
                    ml_num = int(vals[0])
                    robot_pose = [float(vals[1]), float(vals[2]), float(vals[3])]
                    robot_quat = [
                        float(vals[4]),
                        float(vals[5]),
                        float(vals[6]),
                        float(vals[7]),
                    ]

                    test_pose = [testPt[1], testPt[2], testPt[3]]
                    test_quat = [testPt[4], testPt[5], testPt[6], testPt[7]]

                    if self.pose_inside_cube(robot_pose, test_pose, grid_size) == True:
                        f_out.write(
                            "{:d} {:.3f} {:.3f} {:.3f} {:.3f} {:.3f} {:.3f} {:.3f}\n".format(
                                ml_num,
                                robot_pose[0],
                                robot_pose[1],
                                robot_pose[2],
                                robot_quat[0],
                                robot_quat[1],
                                robot_quat[2],
                                robot_quat[3],
                            )
                        )

                        write_features = True
                        self.matchCounter += 1

                    else:
                        write_features = False
                        ml_num = 0

                if (
                    len(vals) == constants.FEAT_COORDS_COLUMNS
                    and write_features
                    and ml_num > 0
                ):
                    f_out.write("{:s}".format(line))
                    ml_num -= 1

    # From the file containing each trajectory pose and its corresponding landmark poses
    # save only the landmark poses in another file
    def feature_extraction(self, databaseBodyFrame_fileIn, database_fileOut):

        print(
            "Extracting features from: {:s} to {:s}".format(
                databaseBodyFrame_fileIn, database_fileOut
            )
        )
        with open(database_fileOut, "w") as f_out:
            with open(databaseBodyFrame_fileIn, "r") as f_in:

                for line in f_in:
                    # Skip empty lines and those starting with comments
                    if re.match("^\s*\n", line) or re.match("^\s*\#", line):
                        continue

                    # Extract all the values separated by spaces
                    vals = []
                    for v in re.split("\s+", line):
                        if v == "":
                            continue
                        vals.append(v)

                    if len(vals) == 3:
                        f_out.write("{:s}".format(line))

    # From the landmark poses remove  all repeated landmarks
    def remove_repeated_poses(self, database_fileOut, feat_only_fileOut):

        lines_seen = set()  # holds lines seen
        print(
            "Removing duplicated feature poses from {:s} and creating {:s}".format(
                database_fileOut, feat_only_fileOut
            )
        )

        match = 0
        numlines = 0
        with open(feat_only_fileOut, "w") as f_out:
            with open(database_fileOut, "r+") as f_in:

                for line in f_in:
                    if line not in lines_seen:
                        f_out.write(line)
                        lines_seen.add(line)
                        match += 1
                    numlines += 1
        print("Found {:d} unique lines in {:d} lines".format(match, numlines))

    # Find any landmark pose within a cube of grid_size from the landmark poses-only file
    # and generate a coverage file with the tested pose (center of the cube) and
    # the number of features found within that cube
    def find_nearby_cube_pose(
        self, px, py, pz, grid_size, feat_only_fileOut, coverage_fileOut
    ):

        num_ml = 0

        with open(coverage_fileOut, "a") as f_out:
            with open(feat_only_fileOut, "r") as f_in:

                for line in f_in:
                    # Skip empty lines and those starting with comments
                    if re.match("^\s*\n", line) or re.match("^\s*\#", line):
                        continue

                    # Extract all the values separated by spaces
                    vals = []
                    for v in re.split("\s+", line):
                        if v == "":
                            continue
                        vals.append(v)

                    if len(vals) == 3:
                        robot_pose = [float(vals[0]), float(vals[1]), float(vals[2])]
                        cube_pose = [px, py, pz]

                        if (
                            self.pose_inside_cube(robot_pose, cube_pose, grid_size)
                            == True
                        ):
                            num_ml += 1
                f_out.write("{:d} {:.3f} {:.3f} {:.3f}\n".format(num_ml, px, py, pz))

    # Equivalent to using math.isclose() since this is Python 2.7x
    def is_close(self, a, b, rel_tol=1e-09, abs_tol=0.0):
        return abs(a - b) <= max(rel_tol * max(abs(a), abs(b)), abs_tol)

    def calculate_grid_params(self, initial_grid_size, start_pos, end_pos):
        dist = abs(end_pos - start_pos)
        num_cubes = math.floor(dist / initial_grid_size)  # float

        dist_delta = 100.0  # arbitrary initial value
        result_grid_size = 0.0
        counter = 1
        max_num_counter = 10000.0

        # Minimizing dist_delta by changing the grid_size
        while self.is_close(dist_delta, 0.0, abs_tol=0.01) == False:
            result_grid_size = initial_grid_size + (
                initial_grid_size * (counter / max_num_counter)
            )
            dist_delta = dist - num_cubes * result_grid_size
            counter += 1
            if counter == max_num_counter:
                print("dist_delta did not converge")
                break

        return [int(num_cubes), result_grid_size]

    def set_grid_params(self):

        # Bays 1-6: Grid params for Z-axis of FWD and AFT walls
        self.grid_params_z16 = self.calculate_grid_params(
            self.grid_size, constants.ZMAX_BAY16, constants.ZMIN_BAY16
        )

        # Bays 1-6: Grid params for X-axis of OVHD and DECK walls
        self.x_init_bay16 = constants.XMIN_BAY16 - self.grid_params_z16[1]
        self.x_fin_bay16 = constants.XMAX_BAY16 + self.grid_params_z16[1]

        self.grid_params_x16 = self.calculate_grid_params(
            self.grid_size, self.x_init_bay16, self.x_fin_bay16
        )

        # Bays 6-8: Grid params for Z-axis of FWD and AFT walls
        self.grid_params_z67 = self.calculate_grid_params(
            self.grid_size, constants.ZMAX_BAY67, constants.ZMIN_BAY67
        )

        # Bays 6-8: Grid params for X-axis of OVHD and DECK walls
        self.x_init_bay67 = constants.XMIN_BAY67 - self.grid_params_z67[1]
        self.x_fin_bay67 = constants.XMAX_BAY67 + self.grid_params_z67[1]

        self.grid_params_x67 = self.calculate_grid_params(
            self.grid_size, self.x_init_bay67, self.x_fin_bay67
        )

    def calculate_coverage_overhead(
        self, bay_num, start_pos_y, stop_pos_y, feat_only_fileOut, coverage_fileOut
    ):

        current_bay = bay_num + 1
        print("Finding matches Overhead Bay" + str(current_bay))

        if current_bay < 6:
            grid_size = self.grid_params_x16[1]
            xlen = self.grid_params_x16[0]
            start_pos_x = self.x_init_bay16
            p_center = grid_size / 2.0  # Center of the cube
            pz = constants.ZMAX_BAY16 + p_center

        else:
            grid_size = self.grid_params_x67[1]
            xlen = self.grid_params_x67[0]
            start_pos_x = self.x_init_bay67
            p_center = grid_size / 2.0  # Center of the cube
            pz = constants.ZMAX_BAY67 + p_center

        ylen = int(abs(stop_pos_y - start_pos_y) / grid_size)

        f_out = open(coverage_fileOut, "a")
        f_out.write("# Wall Overhead_" + str(current_bay) + " " + str(grid_size) + "\n")
        f_out.close()

        for i in range(xlen):
            for j in range(ylen):
                px = (start_pos_x - grid_size * i) - p_center
                py = (start_pos_y - grid_size * j) - p_center
                self.find_nearby_cube_pose(
                    px, py, pz, grid_size, feat_only_fileOut, coverage_fileOut
                )

    def calculate_coverage_forward(
        self, bay_num, start_pos_y, stop_pos_y, feat_only_fileOut, coverage_fileOut
    ):

        current_bay = bay_num + 1
        print("Finding matches Forward Bay" + str(current_bay))

        if current_bay < 6:
            grid_size = self.grid_params_z16[1]
            zlen = self.grid_params_z16[0]
            p_center = grid_size / 2.0  # Center of the cube
            px = constants.XMIN_BAY16 - p_center

        else:
            grid_size = self.grid_params_z67[1]
            zlen = self.grid_params_z67[0]
            p_center = grid_size / 2.0  # Center of the cube
            px = constants.XMIN_BAY67 - p_center

        ylen = int(abs(stop_pos_y - start_pos_y) / grid_size)

        f_out = open(coverage_fileOut, "a")
        f_out.write("# Wall Forward_" + str(current_bay) + " " + str(grid_size) + "\n")
        f_out.close()

        for i in range(ylen):
            for j in range(zlen):
                py = (start_pos_y - grid_size * i) - p_center
                pz = (
                    constants.ZMAX_BAY16 + grid_size * j
                ) + p_center  # Using z_max to have better coverage on the overhead.
                self.find_nearby_cube_pose(
                    px, py, pz, grid_size, feat_only_fileOut, coverage_fileOut
                )

    def calculate_coverage_aft(
        self, bay_num, start_pos_y, stop_pos_y, feat_only_fileOut, coverage_fileOut
    ):

        current_bay = bay_num + 1
        print("Finding matches Aft Bay" + str(current_bay))

        if current_bay < 6:
            grid_size = self.grid_params_z16[1]
            zlen = self.grid_params_z16[0]
            p_center = grid_size / 2.0  # Center of the cube
            px = constants.XMAX_BAY16 + p_center

        else:
            grid_size = self.grid_params_z67[1]
            zlen = self.grid_params_z67[0]
            p_center = grid_size / 2.0  # Center of the cube
            px = constants.XMAX_BAY67 + p_center

        ylen = int(abs(stop_pos_y - start_pos_y) / grid_size)

        f_out = open(coverage_fileOut, "a")
        f_out.write("# Wall Aft_" + str(current_bay) + " " + str(grid_size) + "\n")
        f_out.close()

        for i in range(ylen):
            for j in range(zlen):
                py = (start_pos_y - grid_size * i) - p_center
                pz = (
                    constants.ZMAX_BAY16 + grid_size * j
                ) + p_center  # Using z_max to have better coverage on the overhead.
                self.find_nearby_cube_pose(
                    px, py, pz, grid_size, feat_only_fileOut, coverage_fileOut
                )

    def calculate_coverage_deck(
        self, bay_num, start_pos_y, stop_pos_y, feat_only_fileOut, coverage_fileOut
    ):
        current_bay = bay_num + 1
        print("Finding matches Deck Bay" + str(current_bay))

        if current_bay < 6:
            grid_size = self.grid_params_x16[1]
            xlen = self.grid_params_x16[0]
            start_pos_x = self.x_init_bay16
            p_center = grid_size / 2.0  # Center of the cube
            pz = constants.ZMIN_BAY16 - p_center

        else:
            grid_size = self.grid_params_x16[1]
            xlen = self.grid_params_x67[0]
            start_pos_x = self.x_init_bay67
            p_center = grid_size / 2.0  # Center of the cube
            pz = constants.ZMIN_BAY67 + p_center

        ylen = int(abs(stop_pos_y - start_pos_y) / grid_size)

        f_out = open(coverage_fileOut, "a")
        f_out.write("# Wall Deck_" + str(current_bay) + " " + str(grid_size) + "\n")
        f_out.close()

        for i in range(xlen):
            for j in range(ylen):
                px = (start_pos_x - grid_size * i) - p_center
                py = (start_pos_y - grid_size * j) - p_center
                self.find_nearby_cube_pose(
                    px, py, pz, grid_size, feat_only_fileOut, coverage_fileOut
                )

    def calculate_coverage_airlock(self, feat_only_fileOut, coverage_fileOut):

        f_out = open(coverage_fileOut, "a")
        f_out.write("# Wall Airlock" + " " + str(self.grid_size) + "\n")
        f_out.close()

        print("Finding matches Airlock")
        p_center = self.grid_size / 2.0  # Center of the cube

        xlen = int(
            math.floor(
                abs(constants.XMAX_AIRLK - constants.XMIN_AIRLK) / self.grid_size
            )
        )
        zlen = int(
            math.floor(
                abs(constants.ZMAX_AIRLK - constants.ZMIN_AIRLK) / self.grid_size
            )
        )

        py = constants.YMAX_AIRLK + p_center
        for i in range(xlen):
            for j in range(zlen):
                px = (constants.XMAX_AIRLK + self.grid_size * i) + p_center
                pz = (constants.ZMAX_AIRLK + self.grid_size * j) + p_center
                self.find_nearby_cube_pose(
                    px, py, pz, self.grid_size, feat_only_fileOut, coverage_fileOut
                )

    def find_nearby_features_from_file(self, feat_only_fileOut, coverage_fileOut):

        print(
            "Finding matches from pose file {:s}, creating coverage file: {:s}".format(
                feat_only_fileOut, coverage_fileOut
            )
        )

        f_out = open(coverage_fileOut, "w")
        f_out.write("#Num_ML Cube_Center_Pose (X Y Z)\n")
        f_out.close()

        for bay, positions in constants.BAYS_Y_LIMITS.items():
            start_pos_y = positions[0]
            stop_pos_y = positions[1]

            self.calculate_coverage_overhead(
                bay, start_pos_y, stop_pos_y, feat_only_fileOut, coverage_fileOut
            )
            self.calculate_coverage_forward(
                bay, start_pos_y, stop_pos_y, feat_only_fileOut, coverage_fileOut
            )
            self.calculate_coverage_aft(
                bay, start_pos_y, stop_pos_y, feat_only_fileOut, coverage_fileOut
            )
            self.calculate_coverage_deck(
                bay, start_pos_y, stop_pos_y, feat_only_fileOut, coverage_fileOut
            )

        self.calculate_coverage_airlock(feat_only_fileOut, coverage_fileOut)

    def remove_tmp_file(self, fileToRemove):
        os.remove(fileToRemove)
        print("Removing temporary file: {:s}".format(fileToRemove))


if __name__ == "__main__":

    try:

        if len(sys.argv) < 1:
            print("Usage: " + sys.argv[0] + "<activity_database_file>")
            print(
                "       <activity_database_file>: '/dir/to/activity_database_file.csv' containing robot poses and ML features to compare with\n"
            )
            sys.exit(1)

        activity_database_fileIn = sys.argv[1]

        f_in = activity_database_fileIn.find("_db.csv")
        database_fileOut = (
            activity_database_fileIn[:f_in] + "_features_db.csv"
        )  # Name of file with all features' pose
        feat_only_fileOut = (
            activity_database_fileIn[:f_in] + "_features_db_nonrepeat.csv"
        )  # Name of file with non-repeated features' pose
        coverage_fileOut = (
            activity_database_fileIn[:f_in] + "_coverage_db.csv"
        )  # Name of output file with coverage database

        obj = Coverage_Analyzer()

        obj.set_grid_params()
        obj.feature_extraction(activity_database_fileIn, database_fileOut)
        obj.remove_repeated_poses(database_fileOut, feat_only_fileOut)

        tic = timeit.default_timer()
        obj.find_nearby_features_from_file(feat_only_fileOut, coverage_fileOut)
        toc = timeit.default_timer()
        elapsedTime = toc - tic
        print("Time to create coverage file: {:.2f}s".format(elapsedTime))
        obj.remove_tmp_file(database_fileOut)
        obj.remove_tmp_file(feat_only_fileOut)

    except KeyboardInterrupt:
        print("\n <-CTRL-C EXIT: USER manually exited!->")
        sys.exit(0)
