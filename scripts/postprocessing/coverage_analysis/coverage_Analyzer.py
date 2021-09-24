import math
import os
import re
import subprocess
import sys
import timeit

from tf.transformations import *

""" Determines if a given pose or list of poses in a file is within an Astrobee volume
    based on a database of known poses and their corresponding ML features
"""


class Coverage_Analyzer:
    def __init__(self):

        self.grid_size = 0.30  # 0.15 #0.30 # meters
        self.max_pose_error = (
            (self.grid_size / 2.0) ** 2
        ) * 3  # Max squared error (m^3)   0.0918 0.3675
        self.max_quat_error = 0.52  # 0.2616 # Max relative rotation between two quaternions (in radians equiv to 15 deg) 0.52
        self.matchCounter = 0
        print(
            "Grid size: {:.3f}, Max Pose Error: {:.3f}".format(
                self.grid_size, self.max_pose_error
            )
        )

        # Constants in meters based on JEM keepin dimensions as defined in
        # https://babelfish.arc.nasa.gov/bitbucket/projects/ASTROBEE/repos/astrobee_ops/browse/gds/ControlStationConfig/IssWorld-Default/keepins/keepin.json
        self.xmin_bay16 = 11.99  # Lower FWD corner x coordinate at the entry node, keepin zone from bay 1 to 6
        self.ymin_bay16 = -2.75
        self.zmin_bay16 = 5.92
        self.xmax_bay16 = 9.86  # Higher AFT corner x coordinate at bay 6/7, keepin zone from bay 1 to 6
        self.ymax_bay16 = -9.24
        self.zmax_bay16 = 3.78

        self.xmin_bay67 = 12.34  # Lower FWD corner x coordinate at bay 6/7. keepin zone from bay 6/7 to 8
        self.ymin_bay67 = -9.24
        self.zmin_bay67 = 5.95
        self.xmax_bay67 = 9.54  # Higher AFT corner x coordinate at bay 6/7. keepin zone from bay 6/7 to 8
        self.ymax_bay67 = -11.64
        self.zmax_bay67 = 3.75

        self.xmin_airlk = 11.75  # Lower right side corner, airlock
        self.ymin_airlk = -10.60
        self.zmin_airlk = 6.00
        self.xmax_airlk = 10.25  # Higher left side corner, airlock
        self.ymax_airlk = -10.70
        self.zmax_airlk = 4.55

        # self.num_walls = 9        # Overhead x2 Aft x2, Fwd x2, Deck x2, Airlock (Optional: Entry node, back of airlock)
        self.max_num_features = 0  # Max number of features found in a cube of size defined by self.grid_size
        self.max_cubePose_error = (
            (self.grid_size / 2.0) ** 2
        ) * 3  # (0-self.grid_size/2)**2 + (0-self.grid_size/2)**2 + (0-self.grid_size/2)**2 #Assuming cube center is locally at 0,0,0 then e_squared = 0.0075meters
        # self.output_file = ""     # Name of file with non-repeated features' pose

        self.ylen_bay16 = 1.065  # Constant length on y-axis for bay 1-6
        self.num_bays = 9  # Total number of JEM bays analyzed Bay 1-8, plus airlock

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
    def pose_error(self, robot_pose, test_pose):
        error_squared = (
            (robot_pose[0] - test_pose[0]) ** 2
            + (robot_pose[1] - test_pose[1]) ** 2
            + (robot_pose[2] - test_pose[2]) ** 2
        )
        # print("max_error: {:.4f}, error_sq: {:.4f} ".format(self.max_pose_error, error_squared))

        return error_squared

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
        # print(q1_inverse)

        # q_relative = tf.Transformations.quaternion_multiply(test_quat, q1_inverse)
        q_relative = quaternion_multiply(test_quat, q1_inverse)
        # print(q_relative)

        ang_error = 2 * math.atan2(
            math.sqrt(q_relative[0] ** 2 + q_relative[1] ** 2 + q_relative[2] ** 2),
            q_relative[3],
        )
        # print("max_error: {:.3f}, ang_error: {:.3f} ".format(self.max_quat_error, ang_error))

        # To compensate for a singularity where atan2 = Pi
        tolerance = 0.008  # 0.00 rad = 0.46 deg
        if abs((math.pi * 2) - ang_error) < tolerance:
            ang_error = 0.0
        # print("max_error: {:.3f}, ang_error: {:.3f} ".format(self.max_quat_error, ang_error))

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

                if len(vals) == 8:
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

                    # print("Checking pose {:d} {:.3f} {:.3f} {:.3f} {:.3f} {:.3f} {:.3f} {:.3f}...".format(
                    # ml_num,
                    # robot_pose[0],robot_pose[1], robot_pose[2],
                    # robot_quat[0], robot_quat[1], robot_quat[2], robot_quat[3]))

                    # if (self.pose_error(robot_pose, test_pose) < self.max_pose_error and
                    # self.quat_error(robot_quat, test_quat) < self.max_quat_error):

                    # if (self.pose_error(robot_pose, test_pose) < self.max_pose_error):
                    if self.pose_inside_cube(robot_pose, test_pose, grid_size) == True:
                        # print("Pose {:d} {:.3f} {:.3f} {:.3f} {:.3f} {:.3f} {:.3f} {:.3f}... Passed".format(
                        #     ml_num,
                        #     robot_pose[0],robot_pose[1], robot_pose[2],
                        #     robot_quat[0], robot_quat[1], robot_quat[2], robot_quat[3]))

                        # print("Passed")

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
                        # print("Nope\n")

                if len(vals) == 3 and write_features and ml_num > 0:
                    f_out.write("{:s}".format(line))
                    ml_num -= 1

    def find_nearby_pose_from_file(
        self, testPoint_file, database_fileIn, database_fileOut
    ):

        f_out = open(database_fileOut, "w")
        f_out.write("#Robot and ML Pose results from {:s}\n".format(testPoint_file))
        f_out.write("#Num_ML Robot Pose X Y Z I J K W\n#Feature Pose X Y Z\n")
        f_out.close()

        with open(testPoint_file) as f:

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
                    vals.append(float(v))

                if len(vals) == 8:
                    self.find_nearby_pose(vals, database_fileIn, database_fileOut)

                else:
                    print("There is something wrong in your trajectory file.")

        print(
            "Total matches found: {:d} with Euclidean max error: {:.4f} and angular error: {:.4f}".format(
                self.matchCounter, self.max_pose_error, self.max_quat_error
            )
        )

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
        # self.output_file = re.sub('\.csv$', '', feat_only_fileOut) + "_nonrepeat.csv"
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
    def find_nearby_cubePose(
        self, px, py, pz, grid_size, feat_only_fileOut, coverage_fileOut
    ):

        num_ml = 0

        with open(coverage_fileOut, "a") as f_out:
            # with open(self.output_file, 'r') as f_in:
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

                        # if (self.pose_error(robot_pose, cube_pose) < self.max_cubePose_error):
                        if (
                            self.pose_inside_cube(robot_pose, cube_pose, grid_size)
                            == True
                        ):
                            num_ml += 1
                f_out.write("{:d} {:.3f} {:.3f} {:.3f}\n".format(num_ml, px, py, pz))

    # Equivalent to using math.isclose() since this is Python 2.7x
    def isclose(self, a, b, rel_tol=1e-09, abs_tol=0.0):
        return abs(a - b) <= max(rel_tol * max(abs(a), abs(b)), abs_tol)

    def calculate_grid_params(self, initial_grid_size, start_pos, end_pos):
        # print("start_pos: {:.3f}, end_pos: {:.3f}".format(start_pos, end_pos))
        dist = abs(end_pos - start_pos)
        num_cubes = math.floor(dist / initial_grid_size)  # float

        dist_delta = 100.0  # arbitrary initial value
        result_grid_size = 0.0
        counter = 1
        max_num_counter = 10000.0

        # Minimizing dist_delta by changing the grid_size
        while self.isclose(dist_delta, 0.0, abs_tol=0.01) == False:
            result_grid_size = initial_grid_size + (
                initial_grid_size * (counter / max_num_counter)
            )
            dist_delta = dist - num_cubes * result_grid_size
            counter += 1
            if counter == max_num_counter:
                print("dist_delta did not converge")
                break

        # print("res_grid_size: {:5f}, num_cubes: {:.2f}, dist_delta: {:5f}".format(result_grid_size, num_cubes, dist_delta))

        return [int(num_cubes), result_grid_size]

    def set_grid_params(self):

        # Bays 1-6: Grid params for Z-axis of FWD and AFT walls
        self.grid_params_z16 = self.calculate_grid_params(
            self.grid_size, self.zmax_bay16, self.zmin_bay16
        )
        # print("Bay 1-6, Z-axis, Num Cubes: {:d}, grid_size_z16: {:.8f}".format(self.grid_params_z16[0], self.grid_params_z16[1]))

        # Bays 1-6: Grid params for X-axis of OVHD and DECK walls
        self.x_init_bay16 = self.xmin_bay16 - self.grid_params_z16[1]
        self.x_fin_bay16 = self.xmax_bay16 + self.grid_params_z16[1]

        self.grid_params_x16 = self.calculate_grid_params(
            self.grid_size, self.x_init_bay16, self.x_fin_bay16
        )
        # print("Bays 1-6, X-axis, Num Cubes: {:d}, grid_size_x16: {:.8f}".format(self.grid_params_x16[0], self.grid_params_x16[1]))

        # Bays 6-8: Grid params for Z-axis of FWD and AFT walls
        self.grid_params_z67 = self.calculate_grid_params(
            self.grid_size, self.zmax_bay67, self.zmin_bay67
        )
        # print("Bays 6-8, Z-axis, Num Cubes: {:d}, grid_size_z67: {:.8f}".format(self.grid_params_z67[0], self.grid_params_z67[1]))

        # Bays 6-8: Grid params for X-axis of OVHD and DECK walls
        self.x_init_bay67 = self.xmin_bay67 - self.grid_params_z67[1]
        self.x_fin_bay67 = self.xmax_bay67 + self.grid_params_z67[1]

        self.grid_params_x67 = self.calculate_grid_params(
            self.grid_size, self.x_init_bay67, self.x_fin_bay67
        )
        # print("Bays 6-8, X-axis, Num Cubes: {:d}, grid_size_x67: {:.8f}".format(self.grid_params_x67[0], self.grid_params_x67[1]))

    def calculate_coverage_overhead(
        self, bay_num, start_pos_y, stop_pos_y, feat_only_fileOut, coverage_fileOut
    ):

        current_bay = bay_num + 1
        print("Finding matches Overhead Bay" + str(current_bay))

        # p_center = self.grid_size / 2.0  # Center of the cube
        if current_bay < 6:
            grid_size = self.grid_params_x16[1]
            xlen = self.grid_params_x16[0]
            start_pos_x = self.x_init_bay16
            p_center = grid_size / 2.0  # Center of the cube
            pz = self.zmax_bay16 + p_center

        else:
            grid_size = self.grid_params_x67[1]
            xlen = self.grid_params_x67[0]
            start_pos_x = self.x_init_bay67
            p_center = grid_size / 2.0  # Center of the cube
            pz = self.zmax_bay67 + p_center

        # xlen = int(abs(self.xmax_bay16 - self.xmin_bay16) / self.grid_size)
        # ylen = int(abs(stop_pos_y - start_pos_y) / self.grid_size)
        ylen = int(abs(stop_pos_y - start_pos_y) / grid_size)

        # print("Overhead pz_max: {:.3f}".format(pz))

        f_out = open(coverage_fileOut, "a")
        f_out.write("# Wall Overhead_" + str(current_bay) + " " + str(grid_size) + "\n")
        f_out.close()

        for i in range(xlen):
            for j in range(ylen):
                px = (start_pos_x - grid_size * i) - p_center
                py = (start_pos_y - grid_size * j) - p_center
                self.find_nearby_cubePose(
                    px, py, pz, grid_size, feat_only_fileOut, coverage_fileOut
                )

    def calculate_coverage_forward(
        self, bay_num, start_pos_y, stop_pos_y, feat_only_fileOut, coverage_fileOut
    ):

        current_bay = bay_num + 1
        print("Finding matches Forward Bay" + str(current_bay))

        # p_center = self.grid_size / 2.0  # Center of the cube
        if current_bay < 6:
            grid_size = self.grid_params_z16[1]
            zlen = self.grid_params_z16[0]
            p_center = grid_size / 2.0  # Center of the cube
            px = self.xmin_bay16 - p_center

        else:
            grid_size = self.grid_params_z67[1]
            zlen = self.grid_params_z67[0]
            p_center = grid_size / 2.0  # Center of the cube
            px = self.xmin_bay67 - p_center

        # ylen = int(abs(stop_pos_y - start_pos_y) / self.grid_size)
        # zlen = int(abs(self.zmax_bay16 - self.zmin_bay16) / self.grid_size)
        ylen = int(abs(stop_pos_y - start_pos_y) / grid_size)
        # zlen = grid_params[0]

        # print("zlen: {:d}, start_pos: {:.2f}\n".format(zlen, self.zmin_bay16))

        f_out = open(coverage_fileOut, "a")
        f_out.write("# Wall Forward_" + str(current_bay) + " " + str(grid_size) + "\n")
        f_out.close()

        for i in range(ylen):
            for j in range(zlen):
                py = (start_pos_y - grid_size * i) - p_center
                pz = (
                    self.zmax_bay16 + grid_size * j
                ) + p_center  # Using z_max to have better coverage on the overhead.
                self.find_nearby_cubePose(
                    px, py, pz, grid_size, feat_only_fileOut, coverage_fileOut
                )

    def calculate_coverage_aft(
        self, bay_num, start_pos_y, stop_pos_y, feat_only_fileOut, coverage_fileOut
    ):

        current_bay = bay_num + 1
        print("Finding matches Aft Bay" + str(current_bay))

        # p_center = self.grid_size / 2.0  # Center of the cube
        if current_bay < 6:
            grid_size = self.grid_params_z16[1]
            zlen = self.grid_params_z16[0]
            p_center = grid_size / 2.0  # Center of the cube
            px = self.xmax_bay16 + p_center

        else:
            grid_size = self.grid_params_z67[1]
            zlen = self.grid_params_z67[0]
            p_center = grid_size / 2.0  # Center of the cube
            px = self.xmax_bay67 + p_center

        # ylen = int(abs(stop_pos_y - start_pos_y) / self.grid_size)
        # zlen = int(abs(self.zmax_bay16 - self.zmin_bay16) / self.grid_size)
        ylen = int(abs(stop_pos_y - start_pos_y) / grid_size)
        # zlen = grid_params[0]

        f_out = open(coverage_fileOut, "a")
        f_out.write("# Wall Aft_" + str(current_bay) + " " + str(grid_size) + "\n")
        f_out.close()

        for i in range(ylen):
            for j in range(zlen):
                py = (start_pos_y - grid_size * i) - p_center
                pz = (
                    self.zmax_bay16 + grid_size * j
                ) + p_center  # Using z_max to have better coverage on the overhead.
                self.find_nearby_cubePose(
                    px, py, pz, grid_size, feat_only_fileOut, coverage_fileOut
                )

    def calculate_coverage_deck(
        self, bay_num, start_pos_y, stop_pos_y, feat_only_fileOut, coverage_fileOut
    ):
        current_bay = bay_num + 1
        print("Finding matches Deck Bay" + str(current_bay))

        # p_center = self.grid_size / 2.0  # Center of the cube
        if current_bay < 6:
            grid_size = self.grid_params_x16[1]
            xlen = self.grid_params_x16[0]
            start_pos_x = self.x_init_bay16
            p_center = grid_size / 2.0  # Center of the cube
            pz = self.zmin_bay16 - p_center

        else:
            grid_size = self.grid_params_x16[1]
            xlen = self.grid_params_x67[0]
            start_pos_x = self.x_init_bay67
            p_center = grid_size / 2.0  # Center of the cube
            pz = self.zmin_bay67 + p_center

        # xlen = int(abs(self.xmax_bay16 - self.xmin_bay16) / self.grid_size)
        # ylen = int(abs(stop_pos_y - start_pos_y) / self.grid_size)
        ylen = int(abs(stop_pos_y - start_pos_y) / grid_size)

        # print("Overhead pz_max: {:.3f}".format(pz))

        f_out = open(coverage_fileOut, "a")
        f_out.write("# Wall Deck_" + str(current_bay) + " " + str(grid_size) + "\n")
        f_out.close()

        for i in range(xlen):
            for j in range(ylen):
                px = (start_pos_x - grid_size * i) - p_center
                py = (start_pos_y - grid_size * j) - p_center
                self.find_nearby_cubePose(
                    px, py, pz, grid_size, feat_only_fileOut, coverage_fileOut
                )

    def calculate_coverage_airlock(self, bay_num, feat_only_fileOut, coverage_fileOut):

        f_out = open(coverage_fileOut, "a")
        f_out.write("# Wall Airlock" + " " + str(self.grid_size) + "\n")
        f_out.close()

        print("Finding matches Airlock")
        p_center = self.grid_size / 2.0  # Center of the cube

        xlen = int(math.floor(abs(self.xmax_airlk - self.xmin_airlk) / self.grid_size))
        zlen = int(math.floor(abs(self.zmax_airlk - self.zmin_airlk) / self.grid_size))

        py = self.ymax_airlk + p_center
        for i in range(xlen):
            for j in range(zlen):
                px = (self.xmax_airlk + self.grid_size * i) + p_center
                pz = (self.zmax_airlk + self.grid_size * j) + p_center
                self.find_nearby_cubePose(
                    px, py, pz, self.grid_size, feat_only_fileOut, coverage_fileOut
                )

    def find_nearby_features_from_file(self, feat_only_fileOut, coverage_fileOut):

        print(
            "Finding matches from pose file {:s}, creating coverage file: {:s}".format(
                feat_only_fileOut, coverage_fileOut
            )
        )
        # print("Finding matches from pose file {:s}, creating coverage file: {:s}".format(feat_only_fileOut, coverage_fileOut))
        # p_center = self.grid_size / 2.0  # Center of the cube

        f_out = open(coverage_fileOut, "w")
        f_out.write("#Num_ML Cube_Center_Pose (X Y Z)\n")
        f_out.close()

        for bay in range(self.num_bays):

            if bay == 0:  # Bay 1
                # print("Bay1")
                start_pos_y = self.ymin_bay16
                stop_pos_y = -4.300  # -4.175 # For grid_size -> 0.30m, y= -4.25
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

            elif bay == 1:  # Bay 2
                # print("Bay2")
                start_pos_y = -4.300  # -4.175 #-4.25
                stop_pos_y = (
                    -5.220
                )  # if self.isclose(self.grid_size, 0.30, abs_tol=0.01) else -5.375 #-5.225 #-5.45
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

            elif bay == 2:  # Bay 3
                # print("Bay3")
                start_pos_y = (
                    -5.220
                )  # if self.isclose(self.grid_size, 0.30, abs_tol=0.01) else -5.300 #-5.225 #-5.45
                stop_pos_y = (
                    -6.500
                )  # if self.isclose(self.grid_size, 0.30, abs_tol=0.01) else -6.425 #-6.275 #-6.50
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

            elif bay == 3:  # Bay 4
                # print("Bay4")
                start_pos_y = (
                    -6.430
                )  # if self.isclose(self.grid_size, 0.30, abs_tol=0.01) else -6.350 #-6.275 #-6.35
                stop_pos_y = (
                    -7.550
                )  # if self.isclose(self.grid_size, 0.30, abs_tol=0.01) else -7.400 #-7.55
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

            elif bay == 4:  # Bay 5
                # print("Bay5")
                start_pos_y = (
                    -7.340
                )  # if self.isclose(self.grid_size, 0.30, abs_tol=0.01) else -7.400 #-7.250 #-7.55
                stop_pos_y = (
                    -8.660
                )  # if self.isclose(self.grid_size, 0.30, abs_tol=0.01) else -8.600 #-8.450 #-8.66
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

            elif bay == 5:  # Bay 6
                # print("Bay6")
                start_pos_y = (
                    -8.550
                )  # if self.isclose(self.grid_size, 0.30, abs_tol=0.01) else -8.450 #-8.450 #-8.45
                stop_pos_y = (
                    -9.500
                )  # -9.240 #if self.isclose(self.grid_size, 0.30, abs_tol=0.01) else -9.500 #-9.650
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

            elif bay == 6:  # Bay 7
                # print("Bay7")
                start_pos_y = (
                    -9.480
                )  # if self.isclose(self.grid_size, 0.30, abs_tol=0.01) else -9.500  #-9.65
                stop_pos_y = (
                    -10.800
                )  # if self.isclose(self.grid_size, 0.30, abs_tol=0.01) else -10.700 #-10.700 #-11.00
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

            elif bay == 7:  # Bay 8
                # print("Bay8")
                start_pos_y = (
                    -10.712
                )  # if self.isclose(self.grid_size, 0.30, abs_tol=0.01) else -10.550 #-10.700 #-10.85
                stop_pos_y = (
                    -12.050
                )  # if self.isclose(self.grid_size, 0.30, abs_tol=0.01) else -12.050 #-11.975 #-12.05
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

            elif bay == 8:  # Airlock
                self.calculate_coverage_airlock(
                    bay, feat_only_fileOut, coverage_fileOut
                )

    def remove_tmp_file(self, fileToRemove):
        os.remove(fileToRemove)
        print("Removing temporary file: {:s}".format(fileToRemove))


if __name__ == "__main__":

    try:

        if len(sys.argv) < 1:
            print("Usage: " + sys.argv[0] + "<activity_database_file>")
            print(
                "       <activity_database_file>: 'dir/to/activity_database_file.csv' containing robot poses and ML features to compare with\n"
            )
            # print("       <found_MLPoses_fileOut>: 'dir/to/output_file.csv' containing found nearby /ML features poses near each cube")
            # print("       <coverage_fileOut>: 'dir/to/coverage_fileOut.csv' containing number of ML features and cube pose")
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

        # database_fileOut= sys.argv[2]
        # coverage_fileOut= sys.argv[3]

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
