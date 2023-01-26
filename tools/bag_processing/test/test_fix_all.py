#!/usr/bin/env python
# Copyright (c) 2017, United States Government, as represented by the
# Administrator of the National Aeronautics and Space Administration.
#
# All rights reserved.
#
# The Astrobee platform is licensed under the Apache License, Version 2.0
# (the "License"); you may not use this file except in compliance with the
# License. You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
# License for the specific language governing permissions and limitations
# under the License.

import glob
import os
import subprocess
import sys
import unittest

import rostest


def dosys(cmd):
    sys.stderr.write(cmd + "\n")
    ret = os.system(cmd)
    if ret != 0:
        sys.stderr.write("command failed with return value %d\n" % ret)
    return ret


def get_path(ros_package, subpath):
    cmd = ["catkin_find", "--first-only", ros_package, subpath]
    return subprocess.check_output(cmd).decode("utf-8").strip()


class TestFixAll(unittest.TestCase):
    def get_test_bags_folder(self):
        return get_path("bag_processing", "test/bags")

    def test_fix_all(self):
        fix_all = get_path("bag_processing", "scripts/rosbag_fix_all.py")
        bags_folder = self.get_test_bags_folder()
        bags = os.path.join(bags_folder, "*.bag")

        bag_list = glob.glob(bags)
        self.assertIsNot(len(bag_list), 0)  # check test bags exist

        fix_all_return_value = dosys("%s -d %s" % (fix_all, bags))
        self.assertIs(fix_all_return_value, 0)  # check script success

        fixed_bags = os.path.join(bags_folder, "*.fix_all.bag")
        fixed_bag_list = glob.glob(fixed_bags)
        self.assertEqual(len(bag_list), len(fixed_bag_list))  # check # output files

        # cleanup
        dosys("rm %s" % fixed_bags)


if __name__ == "__main__":
    rostest.rosrun("bag_processing", "test_fix_all", TestFixAll)
