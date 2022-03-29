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
import sys
import unittest

import rostest


def dosys(cmd):
    sys.stderr.write(cmd + "\n")
    ret = os.system(cmd)
    if ret != 0:
        sys.stderr.write("command failed with return value %d\n" % ret)
    return ret


class TestFixAll(unittest.TestCase):
    def test_fix_all(self):
        bp_folder = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
        fix_all = os.path.join(bp_folder, "scripts/rosbag_fix_all.py")
        bags_folder = os.path.join(bp_folder, "test/bags")
        bags = os.path.join(bags_folder, "*.bag")

        bag_list = glob.glob(bags)
        self.assertIsNot(len(bag_list), 0)  # sanity check bags path

        fix_all_return_value = dosys("%s %s" % (fix_all, bags))
        self.assertIs(fix_all_return_value, 0)  # check bags fixed
        dosys("rm %s/*.fix_all.bag" % bags_folder)


if __name__ == "__main__":
    rostest.rosrun("bag_processing", "test_fix_all", TestFixAll)
