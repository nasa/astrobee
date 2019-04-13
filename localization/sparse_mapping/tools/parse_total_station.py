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

import os, sys, re

"""Parse a measurements file as output by the Total Station, and
convert it into a format to be read by build_map when doing
registration.
"""

if len(sys.argv) < 2:
    print("Usage: " + sys.argv[0] + ' input.txt output.txt')
    sys.exit(1)

fileIn  = sys.argv[1]
fileOut = sys.argv[2]

with open(fileIn) as f:

    f_out = open(fileOut, 'w')
    lines = f.readlines()

    f_out.write("# x y z id\n")
    for line in lines:
        # Match: *110001+0000000000111111 81...0+0000000000000942 82...0+0000000000001960 83...0-0000000000000312
        m = re.match("^\*.*?\+0*(.*?)\s+8\d\.\.\.0([\+\-].*?)\s+8\d\.\.\.0([\+\-].*?)\s+8\d\.\.\.0([\+\-].*?)\s+", line)
        if not m:
            continue

        # Strip extra zeros and convert from mm to meters
        pt_id = m.group(1)
        x = int(m.group(2))/1000.0
        y = int(m.group(3))/1000.0
        z = int(m.group(4))/1000.0

        # The id field is just a comment now, it is not needed
        f_out.write(str(x) + " " + str(y) + " " + str(z) + " # " + pt_id + "\n")
        



