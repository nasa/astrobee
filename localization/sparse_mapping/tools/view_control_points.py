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

import sys, os, re, subprocess

"""Parse a file containing registration points for the ISS, and publish them in RViz.
"""

def gen_marker(marker_id, marker_type, Point, Color, scale, text):
    """Place a marker of a given type and color at a given location"""

    marker = """  - 
    header: 
      seq: 1482
      stamp: 
        secs: 1556650754
        nsecs: 179000000
      frame_id: "world"
    ns: "thick_traj"
    id: %d
    type: %d
    action: 0
    pose: 
      position: 
        x: %g
        y: %g
        z: %g
      orientation: 
        x: 0.0
        y: 0.0
        z: 0.0
        w: 1.0
    scale: 
      x: %g
      y: %g
      z: %g
    color: 
      r: %g
      g: %g
      b: %g
      a: 1.0
    lifetime: 
      secs: 0
      nsecs:         0
    frame_locked: False
    text: "%s"
    mesh_resource: ''
    mesh_use_embedded_materials: False
""" % \
    (marker_id, marker_type, Point[0], Point[1], Point[2], 
     scale, scale, scale, Color[0], Color[1], Color[2], text);
    
    return marker;

def write_marker_file(fileIn, fileOut):

    print("Reading: " + fileIn)
    with open(fileIn) as f:

        f_out = open(fileOut, 'w')
        print("Writing: " + fileOut)
        lines = f.readlines()

        f_out.write("markers: \n")
        
        marker_id = 1
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

            if len(vals) < 4:
                print("Skipping invalid line: '" + line + "'")
                continue
            
            Point = [float(vals[0]), float(vals[1]), float(vals[2])]
            marker_text = vals[3]
            
            # Plot a point as a sphere
            marker_id += 1
            marker_type = 2 # plot a small sphere
            marker_scale = 0.01
            marker_color = [1.0, 0.0, 0.0] # red
            marker = gen_marker(marker_id, marker_type, Point, marker_color,
                                marker_scale, marker_text)
            f_out.write(marker)

            # Plot its text label
            marker_id += 1
            marker_type = 9 # plot a text label
            text_scale   = 0.1
            text_color = [1.0, 1.0, 1.0] # white
            marker = gen_marker(marker_id, marker_type, Point, text_color,
                                text_scale, marker_text)
            f_out.write(marker)

# The main program

if len(sys.argv) < 1:
    print("Usage: " + sys.argv[0] + ' registration.txt')
    sys.exit(1)
    
fileIn  = sys.argv[1]
fileOut = fileIn + ".out"

write_marker_file(fileIn, fileOut)

# Publish
cmd = 'rostopic pub --once /loc/registration visualization_msgs/MarkerArray -f ' + fileOut
print("Running: " + cmd)
subprocess.call(cmd.split(" "))

print("Removing: " + fileOut)
os.remove(fileOut)
