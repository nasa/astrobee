#!/usr/bin/python3

#
# Simplify a cmake generated dependency graph.
#
# Cmake has the capability to generate executable/librairies dependency graph.
# However, the generated graphs are unreadable because every library file is
# represented as a unique node. This script groups the libraries files by
# package name.
# The 'groups' variable list the patterns used to re-group the various nodes
# from a same package into a single node. The groups list is specifically
# crafted for ARS: it mostly address the ROS and Gazebo libs, plus some
# key other dependencies.
#
# Usage:
#   1. Generate the dependency graphs with something like
#      cd $BUILD_PATH
#      cmake --graphviz=deps/ars .
#   2. Simplify the desired graphs with something like
#      cd deps
#      $SOURCE_PATH/doc/scripts/simplify_cmake_depsgraph.py ars.executive \
#        > executive.dot
#      dot -Teps executive.dot -o executive.eps
#


import re
import sys

groups = [
    ("nodeROS", "ROS Libraries", "/opt/ros/.+"),
    ("nodeGazebo", "gazebo", "/usr/lib/.+/libgazebo[_a-z0-9]*.so"),
    ("nodeBoost", "boost", "/usr/lib/.+/libboost.+\.so"),
    ("nodeOpenCV", "Open CV", "opencv_[_a-z0-9]+"),
    ("nodeLua", "lua", "/usr/lib/.+/liblua.+\.so"),
    ("nodeTinyXML", "tinyxml", "/usr/lib/.+/libtinyxml.*\.so"),
    ("nodeGflags", "gflags", "/usr/lib/.+/libgflag.*\.so"),
    ("nodeGlog", "glog", "/usr/lib/.+/libglog.*\.so"),
    ("nodeLinux", "Linux System Libraries", "/usr/lib/.*.so")
    #   Not sure if libPocoFoundation should be listed individually
    #    ("nodeLinux", "Linux System Libraries", "/usr/lib/libPoco.*.so"),
    #    ("nodeLinux", "Linux System Libraries", "/usr/lib/.+linux-gnu/.+\.so")
]
nodes = list()


def process_dot(file):
    global nodes
    lines = file.readlines()

    # Identify groups of libraries
    # outer loop is groups: this way the order of the group list is respected
    # and it allows to glob larger pattern after more specific patterns
    # have already been processed
    for g in groups:
        for i, l in enumerate(lines):
            pattern = re.compile('\s"(node[0-9]+)"\s\[\slabel="(' + g[2] + ')"\s.+')
            result = re.search(pattern, l)
            if result:
                lines.pop(i)
                lines.insert(i, l.replace(result.group(2), g[1]))
                nodes.append((result.group(1), g[0]))

    # Replace nodes with common group node name
    for n in nodes:
        lines = [l.replace(n[0], n[1]) for l in lines]

    # Add strict to avoid multiple edges
    lines[0] = "strict " + lines[0]

    # Output the new file
    for l in lines:
        print(l, end=" ")


if len(sys.argv) < 2:
    print("provide input file as first arg")
    exit

f = open(sys.argv[1], "r")

process_dot(f)
# print(nodes)
