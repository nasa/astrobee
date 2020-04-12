#!/bin/bash

#
# Generates a set of dependency graphs
#
# Inputs:
#   BUILD_PATH variable pointing the the Linux build directory,
#              otherwise, the default build path will be used.
#

scriptdir=`dirname $0`

graph_names="ff_nodelet gnc_autocode pmc_actuator_nodelet sparse_mapping \
                executive dds_ros_bridge gazebo_sensor_plugin_perch_cam"

BUILD_PATH=${BUILD_PATH:-${HOME}/freeflyer_build/native}

if [ ! -f $BUILD_PATH/CMakeCache.txt ]
then
  echo "BUILD_PATH=$BUILD_PATH is not a valid build directory!"
  exit 1
fi

srcpath=`cd $scriptdir/../..; pwd`
if [ ! -f $srcpath/freeflyer.doxyfile ]
then
  echo "Something is broken ($srcpath is not valid)!"
  exit 1
fi

pushd $BUILD_PATH
cmake --graphviz=deps/ars .
popd

for g in $graph_names
do
  $scriptdir/simplify_cmake_depsgraph.py $BUILD_PATH/deps/ars.$g \
    > $srcpath/doc/diagrams/$g.dot
done

