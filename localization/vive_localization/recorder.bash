#!/bin/bash

if [[ $# -lt 2 ]] 
then
  echo "Usage:"
  echo "  recorder.sh procedure step [run [info]]"
  echo "    procedure:  procedure nickname"
  echo "    step:       step or section to record"
  echo "    run:        run number (default = 1)"
  echo "    info:       description for the manifest"
  exit -1
fi

proc=$1
step=$2

if [[ $# -gt 2 ]]
then
  run=$3
else
  run="1"
fi

if [[ $# -gt 3 ]]
then
  info=$4
else
  info="n/a"
fi

source /res/astrobee.env
if [[ "$ASTROBEE_WORLD" == "granite" ]]
then 
  zone="UTC+8"
else
  zone="UTC"
fi

# Get a date that match where test is performed
today=`TZ=$zone date +%Y-%m-%d`

# Get the robot name
robot=`cat /etc/robotname`

# Make sure we have a place to write
dest=/data/bags/$robot/$today
mkdir -p $dest

# Craft a filename
start=`TZ=$zone date +%Y%m%d_%H%M`
name="${start}_proc-${proc}_step-${step}_run-${run}"
bagfile=$dest/$name.bag
echo $info > $dest/$name.txt

# ROS Core is running on the LLP
export ROS_MASTER_URI=http://llp:11311

echo "Starting to record $bagfile"
rosbag record \
  -e "/gnc/(.*)|/loc/(.*)|/mob/(.*)|/hw/imu|/hw/vive/(.*)" \
  -x "/mob/mapper/(.*)" \
  -O $bagfile
