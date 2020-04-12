#!/bin/bash

bag_file=$1

slots="top_left top_right bottom_left bottom_right"

echo "Battery IDs from bag ${bag_file%*.bag}"

for s in $slots
do
  echo "In SLOT $s"
  t="/hw/eps/battery/${s}/state"
  rostopic echo $t -b $bag_file | grep serial_number | uniq
done
