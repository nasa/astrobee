#!/bin/bash
#
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

# Check to see if there are arguments
if [ $# -eq 0 ]; then
  echo "Please supply the bag file as an argument"
  exit 1
fi

output_prefix=`basename $1 .bag`

# Extract the following topics as CSV files
for i in /hw/imu /gnc/ekf /gnc/ctl/traj /gnc/control/status /gnc/control/feedback /tf /tf_static /mob/flight_mode /loc/truth/pose /gnc/ctl/command
do
  echo "Extracting $i."
  suffix=`echo $i | sed 's/\//_/g'`
  rostopic echo -b $1 -p $i > ${output_prefix}${suffix}.csv
  sed -i '/no field named/d' ${output_prefix}${suffix}.csv
done

# Zip all our CSV files together so we can give them to GNC
#szip ${output_prefix}_CSV.zip ${output_prefix}_*.csv
mkdir ${output_prefix}-d
mv ${output_prefix}_*.csv ${output_prefix}-d/

echo "Output CSV zip file"
