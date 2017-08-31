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
for i in /hw/imu /gnc/ekf /gnc/ctl/traj /loc/truth /gnc/control/status /gnc/control/feedback /tf /tf_static
do
  echo "Extracting $i."
  suffix=`echo $i | sed 's/\//_/g'`
  rostopic echo -b $1 -p $i > ${output_prefix}${suffix}.csv
  sed -i '/no field named/d' ${output_prefix}${suffix}.csv
done

# Zip all our CSV files together so we can give them to GNC
zip ${output_prefix}_CSV.zip ${output_prefix}_*.csv

# Convert all the CSV to data files for plotting
for i in *.csv; do
  dat_name=`basename $i .csv`
  mv $i ${dat_name}.dat
  sed -i -e 's/,/ /g' ${dat_name}.dat
done

BAG_PREFIX=$output_prefix gnuplot `dirname $0`/plot_astrobee_run.plot
rm *.dat
echo "Output CSV zip file and pdf file."
