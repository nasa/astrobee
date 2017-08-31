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
#
# for use in gnuplot, with CSV outputs from a rosbag

set term pdf dashed
set output "output.pdf"

set style line 1 lt 1 lc rgb "#FF0000" lw 3
set style line 2 lt 1 lc rgb "#00FF00" lw 3
set style line 3 lt 1 lc rgb "#0000FF" lw 3
set style line 4 lt 1 lc rgb "#FF8040" lw 3
set style line 5 lt 3 lc rgb "#8C001A" lw 3
set style line 6 lt 3 lc rgb "#347235" lw 3
set style line 7 lt 3 lc rgb "#0000A0" lw 3
set style line 8 lt 3 lc rgb "#C36241" lw 3

prefix = "`echo $BAG_PREFIX`"

set xtics autofreq 1e9 format ""
set xlabel "Time (s)"
set yrange [-1.5:1.5]
set title "EKF Position"
plot prefix.'_ekf.dat' using 1:25 with lines ls 1 title 'x', \
     prefix.'_ekf.dat' using 1:26 with lines ls 2 title 'y', \
     prefix.'_ekf.dat' using 1:27 with lines ls 3 title 'z'
set autoscale
set title "EKF Quaternion"
plot prefix.'_ekf.dat' using 1:6 with lines ls 1 title 'qx', \
     prefix.'_ekf.dat' using 1:7 with lines ls 2 title 'qy', \
     prefix.'_ekf.dat' using 1:8 with lines ls 3 title 'qz', \
     prefix.'_ekf.dat' using 1:9 with lines ls 4 title 'qw'
set yrange [-1.5:1.5]
set title "Shaper Position"
plot prefix.'_ctl_shaper.dat' using 1:3 with lines ls 1 title 'x', \
     prefix.'_ctl_shaper.dat' using 1:4 with lines ls 2 title 'y', \
     prefix.'_ctl_shaper.dat' using 1:5 with lines ls 3 title 'z'
set autoscale
set title "Shaper Quaternion"
plot prefix.'_ctl_shaper.dat' using 1:6 with lines ls 1 title 'qx', \
     prefix.'_ctl_shaper.dat' using 1:7 with lines ls 2 title 'qy', \
     prefix.'_ctl_shaper.dat' using 1:8 with lines ls 3 title 'qz', \
     prefix.'_ctl_shaper.dat' using 1:9 with lines ls 4 title 'qw'
set title "Shaper Velocity"
plot prefix.'_ctl_shaper.dat' using 1:10 with lines ls 1 title 'x', \
     prefix.'_ctl_shaper.dat' using 1:11 with lines ls 2 title 'y', \
     prefix.'_ctl_shaper.dat' using 1:12 with lines ls 3 title 'z'
set title "Shaper Angular Velocity"
plot prefix.'_ctl_shaper.dat' using 1:13 with lines ls 1 title 'x', \
     prefix.'_ctl_shaper.dat' using 1:14 with lines ls 2 title 'y', \
     prefix.'_ctl_shaper.dat' using 1:15 with lines ls 3 title 'z'
set title "Shaper Acceleration"
plot prefix.'_ctl_shaper.dat' using 1:16 with lines ls 1 title 'x', \
     prefix.'_ctl_shaper.dat' using 1:17 with lines ls 2 title 'y', \
     prefix.'_ctl_shaper.dat' using 1:18 with lines ls 3 title 'z'
set title "Shaper Angular Acceleration"
plot prefix.'_ctl_shaper.dat' using 1:19 with lines ls 1 title 'x', \
     prefix.'_ctl_shaper.dat' using 1:20 with lines ls 2 title 'y', \
     prefix.'_ctl_shaper.dat' using 1:21 with lines ls 3 title 'z'
set title "Trajectory Position"
plot prefix.'_ctl_traj.dat' using 1:3 with lines ls 1 title 'x', \
     prefix.'_ctl_traj.dat' using 1:4 with lines ls 2 title 'y', \
     prefix.'_ctl_traj.dat' using 1:5 with lines ls 3 title 'z'
set title "Trajectory Quaternion"
plot prefix.'_ctl_traj.dat' using 1:6 with lines ls 1 title 'qx', \
     prefix.'_ctl_traj.dat' using 1:7 with lines ls 2 title 'qy', \
     prefix.'_ctl_traj.dat' using 1:8 with lines ls 3 title 'qz', \
     prefix.'_ctl_traj.dat' using 1:9 with lines ls 4 title 'qw'
set title "Control Force"
plot prefix.'_ctl.dat' using 1:5 with lines ls 1 title 'x', \
     prefix.'_ctl.dat' using 1:6 with lines ls 2 title 'y', \
     prefix.'_ctl.dat' using 1:7 with lines ls 3 title 'z'
set title "Control Rotation"
plot prefix.'_ctl.dat' using 1:8 with lines ls 1 title 'qx', \
     prefix.'_ctl.dat' using 1:9 with lines ls 2 title 'qy', \
     prefix.'_ctl.dat' using 1:10 with lines ls 3 title 'qz'
set title "EKF Omega"
plot prefix.'_ekf.dat' using 1:10 with lines ls 1 title 'omega1', \
     prefix.'_ekf.dat' using 1:11 with lines ls 2 title 'omega2', \
     prefix.'_ekf.dat' using 1:12 with lines ls 3 title 'omega3'
set title "EKF Accel Bias"
plot prefix.'_ekf.dat' using 1:22 with lines ls 1 title 'x', \
     prefix.'_ekf.dat' using 1:23 with lines ls 2 title 'y', \
     prefix.'_ekf.dat' using 1:24 with lines ls 3 title 'z', \
     prefix.'_ground_truth.dat' using 1:22 with lines ls 5 title 'True x', \
     prefix.'_ground_truth.dat' using 1:23 with lines ls 6 title 'True y', \
     prefix.'_ground_truth.dat' using 1:24 with lines ls 7 title 'True z'
set yrange [0.0:0.00005]
set title "EKF Covariances"
plot prefix.'_ekf.dat' using 1:37 with lines ls 1 title 'AB Cov x', \
     prefix.'_ekf.dat' using 1:38 with lines ls 2 title 'AB Cov y', \
     prefix.'_ekf.dat' using 1:39 with lines ls 3 title 'AB Cov z', \
     prefix.'_ekf.dat' using 1:34 with lines ls 5 title 'Vel Cov x', \
     prefix.'_ekf.dat' using 1:35 with lines ls 6 title 'Vel Cov y', \
     prefix.'_ekf.dat' using 1:36 with lines ls 7 title 'Vel Cov z'
set autoscale
set title "EKF Velocity"
plot prefix.'_ekf.dat' using 1:16 with lines ls 1 title 'Vel x', \
     prefix.'_ekf.dat' using 1:17 with lines ls 2 title 'Vel y', \
     prefix.'_ekf.dat' using 1:18 with lines ls 3 title 'Vel z', \
     prefix.'_ground_truth.dat' using 1:16 with lines ls 5 title 'True x', \
     prefix.'_ground_truth.dat' using 1:17 with lines ls 6 title 'True y', \
     prefix.'_ground_truth.dat' using 1:18 with lines ls 7 title 'True z'
set title "IMU Acceleration"
plot prefix.'_imu_data.dat' using 1:($30 + 0.1075) with lines ls 1 title 'x', \
     prefix.'_imu_data.dat' using 1:($31 - 0.09415) with lines ls 2 title 'y', \
     prefix.'_imu_data.dat' using 1:($32 + 9.8241) with lines ls 3 title 'z', \
     prefix.'_ground_truth.dat' using 1:19 with lines ls 5 title 'True x', \
     prefix.'_ground_truth.dat' using 1:20 with lines ls 6 title 'True y', \
     prefix.'_ground_truth.dat' using 1:21 with lines ls 7 title 'True z'
set title "All Positions"
set yrange [-1.5:1.5]
plot prefix.'_ekf.dat' using 1:25 with lines ls 1 title 'EKF x', \
     prefix.'_ekf.dat' using 1:26 with lines ls 2 title 'EKF y', \
     prefix.'_ekf.dat' using 1:27 with lines ls 3 title 'EKF z', \
     prefix.'_ctl_shaper.dat' using 1:3 with lines ls 5 title 'Shaper x', \
     prefix.'_ctl_shaper.dat' using 1:4 with lines ls 6 title 'Shaper y', \
     prefix.'_ctl_shaper.dat' using 1:5 with lines ls 7 title 'Shaper z', \
     prefix.'_ctl_traj.dat' using 1:3 with lines title 'Traj x', \
     prefix.'_ctl_traj.dat' using 1:4 with lines title 'Traj y', \
     prefix.'_ctl_traj.dat' using 1:5 with lines title 'Traj z'
set autoscale
set title "All Quaternions"
plot prefix.'_ekf.dat' using 1:6 with lines ls 1 title 'EKF qx', \
     prefix.'_ekf.dat' using 1:7 with lines ls 2 title 'EKF qy', \
     prefix.'_ekf.dat' using 1:8 with lines ls 3 title 'EKF qz', \
     prefix.'_ekf.dat' using 1:9 with lines ls 4 title 'EKF qw', \
     prefix.'_ctl_shaper.dat' using 1:6 with lines ls 5 title 'Shaper qx', \
     prefix.'_ctl_shaper.dat' using 1:7 with lines ls 6 title 'Shaper qy', \
     prefix.'_ctl_shaper.dat' using 1:8 with lines ls 7 title 'Shaper qz', \
     prefix.'_ctl_shaper.dat' using 1:9 with lines ls 8 title 'Shaper qw', \
     prefix.'_ctl_traj.dat' using 1:6 with lines title 'qx', \
     prefix.'_ctl_traj.dat' using 1:7 with lines title 'qy', \
     prefix.'_ctl_traj.dat' using 1:8 with lines title 'qz', \
     prefix.'_ctl_traj.dat' using 1:9 with lines title 'qw'
set title "Truth Position"
plot prefix.'_ekf.dat' using 1:25 with lines ls 1 title 'EKF x', \
     prefix.'_ekf.dat' using 1:26 with lines ls 2 title 'EKF y', \
     prefix.'_ekf.dat' using 1:27 with lines ls 3 title 'EKF z', \
     prefix.'_ground_truth.dat' using 1:25 with lines ls 5 title 'True x', \
     prefix.'_ground_truth.dat' using 1:26 with lines ls 6 title 'True y', \
     prefix.'_ground_truth.dat' using 1:27 with lines ls 7 title 'True z'
set title "Truth Quaternion"
plot prefix.'_ekf.dat' using 1:6 with lines ls 1 title 'EKF qx', \
     prefix.'_ekf.dat' using 1:7 with lines ls 2 title 'EKF qy', \
     prefix.'_ekf.dat' using 1:8 with lines ls 3 title 'EKF qz', \
     prefix.'_ekf.dat' using 1:9 with lines ls 4 title 'EKF qw', \
     prefix.'_ground_truth.dat' using 1:6 with lines ls 1 title 'True qx', \
     prefix.'_ground_truth.dat' using 1:7 with lines ls 2 title 'True qy', \
     prefix.'_ground_truth.dat' using 1:8 with lines ls 3 title 'True qz', \
     prefix.'_ground_truth.dat' using 1:9 with lines ls 4 title 'True qw'
set title "EKF Covariance"
plot prefix.'_ekf.dat' using 1:40 with lines ls 1 title 'x', \
     prefix.'_ekf.dat' using 1:41 with lines ls 2 title 'y', \
     prefix.'_ekf.dat' using 1:42 with lines ls 3 title 'z', \
     prefix.'_localization_registration.dat' using 1:3 title 'registration'
set title "VPP State"
plot prefix.'_vpp_state.dat' using 1:5 with lines ls 1 title 'VPP 1', \
     prefix.'_vpp_state.dat' using 1:6 with lines ls 2 title 'VPP 2', \
     prefix.'_vpp_state.dat' using 1:7 with lines ls 3 title 'VPP 3', \
     prefix.'_vpp_state.dat' using 1:8 with lines ls 4 title 'VPP 4', \
     prefix.'_vpp_state.dat' using 1:9 with lines ls 5 title 'VPP 5', \
     prefix.'_vpp_state.dat' using 1:10 with lines ls 6 title 'VPP 6'
set title "VPP Pitch"
plot prefix.'_vpp_state.dat' using 1:17 with lines ls 1 title 'VPP 1', \
     prefix.'_vpp_state.dat' using 1:18 with lines ls 2 title 'VPP 2', \
     prefix.'_vpp_state.dat' using 1:19 with lines ls 3 title 'VPP 3', \
     prefix.'_vpp_state.dat' using 1:20 with lines ls 4 title 'VPP 4', \
     prefix.'_vpp_state.dat' using 1:21 with lines ls 5 title 'VPP 5', \
     prefix.'_vpp_state.dat' using 1:22 with lines ls 6 title 'VPP 6'
