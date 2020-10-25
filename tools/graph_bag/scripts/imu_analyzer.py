#!/usr/bin/python
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

import imu_measurements
import plot_helpers
import utilities

import matplotlib
matplotlib.use('pdf')
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages

import rosbag
import geometry_msgs


def plot_imu_measurements(pdf, imu_measurements):
  # Acceleration
  plt.figure()
  plot_helpers.plot_vector3ds(imu_measurements.accelerations, imu_measurements.times, 'Acc.')
  plt.xlabel('Time (s)')
  plt.ylabel('Acceleration (m/s^2)')
  plt.title('Acceleration')
  plt.legend(prop={'size': 6})
  pdf.savefig()
  plt.close()

  # Angular Velocity
  plt.figure()
  plot_helpers.plot_vector3ds(imu_measurements.angular_velocities,
               imu_measurements.times, 'Ang. Vel.')
  plt.xlabel('Time (s)')
  plt.ylabel('Angular Velocities')
  plt.title('Angular Velocities')
  plt.legend(prop={'size': 6})
  pdf.savefig()
  plt.close()


def load_imu_msgs(imu_measurements, imu_topic, bag):
  for topic, msg, t in bag.read_messages(imu_topic):
    imu_measurements.add_imu_measurement(msg)


def create_plots(bagfile, output_file):
  bag = rosbag.Bag(bagfile)
  measurements = imu_measurements.ImuMeasurements()
  load_imu_msgs(measurements, '/hw/imu', bag)
  bag.close()
  # filtered_imu_measurements = ...
  # imu_fft = ....

  with PdfPages(output_file) as pdf:
    plot_imu_measurements(pdf, measurements)
