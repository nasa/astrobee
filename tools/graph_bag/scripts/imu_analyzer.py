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
import numpy as np

import rosbag
import geometry_msgs

def get_fft(imu_measurements):
  magnitude = np.fft.rfft(imu_measurements.accelerations.xs)
  freq = np.fft.rfftfreq(len(imu_measurements.times),np.diff(imu_measurements.times)[0])
  return magnitude, freq

def plot_imu_measurements(pdf, imu_measurements, prefix=''):
  # Acceleration
  plt.figure()
  plot_helpers.plot_vector3ds(imu_measurements.accelerations, imu_measurements.times, 'Acc.')
  plt.xlabel('Time (s)')
  plt.ylabel('Acceleration (m/s^2)')
  plt.title(prefix + 'Acceleration')
  plt.legend(prop={'size': 6})
  pdf.savefig()
  plt.close()

  # Angular Velocity
  plt.figure()
  plot_helpers.plot_vector3ds(imu_measurements.angular_velocities,
               imu_measurements.times, 'Ang. Vel.')
  plt.xlabel('Time (s)')
  plt.ylabel('Angular Velocities')
  plt.title(prefix + 'Angular Velocities')
  plt.legend(prop={'size': 6})
  pdf.savefig()
  plt.close()

def plot_fft(pdf, magnitude, frequency, prefix=''):
  plt.figure()
  plt.plot(frequency, np.absolute(magnitude), lw = 1)
  plt.xlabel('Frequency')
  plt.ylabel('Magnitude')
  plt.title(prefix + 'Acceleration FFT')
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

  # TODO(rsoussan): add option to pass x,y,z!
  magnitude, frequency = get_fft(measurements)

  with PdfPages(output_file) as pdf:
    plot_imu_measurements(pdf, measurements, 'Raw Imu ')
    plot_fft(pdf, magnitude, frequency, 'Raw Imu x')
