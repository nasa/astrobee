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
import scipy.signal

import rosbag
import geometry_msgs


def get_fft(data, times, sample_spacing):
  magnitudes = np.fft.rfft(data)
  frequencies = np.fft.rfftfreq(len(times), sample_spacing)
  return magnitudes, frequencies


def butter_lowpass_sos_representation(cutoff_frequency, sample_rate, order=5):
  nyquist_rate = 0.5 * sample_rate
  # From Python Cookbook
  # TODO(rsoussan): Why is this necessary?
  critical_frequency = cutoff_frequency / nyquist_rate
  sos = scipy.signal.butter(order, critical_frequency, btype='low', output='sos', analog=False)
  return sos


def lowpass_filter(data, cutoff_frequency, sample_rate, order=5):
  sos = butter_lowpass_sos_representation(cutoff_frequency, sample_rate, order=order)
  filtered_data = scipy.signal.sosfilt(sos, data)
  return filtered_data


def filter_imu_measurements(imu_measurements, filtered_imu_measurements, cutoff_frequency, sample_rate):
  filtered_imu_measurements.accelerations.xs = lowpass_filter(imu_measurements.accelerations.xs, cutoff_frequency,
                                                              sample_rate)
  filtered_imu_measurements.accelerations.ys = lowpass_filter(imu_measurements.accelerations.ys, cutoff_frequency,
                                                              sample_rate)
  filtered_imu_measurements.accelerations.zs = lowpass_filter(imu_measurements.accelerations.zs, cutoff_frequency,
                                                              sample_rate)
  filtered_imu_measurements.angular_velocities.xs = lowpass_filter(imu_measurements.angular_velocities.xs,
                                                                   cutoff_frequency, sample_rate)
  filtered_imu_measurements.angular_velocities.ys = lowpass_filter(imu_measurements.angular_velocities.ys,
                                                                   cutoff_frequency, sample_rate)
  filtered_imu_measurements.angular_velocities.zs = lowpass_filter(imu_measurements.angular_velocities.zs,
                                                                   cutoff_frequency, sample_rate)
  filtered_imu_measurements.times = imu_measurements.times


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
  plot_helpers.plot_vector3ds(imu_measurements.angular_velocities, imu_measurements.times, 'Ang. Vel.')
  plt.xlabel('Time (s)')
  plt.ylabel('Angular Velocities')
  plt.title(prefix + 'Angular Velocities')
  plt.legend(prop={'size': 6})
  pdf.savefig()
  plt.close()


def plot_fft(pdf, magnitude, frequency, prefix=''):
  plt.figure()
  plt.ylim(top=100)
  plt.plot(frequency, np.absolute(magnitude), lw=1)
  plt.xlabel('Frequency')
  plt.ylabel('Magnitude')
  plt.title(prefix + ' FFT')
  pdf.savefig()
  plt.close()


def plot_filtered_data(pdf, filtered_data, data, filtered_times, times, title):
  plt.figure()
  plt.plot(filtered_times, filtered_data, 'r', alpha=0.5, lw=1, label='Filtered')
  plt.plot(times, data, 'g', alpha=0.5, lw=1, label='Raw')
  plt.xlabel('Time (s)')
  plt.ylabel('Acceleration (m/s^2)')
  plt.title(title)
  plt.legend(prop={'size': 6})
  pdf.savefig()
  plt.close()


def load_imu_msgs(imu_measurements, imu_topic, bag):
  for topic, msg, t in bag.read_messages(imu_topic):
    imu_measurements.add_imu_measurement(msg)


def create_plots(bagfile, filtered_bagfile, output_file, cutoff_frequency):
  bag = rosbag.Bag(bagfile)
  measurements = imu_measurements.ImuMeasurements()
  load_imu_msgs(measurements, '/hw/imu', bag)
  bag.close()

  # Assumes samples are evenly spaced in time
  # This is a fairly safe assumption for imu measurements
  sample_spacing = np.diff(measurements.times)[0]
  sample_rate = 1.0 / sample_spacing

  # Use second bagfile as filtered measurements, otherwise filter with python filter
  filtered_measurements = imu_measurements.ImuMeasurements()
  if (filtered_bagfile):
    filtered_bag = rosbag.Bag(filtered_bagfile)
    load_imu_msgs(filtered_measurements, '/hw/imu', filtered_bag)
    filtered_bag.close()
  else:
    filter_imu_measurements(measurements, filtered_measurements, cutoff_frequency, sample_rate)

  # FFTs
  acceleration_x_fft_magnitudes, acceleration_x_fft_frequencies = get_fft(measurements.accelerations.xs,
                                                                          measurements.times, sample_spacing)
  acceleration_y_fft_magnitudes, acceleration_y_fft_frequencies = get_fft(measurements.accelerations.ys,
                                                                          measurements.times, sample_spacing)
  acceleration_z_fft_magnitudes, acceleration_z_fft_frequencies = get_fft(measurements.accelerations.zs,
                                                                          measurements.times, sample_spacing)
  angular_velocity_x_fft_magnitudes, angular_velocity_x_fft_frequencies = get_fft(measurements.angular_velocities.xs,
                                                                                  measurements.times, sample_spacing)
  angular_velocity_y_fft_magnitudes, angular_velocity_y_fft_frequencies = get_fft(measurements.angular_velocities.ys,
                                                                                  measurements.times, sample_spacing)
  angular_velocity_z_fft_magnitudes, angular_velocity_z_fft_frequencies = get_fft(measurements.angular_velocities.zs,
                                                                                  measurements.times, sample_spacing)

  filtered_acceleration_x_fft_magnitudes, filtered_acceleration_x_fft_frequencies = get_fft(
    filtered_measurements.accelerations.xs, filtered_measurements.times, sample_spacing)
  filtered_acceleration_y_fft_magnitudes, filtered_acceleration_y_fft_frequencies = get_fft(
    filtered_measurements.accelerations.ys, filtered_measurements.times, sample_spacing)
  filtered_acceleration_z_fft_magnitudes, filtered_acceleration_z_fft_frequencies = get_fft(
    filtered_measurements.accelerations.zs, filtered_measurements.times, sample_spacing)
  filtered_angular_velocity_x_fft_magnitudes, filtered_angular_velocity_x_fft_frequencies = get_fft(
    filtered_measurements.angular_velocities.xs, filtered_measurements.times, sample_spacing)
  filtered_angular_velocity_y_fft_magnitudes, filtered_angular_velocity_y_fft_frequencies = get_fft(
    filtered_measurements.angular_velocities.ys, filtered_measurements.times, sample_spacing)
  filtered_angular_velocity_z_fft_magnitudes, filtered_angular_velocity_z_fft_frequencies = get_fft(
    filtered_measurements.angular_velocities.zs, filtered_measurements.times, sample_spacing)

  with PdfPages(output_file) as pdf:
    plot_imu_measurements(pdf, measurements, 'Raw Imu ')

    plot_fft(pdf, acceleration_x_fft_magnitudes, acceleration_x_fft_frequencies, 'Raw Imu FFT Accel x ')
    plot_fft(pdf, acceleration_y_fft_magnitudes, acceleration_y_fft_frequencies, 'Raw Imu FFT Accel y ')
    plot_fft(pdf, acceleration_z_fft_magnitudes, acceleration_z_fft_frequencies, 'Raw Imu FFT Accel z ')
    plot_fft(pdf, angular_velocity_x_fft_magnitudes, angular_velocity_x_fft_frequencies, 'Raw Imu FFT Ang Vel x ')
    plot_fft(pdf, angular_velocity_y_fft_magnitudes, angular_velocity_y_fft_frequencies, 'Raw Imu FFT Ang Vel y ')
    plot_fft(pdf, angular_velocity_z_fft_magnitudes, angular_velocity_z_fft_frequencies, 'Raw Imu FFT Ang Vel z ')

    plot_filtered_data(pdf, filtered_measurements.accelerations.xs, measurements.accelerations.xs,
                       filtered_measurements.times, measurements.times, 'Filtered Accel x')
    plot_filtered_data(pdf, filtered_measurements.accelerations.ys, measurements.accelerations.ys,
                       filtered_measurements.times, measurements.times, 'Filtered Accel y')
    plot_filtered_data(pdf, filtered_measurements.accelerations.zs, measurements.accelerations.zs,
                       filtered_measurements.times, measurements.times, 'Filtered Accel z')
    plot_filtered_data(pdf, filtered_measurements.angular_velocities.xs, measurements.angular_velocities.xs,
                       filtered_measurements.times, measurements.times, 'Filtered Ang Vel x')
    plot_filtered_data(pdf, filtered_measurements.angular_velocities.ys, measurements.angular_velocities.ys,
                       filtered_measurements.times, measurements.times, 'Filtered Ang Vel y')
    plot_filtered_data(pdf, filtered_measurements.angular_velocities.zs, measurements.angular_velocities.zs,
                       filtered_measurements.times, measurements.times, 'Filtered Ang Vel z')

    plot_fft(pdf, filtered_acceleration_x_fft_magnitudes, filtered_acceleration_x_fft_frequencies,
             'Filtered Imu FFT Accel x ')
    plot_fft(pdf, filtered_acceleration_y_fft_magnitudes, filtered_acceleration_y_fft_frequencies,
             'Filtered Imu FFT Accel y ')
    plot_fft(pdf, filtered_acceleration_z_fft_magnitudes, filtered_acceleration_z_fft_frequencies,
             'Filtered Imu FFT Accel z ')
    plot_fft(pdf, filtered_angular_velocity_x_fft_magnitudes, filtered_angular_velocity_x_fft_frequencies,
             'Filtered Imu FFT Ang Vel x ')
    plot_fft(pdf, filtered_angular_velocity_y_fft_magnitudes, filtered_angular_velocity_y_fft_frequencies,
             'Filtered Imu FFT Ang Vel y ')
    plot_fft(pdf, filtered_angular_velocity_z_fft_magnitudes, filtered_angular_velocity_z_fft_frequencies,
             'Filtered Imu FFT Ang Vel z ')
