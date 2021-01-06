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

import environment

import csv
import math
import os
import sys
import time

import matplotlib
matplotlib.use('pdf')
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages

import numpy as np
from numpy.linalg import norm

def run_ekf(astrobee_map, astrobee_bag, output_file, ekf_in_bag = False, features_in_bag=False, image_topic=None, gnc_config='gnc.config'):
  cmd = 'bag_to_csv' if ekf_in_bag else 'bag_to_csv --run_ekf %s' % ('' if features_in_bag else '--gen_features')

  robot_bias = os.path.dirname(os.path.abspath(astrobee_bag)) + '/imu_bias.config'
  cmd = 'rosrun ekf_bag %s %s %s %s %s' % (cmd, astrobee_map, astrobee_bag, output_file, gnc_config)
  print('Not running EKF.' if ekf_in_bag else 'Running EKF.')
  if not ekf_in_bag:
      print( 'Using features from bag.' if features_in_bag else 'Generating features from image.')
  if os.path.isfile(robot_bias):
      cmd += ' ' + robot_bias
  if image_topic is not None:
      cmd += ' --image_topic ' + image_topic      
  os.system(cmd)

def quat_to_euler(q):
  q2q2 = q[1] * q[1]
  x = math.atan2(2 * (q[0] * q[3] + q[1] * q[2]), 1 - 2 * (q[0] * q[0] + q2q2))
  arg = max(-1.0, min(1.0, 2 * (q[1] * q[3] - q[0] * q[2])))
  y = math.asin(arg)
  z = math.atan2(2 * (q[0] * q[1] + q[2] * q[3]), 1 - 2 * (q2q2 + q[2] * q[2]))
  return (x, y, z)

# RMSE of matrix interpreted as n (dim(rows) vectors of dim(col) 
def rmse_matrix(error_matrix):
  return np.sqrt(np.mean(np.sum(np.square(error_matrix), axis=1)))

# Removes the ith entry of a_xs, a_ys, and a_zs if a_times[i] is 
# not in b_times[i].  Assumes timestamp vectors are sorted
def prune_missing_timestamps(a_xs, a_ys, a_zs, a_times, b_times):
  a_index = 0
  pruned_a_matrix = np.empty(shape=(len(b_times), 3)) 
  for b_index, b_time in enumerate(b_times):
    while not np.isclose(a_times[a_index], b_time, rtol=0) and a_times[a_index] < b_time and a_index < len(a_times):
      a_index += 1 
    if not np.isclose(a_times[a_index], b_time, rtol=0, atol=0.02):
      print('Failed to find a time close to b time.')
    pruned_a_matrix[b_index] = np.array([a_xs[a_index], a_ys[a_index], a_zs[a_index]])
  return pruned_a_matrix

# RMSE between two sequences of timestamped positions. Prunes timestamped positions in sequence a
# not present in sequence b. 
def rmse_timestamped_sequences(a_xs, a_ys, a_zs, a_times, b_xs, b_ys, b_zs, b_times):
  a_positions = prune_missing_timestamps(a_xs, a_ys, a_zs, a_times, b_times)
  b_positions = np.column_stack((b_xs, b_ys, b_zs))
  return rmse_matrix(a_positions - b_positions)

#TODO: This does some sort of l2 on stddevs, is this what we want?
def covariance_map(ekf, cov_1_index, cov_2_index, cov_3_index):
  return map(lambda (x, y, z) : math.sqrt(x + y + z),
                            zip(ekf[cov_1_index], ekf[cov_2_index], ekf[cov_3_index]))


class EkfLog(object):
  def __init__(self, filename, start_time=float('-inf'), end_time=float('inf')):
    self.ekf = {'t': [], 'x': [], 'y': [], 'z': [], 'angle1': [], 'angle2': [], 'angle3': [], \
            'vx': [], 'vy': [], 'vz': [], 'ox': [], 'oy': [], 'oz': [], 'ax': [], 'ay': [], 'az': [], \
            'abx': [], 'aby': [], 'abz': [], 'gbx': [], 'gby': [], 'gbz': [], 'c': [], 's': [], \
            'ml_count': [], 'of_count': [], 'mahal' : []}
    for i in range(1, 16):
      self.ekf['cov_' + str(i)] = []
    
    self.mahal = {'times': [], 'boxes': []}
    self.CAMERA_RES = (1280, 960)
    self.HEATMAP_SIZE = (self.CAMERA_RES[0] / 32, self.CAMERA_RES[1] / 32)
    self.vl_heatmap = np.zeros((self.HEATMAP_SIZE[0] + 1, self.HEATMAP_SIZE[1] + 1))
    self.of_heatmap = np.zeros((self.HEATMAP_SIZE[0] + 1, self.HEATMAP_SIZE[1] + 1))
    
    self.vl = {'t': [], 'count': [], 'x': [], 'y': [], 'z': [], 'angle1': [], 'angle2': [], 'angle3': []}
    self.of = {'t': [], 'count': [], 'oldest': [], 'youngest' : [], 'median' : []}
    self.gt = {'t': [], 'x': [], 'y': [], 'z': [], 'angle1': [], 'angle2': [], 'angle3': []}
    
    f = open(filename, 'r')
    for l in f:
      p = l.split(' ')
      if l.startswith('EKF '):
        t = float(p[1])
        if t < start_time or t > end_time:
            continue
        self.ekf['t']        .append(float(p[ 1]))
        self.ekf['x']        .append(float(p[ 2]))
        self.ekf['y']        .append(float(p[ 3]))
        self.ekf['z']        .append(float(p[ 4]))
        self.ekf['angle1']   .append(float(p[ 5]) * 180 / math.pi)
        self.ekf['angle2']   .append(float(p[ 6]) * 180 / math.pi)
        self.ekf['angle3']   .append(float(p[ 7]) * 180 / math.pi)
        self.ekf['vx']       .append(float(p[ 8]))
        self.ekf['vy']       .append(float(p[ 9]))
        self.ekf['vz']       .append(float(p[10]))
        self.ekf['ox']       .append(float(p[11]) * 180 / math.pi)
        self.ekf['oy']       .append(float(p[12]) * 180 / math.pi)
        self.ekf['oz']       .append(float(p[13]) * 180 / math.pi)
        self.ekf['ax']       .append(float(p[14]))
        self.ekf['ay']       .append(float(p[15]))
        self.ekf['az']       .append(float(p[16]))
        self.ekf['abx']      .append(float(p[17]))
        self.ekf['aby']      .append(float(p[18]))
        self.ekf['abz']      .append(float(p[19]))
        self.ekf['gbx']      .append(float(p[20]) * 180 / math.pi)
        self.ekf['gby']      .append(float(p[21]) * 180 / math.pi)
        self.ekf['gbz']      .append(float(p[22]) * 180 / math.pi)
        self.ekf['c']        .append(int(  p[23]))
        self.ekf['s']        .append(int(  p[24]))
        self.ekf['ml_count'] .append(int(  p[25]))
        if self.ekf['ml_count'][-1] == 0:
            self.ekf['ml_count'][-1] = float('nan')
        self.ekf['of_count'] .append(int(  p[26]))
        if self.ekf['of_count'][-1] == 0:
            self.ekf['of_count'][-1] = float('nan')
        for i in range(1, 16):
          self.ekf['cov_' + str(i)].append(math.sqrt(abs(float(p[26 + i]))))
        # convert quaternion to euler angles
        # TODO: shouldn't be using this for covariances
        q = (0.5 * self.ekf['cov_1'][-1],  0.5 * self.ekf['cov_2'][-1], 0.5 * self.ekf['cov_3'][-1], 1.0)
        euler = quat_to_euler(q)
        self.ekf['cov_1'][-1] = euler[0]
        self.ekf['cov_2'][-1] = euler[1]
        self.ekf['cov_3'][-1] = euler[2]
        m = []
        MAHAL_MAX = 30.0
        for i in range(50):
          t = float(p[42 + i])
          if not math.isnan(t) and t > 0.0:
            m.append(min(t, MAHAL_MAX))
        if len(m) > 0:
          self.mahal['times'].append(float(p[1]))
          self.mahal['boxes'].append(m)
        self.ekf['mahal'].append(m)
      elif l.startswith('OF ') and (len(p) - 3) / 4 >= 1:
        t = float(p[1])
        if t > 0.0:
          self.of['t']     .append(t)
          self.of['count'] .append(int(p[2]))
          times = []
          for i in range((len(p) - 3) / 4):
            of_id = float(p[3 + 4 * i + 0])
            origt = float(p[3 + 4 * i + 1])
            u     = float(p[3 + 4 * i + 2])
            v     = float(p[3 + 4 * i + 3])
            times.append(self.of['t'][-1] - origt)
            self.of_heatmap[int(round(u / self.CAMERA_RES[0] * self.HEATMAP_SIZE[0])) + self.HEATMAP_SIZE[0] / 2,
                            int(round(v / self.CAMERA_RES[1] * self.HEATMAP_SIZE[1])) + self.HEATMAP_SIZE[1] / 2] += 1
          self.of['oldest'].append(np.max(times))
          self.of['youngest'].append(np.min(times))
          self.of['median'].append(np.median(times))
      elif l.startswith('VL ') and (len(p) - 9) / 5 >= 1:
        if float(p[1]) > -1.0:
          self.vl['t']        .append(float(p[ 1]))
          self.vl['count']    .append(int(p[ 2]))
          self.vl['x']        .append(float(p[ 3]))
          self.vl['y']        .append(float(p[ 4]))
          self.vl['z']        .append(float(p[ 5]))
          self.vl['angle1']   .append(float(p[ 6]) * 180 / math.pi)
          self.vl['angle2']   .append(float(p[ 7]) * 180 / math.pi)
          self.vl['angle3']   .append(float(p[ 8]) * 180 / math.pi)
          for i in range((len(p) - 9) / 5):
            u = float(p[9 + 5 * i + 0])
            v = float(p[9 + 5 * i + 1])
            x = float(p[9 + 5 * i + 2])
            y = float(p[9 + 5 * i + 3])
            z = float(p[9 + 5 * i + 4])
            self.vl_heatmap[int(round(u / self.CAMERA_RES[0] * self.HEATMAP_SIZE[0])) + self.HEATMAP_SIZE[0] / 2,
                            int(round(v / self.CAMERA_RES[1] * self.HEATMAP_SIZE[1])) + self.HEATMAP_SIZE[1] / 2] += 1
      elif l.startswith('GT '):
        self.gt['t']        .append(float(p[ 1]))
        self.gt['x']        .append(float(p[ 2]))
        self.gt['y']        .append(float(p[ 3]))
        self.gt['z']        .append(float(p[ 4]))
        self.gt['angle1']   .append(float(p[ 5]) * 180 / math.pi)
        self.gt['angle2']   .append(float(p[ 6]) * 180 / math.pi)
        self.gt['angle3']   .append(float(p[ 7]) * 180 / math.pi)
    for key in self.ekf.keys():
        self.ekf[key] = np.array(self.ekf[key])
    #self.vl_heatmap = self.vl_heatmap / np.amax(self.vl_heatmap)
    #self.of_heatmap = self.of_heatmap / np.amax(self.of_heatmap)

  def correct_ground_truth(self):
    # find the best time offset for ground truth
    best_error = float('inf')
    best_offset = 0
    for i in range(125):
      offset = (i - 62.5) / 62.5
      (pos_err, angle_err) = self.evaluate_error(offset)
      err = 100 * pos_err + angle_err # 1 cm = 1 degree error
      if err < best_error:
          best_error = err
          best_offset = offset
    
    # now actually do the shift
    for i in range(len(self.gt['t'])):
      self.gt['t'][i] += best_offset
    return best_offset

  def evaluate_error(self, time_offset = 0.0):
    pos_err = 0.0
    angle_err = 0.0
    err_count = 0
    ekf_i = 0
    for gt_i in range(len(self.gt['t'])):
        t = self.gt['t'][gt_i] + time_offset
        while ekf_i < len(self.ekf['t']) - 1 and self.ekf['t'][ekf_i] < t:
          ekf_i += 1
        if ekf_i >= len(self.ekf['t']) - 1:
            break
        if ekf_i == 0:
          continue
        # now gt_i falls between ekf_i - 1 and ekf_i, we will interpolate for position
        u = (t - self.ekf['t'][ekf_i - 1]) / (self.ekf['t'][ekf_i] - self.ekf['t'][ekf_i - 1])
        x1 = np.array([self.ekf['x'][ekf_i - 1], self.ekf['y'][ekf_i - 1], self.ekf['z'][ekf_i - 1]])
        x2 = np.array([self.ekf['x'][ekf_i    ], self.ekf['y'][ekf_i    ], self.ekf['z'][ekf_i    ]])
        x = x1 + u * (x2 - x1)
        # but just use the older angle, not worth trouble to interpolate in python
        a = np.array([self.ekf['angle1'][ekf_i - 1], self.ekf['angle2'][ekf_i - 1], self.ekf['angle3'][ekf_i - 1]])
        
        err_count += 1
        pos_err += float(norm(np.array([self.gt['x'][gt_i], self.gt['y'][gt_i], self.gt['z'][gt_i]]) - x)) ** 2
        # again, not best metric, but good enough for this
        angle_err += float(norm(np.array([self.gt['angle1'][gt_i], self.gt['angle2'][gt_i], self.gt['angle3'][gt_i]]) - a)) ** 2
    if err_count == 0:
      return (0.0, 0.0)
    return (math.sqrt(pos_err / err_count), math.sqrt(angle_err / err_count))

  def evaluate_features(self):
    total_time = self.ekf['t'][len(self.ekf['t']) - 1] - self.ekf['t'][0]
    total_count = sum(self.vl['count'])
    max_gap = 0
    for i in range(1, len(self.vl['t'])):
      max_gap = max(max_gap, self.vl['t'][i] - self.vl['t'][i-1])
    return (total_count / total_time, max_gap)

  def rmse_using_single_image_sparse_mapping(self):
    rmse_pos = rmse_timestamped_sequences(self.ekf['x'], self.ekf['y'], self.ekf['z'], self.ekf['t'], self.vl['x'], self.vl['y'], self.vl['z'], self.vl['t'])
    rmse_angle = rmse_timestamped_sequences(self.ekf['angle1'], self.ekf['angle2'], self.ekf['angle3'], self.ekf['t'], self.vl['angle1'], self.vl['angle2'], self.vl['angle3'], self.vl['t'])
    return rmse_pos, rmse_angle

  def get_stats(self, job_id):
    stats = []
    stat_names = []
   
    # job id
    stats.append(job_id)
    stat_names.append('job_id') 

    # OF stats
    of_count_no_nans = [0 if math.isnan(val) else val for val in self.ekf['of_count']]
    avg_num_of_features = np.mean(of_count_no_nans)
    stats.append(avg_num_of_features)
    stat_names.append('avg_num_of_features')

    num_of_features_added = np.sum(of_count_no_nans)
    num_of_features_observed = np.sum(self.of['count'])
    ratio_of_features_added = num_of_features_added/num_of_features_observed if num_of_features_observed > 0 else 0
    stats.append(ratio_of_features_added)
    stat_names.append('ratio_of_features_added')

    # ML stats
    ml_count_no_nans = [0 if math.isnan(val) else val for val in self.ekf['ml_count']]
    avg_num_ml_features = np.mean(ml_count_no_nans)
    stats.append(avg_num_ml_features)
    stat_names.append('avg_num_ml_features')

    num_ml_features_added = np.sum(ml_count_no_nans)
    num_ml_features_observed = np.sum(self.vl['count'])
    ratio_ml_features_added = num_ml_features_added/num_ml_features_observed if num_ml_features_observed > 0 else 0
    stats.append(ratio_ml_features_added)
    stat_names.append('ratio_ml_features_added')

    # Mahal
    avg_mahal_distance = np.mean(np.concatenate(self.mahal['boxes']))if len(self.mahal['boxes']) > 0 else 0

    stats.append(avg_mahal_distance)
    stat_names.append('avg_mahal_distance')

    # Covariances
    position_covariances = covariance_map(self.ekf, 'cov_13', 'cov_14', 'cov_15') 
    avg_position_covariance = np.mean(position_covariances)
    stats.append(avg_position_covariance)
    stat_names.append('avg_position_covariance')

    orientation_covariances = covariance_map(self.ekf, 'cov_1', 'cov_2', 'cov_3') 
    avg_orientation_covariance = np.mean(orientation_covariances)
    stats.append(avg_orientation_covariance)
    stat_names.append('avg_orientation_covariance')

    #Compare with groundtruth (if available)
 #   rmse_pos_vs_groundtruth, rmse_angle_vs_groundtruth = self.evaluate_error()
 #   stats.append(rmse_pos_vs_groundtruth)
 #   stat_names.append('rmse_pos_vs_groundtruth')
 #   stats.append(rmse_angle_vs_groundtruth)
 #   stat_names.append('rmse_orientation_vs_groundtruth')

    # Compare with poses from single-image sparse mapping registations
    rmse_pos_vs_single_image_sparse_mapping, rmse_angle_vs_single_image_sparse_mapping = self.rmse_using_single_image_sparse_mapping()
    stats.append(rmse_pos_vs_single_image_sparse_mapping)
    stat_names.append('rmse_pos_vs_single_image_sparse_mapping')
    stats.append(rmse_angle_vs_single_image_sparse_mapping)
    stat_names.append('rmse_orientation_vs_single_image_sparse_mapping')

    return stats, stat_names

  def write_results_to_csv(self, job_id, results_csv_output_file, bagfile):
    with open(results_csv_output_file, 'w') as results_file:
      writer = csv.writer(results_file)
      stats, stat_names = self.get_stats(job_id)
      bag_name = os.path.splitext(os.path.basename(bagfile))[0]
      # TODO(rsoussan): better way to do this
      stats.append(stats[-2])
      stat_names.append('rmse')
      stats.append(bag_name)
      stat_names.append('Bag')
      writer.writerow(stat_names)
      writer.writerow(stats)

  def plot(self, filename, output_text):
    colors = ['r', 'b', 'g']
    ekf = self.ekf
    gt = self.gt
    vl = self.vl
    of = self.of
    
    with PdfPages(filename) as pdf:
      # positions
      plt.figure()
      max_y = 10
      def clip_list(array):
          return [max(-max_y, min(max_y, a)) for a in array]

      plt.plot(ekf['t'], clip_list(ekf['x']), colors[0], linewidth=0.5, label='EKF Pos. (X)')
      plt.plot(ekf['t'], clip_list(ekf['y']), colors[1], linewidth=0.5, label='EKF Pos. (Y)')
      plt.plot(ekf['t'], clip_list(ekf['z']), colors[2], linewidth=0.5, label='EKF Pos. (Z)')
      plt.fill_between(ekf['t'], clip_list(ekf['x']) - ekf['cov_13'], clip_list(ekf['x']) + ekf['cov_13'], facecolor=colors[0], alpha=0.5)
      plt.fill_between(ekf['t'], clip_list(ekf['y']) - ekf['cov_14'], clip_list(ekf['y']) + ekf['cov_14'], facecolor=colors[1], alpha=0.5)
      plt.fill_between(ekf['t'], clip_list(ekf['z']) - ekf['cov_15'], clip_list(ekf['z']) + ekf['cov_15'], facecolor=colors[2], alpha=0.5)
      plt.autoscale(False)
      plt.plot(gt['t'],   clip_list(gt['x']), color=colors[0], linewidth=0.5, dashes=(1, 1), label ='Ground Truth (X)')
      plt.plot(gt['t'],   clip_list(gt['y']), color=colors[1], linewidth=0.5, dashes=(1, 1))
      plt.plot(gt['t'],   clip_list(gt['z']), color=colors[2], linewidth=0.5, dashes=(1, 1))
      plt.plot(vl['t'],   clip_list(vl['x']), color=colors[0], linestyle='None', marker='o', markersize=2, label='Observation (X)')
      plt.plot(vl['t'],   clip_list(vl['y']), color=colors[1], linestyle='None', marker='o', markersize=2)
      plt.plot(vl['t'],   clip_list(vl['z']), color=colors[2], linestyle='None', marker='o', markersize=2)
      plt.xlabel('Time (s)')
      plt.ylabel('Position (m)')
      plt.title('Position')
      plt.legend(prop={'size':6})
      plt.ylim(-max_y + 0.2, max_y + 0.2)
      pdf.savefig()
      
      # angles
      plt.figure()
      plt.plot(ekf['t'], ekf['angle1'], colors[0], linewidth=0.5, label='EKF')
      plt.plot(ekf['t'], ekf['angle2'], colors[1], linewidth=0.5)
      plt.plot(ekf['t'], ekf['angle3'], colors[2], linewidth=0.5)
      plt.fill_between(ekf['t'], ekf['angle1'] - ekf['cov_1'], ekf['angle1'] + ekf['cov_1'], facecolor=colors[0], alpha=0.5)
      plt.fill_between(ekf['t'], ekf['angle2'] - ekf['cov_2'], ekf['angle2'] + ekf['cov_2'], facecolor=colors[1], alpha=0.5)
      plt.fill_between(ekf['t'], ekf['angle3'] - ekf['cov_3'], ekf['angle3'] + ekf['cov_3'], facecolor=colors[2], alpha=0.5)
      plt.autoscale(False)
      plt.plot(gt['t'],   gt['angle1'], color=colors[0], linewidth=0.25, dashes=(1, 1), label='Grount Truth')
      plt.plot(gt['t'],   gt['angle2'], color=colors[1], linewidth=0.25, dashes=(1, 1))
      plt.plot(gt['t'],   gt['angle3'], color=colors[2], linewidth=0.25, dashes=(1, 1))
      plt.plot(vl['t'],   vl['angle1'], color=colors[0], linestyle='None', marker='o', markersize=2, label='Observation')
      plt.plot(vl['t'],   vl['angle2'], color=colors[1], linestyle='None', marker='o', markersize=2)
      plt.plot(vl['t'],   vl['angle3'], color=colors[2], linestyle='None', marker='o', markersize=2)
      plt.xlabel('Time (s)')
      plt.ylabel('Angle ($^\circ$)')
      plt.title('Orientation')
      plt.legend(prop={'size':6})
      pdf.savefig()
      plt.close()
      
      # feature counts
      plt.figure()
      plt.plot(ekf['t'], ekf['ml_count'], linestyle='None', marker='x', markersize=6, color='#FF0000', label='Integrated VL Features')
      plt.plot(ekf['t'], ekf['of_count'], marker='|', markersize=2, color='#0000FF', label='Integrated OF Features')
      plt.plot(vl['t'], vl['count'], linestyle='None', marker='.', markersize=2, color='#B300FF', label='Observed VL Features (at Reg. Time)')
      plt.plot(of['t'], of['count'], linestyle='None', marker=',', markersize=2, color='#00FFb3', label='Observed OF Features (at Reg. Time)')
      plt.xlabel('Time (s)')
      plt.ylabel('Number of Features')
      plt.title('EKF Features')
      plt.legend(prop={'size':6})
      pdf.savefig()
      plt.close()
    
      # mahalnobis distance
      plt.figure()
      if len(self.mahal['boxes']) > 0:
        boxes = plt.boxplot(self.mahal['boxes'], positions=self.mahal['times'], widths=0.2, manage_xticks=False, patch_artist=True)
        plt.setp(boxes['whiskers'], color='Black', linewidth=0.25)
        plt.setp(boxes['caps'], color='Black', linewidth=0.25)
        plt.setp(boxes['medians'], color='Black', linewidth=0.25)
        plt.setp(boxes['fliers'], color='r', marker='x', markersize=1)
        plt.setp(boxes['boxes'], color='Black', facecolor='SkyBlue', linewidth=0.25)
      plt.title('VL Features Mahalnobis Distances')
      plt.xlabel('Time (s)')
      plt.ylabel('Mahalnobis Distance')
      pdf.savefig()
      plt.close()
    
      # linear velocity and acceleration
      plt.figure()
      plt.plot(ekf['t'], ekf['vx'], color=colors[0], linewidth=0.5, label='Velocity')
      plt.plot(ekf['t'], ekf['vy'], color=colors[1], linewidth=0.5)
      plt.plot(ekf['t'], ekf['vz'], color=colors[2], linewidth=0.5)
      plt.fill_between(ekf['t'], ekf['vx'] - ekf['cov_7'], ekf['vx'] + ekf['cov_7'], facecolor=colors[0], alpha=0.5)
      plt.fill_between(ekf['t'], ekf['vy'] - ekf['cov_8'], ekf['vy'] + ekf['cov_8'], facecolor=colors[1], alpha=0.5)
      plt.fill_between(ekf['t'], ekf['vz'] - ekf['cov_9'], ekf['vz'] + ekf['cov_9'], facecolor=colors[2], alpha=0.5)
      plt.title('Velocity')
      plt.xlabel('Time (s)')
      plt.ylabel('Velocity (m/s)')
      plt.ylim(-0.5, 0.5)
      plt.legend(prop={'size':6})
      pdf.savefig()
      plt.close()

      plt.figure()
      plt.plot(ekf['t'], ekf['ax'], color=colors[0], dashes=(1,1), linewidth=0.5, label='Acceleration')
      plt.plot(ekf['t'], ekf['ay'], color=colors[1], dashes=(1,1), linewidth=0.5)
      plt.plot(ekf['t'], ekf['az'], color=colors[2], dashes=(1,1), linewidth=0.5)
      plt.title('Acceleration')
      plt.xlabel('Time (s)')
      plt.ylabel('Acceleration (m/s$^2$)')
      plt.legend(prop={'size':6})
      pdf.savefig()
      plt.close()
      
      # angle and angular velocity
      plt.figure()
      ax = plt.gca()
      ax.plot(ekf['t'], ekf['angle1'], colors[0], linewidth=0.5, label='Angle')
      ax.plot(ekf['t'], ekf['angle2'], colors[1], linewidth=0.5)
      ax.plot(ekf['t'], ekf['angle3'], colors[2], linewidth=0.5)
      ax2 = ax.twinx()
      ax2.plot(ekf['t'], ekf['ox'], color=colors[0], linewidth=0.5, dashes=(1, 1), label='Angular Velocity')
      ax2.plot(ekf['t'], ekf['oy'], color=colors[1], linewidth=0.5, dashes=(1, 1))
      ax2.plot(ekf['t'], ekf['oz'], color=colors[2], linewidth=0.5, dashes=(1, 1))
      ax.set_title('Angular Velocity')
      ax.set_xlabel('Time (s)')
      ax.set_ylabel('Angle ($^\circ$)')
      ax2.set_ylabel('Angular Velocity ($^\circ$/s)')
      lines, labels = ax.get_legend_handles_labels()
      lines2, labels2 = ax2.get_legend_handles_labels()
      ax.legend(lines + lines2, labels + labels2, prop={'size':6})
      pdf.savefig()
      plt.close()
      
      # bias
      plt.figure()
      ax = plt.gca()
      ax.plot(ekf['t'], ekf['abx'], colors[0], linewidth=0.5, label='Accelerometer Bias')
      ax.plot(ekf['t'], ekf['aby'], colors[1], linewidth=0.5)
      ax.plot(ekf['t'], ekf['abz'], colors[2], linewidth=0.5)
      ax.fill_between(ekf['t'], ekf['abx'] - ekf['cov_10'], ekf['abx'] + ekf['cov_10'], facecolor=colors[0], alpha=0.5)
      ax.fill_between(ekf['t'], ekf['aby'] - ekf['cov_11'], ekf['aby'] + ekf['cov_11'], facecolor=colors[1], alpha=0.5)
      ax.fill_between(ekf['t'], ekf['abz'] - ekf['cov_12'], ekf['abz'] + ekf['cov_12'], facecolor=colors[2], alpha=0.5)
      ax2 = ax.twinx()
      ax2.plot(ekf['t'], ekf['gbx'], color=colors[0], linewidth=0.5, dashes=(1, 1), label='Gyrometer Bias')
      ax2.plot(ekf['t'], ekf['gby'], color=colors[1], linewidth=0.5, dashes=(1, 1))
      ax2.plot(ekf['t'], ekf['gbz'], color=colors[2], linewidth=0.5, dashes=(1, 1))
      ax2.fill_between(ekf['t'], ekf['gbx'] - ekf['cov_4'], ekf['gbx'] + ekf['cov_4'], facecolor=colors[0], alpha=0.5)
      ax2.fill_between(ekf['t'], ekf['gby'] - ekf['cov_5'], ekf['gby'] + ekf['cov_5'], facecolor=colors[1], alpha=0.5)
      ax2.fill_between(ekf['t'], ekf['gbz'] - ekf['cov_6'], ekf['gbz'] + ekf['cov_6'], facecolor=colors[2], alpha=0.5)
      ax.set_title('Bias Terms')
      ax.set_xlabel('Time (s)')
      ax.set_ylabel('Accelerometer Bias (m/s$^2$)')
      ax2.set_ylabel('Gyrometer Bias ($^\circ$/s)')
      lines, labels = ax.get_legend_handles_labels()
      lines2, labels2 = ax2.get_legend_handles_labels()
      ax.legend(lines + lines2, labels + labels2, prop={'size':6})
      pdf.savefig()
      plt.close()

      # covariance
      plt.figure()
      plt.plot(ekf['t'], covariance_map(self.ekf, 'cov_13', 'cov_14', 'cov_15'), colors[0], linewidth=0.5, label='Position Covariance')
      plt.plot(ekf['t'], covariance_map(self.ekf, 'cov_7', 'cov_8', 'cov_9'), colors[1], linewidth=0.5, label='Velocity Covariance')
      plt.plot(ekf['t'], covariance_map(self.ekf, 'cov_1', 'cov_2', 'cov_3'), colors[2], linewidth=0.5, label='Orientation Covariance')
      plt.title('Std. Deviation')
      plt.xlabel('Time (s)')
      plt.ylabel('Covariance')
      plt.legend(prop={'size':6})
      pdf.savefig()
      plt.close()
    
      # mahalnobis distance histogram
      plt.figure()
      plt.hist([item for sublist in self.mahal['boxes'] for item in sublist], bins=200, range=(0, 50), normed=True)
      plt.xlabel('Mahalnobis Distance')
      plt.ylabel('pdf')
      plt.title('Mahalnobis Distance Histogram')
      pdf.savefig()
      plt.close()

      plt.figure()
      plt.imshow(np.transpose(self.vl_heatmap), cmap='hot', interpolation='nearest', vmin=0, vmax=np.amax(self.vl_heatmap))
      plt.title('Visual Landmarks Density')
      pdf.savefig()
      plt.close()

      plt.figure()
      plt.plot(of['t'], of['oldest'],   color=colors[0], linewidth=0.5, label='Oldest')
      plt.plot(of['t'], of['median'],   color=colors[1], linewidth=0.5, label='Median')
      plt.plot(of['t'], of['youngest'], color=colors[2], linewidth=0.5, label='Youngest')
      plt.xlabel('Time (s)')
      plt.ylabel('Optical Flow Feature Age (s)')
      plt.title('Optical Flow Feature Age')
      plt.legend(prop={'size':6})
      pdf.savefig()
      plt.close()
      
      plt.figure()
      plt.imshow(np.transpose(self.of_heatmap), cmap='hot', interpolation='nearest', vmin=0, vmax=np.amax(self.vl_heatmap))
      plt.title('Optical Flow Density')
      pdf.savefig()
      plt.close()

      plt.figure()
      plt.axis('off')
      plt.text(0.0, 0.5, output_text)
      pdf.savefig()
      plt.close()

class RunEKFOptions(object):
  def __init__(self, bag_file, map_file, ekf_output_file, pdf_output_file, results_csv_output_file):
    self.bag_file = bag_file
    self.map_file = map_file
    self.ekf_output_file = ekf_output_file 
    self.pdf_output_file = pdf_output_file 
    self.results_csv_output_file = results_csv_output_file 
    self.image_topic = None
    self.job_id = 0 
    self.gnc_config = 'gnc.config' 
    self.robot_name = None
    self.start_time = float('-inf')
    self.end_time = float('inf')
    self.ekf_in_bag = False
    self.features_in_bag = False
    self.cached = False
    self.save_stats = False
    self.make_plots = True

  def set_bag_sweep_params(self, args):
    self.image_topic = args.image_topic
    self.robot_name = args.robot_name
    self.gnc_config = args.gnc_config
    self.save_stats = args.save_stats
    self.make_plots = args.make_plots


  def set_command_line_params(self, args):
    self.image_topic = args.image_topic
    self.robot_name = args.robot_name
    self.start_time = args.start_time
    self.end_time = args.end_time
    self.ekf_in_bag = args.ekf_in_bag
    self.features_in_bag = args.features_in_bag
    self.cached = args.cached
    self.save_stats = args.save_stats
    self.make_plots = args.make_plots

  def set_param_sweep_params(self, image_topic, job_id, gnc_config, save_stats=True, make_plots=True):
    self.image_topic = image_topic
    self.job_id = job_id
    self.gnc_config = gnc_config
    self.save_stats = save_stats
    self.make_plots = make_plots

def run_ekf_and_save_stats(options):
  (astrobee_map, astrobee_bag, robot_config) = environment.initialize_environment(options.map_file, options.bag_file, options.robot_name)

  print("Running EKF for job_id: " + str(options.job_id))

  start = time.time()
  if not options.cached:
    run_ekf(astrobee_map, astrobee_bag, options.ekf_output_file, options.ekf_in_bag, options.features_in_bag, options.image_topic, options.gnc_config)
  run_time = time.time() - start
  output_text = 'Run time: %g\n' % (run_time)

  print("Finished running EKF for job_id " + str(options.job_id) + " in " + str(run_time) + " seconds")

  log = EkfLog(options.ekf_output_file, options.start_time, options.end_time)

  if options.save_stats:
    log.write_results_to_csv(options.job_id, options.results_csv_output_file, astrobee_bag)
    print("Printed csv reults for job_id: " + str(options.job_id))

  if options.make_plots: 
    offset = log.correct_ground_truth()
    (pos_err, angle_err) = log.evaluate_error()
    (fps, max_gap) = log.evaluate_features()
    output_text += 'Ground Truth Time Shift: %g\n' % (offset)
    output_text += 'RMSE Position Error: %g Angular Error: %g\n' % (pos_err, angle_err)
    output_text += 'VL Feature Throughput: %g VLs / s Max Gap: %g s' % (fps, max_gap)
    print output_text
    log.plot(options.pdf_output_file, output_text)
    print 'Saved results to %s' % (options.pdf_output_file)
