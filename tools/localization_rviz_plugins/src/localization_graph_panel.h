/* Copyright (c) 2017, United States Government, as represented by the
 * Administrator of the National Aeronautics and Space Administration.
 *
 * All rights reserved.
 *
 * The Astrobee platform is licensed under the Apache License, Version 2.0
 * (the "License"); you may not use this file except in compliance with the
 * License. You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations
 * under the License.
 */

#ifndef LOCALIZATION_RVIZ_PLUGINS_LOCALIZATION_GRAPH_PANEL_H  // NOLINT
#define LOCALIZATION_RVIZ_PLUGINS_LOCALIZATION_GRAPH_PANEL_H  // NOLINT

#ifndef Q_MOC_RUN
#include <ff_msgs/LocalizationGraph.h>
#include <ff_util/ff_names.h>
#include <graph_localizer/graph_localizer.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <ros/ros.h>
#include <rviz/panel.h>
#include <QLabel>
#endif

class QLineEdit;

namespace localization_rviz_plugins {

class LocalizationGraphPanel : public rviz::Panel {
  Q_OBJECT
 public:  // NOLINT
  explicit LocalizationGraphPanel(QWidget* parent = 0);
 public Q_SLOTS:     // NOLINT
 protected Q_SLOTS:  // NOLINT
 protected:          // NOLINT
  void LocalizationGraphCallback(const ff_msgs::LocalizationGraph::ConstPtr& loc_msg);
  // Factor Counts
  QLabel* of_count_label_;
  QLabel* imu_count_label_;
  // OF status
  QLabel* of_valid_label_;
  QLabel* of_degenerate_label_;
  QLabel* of_behind_camera_label_;
  QLabel* of_outlier_label_;
  QLabel* of_far_point_label_;
  // OF info
  QLabel* of_avg_num_measurements_label_;
  // IMU info
  QLabel* imu_avg_dt_label_;
  QLabel* imu_avg_dp_label_;
  QLabel* imu_avg_dv_label_;
  QLabel* imu_avg_dp_norm_label_;
  QLabel* imu_avg_dv_norm_label_;
  // Graph Latest
  QLabel* latest_velocity_norm_label_;
  QLabel* time_since_latest_label_;

  ros::NodeHandle nh_;
  ros::Subscriber graph_sub_;
};
}  // namespace localization_rviz_plugins

#endif  // LOCALIZATION_RVIZ_PLUGINS_LOCALIZATION_GRAPH_PANEL_H NOLINT
