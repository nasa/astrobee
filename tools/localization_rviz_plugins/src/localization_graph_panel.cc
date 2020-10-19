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

#include "localization_graph_panel.h" // NOLINT

#include <QHBoxLayout>
#include <QPainter>
#include <QVBoxLayout>

namespace localization_rviz_plugins {
LocalizationGraphPanel::LocalizationGraphPanel(QWidget* parent) : rviz::Panel(parent) {
  QHBoxLayout* feature_count_layout = new QHBoxLayout;
  of_count_label_ = new QLabel("OF Factors:");
  imu_count_label_ = new QLabel("Imu Factors:");
  // of_count_label_->setAlignment(Qt::AlignLeft);
  feature_count_layout->addWidget(of_count_label_);
  feature_count_layout->addWidget(imu_count_label_);

  QHBoxLayout* of_result_layout = new QHBoxLayout;
  of_valid_label_ = new QLabel("OF Valid:");
  of_degenerate_label_ = new QLabel("OF Degenerate:");
  of_behind_camera_label_ = new QLabel("OF Behind Camera:");
  of_outlier_label_ = new QLabel("OF Outlier:");
  of_far_point_label_ = new QLabel("OF Far Point:");
  of_result_layout->addWidget(of_valid_label_);
  of_result_layout->addWidget(of_degenerate_label_);
  of_result_layout->addWidget(of_behind_camera_label_);
  of_result_layout->addWidget(of_outlier_label_);
  of_result_layout->addWidget(of_far_point_label_);

  QVBoxLayout* layout = new QVBoxLayout;
  layout->addLayout(feature_count_layout);
  layout->addLayout(of_result_layout);
  setLayout(layout);

  graph_sub_ = nh_.subscribe(TOPIC_GRAPH_LOC, 1, &LocalizationGraphPanel::LocalizationGraphCallback, this,
                             ros::TransportHints().tcpNoDelay());
}

void LocalizationGraphPanel::LocalizationGraphCallback(const ff_msgs::LocalizationGraph::ConstPtr& loc_msg) {
  // TODO(rsoussan): put these somewhere else!
  using Calibration = gtsam::Cal3_S2;
  using Camera = gtsam::PinholeCamera<Calibration>;
  using SmartFactor = gtsam::SmartProjectionPoseFactor<Calibration>;

  // TODO(rsoussan): cleaner way to do this, serialize/deserialize properly
  graph_localizer::GraphLocalizerParams params;
  graph_localizer::GraphLocalizer graph_localizer(params);
  gtsam::deserializeBinary(loc_msg->serialized_graph, graph_localizer);
  int of_factors = 0;
  int imu_factors = 0;
  int of_valid = 0;
  int of_degenerate = 0;
  int of_behind_camera = 0;
  int of_outlier = 0;
  int of_far_point = 0;
  for (const auto factor : graph_localizer.factor_graph()) {
    const auto smart_factor = dynamic_cast<const SmartFactor*>(factor.get());
    if (smart_factor) {
      ++of_factors;
      if (smart_factor->isValid()) ++of_valid;
      if (smart_factor->isDegenerate()) ++of_degenerate;
      if (smart_factor->isPointBehindCamera()) ++of_behind_camera;
      if (smart_factor->isOutlier()) ++of_outlier;
      if (smart_factor->isFarPoint()) ++of_far_point;
    }
    const auto imu_factor = dynamic_cast<gtsam::CombinedImuFactor*>(factor.get());
    if (imu_factor) {
      ++imu_factors;
    }
  }

  // Factor Counts
  QString of_count;
  of_count.setNum(of_factors);
  of_count_label_->setText("OF Factors: " + of_count);
  QString imu_count;
  imu_count.setNum(imu_factors);
  imu_count_label_->setText("IMU Factors: " + imu_count);

  // OF status
  if (of_factors > 0) {
    QString of_valid_percent;
    of_valid_percent.setNum(static_cast<double>(100.0 * of_valid) / of_factors);
    of_valid_label_->setText("OF Valid: " + of_valid_percent + "%");
    // Green if >= 50% valid, yellow if < 50% and > 0%, red if none valid
    const double of_valid_percentage = 100.0 * static_cast<double>(of_valid) / of_factors;
    if (of_valid_percentage < 50)
      of_valid_label_->setStyleSheet("QLabel { background-color : yellow; color : black; }");
    else if (of_valid_percentage <= 0)
      of_valid_label_->setStyleSheet("QLabel { background-color : red; color : white; }");
    else
      of_valid_label_->setStyleSheet("QLabel { background-color : green; color : white; }");

    QString of_degenerate_percent;
    of_degenerate_percent.setNum(static_cast<double>(100.0 * of_degenerate) / of_factors);
    of_degenerate_label_->setText("OF Degenerate: " + of_degenerate_percent + "%");

    QString of_behind_camera_percent;
    of_behind_camera_percent.setNum(static_cast<double>(100.0 * of_behind_camera) / of_factors);
    of_behind_camera_label_->setText("OF Behind Camera: " + of_behind_camera_percent + "%");

    QString of_outlier_percent;
    of_outlier_percent.setNum(static_cast<double>(100.0 * of_outlier) / of_factors);
    of_outlier_label_->setText("OF Outlier: " + of_outlier_percent + "%");

    QString of_far_point_percent;
    of_far_point_percent.setNum(static_cast<double>(100.0 * of_far_point) / of_factors);
    of_far_point_label_->setText("OF Far Point: " + of_far_point_percent + "%");
  }
}
}  // namespace localization_rviz_plugins

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(localization_rviz_plugins::LocalizationGraphPanel, rviz::Panel)
