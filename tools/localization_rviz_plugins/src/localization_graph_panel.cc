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

#include <QHBoxLayout>
#include <QPainter>
#include <QVBoxLayout>

#include "localization_graph_panel.h"  // NOLINT
#include "utilities.h"                 // NOLINT

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

  QHBoxLayout* of_info_layout = new QHBoxLayout;
  of_avg_num_measurements_label_ = new QLabel("OF Avg # Measurements: ");
  of_info_layout->addWidget(of_avg_num_measurements_label_);

  QHBoxLayout* imu_info_layout = new QHBoxLayout;
  imu_avg_dt_label_ = new QLabel("Avg IMU dt: ");
  imu_avg_dp_norm_label_ = new QLabel("Avg IMU dp norm: ");
  imu_avg_dv_norm_label_ = new QLabel("Avg IMU dv norm: ");
  imu_info_layout->addWidget(imu_avg_dt_label_);
  imu_info_layout->addWidget(imu_avg_dp_norm_label_);
  imu_info_layout->addWidget(imu_avg_dv_norm_label_);

  QHBoxLayout* imu_vector_layout = new QHBoxLayout;
  imu_avg_dp_label_ = new QLabel("Avg IMU dp: ");
  imu_avg_dv_label_ = new QLabel("Avg IMU dv: ");
  imu_vector_layout->addWidget(imu_avg_dp_label_);
  imu_vector_layout->addWidget(imu_avg_dv_label_);

  QVBoxLayout* layout = new QVBoxLayout;
  layout->addLayout(feature_count_layout);
  layout->addLayout(of_result_layout);
  layout->addLayout(of_info_layout);
  layout->addLayout(imu_info_layout);
  layout->addLayout(imu_vector_layout);
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
  int of_total_num_measurements = 0;
  double total_imu_dt = 0;
  gtsam::Vector3 total_imu_dp = gtsam::Vector3::Zero();
  gtsam::Vector3 total_imu_dv = gtsam::Vector3::Zero();
  double total_imu_dp_norm = 0;
  double total_imu_dv_norm = 0;
  for (const auto factor : graph_localizer.factor_graph()) {
    const auto smart_factor = dynamic_cast<const SmartFactor*>(factor.get());
    if (smart_factor) {
      ++of_factors;
      of_total_num_measurements += smart_factor->measured().size();
      if (smart_factor->isValid()) ++of_valid;
      if (smart_factor->isDegenerate()) ++of_degenerate;
      if (smart_factor->isPointBehindCamera()) ++of_behind_camera;
      if (smart_factor->isOutlier()) ++of_outlier;
      if (smart_factor->isFarPoint()) ++of_far_point;
    }
    const auto imu_factor = dynamic_cast<gtsam::CombinedImuFactor*>(factor.get());
    if (imu_factor) {
      ++imu_factors;
      total_imu_dt += imu_factor->preintegratedMeasurements().deltaTij();
      const auto imu_combined_nav_state = firstCombinedNavState(graph_localizer, imu_factor);
      const auto imu_predicted_combined_nav_state = pimPredict(graph_localizer, imu_factor);
      if (!imu_predicted_combined_nav_state || !imu_combined_nav_state) {
        LOG(ERROR) << "LocalizationGraphCallback: Failed to get imu nav state and pim predicted nav state.";
      } else {
        const gtsam::Vector3 dp =
            imu_predicted_combined_nav_state->pose().translation() - imu_combined_nav_state->pose().translation();
        total_imu_dp += dp;
        total_imu_dp_norm += dp.norm();
        const gtsam::Vector3 dv = imu_predicted_combined_nav_state->velocity() - imu_combined_nav_state->velocity();
        total_imu_dv += dv;
        total_imu_dv_norm += dv.norm();
      }
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
    of_valid_percent.setNum(static_cast<double>(100.0 * of_valid) / of_factors, 'g', 4);
    of_valid_label_->setText("OF Valid: " + of_valid_percent + "%");
    // Green if >= 50% valid, yellow if < 50% and > 0%, red if 0% valid
    const double of_valid_percentage = 100.0 * static_cast<double>(of_valid) / of_factors;
    if (of_valid_percentage == 0)
      of_valid_label_->setStyleSheet("QLabel { background-color : red; color : white; }");
    else if (of_valid_percentage < 50)
      of_valid_label_->setStyleSheet("QLabel { background-color : yellow; color : black; }");
    else
      of_valid_label_->setStyleSheet("QLabel { background-color : green; color : white; }");

    QString of_degenerate_percent;
    of_degenerate_percent.setNum(static_cast<double>(100.0 * of_degenerate) / of_factors, 'g', 2);
    of_degenerate_label_->setText("OF Degenerate: " + of_degenerate_percent + "%");

    QString of_behind_camera_percent;
    of_behind_camera_percent.setNum(static_cast<double>(100.0 * of_behind_camera) / of_factors, 'g', 2);
    of_behind_camera_label_->setText("OF Behind Camera: " + of_behind_camera_percent + "%");

    QString of_outlier_percent;
    of_outlier_percent.setNum(static_cast<double>(100.0 * of_outlier) / of_factors, 'g', 2);
    of_outlier_label_->setText("OF Outlier: " + of_outlier_percent + "%");

    QString of_far_point_percent;
    of_far_point_percent.setNum(static_cast<double>(100.0 * of_far_point) / of_factors, 'g', 2);
    of_far_point_label_->setText("OF Far Point: " + of_far_point_percent + "%");

    QString of_average_num_measurements;
    of_average_num_measurements.setNum(static_cast<double>(of_total_num_measurements) / of_factors, 'g', 3);
    of_avg_num_measurements_label_->setText("OF Avg # Measurements: " + of_average_num_measurements);
  }

  // IMU status
  if (imu_factors > 0) {
    const double imu_avg_dt = total_imu_dt / imu_factors;
    QString imu_avg_dt_string;
    imu_avg_dt_string.setNum(imu_avg_dt, 'g', 3);
    imu_avg_dt_label_->setText("Avg IMU dt: " + imu_avg_dt_string);
    // Green if <= 0.1, yellow if < .3 and > .1, red if >= 0.3
    if (imu_avg_dt <= 0.1)
      imu_avg_dt_label_->setStyleSheet("QLabel { background-color : green; color : white; }");
    else if (imu_avg_dt < 0.3)
      imu_avg_dt_label_->setStyleSheet("QLabel { background-color : yellow; color : black; }");
    else
      imu_avg_dt_label_->setStyleSheet("QLabel { background-color : red; color : white; }");

    const auto imu_avg_dp = total_imu_dp / imu_factors;
    // TODO(rsoussan): make function to do this, pass prefix string
    QString imu_avg_dp_x_string;
    QString imu_avg_dp_y_string;
    QString imu_avg_dp_z_string;
    imu_avg_dp_x_string.setNum(imu_avg_dp.x(), 'g', 3);
    imu_avg_dp_y_string.setNum(imu_avg_dp.y(), 'g', 3);
    imu_avg_dp_z_string.setNum(imu_avg_dp.z(), 'g', 3);
    imu_avg_dp_label_->setText("Avg IMU dp: (" + imu_avg_dp_x_string + ", " + imu_avg_dp_y_string + ", " +
                               imu_avg_dp_z_string + ")");

    const auto imu_avg_dv = total_imu_dv / imu_factors;
    QString imu_avg_dv_x_string;
    QString imu_avg_dv_y_string;
    QString imu_avg_dv_z_string;
    // TODO(rsoussan): make function to do this, pass prefix string
    imu_avg_dv_x_string.setNum(imu_avg_dv.x(), 'g', 3);
    imu_avg_dv_y_string.setNum(imu_avg_dv.y(), 'g', 3);
    imu_avg_dv_z_string.setNum(imu_avg_dv.z(), 'g', 3);
    imu_avg_dv_label_->setText("Avg IMU dv: (" + imu_avg_dv_x_string + ", " + imu_avg_dv_y_string + ", " +
                               imu_avg_dv_z_string + ")");

    const auto imu_avg_dp_norm = total_imu_dp_norm / imu_factors;
    QString imu_avg_dp_norm_string;
    imu_avg_dp_norm_string.setNum(imu_avg_dp_norm, 'g', 3);
    imu_avg_dp_norm_label_->setText("Avg IMU dp norm: " + imu_avg_dp_norm_string);

    const auto imu_avg_dv_norm = total_imu_dv_norm / imu_factors;
    QString imu_avg_dv_norm_string;
    imu_avg_dv_norm_string.setNum(imu_avg_dv_norm, 'g', 3);
    imu_avg_dv_norm_label_->setText("Avg IMU dv norm: " + imu_avg_dv_norm_string);
  }
}
}  // namespace localization_rviz_plugins

#include <pluginlib/class_list_macros.h>  // NOLINT
PLUGINLIB_EXPORT_CLASS(localization_rviz_plugins::LocalizationGraphPanel, rviz::Panel)
