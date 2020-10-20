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

#include <localization_common/utilities.h>

#include <QHBoxLayout>
#include <QPainter>
#include <QVBoxLayout>

#include <functional>

#include "localization_graph_panel.h"  // NOLINT
#include "utilities.h"                 // NOLINT

namespace {
template <typename Comparator>
// Pass Comparator so green/yellow/red can be sorted in ascending or descending order
void highlightLabel(const double val, const double green_threshold, const double yellow_threshold, QLabel& label,
                    const Comparator comparator = Comparator()) {
  if (comparator(val, green_threshold))
    label.setStyleSheet("QLabel { background-color : green; color : white; }");
  else if (comparator(val, yellow_threshold))
    label.setStyleSheet("QLabel { background-color : yellow; color : black; }");
  else
    label.setStyleSheet("QLabel { background-color : red; color : white; }");
}

void addVectorToLabel(const gtsam::Vector3& vec, const QString& description, QLabel& label, bool add_norm = false,
                      const int precision = 3) {
  QString vec_x_string;
  QString vec_y_string;
  QString vec_z_string;
  vec_x_string.setNum(vec.x(), 'g', precision);
  vec_y_string.setNum(vec.y(), 'g', precision);
  vec_z_string.setNum(vec.z(), 'g', precision);
  QString text = description + ": (" + vec_x_string + ", " + vec_y_string + ", " + vec_z_string + ")";
  if (add_norm) {
    QString vec_norm_string;
    vec_norm_string.setNum(vec.norm(), 'g', precision);
    text += " norm: " + vec_norm_string;
  }
  label.setText(text);
}
}  // namespace

namespace localization_rviz_plugins {
namespace lc = localization_common;

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
  imu_info_layout->addWidget(imu_avg_dt_label_);

  QHBoxLayout* imu_dp_layout = new QHBoxLayout;
  imu_avg_dp_label_ = new QLabel("Avg IMU dp: ");
  imu_dp_layout->addWidget(imu_avg_dp_label_);

  QHBoxLayout* imu_dv_layout = new QHBoxLayout;
  imu_avg_dv_label_ = new QLabel("Avg IMU dv: ");
  imu_dv_layout->addWidget(imu_avg_dv_label_);

  QHBoxLayout* graph_latest_layout = new QHBoxLayout;
  time_since_latest_label_ = new QLabel("Time since latest: ");
  latest_velocity_label_ = new QLabel("Latest Vel: ");
  graph_latest_layout->addWidget(time_since_latest_label_);
  graph_latest_layout->addWidget(latest_velocity_label_);

  QVBoxLayout* layout = new QVBoxLayout;
  layout->addLayout(feature_count_layout);
  layout->addLayout(of_result_layout);
  layout->addLayout(of_info_layout);
  layout->addLayout(imu_info_layout);
  layout->addLayout(imu_dp_layout);
  layout->addLayout(imu_dv_layout);
  layout->addLayout(graph_latest_layout);
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
        const gtsam::Vector3 dv = imu_predicted_combined_nav_state->velocity() - imu_combined_nav_state->velocity();
        total_imu_dv += dv;
      }
    }
  }
  const auto latest_combined_nav_state = graph_localizer.graph_values().LatestCombinedNavState();
  if (latest_combined_nav_state) {
    addVectorToLabel(latest_combined_nav_state->velocity(), "Latest Vel", *latest_velocity_label_, true);

    QString time_since_latest_string;
    const auto current_time = lc::TimeFromRosTime(ros::Time::now());
    const double time_since_latest = current_time - latest_combined_nav_state->timestamp();
    time_since_latest_string.setNum(time_since_latest, 'g', 3);
    highlightLabel<std::less_equal<double>>(time_since_latest, 0.25, 0.35, *time_since_latest_label_);
    time_since_latest_label_->setText("Time since latest: " + time_since_latest_string);
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
    of_valid_percent.setNum(static_cast<double>(100.0 * of_valid) / of_factors, 'g', 3);
    of_valid_label_->setText("OF Valid: " + of_valid_percent + "%");
    const double of_valid_percentage = 100.0 * static_cast<double>(of_valid) / of_factors;
    // Green if >= 50% valid, yellow if < 50% and > 0%, red if 0% valid
    highlightLabel<std::greater_equal<double>>(of_valid_percentage, 50, 0.1, *of_valid_label_);

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
    // Green if <= 0.1, yellow if <= .3 and > .1, red if > 0.3
    highlightLabel<std::less_equal<double>>(imu_avg_dt, 0.1, 0.3, *imu_avg_dt_label_);
    const auto imu_avg_dp = total_imu_dp / imu_factors;
    addVectorToLabel(imu_avg_dp, "Avg IMU dp", *imu_avg_dp_label_, true);

    const auto imu_avg_dv = total_imu_dv / imu_factors;
    addVectorToLabel(imu_avg_dv, "Avg IMU dv", *imu_avg_dv_label_, true);
  }
}
}  // namespace localization_rviz_plugins

#include <pluginlib/class_list_macros.h>  // NOLINT
PLUGINLIB_EXPORT_CLASS(localization_rviz_plugins::LocalizationGraphPanel, rviz::Panel)
