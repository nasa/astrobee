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

#include <graph_localizer/loc_projection_factor.h>
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
                      const int precision = 2) {
  QString text = description + " ";
  if (add_norm) {
    QString vec_norm_string;
    vec_norm_string.setNum(vec.norm(), 'g', precision);
    text += "norm: " + vec_norm_string + ", ";
  }
  QString vec_x_string;
  QString vec_y_string;
  QString vec_z_string;
  vec_x_string.setNum(vec.x(), 'g', precision);
  vec_y_string.setNum(vec.y(), 'g', precision);
  vec_z_string.setNum(vec.z(), 'g', precision);
  text += "vec (" + vec_x_string + ", " + vec_y_string + ", " + vec_z_string + ")";
  label.setText(text);
}

// TODO(rsoussan): unify this with version from graph utilities
double AverageDistanceFromMean(const gtsam::Point2Vector& points) {
  // Calculate mean point and avg distance from mean
  Eigen::Vector2d sum_of_points = Eigen::Vector2d::Zero();
  for (const auto& point : points) {
    sum_of_points += point;
  }
  const Eigen::Vector2d mean_point = sum_of_points / points.size();

  double sum_of_distances_from_mean = 0;
  for (const auto& point : points) {
    const Eigen::Vector2d mean_centered_point = point - mean_point;
    sum_of_distances_from_mean += mean_centered_point.norm();
  }
  const double average_distance_from_mean = sum_of_distances_from_mean / points.size();
  return average_distance_from_mean;
}
}  // namespace

namespace localization_rviz_plugins {
namespace gl = graph_localizer;
namespace lc = localization_common;

LocalizationGraphPanel::LocalizationGraphPanel(QWidget* parent) : rviz::Panel(parent) {
  QHBoxLayout* feature_count_layout = new QHBoxLayout;
  of_count_label_ = new QLabel("OF Factors:");
  imu_count_label_ = new QLabel("Imu Factors:");
  loc_count_label_ = new QLabel("Loc Factors:");
  feature_count_layout->addWidget(of_count_label_);
  feature_count_layout->addWidget(imu_count_label_);
  feature_count_layout->addWidget(loc_count_label_);

  QHBoxLayout* prior_count_layout = new QHBoxLayout;
  pose_prior_count_label_ = new QLabel("Pose Prior Factors:");
  velocity_prior_count_label_ = new QLabel("Vel Prior Factors:");
  bias_prior_count_label_ = new QLabel("Bias Prior Factors:");
  prior_count_layout->addWidget(pose_prior_count_label_);
  prior_count_layout->addWidget(velocity_prior_count_label_);
  prior_count_layout->addWidget(bias_prior_count_label_);

  QHBoxLayout* of_result_layout = new QHBoxLayout;
  of_valid_label_ = new QLabel("OF Valid:");
  of_degenerate_label_ = new QLabel("OF Degenerate:");
  of_result_layout->addWidget(of_valid_label_);
  of_result_layout->addWidget(of_degenerate_label_);

  QHBoxLayout* of_result2_layout = new QHBoxLayout;
  of_behind_camera_label_ = new QLabel("OF Behind Camera:");
  of_outlier_label_ = new QLabel("OF Outlier:");
  of_far_point_label_ = new QLabel("OF Far Point:");
  of_result2_layout->addWidget(of_behind_camera_label_);
  of_result2_layout->addWidget(of_outlier_label_);
  of_result2_layout->addWidget(of_far_point_label_);

  QHBoxLayout* of_info_layout = new QHBoxLayout;
  of_avg_num_measurements_label_ = new QLabel("OF Avg # Measurements: ");
  of_avg_dist_from_mean_label_ = new QLabel("OF Avg Dist From Mean: ");
  of_info_layout->addWidget(of_avg_num_measurements_label_);
  of_info_layout->addWidget(of_avg_dist_from_mean_label_);

  QHBoxLayout* imu_info_layout = new QHBoxLayout;
  imu_avg_dt_label_ = new QLabel("Avg IMU dt: ");
  imu_info_layout->addWidget(imu_avg_dt_label_);

  QHBoxLayout* imu_dp_dt_layout = new QHBoxLayout;
  imu_avg_dp_dt_label_ = new QLabel("Avg IMU dp/dt: ");
  imu_dp_dt_layout->addWidget(imu_avg_dp_dt_label_);

  QHBoxLayout* imu_dv_dt_layout = new QHBoxLayout;
  imu_avg_dv_dt_label_ = new QLabel("Avg IMU dv/dt: ");
  imu_dv_dt_layout->addWidget(imu_avg_dv_dt_label_);

  QHBoxLayout* graph_latest_layout = new QHBoxLayout;
  time_since_latest_label_ = new QLabel("Time since latest: ");
  graph_latest_layout->addWidget(time_since_latest_label_);

  QHBoxLayout* latest_velocity_layout = new QHBoxLayout;
  latest_velocity_label_ = new QLabel("Latest Vel: ");
  latest_velocity_layout->addWidget(latest_velocity_label_);

  QVBoxLayout* layout = new QVBoxLayout;
  layout->addLayout(feature_count_layout);
  layout->addLayout(prior_count_layout);
  layout->addLayout(of_result_layout);
  layout->addLayout(of_result2_layout);
  layout->addLayout(of_info_layout);
  layout->addLayout(imu_info_layout);
  layout->addLayout(imu_dp_dt_layout);
  layout->addLayout(imu_dv_dt_layout);
  layout->addLayout(graph_latest_layout);
  layout->addLayout(latest_velocity_layout);
  setLayout(layout);

  graph_sub_ = nh_.subscribe(TOPIC_GRAPH_LOC, 1, &LocalizationGraphPanel::LocalizationGraphCallback, this,
                             ros::TransportHints().tcpNoDelay());
}

void LocalizationGraphPanel::LocalizationGraphCallback(const ff_msgs::LocalizationGraph::ConstPtr& loc_msg) {
  // TODO(rsoussan): put these somewhere else!
  using Calibration = gtsam::Cal3_S2;
  using Camera = gtsam::PinholeCamera<Calibration>;
  using SmartFactor = gtsam::SmartProjectionPoseFactor<Calibration>;

  gl::GraphLocalizer graph_localizer;
  gtsam::deserializeBinary(loc_msg->serialized_graph, graph_localizer);
  int of_factors = 0;
  int imu_factors = 0;
  int loc_factors = 0;
  int pose_prior_factors = 0;
  int velocity_prior_factors = 0;
  int bias_prior_factors = 0;
  int of_valid = 0;
  int of_degenerate = 0;
  int of_behind_camera = 0;
  int of_outlier = 0;
  int of_far_point = 0;
  int of_total_num_measurements = 0;
  double of_total_avg_dist_from_mean = 0;
  double total_imu_dt = 0;
  gtsam::Vector3 total_imu_dp_dt = gtsam::Vector3::Zero();
  gtsam::Vector3 total_imu_dv_dt = gtsam::Vector3::Zero();
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
      of_total_avg_dist_from_mean += AverageDistanceFromMean(smart_factor->measured());
    }
    const auto imu_factor = dynamic_cast<gtsam::CombinedImuFactor*>(factor.get());
    if (imu_factor) {
      ++imu_factors;
      const double dt = imu_factor->preintegratedMeasurements().deltaTij();
      total_imu_dt += dt;
      const auto imu_combined_nav_state = firstCombinedNavState(graph_localizer, imu_factor);
      const auto imu_predicted_combined_nav_state = pimPredict(graph_localizer, imu_factor);
      if (!imu_predicted_combined_nav_state || !imu_combined_nav_state) {
        LOG(ERROR) << "LocalizationGraphCallback: Failed to get imu nav state and pim predicted nav state.";
      } else {
        const gtsam::Vector3 dp =
            imu_predicted_combined_nav_state->pose().translation() - imu_combined_nav_state->pose().translation();
        total_imu_dp_dt += dp / dt;
        const gtsam::Vector3 dv = imu_predicted_combined_nav_state->velocity() - imu_combined_nav_state->velocity();
        total_imu_dv_dt += dv / dt;
      }
    }
    const auto loc_factor = dynamic_cast<gtsam::LocProjectionFactor<>*>(factor.get());
    if (loc_factor) {
      ++loc_factors;
    }
    // Prior Factors
    const auto pose_prior_factor = dynamic_cast<gtsam::PriorFactor<gtsam::Pose3>*>(factor.get());
    if (pose_prior_factor) {
      ++pose_prior_factors;
    }
    const auto velocity_prior_factor = dynamic_cast<gtsam::PriorFactor<gtsam::Velocity3>*>(factor.get());
    if (velocity_prior_factor) {
      ++velocity_prior_factors;
    }
    const auto bias_prior_factor = dynamic_cast<gtsam::PriorFactor<gtsam::imuBias::ConstantBias>*>(factor.get());
    if (bias_prior_factor) {
      ++bias_prior_factors;
    }
  }
  const auto latest_combined_nav_state = graph_localizer.graph_values().LatestCombinedNavState();
  if (latest_combined_nav_state) {
    addVectorToLabel(latest_combined_nav_state->velocity(), "Latest Vel", *latest_velocity_label_, true);

    QString time_since_latest_string;
    const auto current_time = lc::TimeFromRosTime(ros::Time::now());
    const double time_since_latest = current_time - latest_combined_nav_state->timestamp();
    time_since_latest_string.setNum(time_since_latest, 'g', 2);
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
  QString loc_count;
  loc_count.setNum(loc_factors);
  loc_count_label_->setText("Loc Factors: " + loc_count);
  QString pose_prior_count;
  pose_prior_count.setNum(pose_prior_factors);
  pose_prior_count_label_->setText("Pose Prior Factors: " + pose_prior_count);
  QString velocity_prior_count;
  velocity_prior_count.setNum(velocity_prior_factors);
  velocity_prior_count_label_->setText("Vel Prior Factors: " + velocity_prior_count);
  QString bias_prior_count;
  bias_prior_count.setNum(bias_prior_factors);
  bias_prior_count_label_->setText("Bias Prior Factors: " + bias_prior_count);

  // OF status
  if (of_factors > 0) {
    QString of_valid_percent;
    of_valid_percent.setNum(static_cast<double>(100.0 * of_valid) / of_factors, 'g', 2);
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
    of_average_num_measurements.setNum(static_cast<double>(of_total_num_measurements) / of_factors, 'g', 2);
    of_avg_num_measurements_label_->setText("OF Avg # Measurements: " + of_average_num_measurements);

    QString of_average_dist_from_mean;
    of_average_dist_from_mean.setNum(static_cast<double>(of_total_avg_dist_from_mean) / of_factors, 'g', 2);
    of_avg_dist_from_mean_label_->setText("OF Avg Dist From Mean: " + of_average_dist_from_mean);
  }

  // IMU status
  if (imu_factors > 0) {
    const double imu_avg_dt = total_imu_dt / imu_factors;
    QString imu_avg_dt_string;
    imu_avg_dt_string.setNum(imu_avg_dt, 'g', 2);
    imu_avg_dt_label_->setText("Avg IMU dt: " + imu_avg_dt_string);
    // Green if <= 0.1, yellow if <= .3 and > .1, red if > 0.3
    highlightLabel<std::less_equal<double>>(imu_avg_dt, 0.1, 0.3, *imu_avg_dt_label_);
    const auto imu_avg_dp_dt = total_imu_dp_dt / imu_factors;
    addVectorToLabel(imu_avg_dp_dt, "Avg IMU dp/dt", *imu_avg_dp_dt_label_, true);

    const auto imu_avg_dv_dt = total_imu_dv_dt / imu_factors;
    addVectorToLabel(imu_avg_dv_dt, "Avg IMU dv/dt", *imu_avg_dv_dt_label_, true);
  }
}
}  // namespace localization_rviz_plugins

#include <pluginlib/class_list_macros.h>  // NOLINT
PLUGINLIB_EXPORT_CLASS(localization_rviz_plugins::LocalizationGraphPanel, rviz::Panel)
