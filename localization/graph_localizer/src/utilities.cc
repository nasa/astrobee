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

#include <graph_localizer/utilities.h>

#include <glog/logging.h>

#include <cstdlib>
#include <string>

namespace graph_localizer {
gtsam::Pose3 GtPose(const Eigen::Isometry3d& eigen_pose) {
  return gtsam::Pose3(gtsam::Rot3(eigen_pose.linear().matrix()), eigen_pose.translation());
}

Eigen::Isometry3d LoadTransform(config_reader::ConfigReader& config, const std::string& transform_config_name) {
  Eigen::Vector3d body_t_sensor;
  Eigen::Quaterniond body_Q_sensor;
  if (!msg_conversions::config_read_transform(&config, transform_config_name.c_str(), &body_t_sensor, &body_Q_sensor))
    LOG(FATAL) << "Unspecified transform config: " << transform_config_name;
  Eigen::Isometry3d body_T_sensor = Eigen::Isometry3d::Identity();
  body_T_sensor.translation() = body_t_sensor;
  body_T_sensor.linear() = body_Q_sensor.toRotationMatrix();
  return body_T_sensor;
}

void SetEnvironmentConfigs(const std::string& astrobee_configs_path, const std::string& world) {
  setenv("ASTROBEE_RESOURCE_DIR", (astrobee_configs_path + "/resources").c_str(), true);
  setenv("ASTROBEE_CONFIG_DIR", (astrobee_configs_path + "/config").c_str(), true);
  // TODO(rsoussan): pass this as an argument
  setenv("ASTROBEE_ROBOT", (astrobee_configs_path + "/config/robots/bumble.config").c_str(), true);
  setenv("ASTROBEE_WORLD", world.c_str(), true);
}

bool ValidPointSet(const std::deque<FeaturePoint>& points, const double min_avg_distance_from_mean) {
  if (points.size() < 2) return false;

  // Calculate mean point and avg distance from mean
  Eigen::Vector2d sum_of_points = Eigen::Vector2d::Zero();
  for (const auto& point : points) {
    sum_of_points += point.image_point;
  }
  const Eigen::Vector2d mean_point = sum_of_points / points.size();

  double sum_of_distances_from_mean = 0;
  for (const auto& point : points) {
    const Eigen::Vector2d mean_centered_point = point.image_point - mean_point;
    sum_of_distances_from_mean += mean_centered_point.norm();
  }
  const double average_distance_from_mean = sum_of_distances_from_mean / points.size();
  return (average_distance_from_mean >= min_avg_distance_from_mean);
}
}  // namespace graph_localizer
