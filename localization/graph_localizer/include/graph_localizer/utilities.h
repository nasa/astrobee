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

#ifndef GRAPH_LOCALIZER_UTILITIES_H_
#define GRAPH_LOCALIZER_UTILITIES_H_

#include <config_reader/config_reader.h>
#include <graph_localizer/feature_point.h>
#include <graph_localizer/graph_loc_initialization.h>
#include <graph_localizer/imu_measurement.h>

#include <gtsam/geometry/Pose3.h>

#include <Eigen/Core>

#include <deque>
#include <string>
#include <vector>

namespace graph_localizer {
gtsam::Pose3 GtPose(const Eigen::Isometry3d& eigen_pose);

Eigen::Isometry3d LoadTransform(config_reader::ConfigReader& config, const std::string& transform_config_name);

void SetEnvironmentConfigs(const std::string& astrobee_configs_path, const std::string& world);

void EstimateAndSetImuBiases(const ImuMeasurement& imu_measurement, const int num_imu_measurements_per_bias_estimate,
                             std::vector<ImuMeasurement>& imu_bias_measurements,
                             GraphLocInitialization& graph_loc_initialization);

bool ValidPointSet(const std::deque<FeaturePoint>& points, const double min_avg_distance_from_mean);
}  // namespace graph_localizer

#endif  // GRAPH_LOCALIZER_UTILITIES_H_
