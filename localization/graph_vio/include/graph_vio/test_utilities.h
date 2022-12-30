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

#ifndef GRAPH_VIO_TEST_UTILITIES_H_
#define GRAPH_VIO_TEST_UTILITIES_H_

#include <graph_optimizer/graph_optimizer_params.h>
#include <graph_vio/combined_nav_state_graph_values_params.h>
#include <graph_vio/depth_odometry_factor_adder_params.h>
#include <graph_vio/graph_initializer_params.h>
#include <graph_vio/graph_vio_params.h>
#include <localization_measurements/depth_odometry_measurement.h>
#include <localization_measurements/plane.h>

namespace graph_vio {
localization_measurements::Plane RandomPlane();

localization_measurements::DepthOdometryMeasurement DepthOdometryMeasurementFromPose(
  const Eigen::Isometry3d& pose, const localization_common::Time source_time,
  const localization_common::Time target_time);

CombinedNavStateGraphValuesParams DefaultCombinedNavStateGraphValuesParams();

graph_optimizer::GraphOptimizerParams DefaultGraphOptimizerParams();

CombinedNavStateNodeUpdaterParams DefaultCombinedNavStateNodeUpdaterParams();

GraphInitializerParams DefaultGraphInitializerParams();

GraphVIOParams DefaultGraphVIOParams();

DepthOdometryFactorAdderParams DefaultDepthOdometryFactorAdderParams();
}  // namespace graph_vio
#endif  // GRAPH_VIO_TEST_UTILITIES_H_
