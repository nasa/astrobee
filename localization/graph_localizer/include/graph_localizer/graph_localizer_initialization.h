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
#ifndef GRAPH_LOCALIZER_GRAPH_LOCALIZER_INITIALIZATION_H_
#define GRAPH_LOCALIZER_GRAPH_LOCALIZER_INITIALIZATION_H_

#include <camera/camera_params.h>
#include <config_reader/config_reader.h>
#include <graph_localizer/graph_localizer_params.h>
#include <msg_conversions/msg_conversions.h>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/navigation/ImuBias.h>

#include <string>

namespace graph_localizer {
class GraphLocalizerInitialization {
 public:
  void SetBiases(const gtsam::imuBias::ConstantBias& imu_bias, const bool loaded_from_file = false);
  void SetStartPose(const gtsam::Pose3& global_T_body_start, const double timestamp);
  void RemoveGravityFromBiasIfPossibleAndNecessary();
  bool ReadyToInitialize() const;
  void ResetBiasesAndStartPose();
  void ResetBiasesFromFileAndResetStartPose();
  void ResetStartPose();
  void ResetBiases();
  void ResetBiasesFromFile();
  void StartBiasEstimation();
  bool HasBiases() const;
  bool HasStartPose() const;
  bool HasParams() const;
  bool EstimateBiases() const;
  const GraphLocalizerParams& params() const;
  void LoadGraphLocalizerParams(config_reader::ConfigReader& config);
  bool RemovedGravityFromBiasIfNecessary() const;

 private:
  bool has_biases_ = false;
  bool has_start_pose_ = false;
  bool has_params_ = false;
  bool estimate_biases_ = false;
  bool removed_gravity_from_bias_if_necessary_ = false;
  graph_localizer::GraphLocalizerParams params_;
};
}  // namespace graph_localizer

#endif  // GRAPH_LOCALIZER_GRAPH_LOCALIZER_INITIALIZATION_H_
