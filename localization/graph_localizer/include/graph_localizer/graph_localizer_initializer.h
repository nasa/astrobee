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
#ifndef GRAPH_LOCALIZER_GRAPH_LOCALIZER_INITIALIZER_H_
#define GRAPH_LOCALIZER_GRAPH_LOCALIZER_INITIALIZER_H_

#include <camera/camera_params.h>
#include <config_reader/config_reader.h>
#include <graph_localizer/graph_localizer_params.h>
#include <localization_measurements/timestamped_pose.h>
#include <msg_conversions/msg_conversions.h>

#include <gtsam/geometry/Pose3.h>

#include <string>
#include <vector>

namespace graph_localizer {
class GraphLocalizerInitializer {
 public:
  GraphLocalizerInitializer();
  void SetStartPose(const localization_measurements::TimestampedPose& timestamped_pose);
  bool ReadyToInitialize() const;
  void ResetStartPose();
  bool HasStartPose() const;
  bool HasParams() const;
  const GraphLocalizerParams& params() const;
  void LoadGraphLocalizerParams(config_reader::ConfigReader& config);

 private:
  bool has_start_pose_;
  bool has_params_;
  graph_localizer::GraphLocalizerParams params_;
};
}  // namespace graph_localizer

#endif  // GRAPH_LOCALIZER_GRAPH_LOCALIZER_INITIALIZER_H_
