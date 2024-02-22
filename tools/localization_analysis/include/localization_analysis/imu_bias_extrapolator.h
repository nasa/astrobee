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

#ifndef LOCALIZATION_ANALYSIS_IMU_BIAS_EXTRAPOLATOR_H_
#define LOCALIZATION_ANALYSIS_IMU_BIAS_EXTRAPOLATOR_H_

#include <ff_msgs/GraphVIOState.h>
#include <localization_common/timestamped_set.h>
#include <imu_integration/imu_integrator.h>

#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <string>
#include <vector>

namespace localization_analysis {
class ImuBiasExtrapolator {
 public:
  ImuBiasExtrapolator(const std::string& input_bag_name, const std::string& output_bag_name);
  bool Initialized();
  void Initialize(const localization_common::CombinedNavState& combined_nav_state);
  std::vector<localization_common::CombinedNavState> VIOStateCallback(
    const ff_msgs::GraphVIOState& graph_vio_state_msg);
  void AddExtrapolatedStates();

 private:
  std::unique_ptr<imu_integration::ImuIntegrator> imu_integrator_;
  localization_common::TimestampedSet<localization_common::CombinedNavState> combined_nav_states_;
  localization_common::CombinedNavState latest_extrapolated_state_;
  bool initialized_;
  rosbag::Bag input_bag_;
  rosbag::Bag output_bag_;
};
}  // end namespace localization_analysis

#endif  // LOCALIZATION_ANALYSIS_IMU_BIAS_EXTRAPOLATOR_H_
