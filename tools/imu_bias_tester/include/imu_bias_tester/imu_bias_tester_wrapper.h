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
#ifndef IMU_BIAS_TESTER_IMU_BIAS_TESTER_WRAPPER_H_
#define IMU_BIAS_TESTER_IMU_BIAS_TESTER_WRAPPER_H_

#include <ff_msgs/EkfState.h>
#include <imu_bias_tester/imu_bias_tester.h>
#include <localization_common/combined_nav_state.h>

#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Imu.h>

#include <memory>
#include <string>
#include <vector>

namespace imu_bias_tester {
class ImuBiasTesterWrapper {
 public:
  explicit ImuBiasTesterWrapper(const std::string& graph_config_path_prefix = "");
  // Returns predicted nav states
  std::vector<localization_common::CombinedNavState> LocalizationStateCallback(const ff_msgs::EkfState& loc_msg);
  void ImuCallback(const sensor_msgs::Imu& imu_msg);

 private:
  std::unique_ptr<ImuBiasTester> imu_bias_tester_;
};
}  // namespace imu_bias_tester
#endif  // IMU_BIAS_TESTER_IMU_BIAS_TESTER_WRAPPER_H_
