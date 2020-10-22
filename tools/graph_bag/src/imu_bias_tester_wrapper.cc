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

#include <config_reader/config_reader.h>
#include <graph_bag/imu_bias_tester_wrapper.h>
#include <imu_integration/utilities.h>
#include <localization_common/utilities.h>
#include <localization_measurements/imu_measurement.h>

#include <glog/logging.h>

namespace graph_bag {
namespace ii = imu_integration;
namespace lc = localization_common;
namespace lm = localization_measurements;
ImuBiasTesterWrapper::ImuBiasTesterWrapper() {
  config_reader::ConfigReader config;
  config.AddFile("transforms.config");
  config.AddFile("geometry.config");
  config.AddFile("graph_localizer.config");

  if (!config.ReadFiles()) {
    LOG(FATAL) << "Failed to read config files.";
  }

  imu_integration::ImuIntegratorParams params;
  ii::LoadImuIntegratorParams(config, params);
  imu_bias_tester_.reset(new ImuBiasTester(params));
}

std::vector<lc::CombinedNavState> ImuBiasTesterWrapper::LocalizationStateCallback(const ff_msgs::EkfState& loc_msg) {
  const auto combined_nav_state = lc::CombinedNavStateFromMsg(loc_msg);
  return imu_bias_tester_->PimPredict(combined_nav_state);
}

void ImuBiasTesterWrapper::ImuCallback(const sensor_msgs::Imu& imu_msg) {
  imu_bias_tester_->BufferImuMeasurement(lm::ImuMeasurement(imu_msg));
}
}  // namespace graph_bag
