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

#include <ff_util/ff_names.h>
#include <graph_bag/imu_bias_tester_adder.h>
#include <graph_bag/utilities.h>
#include <imu_bias_tester/imu_bias_tester_wrapper.h>

namespace graph_bag {
ImuBiasTesterAdder::ImuBiasTesterAdder(const std::string& input_bag_name, const std::string& output_bag_name)
    : input_bag_(input_bag_name, rosbag::bagmode::Read), output_bag_(output_bag_name, rosbag::bagmode::Write) {}

void ImuBiasTesterAdder::AddPredictions() {
  rosbag::View view(input_bag_);
  for (const rosbag::MessageInstance msg : view) {
    if (string_ends_with(msg.getTopic(), TOPIC_GRAPH_LOC_STATE)) {
      const ff_msgs::GraphState::ConstPtr localization_msg = msg.instantiate<ff_msgs::GraphState>();
      const auto imu_bias_tester_predicted_states =
        imu_bias_tester_wrapper_.LocalizationStateCallback(*localization_msg);
      SaveImuBiasTesterPredictedStates(imu_bias_tester_predicted_states, output_bag_);
    } else if (string_ends_with(msg.getTopic(), TOPIC_HARDWARE_IMU)) {
      sensor_msgs::ImuConstPtr imu_msg = msg.instantiate<sensor_msgs::Imu>();
      imu_bias_tester_wrapper_.ImuCallback(*imu_msg);
    }
    output_bag_.write(msg.getTopic(), msg.getTime(), msg);
  }
}
}  // namespace graph_bag
