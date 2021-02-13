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
#include <graph_bag/bag_imu_filterer.h>
#include <imu_integration/imu_filter_params.h>
#include <localization_common/utilities.h>
#include <localization_measurements/imu_measurement.h>
#include <msg_conversions/msg_conversions.h>

#include <sensor_msgs/Imu.h>

namespace {
// TODO(rsoussan): Unify this with live measurement simulator, put in utilities
bool string_ends_with(const std::string& str, const std::string& ending) {
  if (str.length() >= ending.length()) {
    return (0 == str.compare(str.length() - ending.length(), ending.length(), ending));
  } else {
    return false;
  }
}
}  // namespace

namespace graph_bag {
namespace ii = imu_integration;
namespace lc = localization_common;
namespace lm = localization_measurements;
namespace mc = msg_conversions;

BagImuFilterer::BagImuFilterer(const std::string& bag_name, const std::string& filtered_bag,
                               const std::string& filter_name)
    : bag_(bag_name, rosbag::bagmode::Read), filtered_bag_(filtered_bag, rosbag::bagmode::Write) {
  ii::ImuFilterParams params;
  // TODO(rsoussan): Allow for different accel and ang vel filters
  params.quiet_accel = filter_name;
  params.quiet_ang_vel = filter_name;
  params.nominal_accel = filter_name;
  params.nominal_ang_vel = filter_name;
  params.fast_accel = filter_name;
  params.fast_ang_vel = filter_name;
  imu_filter_.reset(new ii::ImuFilter(params));
}

void BagImuFilterer::Convert() {
  rosbag::View view(bag_);
  for (const rosbag::MessageInstance m : view) {
    // Convert imu message to filtered imu message
    if (string_ends_with(m.getTopic(), TOPIC_HARDWARE_IMU)) {
      sensor_msgs::ImuConstPtr imu_msg = m.instantiate<sensor_msgs::Imu>();
      const lm::ImuMeasurement imu_measurement(*imu_msg);
      const auto filtered_imu_measurement = imu_filter_->AddMeasurement(imu_measurement);
      if (filtered_imu_measurement) {
        sensor_msgs::Imu filtered_imu_msg;
        mc::VectorToMsg(filtered_imu_measurement->acceleration, filtered_imu_msg.linear_acceleration);
        mc::VectorToMsg(filtered_imu_measurement->angular_velocity, filtered_imu_msg.angular_velocity);
        lc::TimeToHeader(filtered_imu_measurement->timestamp, filtered_imu_msg.header);
        // TODO(rsoussan): Change receive timestamp to account for filter delay?
        filtered_bag_.write(m.getTopic(), m.getTime(), filtered_imu_msg);
      }
    } else {  // Don't change other msgs, write to filtered_bag
      filtered_bag_.write(m.getTopic(), m.getTime(), m);
    }
  }
}
}  // namespace graph_bag
