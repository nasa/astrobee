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

#include <msg_conversions/msg_conversions.h>
#include <parameter_reader/imu_integration.h>
#include <ros_graph_vio/parameter_reader.h>

namespace ros_graph_vio {
namespace mc = msg_conversions;
namespace pr = parameter_reader;

void LoadImuBiasInitializerParams(config_reader::ConfigReader& config, ImuBiasInitializerParams& params,
                                  const std::string& prefix) {
  pr::LoadImuFilterParams(config, params.filter);
  LOAD_PARAM(params.imu_bias_filename, config, prefix + "ibi_");
  LOAD_PARAM(params.num_bias_estimation_measurements, config, prefix + "ibi_");
}

void LoadRosGraphVIOWrapperParams(config_reader::ConfigReader& config, RosGraphVIOWrapperParams& params,
                                  const std::string& prefix) {
  LOAD_PARAM(params.starting_pose_translation_stddev, config, prefix + "rgv_");
  LOAD_PARAM(params.starting_pose_quaternion_stddev, config, prefix + "rgv_");
  LOAD_PARAM(params.starting_velocity_stddev_scale, config, prefix + "rgv_");
  LOAD_PARAM(params.starting_accel_bias_stddev_scale, config, prefix + "rgv_");
  LOAD_PARAM(params.starting_gyro_bias_stddev_scale, config, prefix + "rgv_");
}
}  // namespace ros_graph_vio
