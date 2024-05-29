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
#include <ros_graph_localizer/parameter_reader.h>

namespace ros_graph_localizer {
namespace mc = msg_conversions;
namespace pr = parameter_reader;

void LoadRosGraphLocalizerWrapperParams(config_reader::ConfigReader& config, RosGraphLocalizerWrapperParams& params,
                                        const std::string& prefix) {
  pr::LoadImuIntegratorParams(config, params.imu_integrator);
  LOAD_PARAM(params.extrapolate_dock_pose_with_imu, config, "rgl_" + prefix);
  LOAD_PARAM(params.max_relative_vio_buffer_size, config, "rgl_" + prefix);
  LOAD_PARAM(params.max_duration_between_vl_msgs, config, "rgl_" + prefix);
}

void LoadRosGraphLocalizerNodeletParams(config_reader::ConfigReader& config, RosGraphLocalizerNodeletParams& params,
                                        const std::string& prefix) {
  LOAD_PARAM(params.max_graph_vio_state_buffer_size, config, "rgl_" + prefix);
  LOAD_PARAM(params.max_vl_matched_projections_buffer_size, config, "rgl_" + prefix);
  LOAD_PARAM(params.max_ar_tag_matched_projections_buffer_size, config, "rgl_" + prefix);
  LOAD_PARAM(params.max_imu_buffer_size, config, prefix + "rgl_");
  LOAD_PARAM(params.max_feature_point_buffer_size, config, prefix + "rgl_");
  LOAD_PARAM(params.max_depth_odom_buffer_size, config, prefix + "rgl_");
  LOAD_PARAM(params.max_depth_image_buffer_size, config, prefix + "rgl_");
  LOAD_PARAM(params.max_depth_cloud_buffer_size, config, prefix + "rgl_");
  LOAD_PARAM(params.run_depth_odometry, config, prefix + "rgl_");
  LOAD_PARAM(params.publish_depth_odometry, config, prefix + "rgl_");
  LOAD_PARAM(params.subscribe_to_depth_odometry, config, prefix + "rgl_");
}
}  // namespace ros_graph_localizer
