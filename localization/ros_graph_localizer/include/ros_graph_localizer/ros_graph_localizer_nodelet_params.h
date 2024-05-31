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
#ifndef ROS_GRAPH_LOCALIZER_ROS_GRAPH_LOCALIZER_NODELET_PARAMS_H_
#define ROS_GRAPH_LOCALIZER_ROS_GRAPH_LOCALIZER_NODELET_PARAMS_H_

namespace ros_graph_localizer {
struct RosGraphLocalizerNodeletParams {
  int max_graph_vio_state_buffer_size;
  int max_vl_matched_projections_buffer_size;
  int max_ar_tag_matched_projections_buffer_size;
  int max_imu_buffer_size;
  int max_feature_point_buffer_size;
  int max_depth_odom_buffer_size;
  int max_depth_image_buffer_size;
  int max_depth_cloud_buffer_size;
  // Run depth odometry in nodelet
  bool run_depth_odometry;
  // Receive depth odometry messages from external nodelet
  bool subscribe_to_depth_odometry;
  // Publish depth odometry message if running in nodelet
  bool publish_depth_odometry;
};
}  // namespace ros_graph_localizer

#endif  // ROS_GRAPH_LOCALIZER_ROS_GRAPH_LOCALIZER_NODELET_PARAMS_H_
