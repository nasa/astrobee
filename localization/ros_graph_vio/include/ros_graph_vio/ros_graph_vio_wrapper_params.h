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
#ifndef ROS_GRAPH_VIO_ROS_GRAPH_VIO_WRAPPER_PARAMS_H_
#define ROS_GRAPH_VIO_ROS_GRAPH_VIO_WRAPPER_PARAMS_H_

namespace ros_graph_vio {
struct RosGraphVIOWrapperParams {
  double starting_pose_translation_stddev;
  double starting_pose_quaternion_stddev;
  double starting_velocity_stddev_scale;
  double starting_accel_bias_stddev_scale;
  double starting_gyro_bias_stddev_scale;
};
}  // namespace ros_graph_vio

#endif  // ROS_GRAPH_VIO_ROS_GRAPH_VIO_WRAPPER_PARAMS_H_
