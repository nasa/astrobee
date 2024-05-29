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
#ifndef ROS_GRAPH_LOCALIZER_ROS_GRAPH_LOCALIZER_WRAPPER_PARAMS_H_
#define ROS_GRAPH_LOCALIZER_ROS_GRAPH_LOCALIZER_WRAPPER_PARAMS_H_

#include <imu_integration/imu_integrator_params.h>

#include <boost/serialization/serialization.hpp>

namespace ros_graph_localizer {
struct RosGraphLocalizerWrapperParams {
  imu_integration::ImuIntegratorParams imu_integrator;
  bool extrapolate_dock_pose_with_imu;
  int max_relative_vio_buffer_size;
  double max_duration_between_vl_msgs;

 private:
  // Serialization function
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE& ar, const unsigned int /*version*/) {
    ar& BOOST_SERIALIZATION_NVP(imu_integrator);
    ar& BOOST_SERIALIZATION_NVP(extrapolate_dock_pose_with_imu);
    ar& BOOST_SERIALIZATION_NVP(max_relative_vio_buffer_size);
    ar& BOOST_SERIALIZATION_NVP(max_duration_between_vl_msgs);
  }
};
}  // namespace ros_graph_localizer

#endif  // ROS_GRAPH_LOCALIZER_ROS_GRAPH_LOCALIZER_WRAPPER_PARAMS_H_
