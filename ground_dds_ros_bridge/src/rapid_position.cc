/* Copyright (c) 2017, United State Government, as represented by the
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

#include "ground_dds_ros_bridge/rapid_position.h"

namespace ff {

RapidPositionToRos::RapidPositionToRos(const std::string& subscribe_topic,
                                       const std::string& pub_topic,
                                       const ros::NodeHandle &nh,
                                       const unsigned int queue_size)
  : RapidSubRosPub(subscribe_topic,
                   pub_topic,
                   nh,
                   "RapidPositionToRos",
                   queue_size) {
  // advertise ros topic, make it latched
  pub_ = nh_.advertise<ff_msgs::EkfState>(publish_topic_, queue_size, true);

  // connect to ddsEventLoop
  try {
    dds_event_loop_.connect<rapid::ext::astrobee::EkfState>(this,
            rapid::ext::astrobee::EKF_STATE_TOPIC + subscribe_topic, // topic
            "",                                                      // name
            "AstrobeeEkfStateProfile",                               // profile
            "");                                                     // library
  } catch (std::exception& e) {
    ROS_ERROR_STREAM("RapidPositionToRos exception: " << e.what());
    throw;
  } catch (...) {
    ROS_ERROR("RapidPositionToRos exception unknown");
    throw;
  }

  // start thread
  StartThread();
}

void RapidPositionToRos::CopyTransform3D(geometry_msgs::Pose& pose,
                                         const rapid::Transform3D& transform) {
  pose.position.x = transform.xyz[0];
  pose.position.y = transform.xyz[1];
  pose.position.z = transform.xyz[2];

  pose.orientation.x = transform.rot[0];
  pose.orientation.y = transform.rot[1];
  pose.orientation.z = transform.rot[2];
  pose.orientation.w = transform.rot[3];
}

void RapidPositionToRos::CopyVec3D(geometry_msgs::Vector3& vec_out,
                                   const rapid::Vec3d& vec_in) {
  vec_out.x = vec_in[0];
  vec_out.y = vec_in[1];
  vec_out.z = vec_in[2];
}

void RapidPositionToRos::operator() (rapid::ext::astrobee::EkfState const*
                                                              rapid_ekf_state) {
  ff_msgs::EkfState state;
  util::RapidHeader2Ros(rapid_ekf_state->hdr, &state.header);

  CopyTransform3D(state.pose, rapid_ekf_state->pose);

  CopyVec3D(state.velocity, rapid_ekf_state->velocity);

  CopyVec3D(state.omega, rapid_ekf_state->omega);

  CopyVec3D(state.gyro_bias, rapid_ekf_state->gyro_bias);

  CopyVec3D(state.accel, rapid_ekf_state->accel);

  CopyVec3D(state.accel_bias, rapid_ekf_state->accel_bias);  

  for (int i = 0; i <15; i++) {
    state.cov_diag[i] = rapid_ekf_state->cov_diag[i];
  }

  state.confidence = rapid_ekf_state->confidence;

  state.status = rapid_ekf_state->status;

  state.of_count = rapid_ekf_state->of_count;

  state.ml_count = rapid_ekf_state->ml_count;

  CopyTransform3D(state.hr_global_pose, rapid_ekf_state->hr_global_pose);

  for (int i = 0; i < 50; i++) {
    state.ml_mahal_dists[i] = rapid_ekf_state->ml_mahal_dists[i];
  }

  pub_.publish(state);
}

}  // end namespace ff
