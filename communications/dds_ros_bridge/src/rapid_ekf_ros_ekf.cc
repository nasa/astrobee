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

#include "dds_ros_bridge/rapid_ekf_ros_ekf.h"

namespace ff {

RapidEkfToRos::RapidEkfToRos(
                                        const std::string& subscribe_topic,
                                        const std::string& subscribe_partition,
                                        const std::string& pub_topic,
                                        const ros::NodeHandle &nh,
                                        const unsigned int queue_size)
  : RapidSubRosPub(subscribe_topic,
                   pub_topic,
                   nh,
                   "AstrobeeEkfStateProfile",
                   queue_size) {
  // advertise ros topic
  pub_ = nh_.advertise<ff_msgs::EkfState>(pub_topic, queue_size);
  subscriber_partition_ = subscribe_partition;

  // connect to ddsEventLoop
  // @todo confirm topic suffix has '-'
  try {
    dds_event_loop_.connect<rapid::ext::astrobee::EkfState>(this,
                                            subscribe_topic,        // topic
                                            subscribe_partition,    // name
                                            "AstrobeeEkfStateProfile",  // profile
                                            "");                    // library
  } catch (std::exception& e) {
    ROS_ERROR_STREAM("RapidEkfRosEkf exception: " << e.what());
    throw;
  } catch (...) {
    ROS_ERROR("RapidEkfRosEkf exception unknown");
    throw;
  }

  // start thread
  StartThread();
}

void RapidEkfToRos::CopyTransform3D(const rapid::Transform3D &transform,
                                           geometry_msgs::Pose& pose) {
  pose.position.x = transform.xyz[0];
  pose.position.y = transform.xyz[1];
  pose.position.z = transform.xyz[2];

  pose.orientation.x = transform.rot[0];
  pose.orientation.y = transform.rot[1];
  pose.orientation.z = transform.rot[2];
  pose.orientation.w = transform.rot[3];
}

void RapidEkfToRos::CopyVec3D(const rapid::Vec3d& vec_in,
                                     geometry_msgs::Vector3& vec_out) {
  vec_out.x = vec_in[0];
  vec_out.y = vec_in[1];
  vec_out.z = vec_in[2];
}

void RapidEkfToRos::operator() (rapid::ext::astrobee::EkfState const* ekf_sample) {
  ff_msgs::EkfState ekf_msg;
  util::RapidHeader2Ros(ekf_sample->hdr, &ekf_msg.header, subscriber_partition_);

  CopyTransform3D(ekf_sample->pose, ekf_msg.pose);

  CopyVec3D(ekf_sample->velocity, ekf_msg.velocity);
  CopyVec3D(ekf_sample->omega, ekf_msg.omega);
  CopyVec3D(ekf_sample->gyro_bias, ekf_msg.gyro_bias);
  CopyVec3D(ekf_sample->accel, ekf_msg.accel);
  CopyVec3D(ekf_sample->accel_bias, ekf_msg.accel_bias);

  for (int i = 0; i < 15; i++) {
    ekf_msg.cov_diag[i] = ekf_sample->cov_diag[i];
  }

  ekf_msg.confidence = ekf_sample->confidence;
  ekf_msg.status = ekf_sample->status;
  ekf_msg.of_count = ekf_sample->of_count;
  ekf_msg.ml_count = ekf_sample->ml_count;

  CopyTransform3D(ekf_sample->hr_global_pose, ekf_msg.hr_global_pose);

  for (int i = 0; i < 15; i++) {
    ekf_msg.ml_mahal_dists[i] = ekf_sample->ml_mahal_dists[i];
  }

  pub_.publish(ekf_msg);
}

}  // end namespace ff
