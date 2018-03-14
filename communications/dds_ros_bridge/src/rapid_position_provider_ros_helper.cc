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

#include "dds_ros_bridge/rapid_position_provider_ros_helper.h"

namespace rapid {

PositionProviderRosHelper::PositionProviderRosHelper(
                                      PositionTopicPairParameters const& params,
                                      std::string const& entity_name)
  : PositionProvider(params, entity_name) {
}

void PositionProviderRosHelper::Publish(
                              ff_msgs::EkfState::ConstPtr const& pose_vel_cov) {
  rapid::PositionSample& sample = m_dataSupplier.event();

  sample.hdr.timeStamp = util::RosTime2RapidTime(pose_vel_cov->header.stamp);
  sample.hdr.statusCode = 0;

  // pos
  // double << float32
  sample.pose.xyz[0] = pose_vel_cov->pose.position.x;
  sample.pose.xyz[1] = pose_vel_cov->pose.position.y;
  sample.pose.xyz[2] = pose_vel_cov->pose.position.z;

  // orientation as rapid::RAPID_ROT_QUAT
  // order specified in rapid::BaseTypes X Y Z W
  // float << float32
  sample.pose.rot[0] = pose_vel_cov->pose.orientation.x;
  sample.pose.rot[1] = pose_vel_cov->pose.orientation.y;
  sample.pose.rot[2] = pose_vel_cov->pose.orientation.z;
  sample.pose.rot[3] = pose_vel_cov->pose.orientation.w;

  // linear velocity
  // double << float32
  sample.velocity.xyz[0] = pose_vel_cov->velocity.x;
  sample.velocity.xyz[1] = pose_vel_cov->velocity.y;
  sample.velocity.xyz[2] = pose_vel_cov->velocity.z;

  // angular velocity as rapid::RAPID_ROT_XYZ
  // float << float32
  sample.velocity.rot[0] = pose_vel_cov->omega.x;
  sample.velocity.rot[1] = pose_vel_cov->omega.y;
  sample.velocity.rot[2] = pose_vel_cov->omega.z;

  // Add confidence measure to values
  sample.values.length(1);
  sample.values[0]._d = rapid::RAPID_INT;
  sample.values[0]._u.i = pose_vel_cov->confidence;

  m_dataSupplier.sendEvent();
}

}  // end namespace rapid
