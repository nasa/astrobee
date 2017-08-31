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

#include <ndds/ndds_cpp.h>

#include <string>

#include "dds_ros_bridge/rapid_position_provider_ros_helper.h"
#include "dds_ros_bridge/util.h"

namespace rapid {

PositionProviderRosHelper::PositionProviderRosHelper(
    PositionTopicPairParameters const& params,
    std::string const& entityName)
  : PositionProvider(params, entityName) {
}

void PositionProviderRosHelper::Publish(
  ff_msgs::EkfState::ConstPtr const& poseVelCov) {
  rapid::PositionSample& sample = m_dataSupplier.event();

  sample.hdr.timeStamp = util::RosTime2RapidTime(poseVelCov->header.stamp);
  sample.hdr.statusCode = 0;

  // pos
  // double << float32
  sample.pose.xyz[0] = poseVelCov->pose.position.x;
  sample.pose.xyz[1] = poseVelCov->pose.position.y;
  sample.pose.xyz[2] = poseVelCov->pose.position.z;

  // orientation as rapid::RAPID_ROT_QUAT
  // order specified in rapid::BaseTypes X Y Z W
  // float << float32
  sample.pose.rot[0] = poseVelCov->pose.orientation.x;
  sample.pose.rot[1] = poseVelCov->pose.orientation.y;
  sample.pose.rot[2] = poseVelCov->pose.orientation.z;
  sample.pose.rot[3] = poseVelCov->pose.orientation.w;

  // linear velocity
  // double << float32
  sample.velocity.xyz[0] = poseVelCov->velocity.x;
  sample.velocity.xyz[1] = poseVelCov->velocity.y;
  sample.velocity.xyz[2] = poseVelCov->velocity.z;

  // angular velocity as rapid::RAPID_ROT_XYZ
  // float << float32
  sample.velocity.rot[0] = poseVelCov->omega.x;
  sample.velocity.rot[1] = poseVelCov->omega.y;
  sample.velocity.rot[2] = poseVelCov->omega.z;

  // Add confidence measure to values
  sample.values.length(1);
  sample.values[0]._d = rapid::RAPID_INT;
  sample.values[0]._u.i = poseVelCov->confidence;

  m_dataSupplier.sendEvent();
}

}  // end namespace rapid
