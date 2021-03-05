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

#include "dds_ros_bridge/rapid_position_provider_ros_pose_helper.h"

namespace rapid {

PositionProviderRosPoseHelper::PositionProviderRosPoseHelper(
                                      PositionTopicPairParameters const& params,
                                      std::string const& entity_name)
  : PositionProvider(params, entity_name) {
}

void PositionProviderRosPoseHelper::Publish(
                            geometry_msgs::PoseStamped::ConstPtr const& pose) {
  rapid::PositionSample& sample = m_dataSupplier.event();

  sample.hdr.timeStamp = util::RosTime2RapidTime(pose->header.stamp);
  sample.hdr.statusCode = 0;

  // pos
  // double << float32
  sample.pose.xyz[0] = pose->pose.position.x;
  sample.pose.xyz[1] = pose->pose.position.y;
  sample.pose.xyz[2] = pose->pose.position.z;

  // orientation as rapid::RAPID_ROT_QUAT
  // order specified in rapid::BaseTypesX Y Z W
  // float << float3
  sample.pose.rot[0] = pose->pose.orientation.x;
  sample.pose.rot[1] = pose->pose.orientation.y;
  sample.pose.rot[2] = pose->pose.orientation.z;
  sample.pose.rot[3] = pose->pose.orientation.w;

  m_dataSupplier.sendEvent();
}

}  // end namespace rapid
