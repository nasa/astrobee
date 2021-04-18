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

#ifndef GROUND_DDS_ROS_BRIDGE_RAPID_POSITION_H_
#define GROUND_DDS_ROS_BRIDGE_RAPID_POSITION_H_

#include <string>
#include <memory>

#include "ground_dds_ros_bridge/rapid_sub_ros_pub.h"
#include "ground_dds_ros_bridge/util.h"

#include "ff_msgs/EkfState.h"

#include "AstrobeeConstants.h"
#include "EkfStateSupport.h"

namespace ff {

class RapidPositionToRos : public RapidSubRosPub {
 public:
  RapidPositionToRos(const std::string& subscribe_topic,
                     const std::string& pub_topic,
                     const ros::NodeHandle &nh,
                     const unsigned int queue_size = 10);

  void CopyTransform3D(geometry_msgs::Pose& pose,
                       const rapid::Transform3D& transform);

  void CopyVec3D(geometry_msgs::Vector3& vec_out,
                 const rapid::Vec3d& vec_in);

  // Callback for ddsEventLoop
  void operator() (rapid::ext::astrobee::EkfState const* rapid_ekf_state);
};

}  // end namespace ff

#endif  // GROUND_DDS_ROS_BRIDGE_RAPID_POSITION_H_
