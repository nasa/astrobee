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


#ifndef DDS_ROS_BRIDGE_ROS_GNC_FAM_CMD_STATE_H_
#define DDS_ROS_BRIDGE_ROS_GNC_FAM_CMD_STATE_H_

#include <string>

#include "dds_ros_bridge/ros_sub_rapid_pub.h"
#include "dds_ros_bridge/util.h"

#include "ff_msgs/FamCommand.h"

#include "knDds/DdsTypedSupplier.h"

#include "rapidDds/RapidConstants.h"

#include "rapidIo/RapidIoParameters.h"

#include "rapidUtil/RapidHelper.h"

#include "dds_msgs/AstrobeeConstants.h"
#include "dds_msgs/GncFamCmdStateSupport.h"

namespace ff {

class RosGncFamCmdStateToRapid : public RosSubRapidPub {
 public:
  RosGncFamCmdStateToRapid(const std::string& subscribe_topic,
                           const std::string& pub_topic,
                           const ros::NodeHandle& nh,
                           const unsigned int queue_size = 10);

  void CopyVec3D(rapid::Vec3d& vec_out, const geometry_msgs::Vector3& vec_in);
  void MsgCallback(const ff_msgs::FamCommandConstPtr& msg);
  void PubGncFamCmdState(const ros::TimerEvent& event);
  void SetGncPublishRate(float rate);

 private:
  ff_msgs::FamCommandConstPtr fam_msg_;

  using StateSupplier =
      kn::DdsTypedSupplier<rapid::ext::astrobee::GncFamCmdState>;
  using StateSupplierPtr = std::unique_ptr<StateSupplier>;

  StateSupplierPtr state_supplier_;

  ros::Timer gnc_timer_;
};

}  // end namespace ff

#endif  // DDS_ROS_BRIDGE_ROS_GNC_FAM_CMD_STATE_H_
