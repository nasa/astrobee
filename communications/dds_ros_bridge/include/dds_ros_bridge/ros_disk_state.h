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


#ifndef DDS_ROS_BRIDGE_ROS_DISK_STATE_H_
#define DDS_ROS_BRIDGE_ROS_DISK_STATE_H_

#include <string>
#include <cstring>
#include <memory>

#include "knDds/DdsTypedSupplier.h"

#include "dds_ros_bridge/enum_helper.h"
#include "dds_ros_bridge/ros_sub_rapid_pub.h"
#include "dds_ros_bridge/util.h"

#include "ff_msgs/DiskState.h"
#include "ff_msgs/DiskStateStamped.h"

#include "AstrobeeConstants.h"
#include "DiskStateSupport.h"
#include "DiskConfigSupport.h"

#include "rapidUtil/RapidHelper.h"

namespace ff {

class RosDiskStateToRapid : public RosSubRapidPub {
 public:
  RosDiskStateToRapid(const std::string& subscribeTopic,
                      const std::string& pubTopic,
                      const ros::NodeHandle &nh,
                      const unsigned int queueSize = 10);

  void Callback(ff_msgs::DiskStateStampedConstPtr const& state);
  void CheckAndPublish(ros::TimerEvent const& event);
  void SetPublishRate(float rate);

 private:
  using StateSupplier =
    kn::DdsTypedSupplier<rapid::ext::astrobee::DiskState>;
  using StateSupplierPtr = std::unique_ptr<StateSupplier>;

  StateSupplierPtr s_supplier_;

  using ConfigSupplier =
    kn::DdsTypedSupplier<rapid::ext::astrobee::DiskConfig>;
  using ConfigSupplierPtr = std::unique_ptr<ConfigSupplier>;

  ConfigSupplierPtr c_supplier_;

  bool updated_;
  int num_disks_, llp_start_index_, mlp_start_index_, hlp_start_index_;

  ros::Timer pub_timer_;
};

}  // end namespace ff

#endif  // DDS_ROS_BRIDGE_ROS_DISK_STATE_H_
