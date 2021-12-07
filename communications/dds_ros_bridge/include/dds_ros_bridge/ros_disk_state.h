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

#include <cstring>
#include <string>
#include <memory>

#include "dds_ros_bridge/ros_sub_rapid_pub.h"
#include "dds_ros_bridge/util.h"

#include "ff_msgs/DiskState.h"
#include "ff_msgs/DiskStateStamped.h"

#include "knDds/DdsTypedSupplier.h"

#include "rapidUtil/RapidHelper.h"

#include "dds_msgs/AstrobeeConstants.h"
#include "dds_msgs/DiskStateSupport.h"
#include "dds_msgs/DiskConfigSupport.h"

namespace ff {

class RosDiskStateToRapid : public RosSubRapidPub {
 public:
  RosDiskStateToRapid(const std::string& subscribe_topic,
                      const std::string& pub_topic,
                      const ros::NodeHandle &nh,
                      const unsigned int queue_size = 10);

  void Callback(ff_msgs::DiskStateStampedConstPtr const& state);
  void FillConfigAndState(ff_msgs::DiskStateStampedConstPtr state,
                          unsigned int index,
                          unsigned int size);
  void CheckAndPublish(ros::TimerEvent const& event);
  void SetPublishRate(float rate);

 private:
  using StateSupplier =
    kn::DdsTypedSupplier<rapid::ext::astrobee::DiskState>;
  using StateSupplierPtr = std::unique_ptr<StateSupplier>;

  StateSupplierPtr state_supplier_;

  using ConfigSupplier =
    kn::DdsTypedSupplier<rapid::ext::astrobee::DiskConfig>;
  using ConfigSupplierPtr = std::unique_ptr<ConfigSupplier>;

  ConfigSupplierPtr config_supplier_;

  bool updated_;

  unsigned int llp_disk_size_, mlp_disk_size_, hlp_disk_size_;

  ff_msgs::DiskStateStampedConstPtr llp_state_, mlp_state_, hlp_state_;

  ros::Timer pub_timer_;
};

}  // end namespace ff

#endif  // DDS_ROS_BRIDGE_ROS_DISK_STATE_H_
