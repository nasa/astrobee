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


#ifndef DDS_ROS_BRIDGE_ROS_CPU_STATE_H_
#define DDS_ROS_BRIDGE_ROS_CPU_STATE_H_

#include <string>

#include "knDds/DdsTypedSupplier.h"

#include "dds_ros_bridge/ros_sub_rapid_pub.h"
#include "dds_ros_bridge/util.h"

#include "ff_msgs/CpuState.h"
#include "ff_msgs/CpuStateStamped.h"

#include "AstrobeeConstants.h"
#include "CpuConfigSupport.h"
#include "CpuStateSupport.h"

#include "rapidUtil/RapidHelper.h"

namespace ff {

class RosCpuStateToRapid : public RosSubRapidPub {
 public:
  RosCpuStateToRapid(const std::string& subTopic,
                     const std::string& pubTopic,
                     const ros::NodeHandle &nh,
                     const unsigned int queueSize = 10);

  void Callback(ff_msgs::CpuStateStampedConstPtr const& state);
  void CheckAndPublish(ros::TimerEvent const& event);
  void SetPublishRate(float rate);

 private:
  using ConfigSupplier = kn::DdsTypedSupplier<rapid::ext::astrobee::CpuConfig>;
  using ConfigSupplierPtr = std::unique_ptr<ConfigSupplier>;

  ConfigSupplierPtr c_supplier_;

  using StateSupplier = kn::DdsTypedSupplier<rapid::ext::astrobee::CpuState>;
  using StateSupplierPtr = std::unique_ptr<StateSupplier>;

  StateSupplierPtr s_supplier_;

  bool updated_;

  ros::Timer pub_timer_;
};

}  // end namespace ff

#endif  // DDS_ROS_BRIDGE_ROS_CPU_STATE_H_
