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

#ifndef DDS_ROS_BRIDGE_ROS_ARM_JOINT_SAMPLE_H_
#define DDS_ROS_BRIDGE_ROS_ARM_JOINT_SAMPLE_H_

#include <string>

#include "dds_ros_bridge/ros_sub_rapid_pub.h"
#include "dds_ros_bridge/util.h"

#include "ff_msgs/JointSample.h"
#include "ff_msgs/JointSampleStamped.h"

#include "knDds/DdsTypedSupplier.h"

#include "JointConfigSupport.h"
#include "JointSampleSupport.h"

#include "rapidDds/RapidConstants.h"
#include "rapidUtil/RapidHelper.h"

#include "ros/ros.h"

namespace ff {

class RosArmJointSampleToRapid : public RosSubRapidPub {
 public:
  RosArmJointSampleToRapid(const std::string& subscribe_topic,
                           const std::string& pub_topic,
                           const ros::NodeHandle &nh,
                           unsigned int queue_size = 10);

  void Callback(ff_msgs::JointSampleStampedConstPtr const& sample);

 protected:
  using ConfigSupplier = kn::DdsTypedSupplier<rapid::JointConfig>;
  using ConfigSupplierPtr = std::unique_ptr<ConfigSupplier>;

  using SampleSupplier = kn::DdsTypedSupplier<rapid::JointSample>;
  using SampleSupplierPtr = std::unique_ptr<SampleSupplier>;

  ConfigSupplierPtr config_supplier_;
  SampleSupplierPtr sample_supplier_;
};

}  // end namespace ff

#endif  // DDS_ROS_BRIDGE_ROS_ARM_JOINT_SAMPLE_H_
