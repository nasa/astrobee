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

#ifndef DDS_ROS_BRIDGE_ROS_DATA_TO_DISK_H_
#define DDS_ROS_BRIDGE_ROS_DATA_TO_DISK_H_

#include <string>
#include <memory>

#include "dds_ros_bridge/ros_sub_rapid_pub.h"
#include "dds_ros_bridge/util.h"

#include "ff_msgs/DataToDiskState.h"
#include "ff_msgs/DataTopicsList.h"
#include "ff_msgs/SaveSettings.h"

#include "knDds/DdsTypedSupplier.h"

#include "rapidUtil/RapidHelper.h"

#include "AstrobeeConstants.h"
#include "DataToDiskStateSupport.h"
#include "DataTopicsListSupport.h"

namespace ff {

class RosDataToDiskToRapid : public RosSubRapidPub {
 public:
  RosDataToDiskToRapid(const std::string& state_subscribe_topic,
                       const std::string& topics_subscribe_topic,
                       const std::string& pub_topic,
                       const ros::NodeHandle& nh,
                       const unsigned int queue_size = 10);

  void StateCallback(ff_msgs::DataToDiskStateConstPtr const& state);
  void TopicsCallback(ff_msgs::DataTopicsListConstPtr const& topics);

 private:
  using StateSupplier =
      kn::DdsTypedSupplier<rapid::ext::astrobee::DataToDiskState>;
  using StateSupplierPtr = std::unique_ptr<StateSupplier>;

  StateSupplierPtr state_supplier_;

  using TopicsSupplier =
      kn::DdsTypedSupplier<rapid::ext::astrobee::DataTopicsList>;
  using TopicsSupplierPtr = std::unique_ptr<TopicsSupplier>;

  TopicsSupplierPtr topics_supplier_;

  ros::Subscriber topics_sub_;

  std::string state_subscribe_topic_, topics_subscribe_topic_;
};

}  // end namespace ff

#endif  // DDS_ROS_BRIDGE_ROS_DATA_TO_DISK_H_
