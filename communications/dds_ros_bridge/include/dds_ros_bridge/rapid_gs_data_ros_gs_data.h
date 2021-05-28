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

#ifndef DDS_ROS_BRIDGE_RAPID_GS_DATA_ROS_GS_DATA_H_
#define DDS_ROS_BRIDGE_RAPID_GS_DATA_ROS_GS_DATA_H_

#include <stdint.h>

#include <ros/assert.h>

#include <string>
#include <cstring>
#include <vector>

#include "dds_ros_bridge/rapid_sub_ros_pub.h"
#include "dds_ros_bridge/util.h"

#include "ff_msgs/GuestScienceData.h"

#include "AstrobeeConstants.h"

#include "knDds/DdsTypedSupplier.h"

#include "GuestScienceDataSupport.h"

#include "rapidDds/RapidConstants.h"

#include "rapidUtil/RapidHelper.h"

namespace ff {

class RapidGuestScienceDataToRos : public RapidSubRosPub {
 public:
  RapidGuestScienceDataToRos(const std::string& subscribe_topic,
                          const std::string& subscribe_partition,
                          const std::string& pub_topic,
                          const ros::NodeHandle &nh,
                          const unsigned int queue_size = 10);

    /**
     * call back for ddsEventLoop
     */
  void operator() (rapid::ext::astrobee::GuestScienceData const* data);

 private:
  std::string subscriber_partition_;
};

}  // end namespace ff

#endif  // DDS_ROS_BRIDGE_RAPID_GS_DATA_ROS_GS_DATA_H_
