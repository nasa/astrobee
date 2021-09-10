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

#ifndef DDS_ROS_BRIDGE_ROS_ZONES_RAPID_COMPRESSED_FILE_H_
#define DDS_ROS_BRIDGE_ROS_ZONES_RAPID_COMPRESSED_FILE_H_

#include <cstring>
#include <iostream>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "boost/iostreams/copy.hpp"
#include "boost/iostreams/device/back_inserter.hpp"
#include "boost/iostreams/device/array.hpp"
#include "boost/iostreams/filter/zlib.hpp"
#include "boost/iostreams/filtering_stream.hpp"

#include "dds_ros_bridge/ros_sub_rapid_pub.h"
#include "dds_ros_bridge/util.h"

#include "ff_msgs/GetZones.h"
#include "ff_msgs/Zone.h"

#include "visualization_msgs/MarkerArray.h"

#include "knDds/DdsTypedSupplier.h"

#include "rapidUtil/RapidHelper.h"

#include "dds_msgs/AstrobeeConstants.h"
#include "dds_msgs/CompressedFileSupport.h"

namespace ff {

class RosZonesToRapidCompressedFile : public RosSubRapidPub {
 public:
  RosZonesToRapidCompressedFile(const std::string& subscribe_topic,
                                const std::string& get_zones_srv,
                                const std::string& pub_topic,
                                const ros::NodeHandle &nh,
                                const unsigned int queue_size = 10);

  void Callback(visualization_msgs::MarkerArray::ConstPtr const& zones);

 private:
  using Supplier = kn::DdsTypedSupplier<rapid::ext::astrobee::CompressedFile>;
  using SupplierPtr = std::unique_ptr<Supplier>;

  SupplierPtr file_supplier_;

  ros::ServiceClient get_zones_client_;
};

}  // end namespace ff

#endif  // DDS_ROS_BRIDGE_ROS_ZONES_RAPID_COMPRESSED_FILE_H_
