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

#ifndef DDS_ROS_BRIDGE_RAPID_COMPRESSED_FILE_ROS_COMPRESSED_FILE_H_
#define DDS_ROS_BRIDGE_RAPID_COMPRESSED_FILE_ROS_COMPRESSED_FILE_H_

#include <string>

#include "dds_ros_bridge/rapid_sub_ros_pub.h"
#include "CompressedFile.h"

namespace ff {

class RapidCompressedFileRosCompressedFile : public RapidSubRosPub {
 public:
  RapidCompressedFileRosCompressedFile(const std::string& subscribeTopic,
                                       const std::string& pubTopic,
                                       const ros::NodeHandle &nh,
                                       const unsigned int queueSize = 10);

  // Callback for ddsEventLoop
  void operator() (rapid::ext::astrobee::CompressedFile const* file);
};

}  // end namespace ff

#endif  // DDS_ROS_BRIDGE_RAPID_COMPRESSED_FILE_ROS_COMPRESSED_FILE_H_

