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

#ifndef GROUND_DDS_ROS_BRIDGE_RAPID_IMAGE_ROS_COMPRESSED_IMAGE_H_
#define GROUND_DDS_ROS_BRIDGE_RAPID_IMAGE_ROS_COMPRESSED_IMAGE_H_

#include <string>

#include "ground_dds_ros_bridge/rapid_sub_ros_pub.h"
#include "ground_dds_ros_bridge/util.h"

#include "rapidDds/MIMETypesConstants.h"
#include "rapidDds/RapidConstants.h"

#include "rapidIo/ImageSensorProvider.h"
#include "rapidIo/RapidIoParameters.h"

#include "sensor_msgs/CompressedImage.h"

namespace ff {

class RapidImageRosCompressedImage : public RapidSubRosPub {
 public:
  RapidImageRosCompressedImage(const std::string& subscribe_topic,
                               const std::string& pub_topic,
                               const ros::NodeHandle &nh,
                               const unsigned int queue_size = 10);

  // Callback for ddsEventLoop
  void operator() (rapid::ImageSensorSample const* image);
};

}  // end namespace ff

#endif  // GROUND_DDS_ROS_BRIDGE_RAPID_IMAGE_ROS_COMPRESSED_IMAGE_H_
