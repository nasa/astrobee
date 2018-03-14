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

#ifndef DDS_ROS_BRIDGE_ROS_COMPRESSED_IMAGE_RAPID_IMAGE_H_
#define DDS_ROS_BRIDGE_ROS_COMPRESSED_IMAGE_RAPID_IMAGE_H_

#include <string>

#include "dds_ros_bridge/ros_sub_rapid_pub.h"

#include "rapidDds/MIMETypesConstants.h"
#include "rapidDds/RapidConstants.h"

#include "rapidIo/ImageSensorProvider.h"
#include "rapidIo/RapidIoParameters.h"

#include "sensor_msgs/CompressedImage.h"

namespace ff {

/**
 * RosCompressedImageRapidImage will use the ros::Subscriber to 
 *   subscribe to the compressed image type
 */
class RosCompressedImageRapidImage : public RosSubRapidPub {
 public:
  RosCompressedImageRapidImage(const std::string& subscribe_topic,
                               const std::string& pub_topic,
                               const ros::NodeHandle &nh,
                               const unsigned int queue_size = 10);

  void CallBack(const sensor_msgs::CompressedImage::ConstPtr& msg);

 private:
  std::string GetRapidMimeType(const std::string& ros_format);
  rapid::ImageSensorProviderParameters params_;
  std::shared_ptr<rapid::ImageSensorProvider> provider_;
  const unsigned int MB_;
};

}  // end namespace ff

#endif  // DDS_ROS_BRIDGE_ROS_COMPRESSED_IMAGE_RAPID_IMAGE_H_
