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

#include "dds_ros_bridge/ros_compressed_image_rapid_image.h"

namespace ff {

RosCompressedImageRapidImage::RosCompressedImageRapidImage(
                                            const std::string& subscribe_topic,
                                            const std::string& pub_topic,
                                            const ros::NodeHandle &nh,
                                            const unsigned int queue_size)
  : RosSubRapidPub(subscribe_topic, pub_topic, nh, queue_size), MB_(1048576) {
  std::string subscribe_compresed_topic = subscribe_topic + "/compressed";
  // TODO(all): confirm topic suffix has '-'
  params_.topicSuffix += pub_topic;

  // instantiate provider
  provider_.reset(new rapid::ImageSensorProvider(params_,
                                              "RosCompressedImageRapidImage"));

  // start subscriber
  sub_ = nh_.subscribe(subscribe_compresed_topic,
                       queue_size,
                       &RosCompressedImageRapidImage::CallBack,
                       this);
}

void RosCompressedImageRapidImage::CallBack(
                            const sensor_msgs::CompressedImage::ConstPtr& msg) {
  provider_->setMimeType(GetRapidMimeType(msg->format).c_str());
  if (msg->data.size() > 0 && msg->data.size() < MB_) {
    ROS_ERROR_STREAM("Publishing image. size is " << msg->data.size() << " MB: " << MB_);
    provider_->publishData(&msg->data.front(), msg->data.size());
  } else {
    int size = msg->data.size();
    ROS_ERROR("DDS ROS BRIDGE: Couldn't publish image! image size: %i > %i",
              size,
              MB_);
  }
}

std::string RosCompressedImageRapidImage::GetRapidMimeType(
                                                const std::string& ros_format) {
  // only two accepted values jpeg or png
  if (ros_format.compare("jpeg") == 0)
    return rapid::MIME_IMAGE_JPEG;
  if (ros_format.compare("mono8; jpeg compressed ") == 0)
    return rapid::MIME_IMAGE_JPEG;
  if (ros_format.compare("png") == 0)
    return rapid::MIME_IMAGE_PNG;

  ROS_ERROR_STREAM("DDS ROS Bridge: Unknown camera format: " << ros_format << ".");
  return "";
}

}  // end namespace ff
