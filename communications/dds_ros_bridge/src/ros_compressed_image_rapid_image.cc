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

#include <string>

#include "dds_ros_bridge/ros_compressed_image_rapid_image.h"
#include "rapidDds/RapidConstants.h"
#include "rapidDds/MIMETypesConstants.h"

namespace ff {

RosCompressedImageRapidImage::RosCompressedImageRapidImage(
  const std::string& subscribeTopic, const std::string& pubTopic,
  const ros::NodeHandle &nh, const unsigned int queueSize)
  : RosSubRapidPub(subscribeTopic, pubTopic, nh, queueSize), MB_(1048576) {
  std::string subscribeCompresedTopic = subscribeTopic + "/compressed";
  // TODO(all): confirm topic suffix has '-'
  m_params_.topicSuffix += pubTopic;

  ROS_DEBUG("RosImageRapidImage publishing %s%s",
                rapid::IMAGESENSOR_SAMPLE_TOPIC, m_params_.topicSuffix.c_str());

  // instantiate provider
  m_provider_.reset(new rapid::ImageSensorProvider(m_params_,
    "RosCompressedImageRapidImage"));

  // start subscriber
  m_sub_ = m_nh_.subscribe(subscribeCompresedTopic, queueSize,
    &RosCompressedImageRapidImage::CallBack, this);
}

void RosCompressedImageRapidImage::CallBack(
                            const sensor_msgs::CompressedImage::ConstPtr& msg) {
  m_provider_->setMimeType(GetRapidMimeType(msg->format).c_str());
  if (msg->data.size() > 0 && msg->data.size() < MB_) {
    m_provider_->publishData(&msg->data.front(), msg->data.size());
  } else {
    int size = msg->data.size();
    ROS_ERROR("DDS ROS BRIDGE: Couldn't publish image! image size: %i > %i",
                                                                    size, MB_);
  }
}

std::string RosCompressedImageRapidImage::GetRapidMimeType(
    const std::string& rosFormat) {
  // only two accepted values jpeg or png
  if (rosFormat.compare("jpeg") == 0)
    return rapid::MIME_IMAGE_JPEG;
  if (rosFormat.compare("png") == 0)
    return rapid::MIME_IMAGE_PNG;
  return "";
}

}  // end namespace ff
