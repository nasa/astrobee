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

#include "ground_dds_ros_bridge/rapid_image.h"

namespace ff {

RapidImageToRos::RapidImageToRos(const std::string& subscribe_topic,
                                 const std::string& pub_topic,
                                 const ros::NodeHandle &nh,
                                 const unsigned int queue_size)
  : RapidSubRosPub(subscribe_topic,
                   pub_topic,
                   nh,
                   "RapidImageToRos",
                   queue_size) {
  // advertise ros topic
  publish_topic_ = pub_topic + "/compressed";
  pub_ =
        nh_.advertise<sensor_msgs::CompressedImage>(publish_topic_, queue_size);

  // connect to ddsEventLoop
  try {
    dds_event_loop_.connect<rapid::ImageSensorSample>(this,
                  rapid::IMAGESENSOR_SAMPLE_TOPIC + subscribe_topic,  // topic
                  "",                                                 // name
                  "RapidImageSensorSampleProfile",                    // profile
                  "");                                                // library
  } catch (std::exception& e) {
    ROS_ERROR_STREAM("RapidImageToRos exception: " << e.what());
    throw;
  } catch (...) {
    ROS_ERROR("RapidImageToRos exception unknown");
    throw;
  }

  // start thread
  StartThread();
}

void RapidImageToRos::operator() (rapid::ImageSensorSample const* rapid_img) {
  sensor_msgs::CompressedImage img;
  util::RapidHeader2Ros(rapid_img->hdr, &img.header);

  // only accept 2 values; jpeg or png
  if (std::strcmp(rapid_img->mimeType, rapid::MIME_IMAGE_JPEG) == 0) {
    img.format = "jpeg";
  } else if (std::strcmp(rapid_img->mimeType, rapid::MIME_IMAGE_PNG) == 0) {
    img.format = "png";
  } else {
    ROS_ERROR_STREAM("Unknown image format in ground bridge. Type sent is " << rapid_img->mimeType);
  }

  unsigned char* buf = rapid_img->data.get_contiguous_buffer();
  img.data.reserve(rapid_img->data.length());
  img.data.resize(rapid_img->data.length());
  std::memmove(img.data.data(), buf, rapid_img->data.length());

  pub_.publish(img);
}

}  // end namespace ff
