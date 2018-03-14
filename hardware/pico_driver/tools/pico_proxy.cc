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

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <sensor_msgs/Image.h>
#include <ff_msgs/PicoflexxIntermediateData.h>

#include <string>
#include <map>

// Publishers
ros::Publisher pub_d_;
ros::Publisher pub_a_;
ros::Publisher pub_i_;
ros::Publisher pub_c_;

// Layer extraction
std::vector < cv::Mat > layers(4);

// Depth and confidence matrices
sensor_msgs::Image distance_, confidence_;

struct null_deleter {
    void operator()(void const *) const {}
};

// Called when extended data arrives
void ExtendedCallback(const ff_msgs::PicoflexxIntermediateData::ConstPtr& msg) {
  cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(
    sensor_msgs::ImageConstPtr(&msg->raw, null_deleter()),
    sensor_msgs::image_encodings::TYPE_32FC4);
  cv::split(cv_ptr->image, layers);
  std_msgs::Header header;
  header.stamp = ros::Time::now();

  // Change this for Haz Cam
  header.frame_id = msg->header.frame_id;
  cv_bridge::CvImage d(header, sensor_msgs::image_encodings::TYPE_32FC1, layers[0]);
  // TYPE_8UC1 encoding for Kalibr's calibration data
  cv_bridge::CvImage a(header, sensor_msgs::image_encodings::TYPE_8UC1, layers[1]);
  cv_bridge::CvImage i(header, sensor_msgs::image_encodings::TYPE_32FC1, layers[2]);
  cv_bridge::CvImage n(header, sensor_msgs::image_encodings::TYPE_32FC1, layers[3]);

  // Publish individual images
  pub_d_.publish(d.toImageMsg());
  pub_a_.publish(a.toImageMsg());
  pub_i_.publish(i.toImageMsg());
  pub_c_.publish(n.toImageMsg());
}

// Called when depth image data arrives
void DepthImageCallback(const sensor_msgs::ImageConstPtr& msg) {
  // Prepare distance image
  distance_.header.stamp = ros::Time::now();
  distance_.header.frame_id = msg->header.frame_id;
  distance_.height = msg->height;
  distance_.width = msg->width;
  distance_.is_bigendian = msg->is_bigendian;
  distance_.encoding = sensor_msgs::image_encodings::TYPE_16UC1;
  distance_.step = distance_.width * sizeof(uint16_t);
  distance_.data.resize(distance_.height * distance_.step);
  // Prepare confidenc imahe
  confidence_.header.stamp = ros::Time::now();
  confidence_.header.frame_id = msg->header.frame_id;
  confidence_.height = msg->height;
  confidence_.width = msg->width;
  confidence_.is_bigendian = msg->is_bigendian;
  confidence_.encoding = sensor_msgs::image_encodings::TYPE_8UC1;
  confidence_.step = confidence_.width * sizeof(uint8_t);
  confidence_.data.resize(confidence_.height * confidence_.step);
  // Set pointers to the data fields of the corresponding ROS images
  uint16_t *msg_ptr = const_cast<uint16_t *>(
    reinterpret_cast<const uint16_t *>(&msg->data[0]));
  uint16_t *dist_ptr = reinterpret_cast<uint16_t *>(&distance_.data[0]);
  uint8_t *conf_ptr = reinterpret_cast<uint8_t *>(&confidence_.data[0]);
  // Iterate through the image found in msg
  for (size_t i = 0; i < (distance_.width * distance_.height);
    ++i, ++msg_ptr, ++dist_ptr, ++conf_ptr) {
    *dist_ptr = *msg_ptr & 0x1fff;                     // Grab the distance
    *conf_ptr = (uint8_t)((*msg_ptr & 0xE000) >> 13);  // Grab the confidence
  }
  pub_d_.publish(distance_);
  pub_c_.publish(confidence_);
}

// Main entry point of application
int main(int argc, char **argv) {
  ros::init(argc, argv, "listener");
  ros::NodeHandle n("~");
  if (!n.hasParam("topic"))
    ROS_FATAL("You need to pass a topic to the pico proxy");
  if (!n.hasParam("topic_type"))
    ROS_FATAL("You need to pass a topic type to the pico proxy");
  std::string topic, topic_type;
  n.getParam("topic", topic);
  n.getParam("topic_type", topic_type);
  ROS_INFO_STREAM("Listening on topic " << topic);
  ros::Subscriber sub;
  if (topic_type == "extended") {
    pub_d_ = n.advertise < sensor_msgs::Image > (topic + "/distance/", 1);
    pub_a_ = n.advertise < sensor_msgs::Image > (topic + "/amplitude/", 1);
    pub_i_ = n.advertise < sensor_msgs::Image > (topic + "/intensity/", 1);
    pub_c_ = n.advertise < sensor_msgs::Image > (topic + "/noise/", 1);
    sub = n.subscribe(topic, 1, ExtendedCallback);
  } else if (topic_type == "depth_image") {
    pub_d_ = n.advertise < sensor_msgs::Image > (topic + "/distance", 1);
    pub_c_ = n.advertise < sensor_msgs::Image > (topic + "/confidence", 1);
    sub = n.subscribe(topic, 1, DepthImageCallback);
  } else {
    ROS_FATAL("Unsupported type (must be \"extended\" or \"depth_image\")");
  }
  ros::spin();
  return 0;
}
