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

#include <common/init.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>

static ros::Publisher camera_info_pub, camera_raw_pub;
static ros::Subscriber camera_sub;

static sensor_msgs::CameraInfo caminfo;

// hardcode this, was never ported to lua
void SetInfo() {
  caminfo.K = {7.6946497855829443e+02,
               0,
               6.3522428948586071e+02,
               0,
               7.6875198858016370e+02,
               4.9172890181774102e+02,
               0,
               0,
               1};
  caminfo.P = {7.6946497855829443e+02,
               0,
               6.3522428948586071e+02,
               0,
               0,
               7.6875198858016370e+02,
               4.9172890181774102e+02,
               0,
               0,
               0,
               1,
               0};
  caminfo.D = {-3.2402610654612035e-01, 9.4629740227230974e-02,
               3.8611413028713553e-04, 2.0158706205913819e-04};
  caminfo.R = {1, 0, 0, 0, 1, 0, 0, 0, 1};
  caminfo.height = 960;
  caminfo.width = 1280;
  caminfo.distortion_model = "plumb_bob";
}

void imageCB(const sensor_msgs::Image::ConstPtr &img) {
  caminfo.header = img->header;
  caminfo.header.frame_id = "overhead";
  sensor_msgs::Image img2 = *img;
  img2.header.frame_id = "overhead";
  camera_info_pub.publish(caminfo);
  camera_raw_pub.publish(img2);
}

int main(int argc, char **argv) {
  common::InitFreeFlyerApplication(&argc, &argv);
  ros::init(argc, argv, "overhead_node");

  ros::NodeHandle nh("~");
  // Load Camera Info
  SetInfo();

  camera_raw_pub = nh.advertise<sensor_msgs::Image>("/overhead/image_raw", 1);
  camera_info_pub =
      nh.advertise<sensor_msgs::CameraInfo>("/overhead/camera_info", 1);
  camera_sub = nh.subscribe("/overhead/image", 1, &imageCB);

  ros::spin();
}
