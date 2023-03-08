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

// Core ROS
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

// Shared libraries
#include <ff_common/ff_names.h>
#include <ff_util/ff_nodelet.h>

#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>

// Messages
#include <sensor_msgs/Image.h>

#include <string>
#include <map>

/**
 * \ingroup hardware
 */
namespace debayer {

namespace enc = sensor_msgs::image_encodings;

class DebayerNodelet : public ff_util::FreeFlyerNodelet  {
 public:
  DebayerNodelet() : ff_util::FreeFlyerNodelet(false) {}
  ~DebayerNodelet() {}

 protected:
  void Initialize(ros::NodeHandle *nh) {
    std::string topic, topic_bayer_color, topic_bayer_raw;
    config_name_ = GetName();
    if (GetName() == "nav_cam_debayer") {
      topic = TOPIC_HARDWARE_NAV_CAM;
    } else if (GetName() == "dock_cam_debayer") {
      topic = TOPIC_HARDWARE_DOCK_CAM;
    } else {
      FF_FATAL("Unknown camera driver name.");
    }

    // Advertise color image
    topic_bayer_color = topic + TOPIC_HARDWARE_CAM_SUFFIX_BAYER_COLOR;
    pub_color_ = nh->advertise<sensor_msgs::Image>(topic_bayer_color, 1);

    // Subscribe to the raw bayer topic
    topic_bayer_raw = topic + TOPIC_HARDWARE_CAM_SUFFIX_BAYER_RAW;
    sub_raw_ = nh->subscribe(topic_bayer_raw, 1, &DebayerNodelet::BayerImageCallback, this);
  }

  // Called when depth image data arrives
  void BayerImageCallback(const sensor_msgs::ImageConstPtr& raw_msg) {
    // Next, publish to color
    if (!pub_color_.getNumSubscribers()) return;

    int bit_depth = enc::bitDepth(raw_msg->encoding);

    if (enc::isBayer(raw_msg->encoding)) {
      int type = bit_depth == 8 ? CV_8U : CV_16U;
      const cv::Mat bayer(raw_msg->height, raw_msg->width, CV_MAKETYPE(type, 1),
                          const_cast<uint8_t*>(&raw_msg->data[0]), raw_msg->step);

        sensor_msgs::ImagePtr color_msg = boost::make_shared<sensor_msgs::Image>();
        color_msg->header   = raw_msg->header;
        color_msg->height   = raw_msg->height;
        color_msg->width    = raw_msg->width;
        color_msg->encoding = bit_depth == 8? enc::BGR8 : enc::BGR16;
        color_msg->step     = color_msg->width * 3 * (bit_depth / 8);
        color_msg->data.resize(color_msg->height * color_msg->step);

        cv::Mat color(color_msg->height, color_msg->width, CV_MAKETYPE(type, 3),
                      &color_msg->data[0], color_msg->step);

        int code = -1;
        if (raw_msg->encoding == enc::BAYER_RGGB8 ||
            raw_msg->encoding == enc::BAYER_RGGB16)
          code = cv::COLOR_BayerBG2BGR;
        else if (raw_msg->encoding == enc::BAYER_BGGR8 ||
                 raw_msg->encoding == enc::BAYER_BGGR16)
          code = cv::COLOR_BayerRG2BGR;
        else if (raw_msg->encoding == enc::BAYER_GBRG8 ||
                 raw_msg->encoding == enc::BAYER_GBRG16)
          code = cv::COLOR_BayerGR2BGR;
        else if (raw_msg->encoding == enc::BAYER_GRBG8 ||
                 raw_msg->encoding == enc::BAYER_GRBG16)
          code = cv::COLOR_BayerGB2BGR;

        try {
          cv::cvtColor(bayer, color, code);
        } catch (cv::Exception& e) {
          NODELET_WARN_THROTTLE(30, "cvtColor error: '%s', bayer code: %d, width %d, height %d",
                       e.what(), code, bayer.cols, bayer.rows);
          return;
        }

        pub_color_.publish(color_msg);
    }
  }

 private:
  // ROS
  ros::Subscriber sub_raw_;
  ros::Publisher pub_color_;

  std::string config_name_;
};

PLUGINLIB_EXPORT_CLASS(debayer::DebayerNodelet, nodelet::Nodelet);

}  // namespace debayer
