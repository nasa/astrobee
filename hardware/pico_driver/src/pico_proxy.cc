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
#include <config_reader/config_reader.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// Messages
#include <sensor_msgs/Image.h>
#include <ff_msgs/PicoflexxIntermediateData.h>

#include <string>
#include <map>

/**
 * \ingroup hardware
 */
namespace pico_proxy {

class PicoProxyNodelet : public ff_util::FreeFlyerNodelet  {
 public:
  PicoProxyNodelet() : ff_util::FreeFlyerNodelet(false) {}
  ~PicoProxyNodelet() {}

 protected:
  void Initialize(ros::NodeHandle *nh) {
    std::string topic, topic_type;
    topic = TOPIC_HARDWARE_PICOFLEXX_PREFIX;
    config_name_ = GetName();
    // Read the camera name
    if (config_name_.find("haz") != std::string::npos) {
        topic += TOPIC_HARDWARE_NAME_HAZ_CAM;
    } else if (config_name_.find("perch") != std::string::npos) {
        topic += TOPIC_HARDWARE_NAME_PERCH_CAM;
    } else {
      NODELET_FATAL("You need to pass a camera name to the pico proxy nodelet name");
    }

    // Read the topic type
    if (config_name_.find("extended") != std::string::npos) {
      topic += TOPIC_HARDWARE_PICOFLEXX_SUFFIX_EXTENDED;

      // Read config file for amplitude factor
      config_.AddFile("cameras.config");
      if (!ReadParams())
        return;

      pub_d_ = nh->advertise<sensor_msgs::Image>(topic + "/distance/", 1);
      pub_a_ = nh->advertise<sensor_msgs::Image>(topic + "/amplitude/", 1);
      pub_i_ = nh->advertise<sensor_msgs::Image>(topic + "/intensity/", 1);
      pub_c_ = nh->advertise<sensor_msgs::Image>(topic + "/noise/", 1);
      pub_a_int_ = nh->advertise<sensor_msgs::Image>(topic + "/amplitude_int/", 1);
      sub_ = nh->subscribe(topic, 1, &PicoProxyNodelet::ExtendedCallback, this);

    } else if (config_name_.find("depth_image") != std::string::npos) {
      topic += TOPIC_HARDWARE_PICOFLEXX_SUFFIX_DEPTH_IMAGE;

      pub_d_ = nh->advertise<sensor_msgs::Image>(topic + "/distance", 1);
      pub_c_ = nh->advertise<sensor_msgs::Image>(topic + "/confidence", 1);
      sub_ = nh->subscribe(topic, 1, &PicoProxyNodelet::DepthImageCallback, this);

    } else {
      NODELET_FATAL("The pico proxy nodelet name must have extended or depth_image");
    }
  }

  bool ReadParams(void) {
    // Read the config file
    if (!config_.ReadFiles()) {
      NODELET_FATAL("Failed to read config files.");
      return false;
    }
    // Try and open the config file
    config_reader::ConfigReader::Table pconfig;
    if (!config_.GetTable("picoflexx", &pconfig)) {
      NODELET_FATAL("Picoflexx config not found in LUA.");
      return false;
    }
    // Read the device information from the config table
    config_reader::ConfigReader::Table devices;
    if (!pconfig.GetTable("devices", &devices)) {
      NODELET_FATAL("Lua:Could get devices item in config file");
    }

    for (int i = 0; i < devices.GetSize(); i++) {
      // Get the device info
      config_reader::ConfigReader::Table device_info;
      if (!devices.GetTable(i + 1, &device_info)) {
        NODELET_FATAL("Lua:Could get row in table table");
        continue;
      }

      // Get the parameters
      std::string name;
      if (!device_info.GetStr("name", &name)) {
        NODELET_FATAL("Lua:Could not find row 'name' in table");
        continue;
      }

      if (config_name_.find(name) == std::string::npos)
        continue;

      // Query the amplitude factor
      if (device_info.GetReal("amplitude_factor", &amplitude_factor_))
        return true;
    }
    NODELET_FATAL("You need to pass the amplitude factor to camera parameters");
    return false;
  }

  struct null_deleter {
      void operator()(void const *) const {}
  };

  // Called when extended data arrives
  void ExtendedCallback(const ff_msgs::PicoflexxIntermediateData::ConstPtr& msg) {
    cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(
      sensor_msgs::ImageConstPtr(&msg->raw, null_deleter()),
      sensor_msgs::image_encodings::TYPE_32FC4);
    cv::split(cv_ptr->image, layers_);

    // Keep the same timestamp as the original data
    std_msgs::Header header;
    header.stamp =  msg->header.stamp;
    header.frame_id = msg->header.frame_id;

    cv_bridge::CvImage d(header, sensor_msgs::image_encodings::TYPE_32FC1, layers_[0]);
    cv_bridge::CvImage a(header, sensor_msgs::image_encodings::TYPE_32FC1, layers_[1]);
    cv_bridge::CvImage i(header, sensor_msgs::image_encodings::TYPE_32FC1, layers_[2]);
    cv_bridge::CvImage n(header, sensor_msgs::image_encodings::TYPE_32FC1, layers_[3]);

    // Kalibr cannot handle float images. Hence, we need to create an
    // integer version of the amplitude topic (while keeping the
    // original, as it is used in other contexts).

    // TODO(oalexan1): I found out that kalibr cannot handle a float
    // amplitude.  Hence, here it is multiplied by amplitude_factor and
    // cast to uint16.  The best experimental value for amplitude_factor
    // turned out to be 100, but this may need further
    // experimentation. Casting to uint8 was not enough.  This will need
    // a deeper study. If the amplitude image looks too saturated or too
    // dark, a different amplitude factor can be set when launching this.
    layers_[1].convertTo(img_int_, CV_16UC1, amplitude_factor_, 0);
    cv_bridge::CvImage a_int(header, sensor_msgs::image_encodings::MONO16, img_int_);

    // Publish individual images
    pub_d_.publish(d.toImageMsg());
    pub_a_.publish(a.toImageMsg());
    pub_i_.publish(i.toImageMsg());
    pub_c_.publish(n.toImageMsg());

    pub_a_int_.publish(a_int.toImageMsg());
  }

  // Called when depth image data arrives
  void DepthImageCallback(const sensor_msgs::ImageConstPtr& msg) {
    // Prepare distance image
    distance_.header.stamp = msg->header.stamp;
    distance_.header.frame_id = msg->header.frame_id;
    distance_.height = msg->height;
    distance_.width = msg->width;
    distance_.is_bigendian = msg->is_bigendian;
    distance_.encoding = sensor_msgs::image_encodings::TYPE_16UC1;
    distance_.step = distance_.width * sizeof(uint16_t);
    distance_.data.resize(distance_.height * distance_.step);
    // Prepare confidence image
    confidence_.header.stamp = msg->header.stamp;
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

 private:
  // Configs
  config_reader::ConfigReader config_;
  std::string config_name_;

  // Publishers
  ros::Publisher pub_d_;
  ros::Publisher pub_a_;
  ros::Publisher pub_i_;
  ros::Publisher pub_c_;
  // Subscriber
  ros::Subscriber sub_;

  ros::Publisher pub_a_int_;

  // Layer extraction
  std::vector <cv::Mat> layers_ = std::vector<cv::Mat>(4);
  cv::Mat img_int_;

  // Depth and confidence matrices
  sensor_msgs::Image distance_, confidence_;

  double amplitude_factor_;
};

PLUGINLIB_EXPORT_CLASS(pico_proxy::PicoProxyNodelet, nodelet::Nodelet);

}  // namespace pico_proxy
