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
#include <common/utils.h>
#include <config_reader/config_reader.h>
#include <gnc_ros_wrapper/ekf.h>
#include <sparse_mapping/sparse_map.h>
#include <localization_node/localization.h>
#include <lk_optical_flow/lk_optical_flow.h>
#include <image_transport/image_transport.h>
#include <ff_util/ff_names.h>

#include <Eigen/Core>
#include <gflags/gflags.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>

Eigen::Vector3f QuatToEuler(const geometry_msgs::Quaternion & q) {
  Eigen::Vector3f euler;
  float q2q2 = q.y * q.y;
  euler.x() = atan2(2 * (q.x * q.w + q.y * q.z), 1 - 2 * (q.x * q.x + q2q2));
  float arg = std::max(-1.0, std::min(1.0, 2 * (q.y * q.w - q.x * q.z)));
  euler.y() = asin(arg);
  euler.z() = atan2(2 * (q.x * q.y + q.z * q.w), 1 - 2 * (q2q2 + q.z * q.z));
  return euler;
}

Eigen::Vector3f QuatToEuler(const Eigen::Quaternionf & q) {
  Eigen::Vector3f euler;
  float q2q2 = q.y() * q.y();
  euler.x() = atan2(2 * (q.x() * q.w() + q.y() * q.z()), 1 - 2 * (q.x() * q.x() + q2q2));
  float arg = std::max(-1.0f, std::min(1.0f, 2 * (q.y() * q.w() - q.x() * q.z())));
  euler.y() = asin(arg);
  euler.z() = atan2(2 * (q.x() * q.y() + q.z() * q.w()), 1 - 2 * (q2q2 + q.z() * q.z()));
  return euler;
}

void ReadParams(localization_node::Localizer* loc) {
  config_reader::ConfigReader config;
  config.AddFile("cameras.config");
  config.AddFile("geometry.config");
  config.AddFile("localization.config");
  if (!config.ReadFiles()) {
    ROS_ERROR("Failed to read config files.");
    return;
  }

  loc->ReadParams(&config);
}

int main(int argc, char ** argv) {
  common::InitFreeFlyerApplication(&argc, &argv);

  struct timespec tstart={0, 0}, tend={0, 0};
  struct timespec tstart2={0, 0}, tend2={0, 0};
  clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &tstart);
  clock_gettime(CLOCK_MONOTONIC, &tstart2);

  if (argc < 3) {
    LOG(INFO) << "Usage: " << argv[0] << " map.map bag.bag";
    exit(0);
  }

  rosbag::Bag bag;
  bag.open(argv[2], rosbag::bagmode::Read);

  sparse_mapping::SparseMap map(argv[1], true);
  localization_node::Localizer loc(&map);

  ReadParams(&loc);

  std::vector<std::string> topics;
  topics.push_back(std::string("/") + std::string(TOPIC_HARDWARE_NAV_CAM));
  topics.push_back(std::string("/") + std::string(TOPIC_LOCALIZATION_TRUTH));
  rosbag::View view(bag, rosbag::TopicQuery(topics));

  int progress = 0;
  int images = 0;
  int failures = 0;
  int total_features = 0;
  for (rosbag::MessageInstance const m : view) {
    progress++;

    if (m.isType<sensor_msgs::Image>()) {
      sensor_msgs::ImageConstPtr image_msg = m.instantiate<sensor_msgs::Image>();
      cv_bridge::CvImageConstPtr image;
      try {
        image = cv_bridge::toCvShare(image_msg, sensor_msgs::image_encodings::MONO8);
      } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        break;
      }
      ff_msgs::VisualLandmarks vl_features;
      loc.Localize(image, &vl_features);
      if (vl_features.landmarks.size() == 0)
        failures++;
      images++;
      total_features += vl_features.landmarks.size();
    } else if (m.isType<geometry_msgs::PoseStamped>()) {
      geometry_msgs::PoseStampedConstPtr gt_msg = m.instantiate<geometry_msgs::PoseStamped>();
    }
    common::PrintProgressBar(stdout, static_cast<float>(progress) / view.size());
  }

  bag.close();
  if (images - failures == 0) {
    printf("Failed to localize any images.\n");
  } else {
    printf("Localized %d / %d images with mean of %d features.\n",
         images - failures, images, total_features / (images - failures));
    clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &tend);
    clock_gettime(CLOCK_MONOTONIC, &tend2);
    printf("Time per Frame: %.5f s\n", ((static_cast<double>(tend2.tv_sec) + 1.0e-9*tend2.tv_nsec) -
                                       (static_cast<double>(tstart2.tv_sec) + 1.0e-9*tstart2.tv_nsec)) / images);
    printf("CPU Time per Frame: %.5f s\n", ((static_cast<double>(tend.tv_sec) + 1.0e-9*tend.tv_nsec) -
                                       (static_cast<double>(tstart.tv_sec) + 1.0e-9*tstart.tv_nsec)) / images);
  }
}

