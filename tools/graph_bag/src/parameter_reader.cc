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

#include <graph_bag/parameter_reader.h>
#include <localization_common/utilities.h>
#include <msg_conversions/msg_conversions.h>

namespace graph_bag {
namespace lc = localization_common;
namespace mc = msg_conversions;

void LoadMessageBufferParams(const std::string& message_type, config_reader::ConfigReader& config,
                             MessageBufferParams& params) {
  params.msg_delay = mc::LoadDouble(config, message_type + "_msg_delay");
  params.min_msg_spacing = mc::LoadDouble(config, message_type + "_min_msg_spacing");
}

void LoadLiveMeasurementSimulatorParams(config_reader::ConfigReader& config, const std::string& bag_name,
                                        const std::string& map_file, const std::string& image_topic,
                                        LiveMeasurementSimulatorParams& params) {
  LoadMessageBufferParams("imu", config, params.imu);
  LoadMessageBufferParams("flight_mode", config, params.flight_mode);
  LoadMessageBufferParams("of", config, params.of);
  LoadMessageBufferParams("vl", config, params.vl);
  LoadMessageBufferParams("ar", config, params.ar);
  LoadMessageBufferParams("sm", config, params.sm);
  params.bag_name = bag_name;
  params.map_file = map_file;
  params.image_topic = image_topic;
  params.save_images = mc::LoadBool(config, "save_optical_flow_images") ||
                       mc::LoadBool(config, "save_semantic_matches_images");
}

void LoadCameraDistMap(GraphBagParams& params) {
  const auto& img_size = params.nav_cam_params->GetUndistortedSize();
  cv::Size img_size_cv(img_size[0], img_size[1]);

  params.undist_map_x = cv::Mat(img_size_cv, CV_32FC1);
  params.undist_map_y = cv::Mat(img_size_cv, CV_32FC1);

  for (int x=0; x<img_size_cv.width; x++) {
    for (int y=0; y<img_size_cv.height; y++) {
      Eigen::Vector2d undistorted_point(x, y);
      Eigen::Vector2d distorted_point;
      params.nav_cam_params->Convert<camera::UNDISTORTED, camera::DISTORTED>(undistorted_point, &distorted_point);
      params.undist_map_x.at<float>(y, x) = distorted_point[0];
      params.undist_map_y.at<float>(y, x) = distorted_point[1];
    }
  }
}

void LoadGraphLocalizerSimulatorParams(config_reader::ConfigReader& config, GraphLocalizerSimulatorParams& params) {
  params.optimization_time = mc::LoadDouble(config, "optimization_time");
}

void LoadGraphBagParams(config_reader::ConfigReader& config, GraphBagParams& params) {
  params.save_optical_flow_images = mc::LoadBool(config, "save_optical_flow_images");
  params.save_semantic_matches_images = mc::LoadBool(config, "save_semantic_matches_images");
  params.show_semantic_matches_images = mc::LoadBool(config, "show_semantic_matches_images");
  params.log_relative_time = mc::LoadBool(config, "log_relative_time");
  params.nav_cam_params.reset(new camera::CameraParameters(&config, "nav_cam"));
  params.body_T_nav_cam = lc::LoadTransform(config, "nav_cam_transform");
  params.sparse_mapping_min_num_landmarks = mc::LoadInt(config, "loc_adder_min_num_matches");
  params.ar_min_num_landmarks = mc::LoadInt(config, "ar_tag_loc_adder_min_num_matches");
  LoadCameraDistMap(params);
}
}  // namespace graph_bag
