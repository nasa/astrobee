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
#include <ff_common/init.h>
#include <ff_common/thread.h>
#include <ff_common/utils.h>
#include <sparse_mapping/tensor.h>
#include <sparse_mapping/sparse_map.h>
#include <sparse_mapping/sparse_mapping.h>

#include <sparse_map.pb.h>

#include <gflags/gflags.h>
#include <glog/logging.h>

#include <iostream>
#include <fstream>
#include <algorithm>
#include <thread>

DEFINE_string(input_map, "",
              "Input map created with undistorted images, in a text file.");
DEFINE_string(output_map, "output.map",
              "Output sparse map as expected by Astrobee software.");
DEFINE_bool(bundler_map, false,
            "If true, read the Bundler format. This will be ignored for input .nvm files.");
DEFINE_string(undistorted_camera_params, "",
              "Intrinsics of the undistorted camera. Specify as: "
              "'wid_x wid_y focal_len opt_ctr_x opt_ctr_y'.");
DEFINE_string(distorted_images_list, "",
              "Replace the undistorted images specified on input with distorted images "
              "from this list. The correct value of ASTROBEE_ROBOT must be set.");

// Replace the undistorted images which Theia used with distorted images
void replaceWithDistortedImages(std::vector<std::string> const& undist_images,
                                std::string const& distorted_images_list,
                                sparse_mapping::SparseMap & map) {
  // Must overwrite the camera params for the distorted images
  std::cout << "Using distorted camera parameters for nav_cam for robot: "
            << getenv("ASTROBEE_ROBOT") << ".\n";
  config_reader::ConfigReader config;
  config.AddFile("cameras.config");
  if (!config.ReadFiles()) LOG(FATAL) << "Failed to read config files.\n";

  camera::CameraParameters cam_params(&config, "nav_cam");
  map.SetCameraParameters(cam_params);

  // Replace the undistorted images with distorted ones
  std::vector<std::string> distorted_images;
  std::string image;
  std::ifstream ifs(distorted_images_list.c_str());
  while (ifs >> image)
    distorted_images.push_back(image);

  if (undist_images.size() != distorted_images.size())
    LOG(FATAL) << "The number of distorted images in the list and undistorted ones "
               << "passed on the command line must be the same.\n";

  std::map<std::string, std::string> undist_to_dist;
  for (size_t it = 0; it < undist_images.size(); it++) undist_to_dist[undist_images[it]] = distorted_images[it];

  // Replace the images in the map. Keep in mind that the map
  // may have just a subset of the input images.
  for (size_t it = 0; it < map.cid_to_filename_.size(); it++) {
    auto map_it = undist_to_dist.find(map.cid_to_filename_[it]);
    if (map_it == undist_to_dist.end())
      LOG(FATAL) << "This map image was not specified on input: "
                 << map.cid_to_filename_[it] << ".\n";
    map.cid_to_filename_[it] = map_it->second;
  }
}

int main(int argc, char** argv) {
  ff_common::InitFreeFlyerApplication(&argc, &argv);
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  // Read the images from the command line
  std::vector<std::string> undist_images;
  for (int i = 1; i < argc; i++)
    undist_images.push_back(argv[i]);

  std::cout << "Reading map: " << FLAGS_input_map << std::endl;
  sparse_mapping::SparseMap map(FLAGS_bundler_map, FLAGS_input_map, undist_images);

  if (FLAGS_distorted_images_list == "") {
    // Must overwrite the camera parameters with what is passed on input
    std::vector<double> vals;
    ff_common::parseStr(FLAGS_undistorted_camera_params, vals);
    if (vals.size() < 5)
      LOG(FATAL) << "Could not parse --undistorted_camera_params.\n";
    double widx = vals[0], widy = vals[1], f = vals[2], cx = vals[3], cy = vals[4];
    LOG(INFO) << "Using undistorted camera parameters: "
              << widx << ' ' << widy << ' ' << f << ' ' << cx << ' ' << cy;
    camera::CameraParameters cam_params = camera::CameraParameters(Eigen::Vector2i(widx, widy),
                                                                   Eigen::Vector2d::Constant(f),
                                                                   Eigen::Vector2d(cx, cy));
    map.SetCameraParameters(cam_params);
  } else {
    replaceWithDistortedImages(undist_images, FLAGS_distorted_images_list, map);
  }

  std::cout << "Writing map: " << FLAGS_output_map << std::endl;
  map.Save(FLAGS_output_map);

  google::protobuf::ShutdownProtobufLibrary();

  return 0;
}
