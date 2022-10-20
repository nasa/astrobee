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
DEFINE_string(output_map, "",
              "Output sparse map as expected by Astrobee software.");
DEFINE_bool(no_shift, false,
            "Import the features without shifting them relative to the optical center.");
DEFINE_bool(bundler_map, false,
            "If true, read the Bundler format. This will be ignored for input .nvm files.");
DEFINE_string(undistorted_camera_params, "",
              "Intrinsics of the undistorted camera. Not needed if --distorted_images_list "
              "is specified, as then the camera is set via ASTROBEE_ROBOT. Specify as: "
              "'wid_x wid_y focal_len opt_ctr_x opt_ctr_y'.");
DEFINE_string(undistorted_images_list, "",
              "The full list of undistorted images used to create the sparse map. "
              "If not specified, the images should be passed on the command line. "
              "Note that not all of them may have been used in the map.");
DEFINE_string(distorted_images_list, "",
              "Replace the undistorted images specified on input with distorted images "
              "from this list (one file per line). The correct value of ASTROBEE_ROBOT "
              "must be set.");
namespace {
  // Keep these utilities in a local namespace

  void readLines(std::string const& list, std::vector<std::string> & lines) {
    lines.clear();
    std::string line;
    std::ifstream ifs(list.c_str());
    while (ifs >> line)
      lines.push_back(line);
  }

  // Replace the undistorted images which Theia used with distorted images.
  // The interest points need not be modified as those are always undistorted
  // when saved, even if the map has distorted images.
  void replaceWithDistortedImages(std::vector<std::string> const& undist_images,
                                  std::string const& distorted_images_list,
                                  sparse_mapping::SparseMap & map) {
    // Must overwrite the camera params for the distorted images
    std::cout << "Using distorted camera parameters for nav_cam for robot: "
              << getenv("ASTROBEE_ROBOT") << ".\n";
    config_reader::ConfigReader config;
    config.AddFile("cameras.config");
    if (!config.ReadFiles()) LOG(FATAL) << "Failed to read config files.\n";

    // Save the camera parameters for the given robot.
    camera::CameraParameters camera_params(&config, "nav_cam");
    map.SetCameraParameters(camera_params);

    // Replace the undistorted images with distorted ones
    std::vector<std::string> distorted_images;
    readLines(distorted_images_list, distorted_images);

    if (undist_images.size() != distorted_images.size())
      LOG(FATAL) << "The number of distorted images in the list and undistorted ones "
                 << "passed on the command line must be the same.\n";

    std::map<std::string, std::string> undist_to_dist;
    for (size_t it = 0; it < undist_images.size(); it++)
      undist_to_dist[undist_images[it]] = distorted_images[it];

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
}  // namespace

int main(int argc, char** argv) {
  ff_common::InitFreeFlyerApplication(&argc, &argv);
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  if (FLAGS_input_map == "" || FLAGS_output_map == "")
    LOG(FATAL) << "The input and output maps were not specified.\n";

  if (FLAGS_undistorted_images_list   == "" &&
      FLAGS_distorted_images_list     == "" &&
      FLAGS_undistorted_camera_params == "") {
    std::cout << "No extra options were specified. Assuming that the sparse map "
              << "was made with distorted (original) images.\n" << std::endl;

    config_reader::ConfigReader config;
    config.AddFile("cameras.config");
    if (!config.ReadFiles()) {
      LOG(FATAL) << "Failed to read the robot configuration.";
      return 1;
    }
    camera::CameraParameters camera_params(&config, "nav_cam");

    // Create a sparse map with the given camera parameters
    sparse_mapping::SparseMap map(std::vector<std::string>(), "SURF", camera_params);
    sparse_mapping::ReadNVM(FLAGS_input_map, &map.cid_to_keypoint_map_,
                            &map.cid_to_filename_, &map.pid_to_cid_fid_,
                            &map.pid_to_xyz_, &map.cid_to_cam_t_global_);

    Eigen::Vector2d optical_center = camera_params.GetOpticalOffset();
    Eigen::Vector2d dist_pix, undist_pix;

    for (size_t cid = 0; cid < map.cid_to_keypoint_map_.size(); cid++) {
      Eigen::Vector2d dist_pix;
      for (int fid = 0; fid < map.cid_to_keypoint_map_[cid].cols(); fid++) {
        dist_pix = map.cid_to_keypoint_map_[cid].col(fid);

        if (!FLAGS_no_shift) {
          // Undo the shift relative to the optical center
          dist_pix += optical_center;
        }

        // Undistort and center. Write the result to the map.
        camera_params.Convert<camera::DISTORTED, camera::UNDISTORTED_C>
          (dist_pix, &undist_pix);
        map.cid_to_keypoint_map_[cid].col(fid) = undist_pix;
      }
    }

    // Descriptors are not saved, so let them be empty
    map.cid_to_descriptor_map_.resize(map.cid_to_keypoint_map_.size());

    map.Save(FLAGS_output_map);

    google::protobuf::ShutdownProtobufLibrary();

    return 0;
  }

  if (FLAGS_no_shift)
    LOG(FATAL) << "The --no_shift option does not work with undistorted images.\n";

  // Read the images from a list file or individually specified on the command line
  std::vector<std::string> undist_images;
  if (FLAGS_undistorted_images_list != "") {
    readLines(FLAGS_undistorted_images_list, undist_images);
  } else {
    for (int i = 1; i < argc; i++)
      undist_images.push_back(argv[i]);
  }

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

    // Save the undistorted params in the map
    camera::CameraParameters camera_params
      = camera::CameraParameters(Eigen::Vector2i(widx, widy),
                                 Eigen::Vector2d::Constant(f),
                                 Eigen::Vector2d(cx, cy));
    map.SetCameraParameters(camera_params);

  } else {
    // Replace the images and robot camera params with distorted (original) ones
    replaceWithDistortedImages(undist_images, FLAGS_distorted_images_list, map);
  }

  std::cout << "Writing map: " << FLAGS_output_map << std::endl;
  map.Save(FLAGS_output_map);

  google::protobuf::ShutdownProtobufLibrary();

  return 0;
}
