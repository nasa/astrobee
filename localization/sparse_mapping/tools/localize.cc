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
#include <sparse_mapping/sparse_map.h>
#include <sparse_mapping/reprojection.h>

#include <gflags/gflags.h>
#include <glog/logging.h>

#include <thread>

int main(int argc, char** argv) {
  common::InitFreeFlyerApplication(&argc, &argv);
  if (argc < 3) {
    std::cerr << "Usage: localize map.nvm image.jpg\n";
    std::exit(0);
  }

  // initialize map
  std::string map_file = argv[1];
  std::string img_file = argv[2];
  sparse_mapping::SparseMap map(map_file);

  // localize frame
  camera::CameraModel camera(Eigen::Vector3d(), Eigen::Matrix3d::Identity(), map.GetCameraParameters());
  if (!map.Localize(img_file, &camera)) {
    LOG(ERROR) << "Failed to localize image.";
    return 1;
  }

  // print results
  Eigen::IOFormat CSVFormat(3, 0, ", ", ",   ");
  // for (size_t i = 0; i < map.GetNumFrames(); i++) {
  // camera::CameraModel m(map.GetFrameGlobalTransform(i), map.GetCameraParameters());
  // std::cout << map.GetFrameFilename(i);
  // Eigen::Vector3d camera_pos = m.GetPosition();
  // printf(" map position: (%10.7f, %10.7f, %10.7f) ", camera_pos.x(), camera_pos.y(), camera_pos.z());
  // std::cout << "rotation: (" << m.GetRotation().matrix().format(CSVFormat) << ")\n";
  // }
  // std::cout << "-----------------------------------" << std::endl;
  std::cout << "Localization" << std::endl;
  std::cout << img_file;
  Eigen::Vector3d camera_pos = camera.GetPosition();
  printf(" localized position: (%10.7f, %10.7f, %10.7f) ", camera_pos.x(), camera_pos.y(), camera_pos.z());
  std::cout << "rotation: (" << camera.GetRotation().matrix().format(CSVFormat) << ")\n";

  return 0;
}
