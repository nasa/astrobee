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
  if (argc < 2) {
    std::cerr << "Usage: generate_localization_test map.nvm\n";
    std::exit(0);
  }

  // initialize map
  std::string map_file = argv[1];

  sparse_mapping::SparseMap map(map_file);

  // print results
  for (size_t i = 0; i < map.GetNumFrames() - 1; i++) {
    camera::CameraModel m1(map.GetFrameGlobalTransform(i), map.GetCameraParameters());
    camera::CameraModel m2(map.GetFrameGlobalTransform(i), map.GetCameraParameters());
    std::string name1 = map.GetFrameFilename(i);
    std::string name2 = map.GetFrameFilename(i+1);
    std::string base = name1.substr(0, name1.length() - 11);
    int num1 = stoi(name1.substr(name1.length() - 11, 7));
    int num2 = stoi(name2.substr(name2.length() - 11, 7));
    int between = (num1 + num2) / 2;
    char str[9];
    snprintf(str, sizeof(str), "%07d", between);
    std::string out = base + std::string(str) + std::string(".jpg");
    float factor = static_cast<float>(between) / (num2 - num1);
    std::cout << out;
    Eigen::Vector3d camera_pos = factor * m1.GetPosition() + (1 - factor) * m2.GetPosition();
    Eigen::Matrix3d r = factor * m1.GetRotation() + (1 - factor) * m2.GetRotation();
    printf(" (%g, %g, %g) ", camera_pos.x(), camera_pos.y(), camera_pos.z());
    printf("[%g %g %g, %g %g %g, %g %g %g]\n",
            r(0, 0), r(0, 1), r(0, 2),
            r(1, 0), r(1, 1), r(1, 2),
            r(2, 0), r(2, 1), r(2, 2));
  }

  return 0;
}
