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

#include <Eigen/Geometry>
#include <gflags/gflags.h>
#include <glog/logging.h>

#include <fstream>

DEFINE_bool(show_delta_affines, false,
            "Instead of printing the camera location as an affine,"
            " show the affine between the cameras. Should allow for"
            " easier comparison between map solutions.");
DEFINE_string(show_spec_triplet, "",
              "Specify a triplet that you want to see, ex: '0_3_43'");

int main(int argc, char** argv) {
  common::InitFreeFlyerApplication(&argc, &argv);

  // We require a map input
  if (argc < 2) {
    LOG(ERROR) << "Usage: " << argv[0] << " map.map";
    return 1;
  }

  // Check if input exists
  std::ifstream f(argv[1]);
  if (!f.good()) {
    LOG(FATAL) << "File does not exist: " << argv[1];
  }

  // Load up the map file
  std::string map_file = argv[1];
  sparse_mapping::SparseMap map(map_file);
  LOG(INFO) << "Loaded " << argv[1];
  LOG(INFO) << "\t" << map.GetNumFrames() << " Cameras and "
            << map.GetNumLandmarks() << " Points";

  const Eigen::IOFormat csv_format(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", ", ");

  if (FLAGS_show_delta_affines) {
    std::ofstream of(map_file + ".delta.csv");
    double scaling = 0;
    for (size_t cid = 0; cid < map.GetNumFrames() - 1; cid++) {
      Eigen::Affine3d cam_n_t_global = map.GetFrameGlobalTransform(cid);
      Eigen::Affine3d cam_n1_t_global = map.GetFrameGlobalTransform(cid + 1);
      Eigen::Affine3d delta = (cam_n1_t_global * cam_n_t_global.inverse());

      if (cid == 0) {
        scaling = 1.0 / delta.translation().norm();
      }
      delta.translation() *= scaling;

      LOG(INFO) << "CID " << cid << " cid_n+1_t_cid_n:\n"
                << delta.matrix();
      of << delta.matrix().format(csv_format) << std::endl;
    }
  } else if (!FLAGS_show_spec_triplet.empty()) {
    std::vector<std::string> tokens;
    std::istringstream iss(FLAGS_show_spec_triplet);
    std::string holder;
    while (std::getline(iss, holder, '_')) {
      tokens.push_back(holder);
    }
    if (tokens.size() != 3)
      LOG(FATAL) << "Found incorrect number of tokens: " << tokens.size();

    int cid1 = std::stoi(tokens[0]);
    int cid2 = std::stoi(tokens[1]);
    int cid3 = std::stoi(tokens[2]);

    Eigen::Affine3d
      j_t_i = map.GetFrameGlobalTransform(cid2) * map.GetFrameGlobalTransform(cid1).inverse(),
      k_t_i = map.GetFrameGlobalTransform(cid3) * map.GetFrameGlobalTransform(cid1).inverse(),
      k_t_j = map.GetFrameGlobalTransform(cid3) * map.GetFrameGlobalTransform(cid2).inverse();
    double scale = 1.0 / j_t_i.translation().norm();
    j_t_i.translation() *= scale;
    k_t_i.translation() *= scale;
    k_t_j.translation() *= scale;

    LOG(INFO) << "Affine: " << cid1 << " " << cid2 << "\n"
              << j_t_i.matrix();
    LOG(INFO) << "Affine: " << cid1 << " " << cid3 << "\n"
              << k_t_i.matrix();
    LOG(INFO) << "Affine: " << cid2 << " " << cid3 << "\n"
              << k_t_j.matrix();
  } else {
    std::ofstream of(map_file + ".abs.csv");
    for (size_t cid = 0; cid < map.GetNumFrames(); cid++) {
      Eigen::Affine3d cam_t_global = map.GetFrameGlobalTransform(cid);
      LOG(INFO) << "CID " << cid << " cam_t_global:\n"
                << cam_t_global.matrix();
      of << cam_t_global.matrix().format(csv_format) << std::endl;
    }
  }

  return 0;
}
