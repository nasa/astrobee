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
#include <common/thread.h>
#include <sparse_mapping/tensor.h>
#include <sparse_mapping/sparse_map.h>
#include <sparse_mapping/sparse_mapping.h>
#include <sparse_mapping/reprojection.h>

#include <opencv2/features2d/features2d.hpp>

#include <sparse_map.pb.h>

#include <gflags/gflags.h>
#include <glog/logging.h>

#include <iostream>
#include <fstream>
#include <algorithm>
#include <thread>

// Given a map, extract a sub-map with only given images.
// Alternatively, extract only the images with camera
// centers in given box.

// Can be useful if the map failed to build properly, but parts of it
// are still salvegeable. Those can be extracted, new small maps can
// be created of the region that failed, then all maps can be merged
// together.

// It is suggested that each extracted submap consist only of consecutive
// images (each image overlaps with the one before and after it). Such
// submaps are easier to merge.

// Usage:
// extract_submaps -input_map <input map> -output_map <output map> <images to keep>
// or:
// extract_submaps -input_map <input map> -output_map <output map> -xyz_box "xmin xmax ymin ymax zmin zmax"

DEFINE_string(input_map, "",
              "The input map to use.");

DEFINE_string(output_map, "",
              "Output file containing the extracted map.");

DEFINE_string(xyz_box, "",
              "Output file containing the extracted map.");

DEFINE_bool(skip_bundle_adjustment, false,
            "If specified, in the format xmin xmax ymin ymax zmin zmax, "
            "extract the images with camera center in this box.");

int main(int argc, char** argv) {
  common::InitFreeFlyerApplication(&argc, &argv);
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  if ((argc <= 1 && FLAGS_xyz_box == "") || FLAGS_input_map == "" || FLAGS_output_map == "") {
    LOG(INFO) << "Usage: " << argv[0]
              << " -input_map <input map> -output_map <output map> [ <images to keep> ] "
              << "[ -xyz_box str]";
    return 0;
  }

  // Start with the output being the same as the input
  sparse_mapping::SparseMap out(FLAGS_input_map);

  std::vector<std::string> images_to_keep;
  for (int i = 1; i < argc; i++) {
    images_to_keep.push_back(argv[i]);
  }

  // Alternatively, extract images form this box
  if (FLAGS_xyz_box != "") {
    std::istringstream is(FLAGS_xyz_box);
    double xmin, xmax, ymin, ymax, zmin, zmax;
    if (!(is >> xmin >> xmax >> ymin >> ymax >> zmin >> zmax)) {
      LOG(FATAL) << "Could not parse xyz box.";
    }

    images_to_keep.clear();
    for (size_t cid = 0; cid < out.cid_to_filename_.size(); cid++) {
      Eigen::Vector3d ctr = out.GetFrameGlobalTransform(cid).inverse().translation();
      if (ctr[0] >= xmin && ctr[0] <= xmax &&
          ctr[1] >= ymin && ctr[1] <= ymax &&
          ctr[2] >= zmin && ctr[2] <= zmax) {
        images_to_keep.push_back(out.cid_to_filename_[cid]);
      }
    }
  }

  sparse_mapping::ExtractSubmap(&images_to_keep, &out);

  if (!FLAGS_skip_bundle_adjustment) {
    // Bundle adjust the extracted submap
    bool fix_cameras = false;
    BundleAdjust(fix_cameras, &out);
  }

  out.Save(FLAGS_output_map);

  google::protobuf::ShutdownProtobufLibrary();

  return 0;
}
