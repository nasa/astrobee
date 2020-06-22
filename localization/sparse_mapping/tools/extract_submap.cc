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

// Given a map, extract a sub-map with only specified images.
// Alternatively, extract only the images with given indices (first
// image has index 0). Alternatively, extract the images with the
// camera centers in given box.

// Can be useful if the map failed to build properly, but parts of it
// are still salvegeable. Those can be extracted, new small maps can
// be created of the region that failed, then all maps can be merged
// together.

// It is suggested that each extracted submap consist only of consecutive
// images (each image overlaps with the one before and after it). Such
// submaps are easier to merge.

// Usage:
// extract_submap -input_map <input map> -output_map <output map> <images to keep>
//
// extract_submap -input_map <input map> -output_map <output map> -image_list <file>
// or
// extract_submap -input_map <input map> -output_map <output map> -exclude <images to exclude>
// or
// extract_submap -input_map <input map> -output_map <output map> -cid_range "min_cid max_cid"
// or:
// extract_submap -input_map <input map> -output_map <output map> -xyz_box "xmin xmax ymin ymax zmin zmax"

DEFINE_string(input_map, "",
              "Input map.");

DEFINE_string(output_map, "",
              "Output map.");

DEFINE_string(cid_range, "",
              "Keep only the images with indices in this range (inclusive at both ends).");

DEFINE_string(xyz_box, "",
              "Keep only the images with the camera center in this box. "
              "Use the format: 'xmin xmax ymin ymax zmin zmax'.");

DEFINE_string(image_list, "",
              "Instead of the images being specified on the command line, "
              "read them from a file (one per line).");

DEFINE_bool(skip_bundle_adjustment, false,
            "Skip performing bundle adjustment on the extracted submap.");

DEFINE_bool(exclude, false,
            "Extract the images not provided as input, which is the "
            "opposite of the default behavior.");

int main(int argc, char** argv) {
  ff_common::InitFreeFlyerApplication(&argc, &argv);
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  if ((argc <= 1 && FLAGS_cid_range == "" && FLAGS_xyz_box == "" && FLAGS_image_list == "") ||
      FLAGS_input_map == "" || FLAGS_output_map == "") {
    LOG(INFO) << "Usage: " << argv[0]
              << " -input_map <input map> -output_map <output map> [ -exclude ] [ <images> ] "
              << "[ -image_list ] [ -cid_range 'beg end' ] "
              << "[ -xyz_box 'xmin xmax ymin ymax zmin zmax' ]";
    return 0;
  }

  std::vector<std::string> images;
  if (FLAGS_image_list == "") {
    // Get the images from the command line
    for (int i = 1; i < argc; i++)
      images.push_back(argv[i]);
  } else {
    // Get the images from a file
    std::string image;
    std::ifstream image_handle(FLAGS_image_list);
    while (image_handle >> image)
      images.push_back(image);
  }

  // Start with the output being the same as the input
  sparse_mapping::SparseMap out(FLAGS_input_map);

  std::vector<std::string> images_to_keep;
  if (FLAGS_cid_range == "" && FLAGS_xyz_box == "") {
    if (!FLAGS_exclude) {
      images_to_keep = images;
    } else {
      // Exclude the specified images
      std::set<std::string> exclude;
      for (size_t it = 0; it < images.size(); it++)
        exclude.insert(images[it]);
      for (size_t cid = 0; cid < out.cid_to_filename_.size(); cid++) {
        if (exclude.find(out.cid_to_filename_[cid]) == exclude.end())
          images_to_keep.push_back(out.cid_to_filename_[cid]);
      }
    }
  } else if (FLAGS_cid_range != "") {
    // Keep the images in the given cid range
    std::istringstream is(FLAGS_cid_range);
    size_t min_cid, max_cid;
    if (!(is >> min_cid >> max_cid)) {
        LOG(FATAL) << "Could not parse the cid range.";
    }

    images_to_keep.clear();
    for (size_t cid = 0; cid < out.cid_to_filename_.size(); cid++) {
      if (cid >= min_cid && cid <= max_cid) {
        images_to_keep.push_back(out.cid_to_filename_[cid]);
      }
    }
  } else if (FLAGS_xyz_box != "") {
    // Alternatively, extract images form this box
    std::istringstream is(FLAGS_xyz_box);
    double xmin, xmax, ymin, ymax, zmin, zmax;
    if (!(is >> xmin >> xmax >> ymin >> ymax >> zmin >> zmax)) {
      LOG(FATAL) << "Could not parse the xyz box.";
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
