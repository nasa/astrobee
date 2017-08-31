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

// Merge n maps by merging second into the first, then the third into
// the merged map, etc. It works by finding matches among the maps
// using -num_image_overlaps_at_endpoints and then bringing the second
// map in the coordinate system of the first map. It is suggested that
// registration to real-world coordinate systems be (re-)done after
// maps are merged, as the bundle-adjustment that is required during
// merging may move things around a bit.

// outputs
DEFINE_string(output_map, "merged.map",
              "Output file containing the merged map.");

DEFINE_int32(num_image_overlaps_at_endpoints, 10,
             "Search this many images at the beginning and end of the first map "
             "for matches to this many images at the beginning and end of the "
             "second map.");

DEFINE_double(outlier_factor, 3.0,
              "The factor to use when removing outliers while finding the transform "
              "to align the two maps to merge. A smaller value will discard more "
              "outliers.");

DEFINE_bool(skip_bundle_adjustment, false,
            "If true, do not bundle adjust the merged map.");

int main(int argc, char** argv) {
  common::InitFreeFlyerApplication(&argc, &argv);
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  if (argc < 3) {
    LOG(INFO) << "Usage: " << argv[0] << " <input maps> -output-map <output map>";
    return 0;
  }

  // Ensure we don't over-write one of the inputs
  for (int i = 1; i < argc; i++) {
    if (argv[i] == FLAGS_output_map)
      LOG(FATAL) << "The input and output maps must have different names.";
  }

  if (FLAGS_num_image_overlaps_at_endpoints <= 0)
    LOG(FATAL) << "Must have num_image_overlaps_at_endpoints > 0.";

  // The merged map starts as the first map.
  {
    sparse_mapping::SparseMap A(argv[1]);
    LOG(INFO) << "Initializing " << FLAGS_output_map << " using " << argv[1] << std::endl;
    A.Save(FLAGS_output_map);
  }

  for (int i = 2; i < argc; i++) {
    LOG(INFO) << "Appending " << argv[i] << " to " << FLAGS_output_map << std::endl;
    sparse_mapping::SparseMap A(FLAGS_output_map);
    sparse_mapping::SparseMap B(argv[i]);

    // C starts as A, as SparseMap lacks an empty constructor
    sparse_mapping::SparseMap C = A;

    // Merge
    sparse_mapping::MergeMaps(&A, &B,
                              FLAGS_num_image_overlaps_at_endpoints,
                              FLAGS_outlier_factor,
                              FLAGS_output_map,
                              &C);

    // Bundle adjust the merged map.
    if (!FLAGS_skip_bundle_adjustment) {
      bool fix_cameras = false;
      sparse_mapping::BundleAdjust(fix_cameras, &C);
    }

    C.Save(FLAGS_output_map);
  }

  google::protobuf::ShutdownProtobufLibrary();

  return 0;
}
