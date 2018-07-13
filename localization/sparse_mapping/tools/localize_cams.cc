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
#include <sparse_mapping/sparse_map.h>
#include <sparse_mapping/sparse_mapping.h>
#include <sparse_mapping/reprojection.h>

#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <sparse_map.pb.h>

#include <gflags/gflags.h>
#include <glog/logging.h>

#include <iostream>
#include <fstream>
#include <algorithm>
#include <thread>

// This is a test executable, a way of validating the accuracy of
// localization.

// Given two sets of images of the same indoor environment, create two
// maps ready for localization. That is, maps are built, registered to
// the world coordinate system, rebuilt with BRISK, and then a
// vocabulary database is created. Name those maps reference and source.

// For each source image, localize it against the reference map,
// and compare its camera position and orientation after localization
// with the "known" position and orientation from the source map.

// This is not a fool-proof test, since neither of the two maps
// contains ground truth, rather a simulated version of it, yet it can
// be useful if each individual map is reasonably accurate and
// well-registered.

DEFINE_string(reference_map, "",
              "Reference map to localize against.");
DEFINE_string(source_map, "",
              "Localize images in this map against the reference computed map, "
              "and compare with locations from this map.");

// These are synched up with localization.config. Note that
// -num_similar and -ransac_inlier_tolerance need not be defined as
// flags here, since they already exist in sparse_map.cc.
DEFINE_int32(ransac_iterations, 100,
             "Number of iterations of RANSAC.");
DEFINE_int32(min_features, 200,
             "Minimum number of features to be computed by the feature detector.");
DEFINE_int32(max_features, 800,
             "Maximum number of features to be computed by the feature detector.");
DEFINE_int32(brisk_threshold, 100,
             "The threshold for the BRISK feature detector.");
DEFINE_int32(detection_retries, 2,
             "The number of attempts to acquire the desired number of features with the detector.");

int main(int argc, char** argv) {
  common::InitFreeFlyerApplication(&argc, &argv);
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  sparse_mapping::SparseMap reference(FLAGS_reference_map);
  reference.SetRansacIterations(FLAGS_ransac_iterations);
  reference.SetDetectorParams(FLAGS_min_features, FLAGS_max_features,
                              FLAGS_brisk_threshold, FLAGS_detection_retries);

  Eigen::IOFormat CSVFormat(3, 0, ", ", ",   ");

  sparse_mapping::SparseMap source(FLAGS_source_map);

  if ( !(source.GetCameraParameters() == reference.GetCameraParameters()) )
    LOG(FATAL) << "The source and reference maps don't have the same camera parameters.";

  for (size_t cid = 0; cid < source.GetNumFrames(); cid++) {
    std::string img_file = source.GetFrameFilename(cid);

    // localize frame
    camera::CameraModel localized_cam(Eigen::Vector3d(), Eigen::Matrix3d::Identity(),
                               reference.GetCameraParameters());
    if (!reference.Localize(img_file, &localized_cam)) {
      // Localization failed
      std::cout << "Errors for " << img_file << ": "
                << 1e+6 << " m " << 1e+6 << " degrees" << std::endl;
      continue;
    }

    camera::CameraModel source_cam(source.GetFrameGlobalTransform(cid),
                                   source.GetCameraParameters());
    Eigen::Vector3d expected_angle  = source_cam.GetRotation() * Eigen::Vector3d::UnitX();
    Eigen::Vector3d estimated_angle = localized_cam.GetRotation()   * Eigen::Vector3d::UnitX();
    double angle_err = acos(estimated_angle.dot(expected_angle)) * (180.0 / M_PI);

    std::cout << "Source map position:         " << source_cam.GetPosition().transpose() << "\n";
    std::cout << "Localized position from ref: " << localized_cam.GetPosition().transpose() << "\n";
    std::cout << "Errors for " << img_file << ": "
              << (localized_cam.GetPosition() - source_cam.GetPosition()).norm()
              << " m " << angle_err << " degrees" << "\n";
  }

  google::protobuf::ShutdownProtobufLibrary();

  return 0;
}
