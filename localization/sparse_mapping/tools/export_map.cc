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

// Export a map to nvm format.
// Can be imported back with import_map

DEFINE_string(input_map, "",
              "Input sparse map file, in Astrobee's protobuf format, with .map extension.");
DEFINE_string(output_map, "",
              "Output sparse map in NVM format.");
DEFINE_bool(
  no_shift, false,
  "Save the features without shifting them relative to the optical center. That makes visualizing them easier.");

int main(int argc, char** argv) {
  ff_common::InitFreeFlyerApplication(&argc, &argv);
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  if (FLAGS_input_map == "" || FLAGS_output_map == "")
    LOG(FATAL) << "The input and output maps were not specified.\n";

  sparse_mapping::SparseMap map(FLAGS_input_map);

  camera::CameraParameters camera_params = map.GetCameraParameters();

  // This is very important,
  std::cout << "Saving the nvm file with interest point matches that ";
  if (FLAGS_no_shift)
    std::cout << "are NOT";
  else
    std::cout << "ARE";
  std::cout << " shifted relative to the optical center." << std::endl;

  Eigen::Vector2d optical_center = camera_params.GetOpticalOffset();
  Eigen::Vector2d dist_pix;

  for (size_t cid = 0; cid < map.cid_to_keypoint_map_.size(); cid++) {
    for (int fid = 0; fid < map.cid_to_keypoint_map_[cid].cols(); fid++) {
      // Distort and de-center
      camera_params.Convert<camera::UNDISTORTED_C, camera::DISTORTED>
        (map.cid_to_keypoint_map_[cid].col(fid), &dist_pix);

      if (!FLAGS_no_shift) {
        // Apply the shift. The Astrobee map had the shift relative to
        // image center, but here the shift is relative to optical
        // center.
        dist_pix -= optical_center;
      }

      map.cid_to_keypoint_map_[cid].col(fid) = dist_pix;
    }
  }

  std::cout << "Writing: " << FLAGS_output_map << std::endl;
  sparse_mapping::WriteNVM(map.cid_to_keypoint_map_,
                           map.cid_to_filename_,
                           map.pid_to_cid_fid_,
                           map.pid_to_xyz_,
                           map.cid_to_cam_t_global_,
                           camera_params.GetFocalLength(),
                           FLAGS_output_map);

  google::protobuf::ShutdownProtobufLibrary();
  return 0;
}
