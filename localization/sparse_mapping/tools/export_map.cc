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
DEFINE_string(output_map, "output.nvm",
              "Output sparse map in NVM format.");

int main(int argc, char** argv) {
  ff_common::InitFreeFlyerApplication(&argc, &argv);
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  if (FLAGS_input_map == "" || FLAGS_output_map == "")
    LOG(FATAL) << "The input and output maps were not specified.\n";

  sparse_mapping::SparseMap map(FLAGS_input_map);

  std::cout << "Writing: " << FLAGS_output_map << std::endl;
  sparse_mapping::WriteNVM(map.cid_to_keypoint_map_,
                           map.cid_to_filename_,
                           map.pid_to_cid_fid_,
                           map.pid_to_xyz_,
                           map.cid_to_cam_t_global_,
                           map.camera_params_.GetFocalLength(),
                           FLAGS_output_map);

  google::protobuf::ShutdownProtobufLibrary();
  return 0;
}
