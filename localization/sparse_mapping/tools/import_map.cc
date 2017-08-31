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

#include <sparse_map.pb.h>

#include <gflags/gflags.h>
#include <glog/logging.h>

#include <iostream>
#include <fstream>
#include <algorithm>
#include <thread>

// outputs
DEFINE_string(input_map, "",
              "text file.");
DEFINE_string(output_map, "output.map",
              "Output file containing the matches and control network.");
DEFINE_bool(bundler_map, false,
            "If true, read the bundler or theia format.");

int main(int argc, char** argv) {
  common::InitFreeFlyerApplication(&argc, &argv);
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  std::vector<std::string> files;
  for (int i = 1; i < argc; i++) {
    std::cout << "adding " << argv[i] << std::endl;
    files.push_back(argv[i]);
  }

  std::cout << "input and output " << FLAGS_input_map << ' ' << FLAGS_output_map << std::endl;
  sparse_mapping::SparseMap map(FLAGS_bundler_map, FLAGS_input_map, files);
  map.Save(FLAGS_output_map);

  google::protobuf::ShutdownProtobufLibrary();

  return 0;
}
