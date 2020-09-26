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
#include <gflags/gflags.h>
#include <glog/logging.h>

#include <string>
#include <vector>

DECLARE_bool(alsologtostderr);
DECLARE_bool(colorlogtostderr);

namespace ff_common {
  bool logging_initialized = false;

  void InitFreeFlyerApplication(int* argc, char*** argv) {
    // We will probably always want to log to stderr and not just to buffers
    FLAGS_alsologtostderr = true;
    FLAGS_colorlogtostderr = true;

    // Parse the command line options
    if (!logging_initialized) {
      FLAGS_minloglevel = 0;
      FLAGS_v = 5;
      google::SetLogDestination(google::INFO, "/data/ryan_data/glog/info");
      google::SetLogDestination(google::ERROR, "/data/ryan_data/glog/error");
      google::SetLogDestination(google::WARNING, "/data/ryan_data/glog/warning");
      google::InitGoogleLogging((*argv)[0]);
      logging_initialized = true;
    }
    FREEFLYER_GFLAGS_NAMESPACE::ParseCommandLineFlags(argc, argv, true);
  }

  // ros nodelets get commandline arguments in this form...
  // this function lets us still pass them to gflags
  // main_thread must be true for at most one nodelet in same manager!!!
  // otherwise it will crash
  void InitFreeFlyerApplication(std::vector<std::string> args, bool main_thread) {
    int argc = args.size() + 1;
    const char** argv = (const char**)malloc(sizeof(const char*) * argc);
    argv[0] = "UNKNOWN";
    for (int i = 1; i < argc; i++)
      argv[i] = args[i-1].c_str();
    char** casted_argv = const_cast<char**>(reinterpret_cast<const char**>(argv));
    if (main_thread) {
      InitFreeFlyerApplication(&argc, &casted_argv);
    } else {
      FREEFLYER_GFLAGS_NAMESPACE::ParseCommandLineFlags(&argc, &casted_argv, true);
    }
    free(argv);
  }

  const char* GetConfigDir(void) {
    char* p = getenv("ASTROBEE_CONFIG_DIR");
    if (p == NULL) {
      fprintf(stderr, "ASTROBEE_CONFIG_DIR not set.\n");
      exit(-1);
    }
    return p;
  }

}  // namespace ff_common
