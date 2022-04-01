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

#ifndef LOCALIZATION_ANALYSIS_DEPTH_ODOMETRY_ADDER_H_
#define LOCALIZATION_ANALYSIS_DEPTH_ODOMETRY_ADDER_H_

#include <depth_odometry/depth_odometry_wrapper.h>

// TODO(rsoussan): There is a linking error on melodic for conflicting declarations of lz4
// for flann serialization and ros serialization that occurs when this header is included.
// Once this is fixed (ros version > melodic) these defines and undefines can be removed.
#define LZ4_stream_t LZ4_stream_t_deprecated
#define LZ4_resetStream LZ4_resetStream_deprecated
#define LZ4_createStream LZ4_createStream_deprecated
#define LZ4_freeStream LZ4_freeStream_deprecated
#define LZ4_loadDict LZ4_loadDict_deprecated
#define LZ4_compress_fast_continue LZ4_compress_fast_continue_deprecated
#define LZ4_saveDict LZ4_saveDict_deprecated
#define LZ4_streamDecode_t LZ4_streamDecode_t_deprecated
#define LZ4_compress_continue LZ4_compress_continue_deprecated
#define LZ4_compress_limitedOutput_continue LZ4_compress_limitedOutput_continue_deprecated
#define LZ4_createStreamDecode LZ4_createStreamDecode_deprecated
#define LZ4_freeStreamDecode LZ4_freeStreamDecode_deprecated
#define LZ4_setStreamDecode LZ4_setStreamDecode_deprecated
#define LZ4_decompress_safe_continue LZ4_decompress_safe_continue_deprecated
#define LZ4_decompress_fast_continue LZ4_decompress_fast_continue_deprecated
#include <rosbag/view.h>
#undef LZ4_stream_t
#undef LZ4_resetStream
#undef LZ4_createStream
#undef LZ4_freeStream
#undef LZ4_loadDict
#undef LZ4_compress_fast_continue
#undef LZ4_saveDict
#undef LZ4_streamDecode_t
#undef LZ4_compress_continue
#undef LZ4_compress_limitedOutput_continue
#undef LZ4_createStreamDecode
#undef LZ4_freeStreamDecode
#undef LZ4_setStreamDecode
#undef LZ4_decompress_safe_continue
#undef LZ4_decompress_fast_continue

#include <string>

namespace localization_analysis {
class DepthOdometryAdder {
 public:
  DepthOdometryAdder(const std::string& input_bag_name, const std::string& output_bag_name, const bool save_all_topics,
                     const std::string& config_prefix = "localization/");
  void AddDepthOdometry();

 private:
  depth_odometry::DepthOdometryWrapper depth_odometry_wrapper_;
  rosbag::Bag input_bag_;
  rosbag::Bag output_bag_;
  bool save_all_topics_;
};
}  // namespace localization_analysis

#endif  // LOCALIZATION_ANALYSIS_DEPTH_ODOMETRY_ADDER_H_
