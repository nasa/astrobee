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

#ifndef GRAPH_BAG_DEPTH_ODOMETRY_ADDER_H_
#define GRAPH_BAG_DEPTH_ODOMETRY_ADDER_H_

#include <depth_odometry/depth_odometry_wrapper.h>

#include <rosbag/view.h>

#include <string>

namespace graph_bag {
class DepthOdometryAdder {
 public:
  DepthOdometryAdder(const std::string& input_bag_name, const std::string& output_bag_name, const bool save_all_topics);
  void AddDepthOdometry();

 private:
  depth_odometry::DepthOdometryWrapper depth_odometry_wrapper_;
  rosbag::Bag input_bag_;
  rosbag::Bag output_bag_;
  bool save_all_topics_;
};
}  // namespace graph_bag

#endif  // GRAPH_BAG_DEPTH_ODOMETRY_ADDER_H_
