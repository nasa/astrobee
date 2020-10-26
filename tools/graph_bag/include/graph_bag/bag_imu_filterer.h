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

#ifndef GRAPH_BAG_BAG_IMU_FILTERER_H_
#define GRAPH_BAG_BAG_IMU_FILTERER_H_

#include <imu_integration/imu_filter.h>
#include <imu_integration/test_imu_filter.h>

#include <rosbag/bag.h>

#include <string>

namespace graph_bag {
// Reads through a bag file and filters imu measurements, replacing the old imu measurements with new filtered ones.
// Saves output to a new bagfile.
class BagImuFilterer {
 public:
  BagImuFilterer(const std::string& bag_name, const std::string& imu_topic, const std::string& filtered_bag,
                 const std::string& filter_name);
  void Convert();

 private:
  std::unique_ptr<imu_integration::ImuFilter> imu_filter_;
  rosbag::Bag filtered_bag_;
};
}  // end namespace graph_bag

#endif  // GRAPH_BAG_BAG_IMU_FILTERER_H_
