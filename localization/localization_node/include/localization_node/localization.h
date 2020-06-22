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

#ifndef LOCALIZATION_NODE_LOCALIZATION_H_
#define LOCALIZATION_NODE_LOCALIZATION_H_

#include <sparse_mapping/sparse_map.h>

#include <config_reader/config_reader.h>
#include <cv_bridge/cv_bridge.h>
#include <ff_msgs/VisualLandmarks.h>

namespace localization_node {

class Localizer {
 public:
  explicit Localizer(sparse_mapping::SparseMap* comp_map_ptr);
  ~Localizer();
  void ReadParams(config_reader::ConfigReader* config);
  bool Localize(cv_bridge::CvImageConstPtr image_ptr, ff_msgs::VisualLandmarks* vl);

 private:
  sparse_mapping::SparseMap* map_;
};

};  // namespace localization_node

#endif  // LOCALIZATION_NODE_LOCALIZATION_H_
