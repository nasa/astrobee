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
#ifndef POINT_CLOUD_COMMON_ORGANIZED_NEIGHBOR_H_
#define POINT_CLOUD_COMMON_ORGANIZED_NEIGHBOR_H_

#include <pcl/search/organized.h>

// PCL doesn't allow for setting the projection matrix manually so a custom class is needed
// Make sure to call this after setting the input cloud as PCL will attempt to estimate
// the projection matrix when the input cloud is set.
// TODO(rsoussan): Add distortion to projection
namespace point_cloud_common {
template <typename PointType>
class OrganizedNeighbor : public pcl::search::OrganizedNeighbor<PointType> {
 public:
  typedef boost::shared_ptr<OrganizedNeighbor<PointType>> Ptr;

  void setProjectionMatrix(const Eigen::Matrix<double, 3, 4>& projection_matrix) {
    pcl::search::OrganizedNeighbor<PointType>::projection_matrix_ = projection_matrix.cast<float>();
  }
};
}  // namespace point_cloud_common

#endif  // POINT_CLOUD_COMMON_ORGANIZED_NEIGHBOR_H_
