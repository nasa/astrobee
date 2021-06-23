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

/**
 * This header makes vectors using Eigen objects align properly.
 * It should be included in any file that uses vectors of Eigen objects.
 **/

#ifndef COMMON_EIGEN_VECTORS_H_
#define COMMON_EIGEN_VECTORS_H_

#include <Eigen/Geometry>
#include <Eigen/StdVector>

EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Vector2d)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Vector3d)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Affine3d)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Matrix<double, 3, 4>)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Matrix2Xd)
#endif  // COMMON_EIGEN_VECTORS_H_
