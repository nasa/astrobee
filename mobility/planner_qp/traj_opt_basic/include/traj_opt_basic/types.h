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

#ifndef TRAJ_OPT_BASIC_TYPES_H_
#define TRAJ_OPT_BASIC_TYPES_H_

#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <vector>

// This file is to define typdefs for all the eigen types as either float or
// double
// Add flag "-DTRAJ_OPT_USE_SINGLE_PRECISION" to compile the entire package with
// float,
// otherwise, we will default to use double

namespace traj_opt {

#ifndef TRAJ_OPT_USE_SINGLE_PRECISION
typedef double decimal_t;
#else
typedef float decimal_t;
#endif

typedef Eigen::Matrix<decimal_t, 3, 1> Vec3;
typedef Eigen::Matrix<decimal_t, 4, 1> Vec4;
typedef Eigen::Matrix<decimal_t, 3, 3> Mat3;
typedef Eigen::Matrix<decimal_t, 4, 4> Mat4;

typedef std::vector<Vec4, Eigen::aligned_allocator<Vec4>> Vec4Vec;
typedef std::vector<Vec3, Eigen::aligned_allocator<Vec3>> Vec3Vec;
typedef std::vector<Mat4, Eigen::aligned_allocator<Mat4>> Mat4Vec;

typedef Eigen::Matrix<decimal_t, Eigen::Dynamic, 1> VecD;
typedef Eigen::Matrix<decimal_t, Eigen::Dynamic, 3> MatD3;
typedef Eigen::Matrix<decimal_t, Eigen::Dynamic, Eigen::Dynamic> MatD;

typedef std::vector<VecD> VecDVec;
typedef std::vector<MatD> MatDVec;

typedef Eigen::Quaternion<decimal_t> Quat;
}  // namespace traj_opt
#endif  // TRAJ_OPT_BASIC_TYPES_H_
