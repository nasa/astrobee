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
#ifndef CALIBRATION_UTILITIES_H_
#define CALIBRATION_UTILITIES_H_

#include <camera/camera_params.h>
#include <localization_common/image_correspondences.h>

#include <Eigen/Geometry>

#include <boost/optional.hpp>

namespace calibration {
boost::optional<Eigen::Isometry3d> CameraTTarget(const camera::CameraParameters& camera,
                                                 const localization_common::ImageCorrespondences& matches);
}  // namespace calibration

#endif  // CALIBRATION_UTILITIES_H_
