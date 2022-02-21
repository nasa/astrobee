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
#ifndef POINT_CLOUD_COMMON_PARAMETER_READER_H_
#define POINT_CLOUD_COMMON_PARAMETER_READER_H_

#include <config_reader/config_reader.h>
#include <point_cloud_common/point_to_plane_icp_params.h>
#include <point_cloud_common/point_cloud_with_known_correspondences_aligner_params.h>

namespace point_cloud_common {
void LoadPointToPlaneICPParams(config_reader::ConfigReader& config, PointToPlaneICPParams& params);
void LoadPointCloudWithKnownCorrespondencesAlignerParams(config_reader::ConfigReader& config,
                                                         PointCloudWithKnownCorrespondencesAlignerParams& params);
}  // namespace point_cloud_common
#endif  // POINT_CLOUD_COMMON_PARAMETER_READER_H_
