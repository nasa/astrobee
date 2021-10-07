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
#ifndef DEPTH_ODOMETRY_PARAMETER_READER_H_
#define DEPTH_ODOMETRY_PARAMETER_READER_H_

#include <config_reader/config_reader.h>
#include <depth_odometry/brisk_feature_detector_and_matcher_params.h>
#include <depth_odometry/depth_image_aligner_params.h>
#include <depth_odometry/depth_odometry_nodelet_params.h>
#include <depth_odometry/depth_odometry_params.h>
#include <depth_odometry/lk_optical_flow_feature_detector_and_matcher_params.h>
#include <depth_odometry/surf_feature_detector_and_matcher_params.h>
#include <depth_odometry/icp_params.h>

namespace depth_odometry {
void LoadDepthOdometryNodeletParams(config_reader::ConfigReader& config, DepthOdometryNodeletParams& params);
void LoadDepthOdometryParams(config_reader::ConfigReader& config, DepthOdometryParams& params);
void LoadBriskFeatureDetectorAndMatcherParams(config_reader::ConfigReader& config,
                                              BriskFeatureDetectorAndMatcherParams& params);
void LoadLKOpticalFlowFeatureDetectorAndMatcherParams(config_reader::ConfigReader& config,
                                                      LKOpticalFlowFeatureDetectorAndMatcherParams& params);
void LoadSurfFeatureDetectorAndMatcherParams(config_reader::ConfigReader& config,
                                             SurfFeatureDetectorAndMatcherParams& params);
void LoadDepthImageAlignerParams(config_reader::ConfigReader& config, DepthImageAlignerParams& params);
void LoadICPParams(config_reader::ConfigReader& config, ICPParams& params);
void LoadPointCloudWithKnownCorrespondencesAlignerParams(config_reader::ConfigReader& config,
                                                         PointCloudWithKnownCorrespondencesAlignerParams& params);
}  // namespace depth_odometry
#endif  // DEPTH_ODOMETRY_PARAMETER_READER_H_
