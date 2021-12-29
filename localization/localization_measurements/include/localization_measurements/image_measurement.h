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

#ifndef LOCALIZATION_MEASUREMENTS_IMAGE_MEASUREMENT_H_
#define LOCALIZATION_MEASUREMENTS_IMAGE_MEASUREMENT_H_

#include <localization_common/time.h>
#include <localization_measurements/measurement.h>

#include <opencv2/core.hpp>

namespace localization_measurements {
struct ImageMeasurement : public Measurement {
  ImageMeasurement(const cv::Mat& image, const localization_common::Time timestamp)
      : Measurement(timestamp), image(image) {}
  cv::Mat image;
};
}  // namespace localization_measurements

#endif  // LOCALIZATION_MEASUREMENTS_IMAGE_MEASUREMENT_H_
