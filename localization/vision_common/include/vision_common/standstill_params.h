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
#ifndef VISION_COMMON_STANDSTILL_PARAMS_H_
#define VISION_COMMON_STANDSTILL_PARAMS_H_

#include <boost/serialization/serialization.hpp>

namespace vision_common {
struct StandstillParams {
  // Min number of points for a track to be used for standstill calculation.
  int min_num_points_per_track;
  // Max track duration to consider for standstill.
  double duration;
  // Max average deviation for image point detections in feature tracks for standstill.
  double max_avg_distance_from_mean;

 private:
  // Serialization function
  // TODO(rsoussan): Fix this, causing compile error
  /*friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int file_version) {
    ar& BOOST_SERIALIZATION_NVP(min_num_points_per_track);
    ar& BOOST_SERIALIZATION_NVP(duration);
    ar& BOOST_SERIALIZATION_NVP(max_avg_distance_from_mean);
  }*/
};
}  // namespace vision_common

#endif  // VISION_COMMON_STANDSTILL_PARAMS_H_
