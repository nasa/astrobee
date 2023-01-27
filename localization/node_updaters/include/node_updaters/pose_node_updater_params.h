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
#ifndef NODE_UPDATERS_POSE_NODE_UPDATER_PARAMS_H_
#define NODE_UPDATERS_POSE_NODE_UPDATER_PARAMS_H_

#include <localization_common/time.h>

#include <gtsam/geometry/Pose3.h>

#include <boost/serialization/serialization.hpp>

namespace node_updaters {
using Base = TimestampedNodeUpdaterParams<gtsam::Pose3>;
struct PoseNodeUpdaterParams : Base {
  double starting_prior_translation_stddev;
  double starting_prior_quaternion_stddev;

 private:
  // Serialization function
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE& ar, const unsigned int /*version*/) {
    ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(Base);
    ar& BOOST_SERIALIZATION_NVP(starting_prior_translation_stddev);
    ar& BOOST_SERIALIZATION_NVP(starting_prior_quaternion_stddev);
  }
};
}  // namespace node_updaters

#endif  // NODE_UPDATERS_POSE_NODE_UPDATER_PARAMS_H_