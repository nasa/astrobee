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

#ifndef GRAPH_LOCALIZER_LOC_POSE_FACTOR_H_
#define GRAPH_LOCALIZER_LOC_POSE_FACTOR_H_

#include <gtsam/slam/PriorFactor.h>

namespace gtsam {
// Allows for differentiation between loc pose factors and other pose priors, perhaps
// added as starting priors or marginalization factors
class LocPoseFactor : public PriorFactor<Pose3> {
 private:
  typedef PriorFactor<Pose3> Base;

 public:
  typedef typename boost::shared_ptr<LocPoseFactor> shared_ptr;

  LocPoseFactor() {}

  virtual ~LocPoseFactor() {}

  LocPoseFactor(Key key, const Pose3& prior, const SharedNoiseModel& model = nullptr) : Base(key, prior, model) {}

  LocPoseFactor(Key key, const Pose3& prior, const Matrix& covariance) : Base(key, prior, covariance) {}
};
}  // namespace gtsam

#endif  // GRAPH_LOCALIZER_LOC_POSE_FACTOR_H_
