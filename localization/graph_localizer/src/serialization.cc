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

#include <graph_localizer/serialization.h>

#include <gtsam/base/serialization.h>

#include <boost/serialization/serialization.hpp>

// Value types used in GraphValues
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/navigation/NavState.h>

// Export all classes derived from Value so they can
// be properly serialized using base class (value) pointers in GraphValues
GTSAM_VALUE_EXPORT(gtsam::Vector3);
GTSAM_VALUE_EXPORT(gtsam::Pose3);
GTSAM_VALUE_EXPORT(gtsam::imuBias::ConstantBias);

namespace graph_localizer {

std::string SerializeBinary(const GraphLocalizer& graph_localizer) { return gtsam::serializeBinary(graph_localizer); }
}  // namespace graph_localizer
