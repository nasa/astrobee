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
#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/navigation/NavState.h>

// Factors used in NonlinearFactorGraph
#include <graph_localizer/loc_pose_factor.h>
#include <graph_localizer/loc_projection_factor.h>
#include <graph_localizer/robust_smart_projection_pose_factor.h>
#include <gtsam/nonlinear/LinearContainerFactor.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/ProjectionFactor.h>
#include <gtsam/slam/SmartProjectionPoseFactor.h>

// Export all classes derived from Value so they can
// be properly serialized using base class (value) pointers in GraphValues
GTSAM_VALUE_EXPORT(gtsam::Cal3_S2);
GTSAM_VALUE_EXPORT(gtsam::imuBias::ConstantBias);
GTSAM_VALUE_EXPORT(gtsam::PinholeCamera<gtsam::Cal3_S2>);
GTSAM_VALUE_EXPORT(gtsam::Pose3);
GTSAM_VALUE_EXPORT(gtsam::Vector3);

// Export all factors and noise models used in factor graph so
// these can be serialized using their base class pointers in
// the gtsam::NonlinearFactorGraph
using LocProjectionFactor = gtsam::LocProjectionFactor<gtsam::Pose3, gtsam::Point3, gtsam::Cal3_S2>;
BOOST_CLASS_EXPORT_GUID(gtsam::PriorFactor<gtsam::Pose3>, "gtsam::PriorFactorPose3");
BOOST_CLASS_EXPORT_GUID(gtsam::PriorFactor<gtsam::Vector3>, "gtsam::PriorFactorVector3");
BOOST_CLASS_EXPORT_GUID(gtsam::PriorFactor<gtsam::imuBias::ConstantBias>, "gtsam::PriorFactorConstantBias");
BOOST_CLASS_EXPORT_GUID(LocProjectionFactor, "gtsam::LocProjectionFactor");
BOOST_CLASS_EXPORT_GUID(gtsam::LocPoseFactor, "gtsam::LocPoseFactor");
using ProjectionFactor = gtsam::GenericProjectionFactor<gtsam::Pose3, gtsam::Point3>;
BOOST_CLASS_EXPORT_GUID(ProjectionFactor, "gtsam::GenericProjectionFactor");
BOOST_CLASS_EXPORT_GUID(gtsam::RobustSmartProjectionPoseFactor<gtsam::Cal3_S2>,
                        "gtsam::RobustSmartProjectionPoseFactorCal3_S2");
BOOST_CLASS_EXPORT_GUID(gtsam::SmartProjectionPoseFactor<gtsam::Cal3_S2>, "gtsam::SmartProjectionPoseFactorCal3_S2");
BOOST_CLASS_EXPORT_GUID(gtsam::CombinedImuFactor, "gtsam::CombinedImuFactor");
BOOST_CLASS_EXPORT_GUID(gtsam::LinearContainerFactor, "gtsam::LinearContainerFactor");

// Add all possible noise models used by factors
BOOST_CLASS_EXPORT_GUID(gtsam::noiseModel::Constrained, "gtsam_noiseModel_Constrained");
BOOST_CLASS_EXPORT_GUID(gtsam::noiseModel::Diagonal, "gtsam_noiseModel_Diagonal");
BOOST_CLASS_EXPORT_GUID(gtsam::noiseModel::Gaussian, "gtsam_noiseModel_Gaussian");
BOOST_CLASS_EXPORT_GUID(gtsam::noiseModel::Unit, "gtsam_noiseModel_Unit");
BOOST_CLASS_EXPORT_GUID(gtsam::noiseModel::Isotropic, "gtsam_noiseModel_Isotropic");
BOOST_CLASS_EXPORT_GUID(gtsam::noiseModel::Robust, "gtsam_noiseModel_Robust");

BOOST_CLASS_EXPORT_GUID(gtsam::noiseModel::mEstimator::Base, "gtsam_noiseModel_mEstimator_Base");
BOOST_CLASS_EXPORT_GUID(gtsam::noiseModel::mEstimator::Null, "gtsam_noiseModel_mEstimator_Null");
BOOST_CLASS_EXPORT_GUID(gtsam::noiseModel::mEstimator::Fair, "gtsam_noiseModel_mEstimator_Fair");
BOOST_CLASS_EXPORT_GUID(gtsam::noiseModel::mEstimator::Huber, "gtsam_noiseModel_mEstimator_Huber");
BOOST_CLASS_EXPORT_GUID(gtsam::noiseModel::mEstimator::Tukey, "gtsam_noiseModel_mEstimator_Tukey");

BOOST_CLASS_EXPORT_GUID(gtsam::SharedNoiseModel, "gtsam_SharedNoiseModel");
BOOST_CLASS_EXPORT_GUID(gtsam::SharedDiagonal, "gtsam_SharedDiagonal");

namespace graph_localizer {

std::string SerializeBinary(const GraphLocalizer& graph_localizer) { return gtsam::serializeBinary(graph_localizer); }
}  // namespace graph_localizer
