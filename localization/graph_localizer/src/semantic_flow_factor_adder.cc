#include <graph_localizer/semantic_flow_factor_adder.h>
#include <gtsam/slam/SmartProjectionPoseFactor.h>
#include <graph_localizer/graph_localizer.h>

namespace graph_localizer {
namespace go = graph_optimizer;
namespace sym = gtsam::symbol_shorthand;
SemanticFlowFactorAdder::SemanticFlowFactorAdder(const SemanticFlowFactorAdderParams& params,
                                                 std::shared_ptr<const SemanticObjectTracker> object_tracker)
                                                : SemanticFlowFactorAdder::Base(params), object_tracker_(object_tracker) {
  smart_projection_params_.verboseCheirality = params.verbose_cheirality;
  smart_projection_params_.setRankTolerance(1e-9);
  smart_projection_params_.setLandmarkDistanceThreshold(params.landmark_distance_threshold);
  smart_projection_params_.setDynamicOutlierRejectionThreshold(params.dynamic_outlier_rejection_threshold);
  smart_projection_params_.setRetriangulationThreshold(params.retriangulation_threshold);
  if (params.rotation_only_fallback) smart_projection_params_.setDegeneracyMode(gtsam::DegeneracyMode::HANDLE_INFINITY);
  smart_projection_params_.setEnableEPI(params.enable_EPI);
}

std::vector<go::FactorsToAdd> SemanticFlowFactorAdder::AddFactors() {
  go::FactorsToAdd factors_to_add(go::GraphActionCompleterType::SmartFactor);

  const auto tracks = object_tracker_->Tracks();

  //LogDebug("SemanticAddFactors: " << params().cam_intrinsics->fx() << ", " << params().cam_intrinsics->fy()
  //         << ", " << params().cam_intrinsics->px() << ", " << params().cam_intrinsics->py());
  for (const auto& track : tracks) {
    const int num_object_track_points = track->size();
    if (num_object_track_points <= 0) continue;
    const double noise_scale = params().noise_scale * num_object_track_points;
    const auto noise = gtsam::noiseModel::Isotropic::Sigma(2, noise_scale * params().cam_noise->sigma());

    const auto pts = track->LatestDets();
    SharedRobustSmartFactor smart_factor = 
      boost::make_shared<RobustSmartFactor>(noise, params().cam_intrinsics, params().body_T_cam, smart_projection_params_,
                                            params().rotation_only_fallback, params().robust, params().huber_k);
    go::KeyInfos key_infos;
    key_infos.reserve(pts.size());

    int uninitialized_key_index = 0;
    for (const auto& pt : pts) {
      const go::KeyInfo key_info(&sym::P, go::NodeUpdaterType::CombinedNavState, pt.timestamp);
      key_infos.emplace_back(key_info);
      smart_factor->add(Camera::Measurement(pt.image_point), key_info.MakeKey(uninitialized_key_index++));
    }
    factors_to_add.push_back({key_infos, smart_factor});
  }

  LogDebug("AddSemanticFactors: Added " << factors_to_add.size() << " smart factors.");
  return {factors_to_add};
}
} // namespace graph_localizer
