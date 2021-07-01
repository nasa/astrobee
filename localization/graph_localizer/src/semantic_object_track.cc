#include <graph_localizer/semantic_object_track.h>

namespace graph_localizer {
SemanticObjectTrack::SemanticObjectTrack(const localization_measurements::SemanticDet& semantic_det) {
  cls_type_ = semantic_det.class_id;
  AddMeasurement(semantic_det);
}

void SemanticObjectTrack::AddMeasurement(const localization_measurements::SemanticDet& semantic_det) {
  if (cls_type_ == semantic_det.class_id) {
    dets_.emplace(semantic_det.timestamp, semantic_det);
  }
}

void SemanticObjectTrack::RemoveOldMeasurements(const localization_common::Time oldest_allowed_timestamp) {
  dets_.erase(dets_.begin(), dets_.lower_bound(oldest_allowed_timestamp));
}

bool SemanticObjectTrack::empty() const {
  return dets_.empty();
}

float SemanticObjectTrack::Distance(const localization_measurements::SemanticDet& semantic_det) {
  if (cls_type_ != semantic_det.class_id) return -1;
  const auto latest_det = LatestDet();
  if (!latest_det) return -1;
  return (semantic_det.image_point - latest_det->image_point).norm();
}

boost::optional<localization_measurements::SemanticDet> SemanticObjectTrack::LatestDet() const {
  if (empty()) return boost::none;
  return dets_.crbegin()->second;
}

std::vector<localization_measurements::SemanticDet> SemanticObjectTrack::LatestDets() const {
  std::vector<localization_measurements::SemanticDet> latest_dets;
  for (const auto& det : dets_) {
    latest_dets.push_back(det.second);
  }
  return latest_dets;
}

boost::optional<localization_common::Time> SemanticObjectTrack::LatestTimestamp() const {
  if (empty()) return boost::none;
  return dets_.crbegin()->first;
}

boost::optional<localization_common::Time> SemanticObjectTrack::OldestTimestamp() const {
  if (empty()) return boost::none;
  return dets_.cbegin()->first;
}
} // namespace graph_localizer
