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

#ifndef SPARSE_MAPPING_SPARSE_MAP_H_
#define SPARSE_MAPPING_SPARSE_MAP_H_

#include <ff_common/eigen_vectors.h>
#include <interest_point/matching.h>
#include <sparse_mapping/vocab_tree.h>
#include <sparse_mapping/sparse_mapping.h>
#include <sparse_mapping/localization_parameters.h>
#include <camera/camera_model.h>
#include <camera/camera_params.h>

#include <boost/optional.hpp>
#include <Eigen/Geometry>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc.hpp>

#include <map>
#include <mutex>
#include <numeric>
#include <set>
#include <string>
#include <vector>
#include <utility>
#include <limits>

namespace cv {
  class Mat;
  class DMatch;
}

namespace sparse_mapping {

// Non-member function InitializeCidFidToPid() that we will use within
// this class and outside of it as well.
void InitializeCidFidToPid(int num_cid,
                           std::vector<std::map<int, int> > const& pid_to_cid_fid,
                           std::vector<std::map<int, int> > * cid_fid_to_pid);

/**
 * A class representing a sparse map, which consists of a collection
 * of keyframes and detected features. To localize, an image's features
 * are matched to the keyframes in the map. They keyframe features have known
 * positions and the camera pose can be estimated with ransac.
 **/
struct SparseMap {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   * Constructs a new sparse map from a list of image files and their
   * associate keypoint and descriptor files. If use_cached_features
   * is set to false, it reads the image files and performs feature
   * detection instead. Does not perform bundle adjustment.
   **/
  SparseMap(const std::vector<std::string> & filenames,
            const std::string & detector,
            const camera::CameraParameters & params);

  /**
   * Constructs a new sparse map from a protobuf file, with specified
   * vocabulary tree and optional parameters.
   **/
  SparseMap(const std::string & protobuf_file,
            bool localization = false);

  /**
     Form a sparse map with given cameras/images, and no features
  **/
  SparseMap(const std::vector<Eigen::Affine3d>& cid_to_cam_t,
            const std::vector<std::string> & filenames,
            const std::string & detector,
            const camera::CameraParameters & params);


  SparseMap(bool bundler_format, std::string const& filename, std::vector<std::string> const& files);

  // Set default loc params using FLAG values
  void SetDefaultLocParams();

  // Set loc params
  void SetLocParams(const LocalizationParameters& loc_params);
  const LocalizationParameters& loc_params() const { return loc_params_; }
  LocalizationParameters& loc_params() { return loc_params_; }

  void SetDetectorParams(int min_features, int max_features, int retries, double min_thresh, double default_thresh,
                         double max_thresh, double too_many_ratio, double too_few_ratio);

  /**
   * Detect features in given images
   **/
  void DetectFeatures();

  /**
   * Save the map to a protobuf file.
   **/
  void Save(const std::string & protobuf_file) const;

  /**
   * Estimate the camera pose for an image file.
   **/
  bool Localize(std::string const& img_file, camera::CameraModel* pose,
      std::vector<Eigen::Vector3d>* inlier_landmarks = NULL,
                std::vector<Eigen::Vector2d>* inlier_observations = NULL,
                std::vector<int> * cid_list = NULL);
  bool Localize(const cv::Mat & image,
      camera::CameraModel* pose, std::vector<Eigen::Vector3d>* inlier_landmarks = NULL,
                std::vector<Eigen::Vector2d>* inlier_observations = NULL,
                std::vector<int> * cid_list = NULL);
  bool Localize(cv::Mat const& test_descriptors, Eigen::Matrix2Xd const& test_keypoints, camera::CameraModel* pose,
                std::vector<Eigen::Vector3d>* inlier_landmarks, std::vector<Eigen::Vector2d>* inlier_observations,
                std::vector<int>* cid_list = NULL, const cv::Mat& image = cv::Mat());

  // access map frames
  /**
   * Get the number of keyframes in the map.
   **/
  size_t GetNumFrames(void) const {return cid_to_filename_.size();}
  /**
   * Get the filename of a keyframe in the map.
   **/
  const std::string & GetFrameFilename(int frame) const {return cid_to_filename_[frame];}
  /**
   * Get the global camera transform for a keyframe in the map.
   **/
  const Eigen::Affine3d & GetFrameGlobalTransform(int frame) const
        {return cid_to_cam_t_global_[frame];}

  void SetFrameGlobalTransform(int frame, const Eigen::Affine3d & transform) {
    cid_to_cam_t_global_[frame] = transform;
  }
  /**
   * Get the keypoint coordinates in the specified frame.
   **/
  const Eigen::Matrix2Xd & GetFrameKeypoints(int frame) const {return cid_to_keypoint_map_[frame];}
  /**
   * Get the descriptor for a frame and feature.
   **/
  cv::Mat GetDescriptor(int frame, int fid) const { return cid_to_descriptor_map_[frame].row(fid);}
  /**
   * Returns map of feature ids to landmark ids for the specified frame.
   **/
  const std::map<int, int> & GetFrameFidToPidMap(int frame) const {return cid_fid_to_pid_[frame];}

  // access map landmarks
  /**
   * Get the number of landmark points in the map.
   **/
  size_t GetNumLandmarks(void) const {return pid_to_xyz_.size();}
  /**
   * Get the global position of the specified landmark.
   **/
  Eigen::Vector3d GetLandmarkPosition(int landmark) const {return pid_to_xyz_[landmark];}
  /**
   * Return a map for a specified landmark, matching the ids of all the keyframes that landmark
   * was seen in to the feature id within that frame.
   **/
  const std::map<int, int> & GetLandmarkCidToFidMap(int landmark) const {return pid_to_cid_fid_[landmark];}

  // access and modify parameters
  /**
   * Return the parameters of the camera used to construct the map.
   **/
  camera::CameraParameters GetCameraParameters(void) const {return camera_params_;}
  void SetCameraParameters(camera::CameraParameters camera_params) {camera_params_ = camera_params;}
  /**
   * Return the number of observations. Use this number to divide the final error to find the average pixel error.
   **/
  size_t GetNumObservations(void) const {return std::accumulate(pid_to_cid_fid_.begin(),
                                                                pid_to_cid_fid_.end(),
                                                                0, [](size_t v, std::map<int, int> const& map)
                                                                { return v + map.size(); }); }
  /**
   * Return the transform to real world coordinates.
   **/
  Eigen::Affine3d GetWorldTransform() const {return world_transform_;}

  /**
   * Apply given transform to camera positions and 3D points
   **/
  void ApplyTransform(Eigen::Affine3d const& T) {
    sparse_mapping::TransformCamerasAndPoints(T, &cid_to_cam_t_global_, &pid_to_xyz_);
  }

  // Load map. If localization is true, load only the parts of the map
  // needed for localization.
  void Load(const std::string & protobuf_file, bool localization = false);

  // Optionally load keypoints. Call this after loading the map, if for example
  // the user needs to use the keypoints for visualization or essential matrix
  // estimation and those params differ from the default FLAG values.
  void LoadKeypoints(const std::string & protobuf_file);

  // construct from pid_to_cid_fid
  void InitializeCidFidToPid();

  // detect features with opencv
  void DetectFeaturesFromFile(std::string const& filename,
                              bool multithreaded,
                              cv::Mat* descriptors,
                              Eigen::Matrix2Xd* keypoints);
  void DetectFeatures(cv::Mat const& image,
                      bool multithreaded,
                      cv::Mat* descriptors,
                      Eigen::Matrix2Xd* keypoints);
  // delete feature descriptors with no matching landmark
  void PruneMap(void);

  std::string GetDetectorName() { return detector_.GetDetectorName(); }
  interest_point::FeatureDetector& detector() { return detector_; }
  int GetHistogramEqualization() {return loc_params_.histogram_equalization;}

  // stored in map file
  std::vector<std::string> cid_to_filename_;
  // TODO(bcoltin) replace Eigen2Xd everywhere with one keypoint class
  std::vector<Eigen::Matrix2Xd > cid_to_keypoint_map_;
  std::vector<std::map<int, int> > pid_to_cid_fid_;
  std::vector<Eigen::Vector3d> pid_to_xyz_;
  std::vector<Eigen::Affine3d > cid_to_cam_t_global_;
  std::vector<cv::Mat> cid_to_descriptor_map_;
  // generated on load
  std::vector<std::map<int, int> > cid_fid_to_pid_;
  std::vector<std::map<int, int>> cid_to_matching_cid_counts_;

  interest_point::FeatureDetector detector_;
  camera::CameraParameters camera_params_;
  mutable sparse_mapping::VocabDB vocab_db_;  // TODO(oalexan1): Mutable means someone is doing something wrong.
  LocalizationParameters loc_params_;
  std::string protobuf_file_;

  // e.g, 10th db image is 3rd image in cid_to_filename_
  std::map<int, int> db_to_cid_map_;

  // If datastructure is available, match only pairs of cids
  // that are present in it (this info can come from example from
  // a map that was built previously with the same images but
  // a different descriptor.
  std::map< int, std::set<int> > cid_to_cid_;  // TODO(oalexan1): Need not be a member

  // These are used to register the map to real world coordinates
  // with information provided by the user.
  // TODO(oalexan1): These need not be members
  Eigen::Affine3d world_transform_;
  std::vector<Eigen::Matrix2Xd> user_cid_to_keypoint_map_;
  std::vector<std::map<int, int> > user_pid_to_cid_fid_;
  std::vector<Eigen::Vector3d> user_pid_to_xyz_;

  cv::Ptr<cv::CLAHE> clahe_;
  boost::optional<int> best_previous_cid_;
  std::mutex mutex_detector_;

 private:
  // Create an empty map. It is strongly recommended to not use this function,
  // as it requires carefully initializing many members. Hence this is made
  // private. Consider using the other constructors.
  SparseMap();

  // I found out the hard way that sparse maps cannot be copied
  // correctly, hence prohibit this. The only good way seems to be to
  // load a copy from disk. (oalexan1)
  SparseMap(SparseMap &);
  SparseMap& operator=(const SparseMap&);

  // Reorder the images in the map and the rest of the data accordingly
  void reorderMap(std::map<int, int> const& old_cid_to_new_cid);
};
}  // namespace sparse_mapping

#endif  // SPARSE_MAPPING_SPARSE_MAP_H_
