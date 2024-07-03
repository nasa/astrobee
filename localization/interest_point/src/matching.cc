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

#include <interest_point/BAD.h>
#include <interest_point/brisk.h>
#include <interest_point/matching.h>
#include <opencv2/xfeatures2d.hpp>

#include <Eigen/Core>
#include <gflags/gflags.h>
#include <glog/logging.h>

#include <iostream>
#include <vector>
// Note: if any of these values are manually set by the user in
// build_map, the localize script must be invoked with precisely the
// same settings!
// TODO(oalexan1): Ideally the settings used here must be saved in the
// map file, for the localize executable to read them from there.
DEFINE_int32(hamming_distance, 85,
             "A smaller value keeps fewer but more reliable binary descriptor matches.");
DEFINE_double(goodness_ratio, 0.8,
              "A smaller value keeps fewer but more reliable descriptor matches.");
DEFINE_int32(orgbrisk_octaves, 4,
             "Number of octaves, or scale spaces, that BRISK will evaluate.");
DEFINE_double(orgbrisk_pattern_scale, 1.0,
             "The pattern scale to use for BRISK.");

// Customize the feature detectors.
DEFINE_int32(detection_retries, 5,
             "Number of attempts to acquire the desired number of features with the detector.");
// SURF detector
DEFINE_int32(min_surf_features, 1000,
             "Minimum number of features to be computed using SURF.");
DEFINE_int32(max_surf_features, 5000,
             "Maximum number of features to be computed using SURF.");
DEFINE_double(min_surf_threshold, 1.1,
              "Minimum threshold for feature detection using SURF.");
DEFINE_double(default_surf_threshold, 10,
              "Default threshold for feature detection using SURF.");
DEFINE_double(max_surf_threshold, 1000,
              "Maximum threshold for feature detection using SURF.");
DEFINE_double(surf_too_many_ratio, 1.1,
              "Ratio to increase the dynamic feature threshold by if too many features are detected.");
DEFINE_double(surf_too_few_ratio, 0.9,
              "Ratio to reduce the dynamic feature threshold by if too few features are detected.");

// Binary detector
DEFINE_int32(min_brisk_features, 1000,
             "Minimum number of features to be computed using ORGBRISK.");
DEFINE_int32(max_brisk_features, 5000,
             "Maximum number of features to be computed using ORGBRISK.");
DEFINE_double(min_brisk_threshold, 10,
              "Minimum threshold for feature detection using ORGBRISK.");
DEFINE_double(default_brisk_threshold, 90,
              "Default threshold for feature detection using ORGBRISK.");
DEFINE_double(max_brisk_threshold, 110,
              "Maximum threshold for feature detection using ORGBRISK.");
DEFINE_double(brisk_too_many_ratio, 1.25,
              "Ratio to increase the dynamic feature threshold by if too many features are detected.");
DEFINE_double(brisk_too_few_ratio, 0.8,
              "Ratio to reduce the dynamic feature threshold by if too few features are detected.");



namespace interest_point {

DynamicDetector::DynamicDetector(int min_features, int max_features, int max_retries, double min_thresh,
                                 double default_thresh, double max_thresh, double too_many_ratio, double too_few_ratio)
    : min_features_(min_features),
      max_features_(max_features),
      max_retries_(max_retries),
      min_thresh_(min_thresh),
      default_thresh_(default_thresh),
      max_thresh_(max_thresh),
      dynamic_thresh_(default_thresh),
      too_many_ratio_(too_many_ratio),
      too_few_ratio_(too_few_ratio),
      last_keypoint_count_(0) {}

void DynamicDetector::GetDetectorParams(int& min_features, int& max_features, int& max_retries, double& min_thresh,
                                        double& default_thresh, double& max_thresh, double& too_many_ratio,
                                        double& too_few_ratio) {
  min_features = min_features_;
  max_features = max_features_;
  max_retries = max_retries_;
  min_thresh = min_thresh_;
  default_thresh = default_thresh_;
  max_thresh = max_thresh_;
  too_many_ratio = too_many_ratio_;
  too_few_ratio = too_few_ratio_;
}

  void DynamicDetector::Detect(const cv::Mat& image,
                               std::vector<cv::KeyPoint>* keypoints,
                               cv::Mat* keypoints_description) {
    // Sometimes we want a placeholder detector for initialization. Yet
    // that one cannot be used until it is configured.
    if (default_thresh_ <= 0)
      LOG(FATAL) << "The detector parameters have not been set.";

    for (unsigned int i = 0; i < max_retries_; i++) {
      keypoints->clear();
      DetectImpl(image, keypoints);
      last_keypoint_count_ = keypoints->size();
      if (keypoints->size() < min_features_)
        TooFew();
      else if (keypoints->size() > max_features_)
        TooMany();
      else
        break;
    }
    ComputeImpl(image, keypoints, keypoints_description);
  }

  class BriskDynamicDetector : public DynamicDetector {
   public:
    BriskDynamicDetector(int min_features, int max_features, int max_retries, double min_thresh, double default_thresh,
                         double max_thresh, double too_many_ratio, double too_few_ratio)
        : DynamicDetector(min_features, max_features, max_retries, min_thresh, default_thresh, max_thresh,
                          too_many_ratio, too_few_ratio) {
      Reset();
    }

    void Reset(void) {
      brisk_ = interest_point::BRISK::create(dynamic_thresh_, FLAGS_orgbrisk_octaves,
                                 FLAGS_orgbrisk_pattern_scale);
    }

    virtual void DetectImpl(const cv::Mat& image, std::vector<cv::KeyPoint>* keypoints) {
      brisk_->detect(image, *keypoints);
    }
    virtual void ComputeImpl(const cv::Mat& image, std::vector<cv::KeyPoint>* keypoints,
                             cv::Mat* keypoints_description) {
      brisk_->compute(image, *keypoints, *keypoints_description);
    }
    virtual void TooMany(void) {
      dynamic_thresh_ *= too_many_ratio_;
      dynamic_thresh_ = static_cast<int>(dynamic_thresh_);  // for backwards compatibility
      if (dynamic_thresh_ > max_thresh_)
        dynamic_thresh_ = max_thresh_;
      brisk_->setThreshold(dynamic_thresh_);
    }
    virtual void TooFew(void) {
      dynamic_thresh_ *= too_few_ratio_;
      dynamic_thresh_ = static_cast<int>(dynamic_thresh_);  // for backwards compatibility
      if (dynamic_thresh_ < min_thresh_)
        dynamic_thresh_ = min_thresh_;
      brisk_->setThreshold(dynamic_thresh_);
    }

   private:
    cv::Ptr<interest_point::BRISK> brisk_;
  };

  class SurfDynamicDetector : public DynamicDetector {
   public:
    SurfDynamicDetector(int min_features, int max_features, int max_retries, double min_thresh, double default_thresh,
                        double max_thresh, double too_many_ratio, double too_few_ratio)
        : DynamicDetector(min_features, max_features, max_retries, min_thresh, default_thresh, max_thresh,
                          too_many_ratio, too_few_ratio) {
      surf_ = cv::xfeatures2d::SURF::create(dynamic_thresh_);
    }

    virtual void DetectImpl(const cv::Mat& image, std::vector<cv::KeyPoint>* keypoints) {
      surf_->detect(image, *keypoints);
    }
    virtual void ComputeImpl(const cv::Mat& image, std::vector<cv::KeyPoint>* keypoints,
                             cv::Mat* keypoints_description) {
      surf_->compute(image, *keypoints, *keypoints_description);
    }
    virtual void TooMany(void) {
      dynamic_thresh_ *= too_many_ratio_;
      if (dynamic_thresh_ > max_thresh_)
        dynamic_thresh_ = max_thresh_;
      surf_->setHessianThreshold(static_cast<float>(dynamic_thresh_));
    }
    virtual void TooFew(void) {
      dynamic_thresh_ *= too_few_ratio_;
      if (dynamic_thresh_ < min_thresh_)
        dynamic_thresh_ = min_thresh_;
      surf_->setHessianThreshold(static_cast<float>(dynamic_thresh_));
    }

   private:
    cv::Ptr<cv::xfeatures2d::SURF> surf_;
  };

  class TeblidDynamicDetector : public DynamicDetector {
   public:
    TeblidDynamicDetector(int min_features, int max_features, int max_retries, double min_thresh, double default_thresh,
                          double max_thresh, double too_many_ratio, double too_few_ratio, bool use_512 = true)
        : DynamicDetector(min_features, max_features, max_retries, min_thresh, default_thresh, max_thresh,
                          too_many_ratio, too_few_ratio) {
      if (use_512) {
        teblid_ = upm::BAD::create(5.0, upm::BAD::SIZE_512_BITS);
      } else {
        teblid_ = upm::BAD::create(5.0, upm::BAD::SIZE_256_BITS);
      }
      Reset();
    }

    void Reset(void) {
      brisk_ = interest_point::BRISK::create(dynamic_thresh_, FLAGS_orgbrisk_octaves,
                                 FLAGS_orgbrisk_pattern_scale);
    }

    virtual void DetectImpl(const cv::Mat& image, std::vector<cv::KeyPoint>* keypoints) {
      brisk_->detect(image, *keypoints);
    }
    virtual void ComputeImpl(const cv::Mat& image, std::vector<cv::KeyPoint>* keypoints,
                             cv::Mat* keypoints_description) {
      teblid_->compute(image, *keypoints, *keypoints_description);
    }
    virtual void TooMany(void) {
      double threshold_ratio = too_many_ratio_;
      const int keypoint_diff = std::abs(last_keypoint_count_ - min_features_);
      // Scale ratio as get close to edge of too few keypoints to avoid undershoot
      constexpr double kKeypointDiff = 500;
      if (keypoint_diff < kKeypointDiff) {
        threshold_ratio += (1.0 - threshold_ratio)*(kKeypointDiff - keypoint_diff)/kKeypointDiff;
      }
      dynamic_thresh_ *= threshold_ratio;
      dynamic_thresh_ = std::lround(dynamic_thresh_);  // for backwards compatibility
      if (dynamic_thresh_ > max_thresh_)
        dynamic_thresh_ = max_thresh_;
      brisk_->setThreshold(dynamic_thresh_);
    }
    virtual void TooFew(void) {
      double threshold_ratio = too_few_ratio_;
      const int keypoint_diff = std::abs(last_keypoint_count_ - max_features_);
      // Scale ratio as get close to edge of too many keypoints to avoid overshoot
      constexpr double kKeypointDiff = 1000;
      if (keypoint_diff < kKeypointDiff) {
        threshold_ratio += (1.0 - threshold_ratio)*(kKeypointDiff - keypoint_diff)/kKeypointDiff;
      }
      dynamic_thresh_ *= threshold_ratio;
      dynamic_thresh_ = std::lround(dynamic_thresh_);  // for backwards compatibility
      if (dynamic_thresh_ < min_thresh_)
        dynamic_thresh_ = min_thresh_;
      brisk_->setThreshold(dynamic_thresh_);
    }

   private:
    cv::Ptr<cv::Feature2D> teblid_;
    cv::Ptr<interest_point::BRISK> brisk_;
  };

  FeatureDetector::FeatureDetector(std::string const& detector_name, int min_features, int max_features, int retries,
                                   double min_thresh, double default_thresh, double max_thresh, double too_many_ratio,
                                   double too_few_ratio) {
    detector_ = NULL;
    Reset(detector_name, min_features, max_features, retries,
          min_thresh, default_thresh, max_thresh, too_many_ratio, too_few_ratio);
  }

  void FeatureDetector::GetDetectorParams(int & min_features, int & max_features, int & max_retries,
                                          double & min_thresh, double & default_thresh,
                                          double & max_thresh, double & too_many_ratio, double & too_few_ratio) {
    if (detector_ == NULL)
      LOG(FATAL) << "The detector was not set.";
    detector_->GetDetectorParams(min_features, max_features, max_retries,
                                 min_thresh, default_thresh, max_thresh, too_many_ratio, too_few_ratio);
  }

  FeatureDetector::~FeatureDetector(void) {
    if (detector_ != NULL) {
      delete detector_;
      detector_ = NULL;
    }
  }

  void FeatureDetector::Reset(std::string const& detector_name, int min_features, int max_features, int retries,
                              double min_thresh, double default_thresh, double max_thresh, double too_many_ratio,
                              double too_few_ratio) {
    detector_name_ = detector_name;

    if (detector_ != NULL) {
      delete detector_;
      detector_ = NULL;
    }

    // Populate the defaults
    if (max_features <= 0) {
      if (detector_name == "SURF") {
        min_features   = FLAGS_min_surf_features;
        max_features   = FLAGS_max_surf_features;
        retries        = FLAGS_detection_retries;
        min_thresh     = FLAGS_min_surf_threshold;
        default_thresh = FLAGS_default_surf_threshold;
        max_thresh     = FLAGS_max_surf_threshold;
        too_many_ratio = FLAGS_surf_too_many_ratio;
        too_few_ratio  = FLAGS_surf_too_few_ratio;
      } else if (detector_name == "ORGBRISK" || detector_name == "TEBLID512" ||
                 detector_name == "TEBLID256") {
        min_features   = FLAGS_min_brisk_features;
        max_features   = FLAGS_max_brisk_features;
        retries        = FLAGS_detection_retries;
        min_thresh     = FLAGS_min_brisk_threshold;
        default_thresh = FLAGS_default_brisk_threshold;
        max_thresh     = FLAGS_max_brisk_threshold;
        too_many_ratio = FLAGS_brisk_too_many_ratio;
        too_few_ratio  = FLAGS_brisk_too_few_ratio;
      } else {
        LOG(FATAL) << "Unimplemented feature detector: " << detector_name;
      }
    }

    // Loading the detector
    if (detector_name == "ORGBRISK")
      detector_ = new BriskDynamicDetector(min_features, max_features, retries,
                                           min_thresh, default_thresh, max_thresh, too_many_ratio, too_few_ratio);
    else if (detector_name == "SURF")
      detector_ = new SurfDynamicDetector(min_features, max_features, retries,
                                          min_thresh, default_thresh, max_thresh, too_many_ratio, too_few_ratio);
    else if (detector_name == "TEBLID512")
      detector_ = new TeblidDynamicDetector(min_features, max_features, retries,
                                          min_thresh, default_thresh, max_thresh, too_many_ratio, too_few_ratio, true);
    else if (detector_name == "TEBLID256")
      detector_ = new TeblidDynamicDetector(min_features, max_features, retries,
                                          min_thresh, default_thresh, max_thresh, too_many_ratio, too_few_ratio, false);
    else
      LOG(FATAL) << "Unimplemented feature detector: " << detector_name;

    LOG(INFO) << "Using descriptor: " << detector_name;
  }

  void FeatureDetector::Detect(const cv::Mat& image,
                               std::vector<cv::KeyPoint>* keypoints,
                               cv::Mat* keypoints_description) {
    if (detector_ == NULL)
      LOG(FATAL) << "The detector was not initialized.";

    detector_->Detect(image, keypoints, keypoints_description);

    // Normalize the image points relative to the center of the image
    for (cv::KeyPoint& key : *keypoints) {
      key.pt.x -= image.cols/2.0;
      key.pt.y -= image.rows/2.0;
    }
  }

  void FindMatches(const cv::Mat& img1_descriptor_map, const cv::Mat& img2_descriptor_map,
                   std::vector<cv::DMatch>* matches, boost::optional<int> hamming_distance,
                   boost::optional<double> goodness_ratio) {
    CHECK(img1_descriptor_map.depth() ==
          img2_descriptor_map.depth())
      << "Mixed descriptor types. Did you mash BRISK with SIFT/SURF?";

    if (!hamming_distance) hamming_distance = FLAGS_hamming_distance;
    if (!goodness_ratio) goodness_ratio = FLAGS_goodness_ratio;

    // Check for early exit conditions
    matches->clear();
    if (img1_descriptor_map.rows == 0 ||
        img2_descriptor_map.rows == 0)
      return;

    if (img1_descriptor_map.depth() == CV_8U) {
      // Binary descriptor
      cv::FlannBasedMatcher matcher(cv::makePtr<cv::flann::LshIndexParams>(3, 18, 2));
      std::vector<std::vector<cv::DMatch> > possible_matches;
      matcher.knnMatch(img1_descriptor_map, img2_descriptor_map, possible_matches, 2);
      matches->clear();
      matches->reserve(possible_matches.size());
      for (std::vector<cv::DMatch> const& best_pair : possible_matches) {
        if (best_pair.size() == 0 || best_pair.at(0).distance > *hamming_distance) continue;
        if (best_pair.size() == 1) {
          // This was the only best match, push it.
          matches->push_back(best_pair.at(0));
        } else {
          // Push back a match only if it is a certain percent better than the next best.
          if (best_pair.at(0).distance < *goodness_ratio * best_pair.at(1).distance) {
            matches->push_back(best_pair[0]);
          }
        }
      }
    } else {
      // Traditional floating point descriptor
      cv::FlannBasedMatcher matcher;
      std::vector<std::vector<cv::DMatch> > possible_matches;
      matcher.knnMatch(img1_descriptor_map, img2_descriptor_map, possible_matches, 2);
      matches->clear();
      matches->reserve(possible_matches.size());
      for (std::vector<cv::DMatch> const& best_pair : possible_matches) {
        if (best_pair.size() == 1) {
          // This was the only best match, push it.
          matches->push_back(best_pair.at(0));
        } else {
          // Push back a match only if it is 25% better than the next best.
          if (best_pair.at(0).distance < *goodness_ratio * best_pair.at(1).distance) {
            matches->push_back(best_pair[0]);
          }
        }
      }
    }
  }
}  // namespace interest_point
