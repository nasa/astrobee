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

#include <interest_point/matching.h>
#include <interest_point/brisk.h>
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
DEFINE_int32(hamming_distance, 90,
             "A smaller value keeps fewer but more reliable binary descriptor matches.");
DEFINE_double(goodness_ratio, 0.8,
              "A smaller value keeps fewer but more reliable float descriptor matches.");
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
// ORGBRISK detector
DEFINE_int32(min_brisk_features, 400,
             "Minimum number of features to be computed using ORGBRISK.");
DEFINE_int32(max_brisk_features, 800,
             "Maximum number of features to be computed using ORGBRISK.");
DEFINE_double(min_brisk_threshold, 20,
              "Minimum threshold for feature detection using ORGBRISK.");
DEFINE_double(default_brisk_threshold, 90,
              "Default threshold for feature detection using ORGBRISK.");
DEFINE_double(max_brisk_threshold, 110,
              "Maximum threshold for feature detection using ORGBRISK.");

namespace interest_point {

  DynamicDetector::DynamicDetector(int min_features, int max_features, int max_retries,
                                   double min_thresh, double default_thresh, double max_thresh):
    min_features_(min_features), max_features_(max_features), max_retries_(max_retries),
    min_thresh_(min_thresh), default_thresh_(default_thresh), max_thresh_(max_thresh),
    dynamic_thresh_(default_thresh) {}

  void DynamicDetector::GetDetectorParams(int & min_features, int & max_features, int & max_retries,
                                          double & min_thresh, double & default_thresh,
                                          double & max_thresh) {
    min_features = min_features_; max_features = max_features_; max_retries = max_retries_;
    min_thresh = min_thresh_; default_thresh = default_thresh_; max_thresh = max_thresh_;
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
    BriskDynamicDetector(int min_features, int max_features, int max_retries,
                         double min_thresh, double default_thresh, double max_thresh)
      : DynamicDetector(min_features, max_features, max_retries,
                        min_thresh, default_thresh, max_thresh) {
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
      dynamic_thresh_ *= 1.25;
      dynamic_thresh_ = static_cast<int>(dynamic_thresh_);  // for backwards compatibility
      if (dynamic_thresh_ > max_thresh_)
        dynamic_thresh_ = max_thresh_;
      brisk_->setThreshold(dynamic_thresh_);
    }
    virtual void TooFew(void) {
      dynamic_thresh_ *= 0.8;
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
    SurfDynamicDetector(int min_features, int max_features, int max_retries,
                        double min_thresh, double default_thresh, double max_thresh)
      : DynamicDetector(min_features, max_features, max_retries,
                        min_thresh, default_thresh, max_thresh) {
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
      dynamic_thresh_ *= 1.1;
      if (dynamic_thresh_ > max_thresh_)
        dynamic_thresh_ = max_thresh_;
      surf_->setHessianThreshold(static_cast<float>(dynamic_thresh_));
    }
    virtual void TooFew(void) {
      dynamic_thresh_ *= 0.9;
      if (dynamic_thresh_ < min_thresh_)
        dynamic_thresh_ = min_thresh_;
      surf_->setHessianThreshold(static_cast<float>(dynamic_thresh_));
    }

   private:
    cv::Ptr<cv::xfeatures2d::SURF> surf_;
  };

  FeatureDetector::FeatureDetector(std::string const& detector_name,
                                   int min_features, int max_features, int retries,
                                   double min_thresh, double default_thresh, double max_thresh) {
    detector_ = NULL;
    Reset(detector_name, min_features, max_features, retries,
          min_thresh, default_thresh, max_thresh);
  }

  void FeatureDetector::GetDetectorParams(int & min_features, int & max_features, int & max_retries,
                                          double & min_thresh, double & default_thresh,
                                          double & max_thresh) {
    if (detector_ == NULL)
      LOG(FATAL) << "The detector was not set.";
    detector_->GetDetectorParams(min_features, max_features, max_retries,
                                 min_thresh, default_thresh, max_thresh);
  }

  FeatureDetector::~FeatureDetector(void) {
    if (detector_ != NULL) {
      delete detector_;
      detector_ = NULL;
    }
  }

  void FeatureDetector::Reset(std::string const& detector_name,
                              int min_features, int max_features, int retries,
                              double min_thresh, double default_thresh, double max_thresh) {
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
      } else if (detector_name == "ORGBRISK") {
        min_features   = FLAGS_min_brisk_features;
        max_features   = FLAGS_max_brisk_features;
        retries        = FLAGS_detection_retries;
        min_thresh     = FLAGS_min_brisk_threshold;
        default_thresh = FLAGS_default_brisk_threshold;
        max_thresh     = FLAGS_max_brisk_threshold;
      } else {
        LOG(FATAL) << "Unimplemented feature detector: " << detector_name;
      }
    }

    // Loading the detector
    if (detector_name == "ORGBRISK")
      detector_ = new BriskDynamicDetector(min_features, max_features, retries,
                                           min_thresh, default_thresh, max_thresh);
    else if (detector_name == "SURF")
      detector_ = new SurfDynamicDetector(min_features, max_features, retries,
                                          min_thresh, default_thresh, max_thresh);
    else
      LOG(FATAL) << "Unimplemented feature detector: " << detector_name;
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

  void FindMatches(const cv::Mat & img1_descriptor_map,
                   const cv::Mat & img2_descriptor_map, std::vector<cv::DMatch> * matches) {
    CHECK(img1_descriptor_map.depth() ==
          img2_descriptor_map.depth())
      << "Mixed descriptor types. Did you mash BRISK with SIFT/SURF?";

    // Check for early exit conditions
    matches->clear();
    if (img1_descriptor_map.rows == 0 ||
        img2_descriptor_map.rows == 0)
      return;

    if (img1_descriptor_map.depth() == CV_8U) {
      // Binary descriptor

      // cv::BFMatcher matcher(cv::NORM_HAMMING, true  /* Forward & Backward matching */);
      cv::FlannBasedMatcher matcher(cv::makePtr<cv::flann::LshIndexParams>(3, 18, 2));
      matcher.match(img1_descriptor_map, img2_descriptor_map, *matches);

      // Select only inlier matches that meet a BRISK threshold of
      // of FLAGS_hamming_distance.
      // TODO(oalexan1) This needs further study.
      std::vector<cv::DMatch> inlier_matches;
      inlier_matches.reserve(matches->size());  // This saves time in allocation
      for (cv::DMatch const& dmatch : *matches) {
        if (dmatch.distance < FLAGS_hamming_distance) {
          inlier_matches.push_back(dmatch);
        }
      }
      matches->swap(inlier_matches);  // Doesn't invoke a copy of all elements.
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
          if (best_pair.at(0).distance < FLAGS_goodness_ratio * best_pair.at(1).distance) {
            matches->push_back(best_pair[0]);
          }
        }
      }
    }
  }
}  // namespace interest_point
