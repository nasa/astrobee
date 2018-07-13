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
DEFINE_int32(surf_mode, 1,
             "0: Use default SURF, 1: use dynamic adaptive SURF, 2: Use grid adaptive SURF.");

DEFINE_int32(orgbrisk_octaves, 4,
             "Number of octaves, or scale spaces, that BRISK will evaluate.");
DEFINE_double(orgbrisk_pattern_scale, 1.0,
             "The pattern scale to use for BRISK.");

namespace interest_point {

  DynamicDetector::DynamicDetector(unsigned int min_features, unsigned int max_features, unsigned int max_retries) :
         min_features_(min_features), max_features_(max_features), max_retries_(max_retries) {}

  void DynamicDetector::Detect(const cv::Mat& image,
                               std::vector<cv::KeyPoint>* keypoints,
                               cv::Mat* keypoints_description) {
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
    BriskDynamicDetector(unsigned int min_features, unsigned int max_features,
                         unsigned int max_retries, int threshold)
         : DynamicDetector(min_features, max_features, max_retries), threshold_(threshold) {
      Reset();
    }

    void Reset(void) {
      brisk_ = cv::BRISK::create(threshold_, FLAGS_orgbrisk_octaves,
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
      threshold_ *= 1.3;
      if (threshold_ > 200)
        threshold_ = 200;
      Reset();
    }
    virtual void TooFew(void) {
      threshold_ *= 0.6;
      if (threshold_ < 5)
        threshold_ = 5;
      Reset();
    }

   private:
    cv::Ptr<cv::BRISK> brisk_;
    int threshold_;
  };

  class SurfDynamicDetector : public DynamicDetector {
   public:
    SurfDynamicDetector(unsigned int min_features, unsigned int max_features, unsigned int max_retries, float threshold)
         : DynamicDetector(min_features, max_features, max_retries), threshold_(threshold) {
      surf_ = cv::xfeatures2d::SURF::create(threshold);
    }

    virtual void DetectImpl(const cv::Mat& image, std::vector<cv::KeyPoint>* keypoints) {
      surf_->detect(image, *keypoints);
    }
    virtual void ComputeImpl(const cv::Mat& image, std::vector<cv::KeyPoint>* keypoints,
                             cv::Mat* keypoints_description) {
      surf_->compute(image, *keypoints, *keypoints_description);
    }
    virtual void TooMany(void) {
      threshold_ *= 1.1;
      surf_->setHessianThreshold(threshold_);
    }
    virtual void TooFew(void) {
      threshold_ *= 0.9;
      if (threshold_ < 1.1)
        threshold_ = 1.1;
      surf_->setHessianThreshold(threshold_);
    }

   private:
    cv::Ptr<cv::xfeatures2d::SURF> surf_;
    float threshold_;
  };

  FeatureDetector::FeatureDetector(std::string const& detector_name,
                                   int min_features, int max_features,
                                   int brisk_threshold, int retries) {
    detector_ = NULL;
    Reset(detector_name, min_features, max_features, brisk_threshold, retries);
  }

  FeatureDetector::~FeatureDetector(void) {
    if (detector_ != NULL) {
      delete detector_;
      detector_ = NULL;
    }
  }

  void FeatureDetector::Reset(std::string const& detector_name,
                              int min_features, int max_features,
                              int brisk_threshold, int retries) {
    detector_name_   = detector_name;

    if (detector_ != NULL) {
      delete detector_;
      detector_ = NULL;
    }

    // Loading the detector
    if (detector_name == "ORGBRISK") {
      detector_ = new BriskDynamicDetector(min_features, max_features,
                                           retries, brisk_threshold);
    } else if (detector_name == "SURF") {
      // Note that we use a fixed threshold of 10, rather than brisk_threshold.
      detector_ = new SurfDynamicDetector(min_features, max_features, retries, 10.0);
    } else {
      LOG(FATAL) << "Unimplemented feature detector " << detector_name;
    }
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
