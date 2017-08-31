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
#ifndef INTEREST_POINT_MATCHING_H_
#define INTEREST_POINT_MATCHING_H_

#include <opencv2/features2d/features2d.hpp>
#include <Eigen/Core>

#include <vector>
#include <string>
#include <map>

namespace interest_point {

  class DynamicDetector {
   public:
    DynamicDetector(unsigned int min_features, unsigned int max_features, unsigned int max_retries);
    virtual ~DynamicDetector(void) {}
    void Detect(const cv::Mat& image,
                        std::vector<cv::KeyPoint>* keypoints,
                        cv::Mat* keypoints_description);
    virtual void DetectImpl(const cv::Mat& image,
                            std::vector<cv::KeyPoint>* keypoints) = 0;
    virtual void ComputeImpl(const cv::Mat& image,
                            std::vector<cv::KeyPoint>* keypoints,
                            cv::Mat* keypoints_description) = 0;
    virtual void TooFew(void) = 0;
    virtual void TooMany(void) = 0;
   private:
    unsigned int min_features_;
    unsigned int max_features_;
    unsigned int max_retries_;
  };

  class FeatureDetector {
   private:
    DynamicDetector* detector_;
    std::string detector_name_;

   public:
    FeatureDetector(std::string const& detector_name = "SURF",
                    int min_features = 400, int max_features = 1000,
                    int brisk_threshold = 15, int retries = 5);
    ~FeatureDetector(void);

    void Reset(std::string const& detector_name,
               int min_features = 400, int max_features = 1000,
               int brisk_threshold = 15, int retries = 5);

    void Detect(const cv::Mat& image,
                    std::vector<cv::KeyPoint>* keypoints,
                    cv::Mat* keypoints_description);

    std::string GetDetectorName() const {return detector_name_;}

    friend bool operator== (FeatureDetector const& A, FeatureDetector const& B) {
      return (A.detector_name_ == B.detector_name_);
    }
  };

  /**
   * descriptor is what opencv descriptor was used to make the descriptors
   * the descriptor maps are the features in the two images
   * matches is output to contain the matching features between the two images
   **/
  void FindMatches(const cv::Mat & img1_descriptor_map,
                   const cv::Mat & img2_descriptor_map,
                   std::vector<cv::DMatch> * matches);
}  // namespace interest_point

#endif  // INTEREST_POINT_MATCHING_H_
