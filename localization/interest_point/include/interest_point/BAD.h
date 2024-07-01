/**
 * @copyright 2021 Xoan Iago Suarez Canosa. All rights reserved.
 * Constact: iago.suarez@thegraffter.com
 * Software developed in the PhD: Low-level vision for resource-limited devices
 * Repo link: https://github.com/iago-suarez/efficient-descriptors
 * License: Apache 2
 * TODO(rsoussan): Remove this and BAD.cpp and switch to opencv TEBLID if opencv
 * version upgraded.
 */

#ifndef INTEREST_POINT_BAD_H_
#define INTEREST_POINT_BAD_H_

#include <opencv2/opencv.hpp>

#include <vector>
#include <string>

namespace upm {

/**
 * Implementation of the Box Average Difference (BAD) descriptor. The method uses features
 * computed from the difference of the average gray values of two boxes in the patch.
 *
 * Each pair of boxes is represented with a BoxPairParams struct. After obtaining the feature
 * from them, the i'th feature is thresholded with thresholds_[i], producing the binary
 * weak-descriptor.
 */
class BAD : public cv::Feature2D {
 public:
  /**
     * @brief  Descriptor number of bits, each bit is a weak-descriptor.
     * The user can choose between 512 or 256 bits.
     */
  enum BadSize {
    SIZE_512_BITS = 100, SIZE_256_BITS = 101,
  };

  /**
   * @param scale_factor Adjust the sampling window around detected keypoints:
    - <b> 1.00f </b> should be the scale for ORB keypoints
    - <b> 6.75f </b> should be the scale for SIFT detected keypoints
    - <b> 6.25f </b> is default and fits for KAZE, SURF detected keypoints
    - <b> 5.00f </b> should be the scale for AKAZE, MSD, AGAST, FAST, BRISK keypoints
   * @param n_bits Determine the number of bits in the descriptor. Should be either
     BAD::SIZE_512_BITS or BAD::SIZE_256_BITS.
   */
  explicit BAD(float scale_factor = 1.0f, BadSize n_bits = SIZE_512_BITS);

  /**
   * @brief Creates the BAD descriptor.
   * @param scale_factor Adjust the sampling window around detected keypoints:
    - <b> 1.00f </b> should be the scale for ORB keypoints
    - <b> 6.75f </b> should be the scale for SIFT detected keypoints
    - <b> 6.25f </b> is default and fits for KAZE, SURF detected keypoints
    - <b> 5.00f </b> should be the scale for AKAZE, MSD, AGAST, FAST, BRISK keypoints
   * @param n_bits
   * @return
   */
  static cv::Ptr<BAD> create(float scale_factor = 1.0f, BadSize n_bits = SIZE_512_BITS) {
    return cv::makePtr<upm::BAD>(scale_factor, n_bits);
  }

  /** @brief Computes the descriptors for a set of keypoints detected in an image (first variant) or image set
  (second variant).

  @param image Image.
  @param keypoints Input collection of keypoints. Keypoints for which a descriptor cannot be
  computed are removed. Sometimes new keypoints can be added, for example: SIFT duplicates keypoint
  with several dominant orientations (for each orientation).
  @param descriptors Computed descriptors. In the second variant of the method descriptors[i] are
  descriptors computed for a keypoints[i]. Row j is the keypoints (or keypoints[i]) is the
  descriptor for keypoint j-th keypoint.
   */
  void compute(cv::InputArray image,
               CV_OUT CV_IN_OUT std::vector<cv::KeyPoint> &keypoints,
               cv::OutputArray descriptors) override;

  int descriptorSize() const override { return box_params_.size() / 8; }
  int descriptorType() const override { return CV_8UC1; }
  int defaultNorm() const override { return cv::NORM_HAMMING; }
  bool empty() const override { return false; }

  cv::String getDefaultName() const override { return std::string("BAD") + std::to_string(box_params_.size()); }

  // Struct representing a pair of boxes in the patch
  struct BoxPairParams {
    int x1, x2, y1, y2, boxRadius;
  };

 protected:
  // Computes the BADdescriptor
  void computeBAD(const cv::Mat &integral_img,
                  const std::vector<cv::KeyPoint> &keypoints,
                  cv::Mat &descriptors);

  std::vector<float> thresholds_;
  std::vector<BoxPairParams> box_params_;
  float scale_factor_ = 1;
  cv::Size patch_size_ = {32, 32};
};

}  // namespace upm
#endif  // INTEREST_POINT_BAD_H_
