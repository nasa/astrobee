/**
 * @copyright 2021 Xoan Iago Suarez Canosa. All rights reserved.
 * Constact: iago.suarez@thegraffter.com
 * Software developed in the PhD: Low-level vision for resource-limited devices
 */
#ifndef EFFICIENT_DESCRIPTORS_PATCHSIFT_H_
#define EFFICIENT_DESCRIPTORS_PATCHSIFT_H_

#include <vector>
#include <opencv2/opencv.hpp>

namespace upm {

/**
 * Implementation of the SIFT descriptor:
 * Lowe, D. G. (2004). Distinctive image features from scale-invariant keypoints.
 * International journal of computer vision, 60(2), 91-110.
 *
 * This code does not include the detection part. Instead, it is intended to be used with any
 */
class PatchSIFT : public cv::Feature2D {
 public:
  explicit PatchSIFT(float kp_scale,  // Default 1 / 6.0
                     float cropping_scale,
                     double sigma = 1.6,
                     bool step1_pyramid = true,
                     bool step4_gaussian_weight = true,
                     bool step6_trilinear_interp = true,
                     bool step7_l2_normalization = true,
                     bool step8_trim_bigvals = true,
                     bool step9_uchar_scaling = true);

  static cv::Ptr<cv::Feature2D> create(float kp_scale,  // Default 1 / 6.0
                                       float cropping_scale,
                                       double sigma = 1.6,
                                       bool step1_pyramid = true,
                                       bool step4_gaussian_weight = true,
                                       bool step6_trilinear_interp = true,
                                       bool step7_l2_normalization = true,
                                       bool step8_trim_bigvals = true,
                                       bool step9_uchar_scaling = true);

  //! returns the descriptor size in floats (128)
  int descriptorSize() const override;
  int descriptorType() const override { return CV_32FC1; }
  int defaultNorm() const override { return cv::NORM_L2; }

  void compute(cv::InputArray image,
               CV_OUT CV_IN_OUT std::vector<cv::KeyPoint> &keypoints,
               cv::OutputArray descriptors) override;

  void describeKeypoint(const cv::Mat &image, cv::Mat &descriptor);

  void calcSIFTDescriptor(const cv::Mat &img, cv::Mat descriptor);

  inline const cv::Size &GetPatchSize() const { return patch_size; }
  inline const float &GetCroppingScale() const { return cropping_scale; }

  static void rectifyPatch(const cv::Mat &image,
                           const cv::KeyPoint &kp,
                           const cv::Size &patchSize,
                           cv::Mat &patch,
                           float scale_factor);

 protected:
  // The scale of the keypoint relative to the patch size. If will be used to modify the variance
  // of the gaussian that give more weight to the pixels in the center
  float kp_scale;
  double sigma;
  // default number of bins per histogram in descriptor array
  int ori_bins = 8;
  // default width of descriptor histogram array
  int r_bins = 4;
  // default width of descriptor histogram array
  int c_bins = 4;
  // threshold on magnitude of elements of descriptor vector
  float magnitude_th = 0.2f;
  // factor used to convert floating-point descriptor to unsigned char
  float int_descr_factor = 512.f;
  // Steps of the algorithm we can configure
  bool step1_pyramid = true;
  bool step4_gaussian_weight = true;
  bool step6_trilinear_interp = true;
  bool step7_l2_normalization = true;
  bool step8_trim_bigvals = true;
  bool step9_uchar_scaling = true;

  cv::Size patch_size{32, 32};
  float cropping_scale = 1.0f;
};

/******************************* Defs and macros *****************************/

// assumed gaussian blur for input image
static const float SIFT_INIT_SIGMA = 0.5f;

// determines the size of a single descriptor orientation histogram
static const float SIFT_DESCR_SCL_FCTR = 3.f;

class RootPatchSIFT : public PatchSIFT {
 public:
  RootPatchSIFT(float kp_scale,  // Default 1 / 6.0
                float cropping_scale, double sigma = 1.6, bool step1_pyramid = true, bool step4_gaussian_weight = true,
                bool step6_trilinear_interp = true, bool step7_l2_normalization = true, bool step8_trim_bigvals = true,
                bool step9_uchar_scaling = true)
      : PatchSIFT(kp_scale, cropping_scale, sigma, step1_pyramid, step4_gaussian_weight, step6_trilinear_interp,
                  step7_l2_normalization, step8_trim_bigvals, step9_uchar_scaling) {}

  void compute(cv::InputArray image,
               CV_OUT CV_IN_OUT std::vector<cv::KeyPoint> &keypoints,
               cv::OutputArray _descriptors) override {
    PatchSIFT::compute(image, keypoints, _descriptors);
    cv::Mat descriptors = _descriptors.getMat();
    for (int i = 0; i < descriptors.rows; i++) {
      double norm = cv::norm(descriptors.row(i), cv::NORM_L1);
      cv::sqrt(descriptors.row(i) / norm, descriptors.row(i));
    }
  }

  static cv::Ptr<cv::Feature2D> create(float kp_scale,  // Default 1 / 6.0
                                       float cropping_scale, double sigma = 1.6, bool step1_pyramid = true,
                                       bool step4_gaussian_weight = true, bool step6_trilinear_interp = true,
                                       bool step7_l2_normalization = true, bool step8_trim_bigvals = true,
                                       bool step9_uchar_scaling = true) {
    return cv::makePtr<RootPatchSIFT>(kp_scale,
                                      cropping_scale,
                                      sigma,
                                      step1_pyramid,
                                      step4_gaussian_weight,
                                      step6_trilinear_interp,
                                      step7_l2_normalization,
                                      step8_trim_bigvals,
                                      step9_uchar_scaling);
  }
};

}  // namespace upm
#endif  // EFFICIENT_DESCRIPTORS_PATCHSIFT_H_
