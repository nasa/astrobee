/**
 * @copyright 2021 Xoan Iago Suarez Canosa. All rights reserved.
 * Constact: iago.suarez@thegraffter.com
 * Software developed in the PhD: Low-level vision for resource-limited devices
 */
#ifndef EFFICIENT_DESCRIPTORS_HASHSIFT_H_
#define EFFICIENT_DESCRIPTORS_HASHSIFT_H_

#include <opencv2/opencv.hpp>
#include <string>
#include <memory>
#include <interest_point/PatchSIFT.h>

namespace upm {
/**
 * HashSIFT descriptor described in the article
 * "Suarez, I., Buenaposada, J. M., & Baumela, L. (2021). Revisiting Binary Local Image
 * Description for Resource Limited Devices. IEEE Robotics and Automation Letters."
 *
 * The descriptor computes first the SIFT features using the class PatchSIFT and then it hashes
 * the floating point SIFT descriptor with a pre-learnt linear projection: b_matrix_
 * The weights b_matrix_ are loaded from the files HashSiftWeights256.i and HashSiftWeights512.i.
 */
class HashSIFT : public cv::Feature2D {
 public:
  /**
  * @brief  Descriptor number of bits, each bit is a weak-descriptor.
  * The user can choose between 512 or 256 bits.
  */
  enum HashSiftSize {
    SIZE_512_BITS = 100, SIZE_256_BITS = 101,
  };

  /**
   *
   * @param cropping_scale Determines the size of the patch cropped for description.
   * The diameter of the patch will be: cropping_scale * kp.size
   * @param n_bits The number of bits of the descriptor (512 or 256)
   * @param sigma The standard deviation of the gaussian smoothing filter applied to the patch
   * before description.
   */
  explicit HashSIFT(float cropping_scale,
                    HashSiftSize n_bits = SIZE_256_BITS,
                    double sigma = 1.6);

  /**
   * Creates a pointer to a new instance
   * @param cropping_scale Determines the size of the patch cropped for description.
   * The diameter of the patch will be: cropping_scale * kp.size
   * @param n_bits The number of bits of the descriptor (512 or 256)
   * @param sigma The standard deviation of the gaussian smoothing filter applied to the patch
   * before description.
   * @return A pointer to the new cv::Feature2D descriptor object
   */
  static cv::Ptr<cv::Feature2D> create(float cropping_scale,
                                       HashSiftSize n_bits = SIZE_256_BITS,
                                       double sigma = 1.6);

  /**
 * Computes the descriptors corresponding to the specified Keypoints
 * @param image The input grayscale image
 * @param keypoints The detected input keypoints
 * @param descriptors The output descriptors. The shape of the output cv::Mat will be:
 * n_keypoints x (nbits / 8)
 */
  void compute(cv::InputArray image, CV_OUT CV_IN_OUT std::vector<cv::KeyPoint>& keypoints,
               cv::OutputArray descriptors) override;

  int descriptorSize() const override { return nbits_ / 8; }
  int descriptorType() const override { return CV_8UC1; }
  int defaultNorm() const override { return cv::NORM_HAMMING; }

 private:
  /**
   * @brief Transforms the wlResponses with the B^T matrix and apply the sign operation.
   * The matrix multiplication is done using floating point arithmetic from OpenCV
   * @param wlResponses A floating point matrix with the WL responses. the shape is:
   * (n_descriptors, n_wls) i.e. each descriptor is a row of the matrix
   * @param descriptors The Output matrix where we are going to store  the resulting binary descriptors.
   */
  void matmulAndSign(const cv::Mat &wlResponses, cv::Mat descriptors);

  std::shared_ptr<PatchSIFT> sift_{};
  int nbits_{-1};
  cv::Mat_<float> b_matrix_;
  cv::Mat_<float> transp_b_matrix_;
};

}  // namespace upm
#endif  // EFFICIENT_DESCRIPTORS_HASHSIFT_H_
