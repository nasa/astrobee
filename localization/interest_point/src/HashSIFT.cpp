/**
 * @copyright 2021 Xoan Iago Suarez Canosa. All rights reserved.
 * Constact: iago.suarez@thegraffter.com
 * Software developed in the PhD: Low-level vision for resource-limited devices
 */
#include <interest_point/HashSIFT.h>

namespace upm {

HashSIFT::HashSIFT(float cropping_scale,
                   HashSiftSize n_bits,
                   double sigma) {
  if (n_bits == SIZE_512_BITS) {
    #include <interest_point/HashSiftWeights512.h>
    b_matrix_ = cv::Mat(512, 129, CV_64FC1, HASH_SIFT_512_VALS);
    b_matrix_.convertTo(b_matrix_, CV_32FC1);
  } else {
    #include <interest_point/HashSiftWeights256.h>
    b_matrix_ = cv::Mat(256, 129, CV_64FC1, HASH_SIFT_256_VALS);
    b_matrix_.convertTo(b_matrix_, CV_32FC1);
  }

  nbits_ = b_matrix_.rows;
  transp_b_matrix_ = b_matrix_.t();

  // 1 / 6.75f
  sift_ = std::make_shared<PatchSIFT>(1 / 6.0, cropping_scale, sigma);
}

cv::Ptr<cv::Feature2D> HashSIFT::create(float cropping_scale,
                                        HashSiftSize n_bits,
                                        double sigma) {
  return cv::makePtr<HashSIFT>(cropping_scale, n_bits, sigma);
}

void HashSIFT::compute(cv::InputArray& _image, std::vector<cv::KeyPoint>& keypoints, cv::OutputArray& _descriptors) {
  cv::Mat image = _image.getMat();

  if (image.empty())
    return;

  if (keypoints.empty()) {
    // clean output buffer (it may be reused with "allocated" data)
    _descriptors.release();
    return;
  }

  assert(image.type() == CV_8UC1);

  // Create the output array of descriptors
  _descriptors.create((int)keypoints.size(), descriptorSize(), descriptorType());
  cv::Mat descriptors = _descriptors.getMat();
  cv::Mat sift_descriptors(keypoints.size(), 129, CV_32FC1);
  sift_descriptors.col(0).setTo(1);
  cv::Mat wlInt8tResponses(keypoints.size(), 129, CV_8UC1);

#ifndef UPM_PARALLEL_HASH_SIFT
  const cv::Range range(0, keypoints.size());
#else
  parallel_for_(cv::Range(0, keypoints.size()), [&](const cv::Range &range) {
#endif
    cv::Mat patch(sift_->GetPatchSize(), CV_8UC1);
    for (int r = range.start; r < range.end; r++) {
      PatchSIFT::rectifyPatch(image, keypoints[r], sift_->GetPatchSize(), patch,
                              sift_->GetCroppingScale());
      sift_->calcSIFTDescriptor(patch,
                                sift_descriptors(cv::Range(r, r + 1),
                                                 cv::Range(1, sift_descriptors.cols)));
    }

    // Compute the linear projection and hash
    matmulAndSign(sift_descriptors.rowRange(range), descriptors.rowRange(range));

#ifdef UPM_PARALLEL_HASH_SIFT
  });
#endif
  }  // namespace upm

void HashSIFT::matmulAndSign(const cv::Mat &wlResponses, cv::Mat descriptors) {
  assert(wlResponses.rows == descriptors.rows);
  cv::Mat tmp = wlResponses * transp_b_matrix_;
  float *pTmp = tmp.ptr<float>();
  uint8_t *p_descr = descriptors.ptr<uint8_t>();
  uint8_t byte;
  int d, b;
  for (d = 0; d < descriptors.rows; d++) {
    for (b = 0; b < tmp.cols / 8; b++) {
      byte = 0;
      byte |= (*pTmp++ > 0) << 7;
      byte |= (*pTmp++ > 0) << 6;
      byte |= (*pTmp++ > 0) << 5;
      byte |= (*pTmp++ > 0) << 4;
      byte |= (*pTmp++ > 0) << 3;
      byte |= (*pTmp++ > 0) << 2;
      byte |= (*pTmp++ > 0) << 1;
      byte |= (*pTmp++ > 0) << 0;
      *p_descr++ = byte;
    }  // End of for each bit
  }  // End of for each keypoint
}
}
