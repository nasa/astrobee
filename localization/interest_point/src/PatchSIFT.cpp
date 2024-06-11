/**
 * @copyright 2021 Xoan Iago Suarez Canosa. All rights reserved.
 * Constact: iago.suarez@thegraffter.com
 * Software developed in the PhD: Low-level vision for resource-limited devices
 */
#include <interest_point/PatchSIFT.h>

namespace upm {

void PatchSIFT::calcSIFTDescriptor(const cv::Mat &gray, cv::Mat descriptor) {
  assert(descriptor.type() == CV_32FC1);
  assert(descriptor.rows == 1 && descriptor.cols == descriptorSize());
  assert(gray.channels() == 1);

  int i, j, k, r, c;

  // Step 1: Filter the image using a gaussian filter of size sigma
  cv::Mat img;
  if (step1_pyramid) {
    float sig_diff = sqrtf(std::max(float(sigma) * float(sigma) - SIFT_INIT_SIGMA * SIFT_INIT_SIGMA, 0.01f));
    cv::GaussianBlur(gray, img, cv::Size(), sig_diff, sig_diff);
  } else {
    img = gray;
  }
  // Step 2: We assume a pre-oriented and pre-scale patch of fixed size

  // Step 3: Compute the image derivatives
  // X and Y derivatives
  cv::Mat X(img.rows - 2, img.cols - 2, CV_32FC1);
  cv::Mat Y(img.rows - 2, img.cols - 2, CV_32FC1);
  // Row and column bins
  cv::Mat RBin(img.rows - 2, img.cols - 2, CV_32FC1);
  cv::Mat CBin(img.rows - 2, img.cols - 2, CV_32FC1);

  // The radius of the keypoint in pixels
  const float cell_width = SIFT_DESCR_SCL_FCTR * (kp_scale * img.cols * 0.5f);
  const float cell_height = SIFT_DESCR_SCL_FCTR * (kp_scale * img.rows * 0.5f);

  float *pX = X.ptr<float>(), *pY = Y.ptr<float>(), *pRBin = RBin.ptr<float>(),
      *pCBin = CBin.ptr<float>();
  float c_rot, r_rot;
  uint8_t *p, *pImg = img.ptr<uint8_t>();

  int img_width = img.cols, pos = 0;
  // Compute image derivatives
  for (i = 1; i < (img.rows - 1); i++) {
    p = pImg + i * img_width + 1;
    for (j = 1; j < (img.cols - 1); j++) {
      // Compute the derivative using the previous and next pixels
      pX[pos] = *(p + 1) - *(p - 1);
      pY[pos] = *(p - img_width) - *(p + img_width);

      // Assign each pixels to its bin
      c_rot = (j - img.cols / 2.0f) / cell_width;
      r_rot = (i - img.rows / 2.0f) / cell_height;
      // Check that the indices of the histogram are correct
      assert((r_rot + r_bins / 2 - 0.5f) > -2 && (r_rot + r_bins / 2 - 0.5f) <= r_bins);
      assert((c_rot + c_bins / 2 - 0.5f) > -2 && (c_rot + c_bins / 2 - 0.5f) <= c_bins);
      pRBin[pos] = r_rot + r_bins / 2 - 0.5f;
      pCBin[pos] = c_rot + c_bins / 2 - 0.5f;
      pos++;
      p++;
    }
  }

  // Compute the gradient direction and magnitude
  cv::Mat Mag, Ori;
  cv::cartToPolar(X, Y, Mag, Ori);
  float *pMag = Mag.ptr<float>(), *pOri = Ori.ptr<float>();

  // Step 4: Create the gaussian weighting assigned to each pixel
  if (step4_gaussian_weight) {
    cv::Mat W = cv::Mat::ones(img.rows - 2, img.cols - 2, CV_32FC1);
    const float kp_radius = kp_scale * img.rows * 0.5f;
    float kernel_sigma = 0.5 * c_bins * SIFT_DESCR_SCL_FCTR * kp_radius;
    float dx, dy, dist_to_center2;
    float *pW = W.ptr<float>();
    for (r = 0; r < W.rows; r++) {
      for (c = 0; c < W.cols; c++) {
        dx = r - W.cols / 2.0f;
        dy = c - W.rows / 2.0f;
        dist_to_center2 = dx * dx + dy * dy;
        *pW = -dist_to_center2 / (2 * kernel_sigma * kernel_sigma);
        pW++;
      }
    }
    // Multiply the gradient magnitude by the importance of each pixel
    cv::exp(W, W);
    Mag = Mag.mul(W);
  }

  // Step 5: Build the histogram of orientations
  // We have an histogram bigger than usual to capture the circular
  float histogram[r_bins + 2][c_bins + 2][ori_bins + 2];
  // Fill the histogram with zeros
  std::fill(&histogram[0][0][0], &histogram[r_bins + 1][c_bins + 1][ori_bins + 1], 0);
  float bins_per_rad = ori_bins / (2 * M_PI);
  int r0, c0, o0;
  float rbin, obin, mag, cbin, v_r1, v_rc11, v_rc01, v_rco111, v_rco101, v_rco011, v_rco001, v_r0,
      v_rc10, v_rc00,
      v_rco110, v_rco100, v_rco010, v_rco000;
  pos = 0;
  for (r = 0; r < Ori.rows; r++) {
    for (c = 0; c < Ori.cols; c++) {
      rbin = pRBin[pos];
      cbin = pCBin[pos];
      obin = pOri[pos] * bins_per_rad;
      mag = pMag[pos];

      // Step6: Histogram update using tri-linear interpolation
      if (step6_trilinear_interp) {
        r0 = cvFloor(rbin);
        c0 = cvFloor(cbin);
        o0 = cvFloor(obin);
        rbin -= r0;
        cbin -= c0;
        obin -= o0;

        // If the orientation is out of the bin center it
        if (o0 < 0) o0 += ori_bins;
        if (o0 >= ori_bins) o0 -= ori_bins;

        v_r1 = mag * rbin, v_r0 = mag - v_r1;
        v_rc11 = v_r1 * cbin, v_rc10 = v_r1 - v_rc11;
        v_rc01 = v_r0 * cbin, v_rc00 = v_r0 - v_rc01;
        v_rco111 = v_rc11 * obin, v_rco110 = v_rc11 - v_rco111;
        v_rco101 = v_rc10 * obin, v_rco100 = v_rc10 - v_rco101;
        v_rco011 = v_rc01 * obin, v_rco010 = v_rc01 - v_rco011;
        v_rco001 = v_rc00 * obin, v_rco000 = v_rc00 - v_rco001;

        histogram[r0 + 1][c0 + 1][o0] += v_rco000;
        histogram[r0 + 1][c0 + 1][o0 + 1] += v_rco001;
        histogram[r0 + 1][c0 + 2][o0] += v_rco010;
        histogram[r0 + 1][c0 + 2][o0 + 1] += v_rco011;
        histogram[r0 + 2][c0 + 1][o0] += v_rco100;
        histogram[r0 + 2][c0 + 1][o0 + 1] += v_rco101;
        histogram[r0 + 2][c0 + 2][o0] += v_rco110;
        histogram[r0 + 2][c0 + 2][o0 + 1] += v_rco111;
      } else {
        // Hard nearest neighbour assignation
        r0 = cvRound(rbin);
        c0 = cvRound(cbin);
        o0 = cvRound(obin);

        if (o0 < 0) o0 += ori_bins;
        if (o0 >= ori_bins) o0 -= ori_bins;

        histogram[r0 + 1][c0 + 1][o0] += mag;
      }
      pos++;
    }
  }

  // Finalize histogram, since the orientation histograms are circular
  float *p_dscr = descriptor.ptr<float>();
  for (r = 0; r < r_bins; r++)
    for (c = 0; c < c_bins; c++) {
      // Increase the value in the penultimate orientation bin in the first one
      histogram[r + 1][c + 1][0] += histogram[r + 1][c + 1][ori_bins];
      // Increase the value in last orientation bin in the second one
      histogram[r + 1][c + 1][1] += histogram[r + 1][c + 1][ori_bins + 1];
      // Copy the values in the histogram to the output destination
      for (k = 0; k < ori_bins; k++)
        p_dscr[(r * r_bins + c) * ori_bins + k] = histogram[r + 1][c + 1][k];
    }

  // Step 7: Apply L2 normalization
  if (step7_l2_normalization) {
    float dscr_norm = std::max(float(cv::norm(descriptor)), FLT_EPSILON);
    descriptor /= dscr_norm;
  }

  // Step 8: Trim Big Values
  if (step8_trim_bigvals) {
    for (i = 0; i < descriptor.cols; i++) p_dscr[i] = std::min(p_dscr[i], magnitude_th);
    float dscr_norm = std::max(float(cv::norm(descriptor)), FLT_EPSILON);
    descriptor /= dscr_norm;
  }

  // Optional Step 9: Scale the result, so that it can be easily converted to byte array
  if (step9_uchar_scaling) {
    for (k = 0; k < descriptor.cols; k++) {
      p_dscr[k] = cv::saturate_cast<uchar>(p_dscr[k] * int_descr_factor);
    }
  }
}

//////////////////////////////////////////////////////////////////////////////////////////

PatchSIFT::PatchSIFT(float kp_scale,
                     float cropping_scale,
                     double sigma,
                     bool step1_pyramid,
                     bool step4_gaussian_weight,
                     bool step6_trilinear_interp,
                     bool step7_l2_normalization,
                     bool step8_trim_bigvals,
                     bool step9_uchar_scaling)
    : kp_scale(kp_scale),
      cropping_scale(cropping_scale),
      sigma(sigma),
      step1_pyramid(step1_pyramid),
      step4_gaussian_weight(step4_gaussian_weight),
      step6_trilinear_interp(step6_trilinear_interp),
      step7_l2_normalization(step7_l2_normalization),
      step8_trim_bigvals(step8_trim_bigvals),
      step9_uchar_scaling(step9_uchar_scaling) {
}

void PatchSIFT::compute(const cv::_InputArray& _image, std::vector<cv::KeyPoint>& keypoints,
                        const cv::_OutputArray& _descriptors) {
  cv::Mat image = _image.getMat();
  _descriptors.create((int)keypoints.size(), descriptorSize(), descriptorType());
  cv::Mat descriptors = _descriptors.getMat();

  auto describe_kps_range = [&](const cv::Range& range) {
    for (int r = range.start; r < range.end; r++) {
      cv::Mat patch;
      rectifyPatch(image, keypoints[r], patch_size, patch, cropping_scale);
      calcSIFTDescriptor(patch, descriptors.row(r));
    }
  };
#ifdef UPM_PARALLEL_SIFT
  parallel_for_(cv::Range(0, keypoints.size()), describe_kps_range);
#else
  describe_kps_range(cv::Range(0, keypoints.size()));
#endif
}

int PatchSIFT::descriptorSize() const {
  return r_bins * c_bins * ori_bins;
}

void PatchSIFT::describeKeypoint(const cv::Mat &image, cv::Mat &descriptor) {
  if (image.empty()) {
    descriptor.release();
    return;
  }

  if (image.empty() || image.depth() != CV_8U)
    CV_Error(cv::Error::StsBadArg, "image is empty or has incorrect depth (!=CV_8U)");

  cv::Mat gray;
  if (image.channels() == 3 || image.channels() == 4)
    cvtColor(image, gray, cv::COLOR_BGR2GRAY);
  else
    gray = image;

  descriptor.create(1, descriptorSize(), CV_32F);
  calcSIFTDescriptor(gray, descriptor);
}

cv::Ptr<cv::Feature2D> PatchSIFT::create(float kp_scale,
                                         float cropping_scale,
                                         double sigma,
                                         bool step1_pyramid,
                                         bool step4_gaussian_weight,
                                         bool step6_trilinear_interp,
                                         bool step7_l2_normalization,
                                         bool step8_trim_bigvals,
                                         bool step9_uchar_scaling) {
  return cv::makePtr<PatchSIFT>(kp_scale,
                                cropping_scale,
                                sigma,
                                step1_pyramid,
                                step4_gaussian_weight,
                                step6_trilinear_interp,
                                step7_l2_normalization,
                                step8_trim_bigvals,
                                step9_uchar_scaling);
}
void PatchSIFT::rectifyPatch(const cv::Mat &image,
                             const cv::KeyPoint &kp,
                             const cv::Size &patchSize,
                             cv::Mat &patch,
                             float scale_factor) {
  const float s = scale_factor * kp.size / (0.5f * (patchSize.width + patchSize.height));

  const float cosine = (kp.angle >= 0) ? cos(kp.angle * (float)CV_PI / 180.0f) : 1.f;
  const float sine = (kp.angle >= 0) ? sin(kp.angle * (float)CV_PI / 180.0f) : 0.f;

  cv::Matx23f M(s * cosine, -s * sine, (-s * cosine + s * sine) * patchSize.width / 2.0f + kp.pt.x,
                s * sine, s * cosine, (-s * sine - s * cosine) * patchSize.height / 2.0f + kp.pt.y);
  warpAffine(image,
             patch,
             M,
             patchSize,
             cv::WARP_INVERSE_MAP + cv::INTER_CUBIC + cv::WARP_FILL_OUTLIERS,
             cv::BORDER_REPLICATE);
}

}  // namespace upm
