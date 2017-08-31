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

#ifndef CAMERA_CAMERA_PARAMS_H_
#define CAMERA_CAMERA_PARAMS_H_

#include <config_reader/config_reader.h>
#include <Eigen/Core>

#include <string>
#include <vector>
#include <algorithm>

// Forward declare mat type so that we don't have to include OpenCV if we're
// not going to use it.
namespace cv {
  class Mat;
}  // end namespace cv

// Functionality for undistorting and re-distorting images
namespace camera {
  const char distortion_msg[] = "Tangent distortion constant for this "
    "camera. Use 0.923169 for ISS imagery, and 0.925417 for images "
    "from the tango device.";

  // Definition of camera frames
  //
  // RAW - This is how the image came to us.
  // DISTORTED - Similar to RAW, but we've cropped out everything that is
  // non-image
  // DISTORTED_C - Same as DISTORTED, but the origin is the center of the image
  // UNDISTORTED - A new camera frame that is create by remove lens distortion
  // and placing the optical center directly center of the image.
  // UNDISTORTED_C - Same as UNDISTORTED, but the origin is the center of the image
  enum {
    RAW,
    DISTORTED,
    DISTORTED_C,
    UNDISTORTED,
    UNDISTORTED_C
  };

  class CameraParameters {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    // Please load from file or specify all variables
    CameraParameters() = delete;
    explicit CameraParameters(std::string const& filename, std::string const& base_dir = "");
    explicit CameraParameters(config_reader::ConfigReader* config, const char* name);
    CameraParameters(Eigen::Vector2i const& image_size,
                     Eigen::Vector2d const& focal_length,
                     Eigen::Vector2d const& optical_center,
                     Eigen::VectorXd const& distortion = Eigen::VectorXd());

    // Used to create a remap table. This table is the same size as the
    // UNDISTORTED image, where every pixel's value is the cooresponding pixel
    // location in the DISTORTED image.
    void GenerateRemapMaps(cv::Mat* remap_map);

    // Conversion utilities
    template <int SRC, int DEST>
    void Convert(Eigen::Vector2d const& input, Eigen::Vector2d *output) const {
      throw("Please use the explicitly specified conversions by using the correct enum.");
    }

    // Utility to create intrinsic matrix for the correct coordinate frame
    template <int FRAME>
    Eigen::Matrix3d GetIntrinsicMatrix() const {
      throw("Please use the explicitly specified frame by using the correct enum.");
    }

    // Image size info. We don't give direct access because if the size
    // changes, we need to recompute some temporary variables.
    //   These apply to coordinate frames DISTORTED, DISTORTED_C
    void SetDistortedSize(Eigen::Vector2i const& image_size);
    const Eigen::Vector2i& GetDistortedSize() const;
    const Eigen::Vector2d& GetDistortedHalfSize() const;

    //   These apply to UNDISTORTED, UNDISTORTED_C
    void SetUndistortedSize(Eigen::Vector2i const& image_size);
    const Eigen::Vector2i& GetUndistortedSize() const;
    const Eigen::Vector2d& GetUndistortedHalfSize() const;

    // Start of DISTORTED inside RAW frame
    void SetCropOffset(Eigen::Vector2i const& crop);
    const Eigen::Vector2i& GetCropOffset() const;
    // Optical offset in DISTORTED frame
    void SetOpticalOffset(Eigen::Vector2d const& offset);
    const Eigen::Vector2d& GetOpticalOffset() const;

    // Helper function that approximates the focal length as a single value.
    // Will produce wrong results if pixels are not square. Should be exactly
    // correct if image is UNDISTORTED.
    void SetFocalLength(Eigen::Vector2d const&);
    double GetFocalLength() const;
    const Eigen::Vector2d& GetFocalVector() const;

    // This will change the lens distortion type based on the size of the
    // vector. Size 0, no lens distortion, Size 1, FOV, Size 4 or 5, TSAI.
    void SetDistortion(Eigen::VectorXd const& distortion);
    const Eigen::VectorXd& GetDistortion() const;

    // Comparison operator
    friend bool operator== (CameraParameters const& A, CameraParameters const& B) {
      return (A.crop_offset_            == B.crop_offset_            &&
              A.distorted_image_size_   == B.distorted_image_size_   &&
              A.undistorted_image_size_ == B.undistorted_image_size_ &&
              A.distorted_half_size_    == B.distorted_half_size_    &&
              A.focal_length_           == B.focal_length_           &&
              A.optical_offset_         == B.optical_offset_         &&
              A.distortion_coeffs_      == B.distortion_coeffs_      &&
              A.distortion_precalc1_    == B.distortion_precalc1_    &&
              A.distortion_precalc2_    == B.distortion_precalc2_    &&
              A.distortion_precalc3_    == B.distortion_precalc3_);
    }

   private:
    // Converts UNDISTORTED_C to DISTORTED_C
    void DistortCentered(Eigen::Vector2d const& undistorted_c,
                         Eigen::Vector2d* distorted_c) const;
    // Converts DISTORTED_C to UNDISTORTED_C
    void UndistortCentered(Eigen::Vector2d const& distorted_c,
                           Eigen::Vector2d* undistorted_c) const;

    // Members
    Eigen::Vector2i
      crop_offset_,             // Start of DISTORTED in RAW frame
      distorted_image_size_,    // Applies to DISTORTED, DISTORTED_C
      undistorted_image_size_;  // Applies to UNDISTORTED, UNDISTORTED_C,
                                // this is bigger than distorted image size
                                // because we need to expand the sensor to
                                // capture the unravelling from lens distortion
                                // removal.
    Eigen::Vector2d distorted_half_size_, undistorted_half_size_;
    Eigen::Vector2d
      focal_length_,    // In pixels, applies to DISTORTED, UNDISTORTED,
                        // DISTORTED_C, UNDISTORTED_C
      optical_offset_;  // Optical offset in DISTORTED frame

    // Distortion coefficients are in an arbitrary sized vector. The length of
    // the vector tells us what lens distortion model we are using. Length 0 =
    // No lens distortion, ideal camera. Length 1 = FOV/Tangent model, Length 4
    // or 5 = TSAI/OpenCV model.
    Eigen::VectorXd distortion_coeffs_;
    double distortion_precalc1_, distortion_precalc2_, distortion_precalc3_;
  };

#define DECLARE_CONVERSION(TYPEA, TYPEB) \
  template <>  \
  void CameraParameters::Convert<TYPEA, TYPEB>(Eigen::Vector2d const& input, Eigen::Vector2d *output) const
  DECLARE_CONVERSION(RAW, DISTORTED);
  DECLARE_CONVERSION(DISTORTED, RAW);
  DECLARE_CONVERSION(UNDISTORTED_C, DISTORTED_C);
  DECLARE_CONVERSION(DISTORTED_C, UNDISTORTED_C);
  DECLARE_CONVERSION(UNDISTORTED, UNDISTORTED_C);
  DECLARE_CONVERSION(UNDISTORTED_C, UNDISTORTED);
  DECLARE_CONVERSION(DISTORTED, UNDISTORTED);
  DECLARE_CONVERSION(UNDISTORTED, DISTORTED);
  DECLARE_CONVERSION(DISTORTED, UNDISTORTED_C);
  DECLARE_CONVERSION(UNDISTORTED_C, DISTORTED);
#undef DECLARE_CONVERSION

#define DECLARE_INTRINSIC(TYPE) \
  template <>  \
  Eigen::Matrix3d CameraParameters::GetIntrinsicMatrix<TYPE>() const
  DECLARE_INTRINSIC(RAW);
  DECLARE_INTRINSIC(DISTORTED);
  DECLARE_INTRINSIC(DISTORTED_C);
  DECLARE_INTRINSIC(UNDISTORTED);
  DECLARE_INTRINSIC(UNDISTORTED_C);
#undef DECLARE_INTRINSIC
}  // namespace camera

#endif  // CAMERA_CAMERA_PARAMS_H_
