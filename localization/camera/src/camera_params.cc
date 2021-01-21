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

#include <camera/camera_params.h>

#include <config_reader/config_reader.h>
#include <Eigen/Dense>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <fstream>
#include <iostream>

camera::CameraParameters::CameraParameters(Eigen::Vector2i const& image_size,
    Eigen::Vector2d const& focal_length,
    Eigen::Vector2d const& optical_center,
    Eigen::VectorXd const& distortion) {
  SetDistortedSize(image_size);
  SetUndistortedSize(image_size);
  focal_length_ = focal_length;
  optical_offset_ = optical_center;
  crop_offset_.setZero();
  SetDistortion(distortion);
}

camera::CameraParameters::CameraParameters(std::string const& calibration_file,
                                           std::string const & base_dir) {
  std::ifstream f(calibration_file);
  // it may be a relative path to the map file's directory
  if (!f.good() && base_dir != "") {
    f.open(base_dir + "/" + calibration_file);
  }
  if (!f.good()) {
    LOG(FATAL) <<  "Calibration file does not exist: " << calibration_file;
  }

  cv::FileStorage fs(calibration_file, cv::FileStorage::READ);

  cv::Mat cam_mat, dist_coeffs;
  Eigen::Vector2i size;

  // Read in distorted image size.
  fs["width"]  >> size[0];
  fs["height"] >> size[1];
  if (size[0] == 0 || size[1] == 0) {
    fs["image_width"]  >> size[0];
    fs["image_height"]  >> size[1];
  }
  SetDistortedSize(size);

  // Read in focal length and optical offset
  fs["intrinsic_matrix"] >> cam_mat;
  if (cam_mat.rows == 0 || cam_mat.cols == 0)
    fs["camera_matrix"] >> cam_mat;
  optical_offset_ << cam_mat.at<double>(0, 2), cam_mat.at<double>(1, 2);
  focal_length_ << cam_mat.at<double>(0, 0), cam_mat.at<double>(1, 1);

  // Read in the distortion coefficients
  fs["distortion"] >> dist_coeffs;
  if (dist_coeffs.rows == 0 || dist_coeffs.cols == 0)
    fs["distortion_coefficients"] >> dist_coeffs;
  Eigen::VectorXd buffer(dist_coeffs.cols * dist_coeffs.rows);
  for (int32_t i = 0; i < buffer.size(); i++) {
    buffer[i] = dist_coeffs.at<double>(i);
  }
  SetDistortion(buffer);

  // Load up the undistorted image size and handle if it was specified.
  fs["undistorted_width"] >> size[0];
  fs["undistorted_height"] >> size[1];
  if (!(size.array() > 0).all()) {
    size = distorted_image_size_;
  }
  SetUndistortedSize(size);

  // Read in crop offset
  fs["crop_x"] >> size[0];  // These default to zero if not specified
  fs["crop_y"] >> size[1];
  crop_offset_ = size;
}

camera::CameraParameters::CameraParameters(config_reader::ConfigReader* config, const char* name) {
  cv::Mat cam_mat(3, 3, CV_64F);
  Eigen::Vector2i size;

  config_reader::ConfigReader::Table camera(config, name);
  // Read in distorted image size.
  if (!camera.GetInt("width", &size[0]))
    fprintf(stderr, "Could not read camera width.");
  if (!camera.GetInt("height", &size[1]))
    fprintf(stderr, "Could not read camera height.");
  SetDistortedSize(size);

  config_reader::ConfigReader::Table vector(&camera, "intrinsic_matrix");
  for (int i = 0; i < 9; i++) {
    if (!vector.GetReal((i + 1), &cam_mat.at<double>(i / 3, i % 3))) {
      fprintf(stderr, "Failed to read vector intrinsic_matrix.");
      break;
    }
  }
  // Read in focal length and optical offset
  optical_offset_ << cam_mat.at<double>(0, 2), cam_mat.at<double>(1, 2);
  focal_length_ << cam_mat.at<double>(0, 0), cam_mat.at<double>(1, 1);

  // Read in the distortion coefficients (if not a real number, may be a list)
  Eigen::VectorXd buffer(1);
  if (!camera.GetReal("distortion_coeff", &buffer[0])) {
    config_reader::ConfigReader::Table dist(&camera, "distortion_coeff");
    buffer.resize(dist.GetSize());
    for (int i = 0; i < dist.GetSize(); i++) {
      if (!dist.GetReal(i + 1, &buffer[i]))
        fprintf(stderr, "Could not read camera distortion_coeff.");
    }
  }
  SetDistortion(buffer);

  if (!camera.GetInt("undistorted_width", &size[0]))
    fprintf(stderr, "Could not read camera undistorted_width.");
  if (!camera.GetInt("undistorted_height", &size[1]))
    fprintf(stderr, "Could not read camera undistorted_height.");
  SetUndistortedSize(size);

  // right now all our crops are zero
  // ff_common::ConfigReader::Table crop_table(&camera, "crop");
  // // Read in crop offset
  // if (!crop_table.GetInt("x", &size[0]))
  //   size[0] = 0;
  // if (!crop_table.GetInt("y", &size[1]))
  //   size[1] = 0;
  // crop_offset_ = size;
}

void camera::CameraParameters::SetDistortedSize(Eigen::Vector2i const& image_size) {
  distorted_image_size_ = image_size;
  distorted_half_size_ = image_size.cast<double>() / 2;
}

const Eigen::Vector2i& camera::CameraParameters::GetDistortedSize() const {
  return distorted_image_size_;
}

const Eigen::Vector2d& camera::CameraParameters::GetDistortedHalfSize() const {
  return distorted_half_size_;
}

void camera::CameraParameters::SetUndistortedSize(Eigen::Vector2i const& image_size) {
  undistorted_image_size_ = image_size;
  undistorted_half_size_ = image_size.cast<double>() / 2;
}

const Eigen::Vector2i& camera::CameraParameters::GetUndistortedSize() const {
  return undistorted_image_size_;
}

const Eigen::Vector2d& camera::CameraParameters::GetUndistortedHalfSize() const {
  return undistorted_half_size_;
}

void camera::CameraParameters::SetCropOffset(Eigen::Vector2i const& crop) {
  crop_offset_ = crop;
}

const Eigen::Vector2i& camera::CameraParameters::GetCropOffset() const {
  return crop_offset_;
}

void camera::CameraParameters::SetOpticalOffset(Eigen::Vector2d const& offset) {
  optical_offset_ = offset;
}

const Eigen::Vector2d& camera::CameraParameters::GetOpticalOffset() const {
  return optical_offset_;
}

void camera::CameraParameters::SetFocalLength(Eigen::Vector2d const& f) {
  focal_length_ = f;
}

double camera::CameraParameters::GetFocalLength() const {
  return focal_length_.mean();
}

const Eigen::Vector2d& camera::CameraParameters::GetFocalVector() const {
  return focal_length_;
}

void camera::CameraParameters::SetDistortion(Eigen::VectorXd const& distortion) {
  distortion_coeffs_ = distortion;

  // Ensure variables are initialized
  distortion_precalc1_ = 0;
  distortion_precalc2_ = 0;
  distortion_precalc3_ = 0;

  switch (distortion_coeffs_.size()) {
  case 0:
    // No lens distortion!
    break;
  case 1:
    // FOV model
    // inverse alpha
    distortion_precalc1_ = 1 / distortion[0];
    // Inside tangent function
    distortion_precalc2_ = 2 * tan(distortion[0] / 2);
    break;
  case 4:
    // Fall through intended.
  case 5:
    // Tsai model
    // There doesn't seem like there are any precalculations we can use.
    break;
  default:
    LOG(FATAL) << "Recieved irregular distortion vector size. Size = " << distortion_coeffs_.size();
  }
}

const Eigen::VectorXd& camera::CameraParameters::GetDistortion() const {
  return distortion_coeffs_;
}

void camera::CameraParameters::DistortCentered(Eigen::Vector2d const& undistorted_c,
                                               Eigen::Vector2d* distorted_c) const {
  // We assume that input x and y are pixel values that have
  // undistorted_len_x/2.0 and undistorted_len_y/2.0 subtracted from
  // them. The outputs will have distorted_len_x/2.0 and
  // distorted_len_y/2.0 subtracted from them.
  if (distortion_coeffs_.size() == 0) {
    // There is no distortion
    *distorted_c = undistorted_c + optical_offset_ - distorted_half_size_;
  } else if (distortion_coeffs_.size() == 1) {
    // This is the FOV model
    Eigen::Vector2d norm = undistorted_c.cwiseQuotient(focal_length_);
    double ru = norm.norm();
    double rd = atan(ru * distortion_precalc2_) * distortion_precalc1_;
    double conv;
    if (ru > 1e-5) {
      conv = rd / ru;
    } else {
      conv = 1;
    }
    *distorted_c = (optical_offset_ - distorted_half_size_) +
      conv * norm.cwiseProduct(focal_length_);
  } else if (distortion_coeffs_.size() == 4 ||
             distortion_coeffs_.size() == 5) {
    // Tsai lens distortion
    double k1 = distortion_coeffs_[0];
    double k2 = distortion_coeffs_[1];
    double p1 = distortion_coeffs_[2];
    double p2 = distortion_coeffs_[3];
    double k3 = 0;
    if (distortion_coeffs_.size() == 5)
      k3 = distortion_coeffs_[4];

    // To relative coordinates
    Eigen::Vector2d norm = undistorted_c.cwiseQuotient(focal_length_);
    double r2 = norm.squaredNorm();

    // Radial distortion
    double radial_dist = 1 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2;
    *distorted_c = radial_dist * norm;

    // Tangential distortion
    *distorted_c +=
      Eigen::Vector2d(2 * p1 * norm[0] * norm[1] + p2 * (r2 + 2 * norm[0] * norm[0]),
                      p1 * (r2 + 2 * norm[1] * norm[1]) + 2 * p2 * norm[0] * norm[1]);

    // Back to absolute coordinates.
    *distorted_c = distorted_c->cwiseProduct(focal_length_) +
      (optical_offset_ - distorted_half_size_);
  } else {
    LOG(ERROR) << "Unknown distortion vector size!";
  }
}

void camera::CameraParameters::UndistortCentered(Eigen::Vector2d const& distorted_c,
                                                 Eigen::Vector2d *undistorted_c) const {
  // We assume that input x and y are pixel values that have
  // distorted_len_x and distorted_len_y subtracted from them. The
  // outputs will have undistorted_len_x and undistorted_len_y
  // subtracted from them.
  if (distortion_coeffs_.size() == 0) {
    // No lens distortion
    *undistorted_c = distorted_c - (optical_offset_ - distorted_half_size_);
  } else if (distortion_coeffs_.size() == 1) {
    // FOV lens distortion
    Eigen::Vector2d norm =
      (distorted_c - (optical_offset_ - distorted_half_size_)).cwiseQuotient(focal_length_);
    double rd = norm.norm();
    double ru = tan(rd * distortion_coeffs_[0]) / distortion_precalc2_;
    double conv = 1.0;
    if (rd > 1e-5)
      conv = ru / rd;
    *undistorted_c = conv * norm.cwiseProduct(focal_length_);
  } else if (distortion_coeffs_.size() == 4 ||
             distortion_coeffs_.size() == 5) {
    // Tsai lens distortion
    cv::Mat src(1, 1, CV_64FC2);
    cv::Mat dst(1, 1, CV_64FC2);
    Eigen::Map<Eigen::Vector2d> src_map(src.ptr<double>()), dst_map(dst.ptr<double>());
    cv::Mat dist_int_mat(3, 3, cv::DataType<double>::type),
      undist_int_mat(3, 3, cv::DataType<double>::type);
    cv::Mat cvdist;
    cv::eigen2cv(distortion_coeffs_, cvdist);
    cv::eigen2cv(GetIntrinsicMatrix<DISTORTED>(), dist_int_mat);
    cv::eigen2cv(GetIntrinsicMatrix<UNDISTORTED>(), undist_int_mat);
    src_map = distorted_c + distorted_half_size_;
    cv::undistortPoints(src, dst, dist_int_mat, cvdist, cv::Mat(), undist_int_mat);
    *undistorted_c = dst_map - undistorted_half_size_;
  } else {
    LOG(ERROR) << "Unknown distortion vector size!";
  }
}

// The 'scale' variable is useful when we have the distortion model for a given
// image, and want to apply it to a version of that image at a different resolution,
// with 'scale' being the ratio of the width of the image at different resolution
// and the one at the resolution at which the distortion model is computed.
void camera::CameraParameters::GenerateRemapMaps(cv::Mat* remap_map, double scale) {
  remap_map->create(scale*undistorted_image_size_[1], scale*undistorted_image_size_[0], CV_32FC2);
  Eigen::Vector2d undistorted, distorted;
  for (undistorted[1] = 0; undistorted[1] < scale*undistorted_image_size_[1]; undistorted[1]++) {
    for (undistorted[0] = 0; undistorted[0] < scale*undistorted_image_size_[0]; undistorted[0]++) {
      Convert<UNDISTORTED, DISTORTED>(undistorted/scale, &distorted);
      remap_map->at<cv::Vec2f>(undistorted[1], undistorted[0])[0] = scale*distorted[0];
      remap_map->at<cv::Vec2f>(undistorted[1], undistorted[0])[1] = scale*distorted[1];
    }
  }
}


namespace camera {

  // Conversion function helpers
#define DEFINE_CONVERSION(TYPEA, TYPEB) \
  template <> \
  void camera::CameraParameters::Convert<TYPEA, TYPEB>(Eigen::Vector2d const& input, Eigen::Vector2d *output) const

  DEFINE_CONVERSION(RAW, DISTORTED) {
    *output = input - crop_offset_.cast<double>();
  }
  DEFINE_CONVERSION(DISTORTED, RAW) {
    *output = input + crop_offset_.cast<double>();
  }
  DEFINE_CONVERSION(UNDISTORTED_C, DISTORTED_C) {
    DistortCentered(input, output);
  }
  DEFINE_CONVERSION(DISTORTED_C, UNDISTORTED_C) {
    UndistortCentered(input, output);
  }
  DEFINE_CONVERSION(UNDISTORTED, UNDISTORTED_C) {
    *output = input - undistorted_half_size_;
  }
  DEFINE_CONVERSION(UNDISTORTED_C, UNDISTORTED) {
    *output = input + undistorted_half_size_;
  }
  DEFINE_CONVERSION(DISTORTED, UNDISTORTED) {
    Convert<DISTORTED_C, UNDISTORTED_C>(input - distorted_half_size_, output);
    *output += undistorted_half_size_;
  }
  DEFINE_CONVERSION(UNDISTORTED, DISTORTED) {
    Eigen::Vector2d centered_output;
    Convert<UNDISTORTED, UNDISTORTED_C>(input, output);
    Convert<UNDISTORTED_C, DISTORTED_C>(*output, &centered_output);
    *output = centered_output + distorted_half_size_;
  }
  DEFINE_CONVERSION(DISTORTED, UNDISTORTED_C) {
    Convert<DISTORTED_C, UNDISTORTED_C>(input - distorted_half_size_, output);
  }
  DEFINE_CONVERSION(UNDISTORTED_C, DISTORTED) {
    Convert<UNDISTORTED_C, DISTORTED_C>(input, output);
    *output += distorted_half_size_;
  }

#undef DEFINE_CONVERSION

  // Helper functions to give the intrinsic matrix
#define DEFINE_INTRINSIC(TYPE) \
  template <> \
  Eigen::Matrix3d camera::CameraParameters::GetIntrinsicMatrix<TYPE>() const

  DEFINE_INTRINSIC(RAW) {
    Eigen::Matrix3d k = focal_length_.homogeneous().asDiagonal();
    k.block<2, 1>(0, 2) = optical_offset_ + crop_offset_.cast<double>();
    return k;
  }
  DEFINE_INTRINSIC(DISTORTED) {
    Eigen::Matrix3d k = focal_length_.homogeneous().asDiagonal();
    k.block<2, 1>(0, 2) = optical_offset_;
    return k;
  }
  DEFINE_INTRINSIC(DISTORTED_C) {
    Eigen::Matrix3d k = focal_length_.homogeneous().asDiagonal();
    k.block<2, 1>(0, 2) = optical_offset_ - distorted_half_size_;
    return k;
  }
  DEFINE_INTRINSIC(UNDISTORTED) {
    Eigen::Matrix3d k = focal_length_.homogeneous().asDiagonal();
    k.block<2, 1>(0, 2) = undistorted_half_size_;
    return k;
  }
  DEFINE_INTRINSIC(UNDISTORTED_C) {
    Eigen::Matrix3d k = focal_length_.homogeneous().asDiagonal();
    return k;
  }

#undef DEFINE_INTRINSIC

}  // end namespace camera
