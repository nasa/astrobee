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

#include <Eigen/Geometry>
#include <ceres/rotation.h>
#include <gtest/gtest.h>

#include <functional>
#include <list>
#include <memory>
#include <string>
#include <vector>

#define EXPECT_VECTOR3D_NEAR(p1, p2, t) EXPECT_NEAR(p1[0], p2[0], t); EXPECT_NEAR(p1[1], p2[1], t); \
  EXPECT_NEAR(p1[2], p2[2], t);

#define EXPECT_VECTOR2D_NEAR(p1, p2, t) EXPECT_NEAR(p1[0], p2[0], t); EXPECT_NEAR(p1[1], p2[1], t);

void Function(camera::CameraParameters const& params) {
  EXPECT_VECTOR2D_NEAR(Eigen::Vector2i(1200, 600), params.GetDistortedSize(), 1e-3);
  EXPECT_VECTOR2D_NEAR(Eigen::Vector2i(600, 300), params.GetDistortedHalfSize(), 1e-3);
  EXPECT_VECTOR2D_NEAR(Eigen::Vector2i(1200, 600), params.GetUndistortedSize(), 1e-3);
  EXPECT_VECTOR2D_NEAR(Eigen::Vector2i(600, 300), params.GetUndistortedHalfSize(), 1e-3);
  EXPECT_VECTOR2D_NEAR(Eigen::Vector2d(300, 200), params.GetFocalVector(), 1e-3);
  EXPECT_VECTOR2D_NEAR(Eigen::Vector2d(660.7, 500.3), params.GetOpticalOffset(), 1e-3);
  EXPECT_VECTOR2D_NEAR(Eigen::Vector2i(50, 100), params.GetCropOffset(), 1e-3);
}

TEST(camera_params, creation) {
  camera::CameraParameters params(
      Eigen::Vector2i(1200, 600),
      Eigen::Vector2d(300, 200),
      Eigen::Vector2d(660.7, 500.3));
  EXPECT_VECTOR2D_NEAR(params.GetDistortedHalfSize(), Eigen::Vector2i(600, 300), 1e-3);
  EXPECT_VECTOR2D_NEAR(params.GetUndistortedHalfSize(), Eigen::Vector2i(600, 300), 1e-3);
  EXPECT_EQ(0, params.GetDistortion().size());
  EXPECT_NEAR(250, params.GetFocalLength(), 1e-3);

  // Make RAW and DISTORT be different frames
  params.SetCropOffset(Eigen::Vector2i(50, 100));
  EXPECT_VECTOR2D_NEAR(params.GetCropOffset(), Eigen::Vector2i(50, 100), 1e-3);

  // Verify we get the correct intrinsic matrix
  Eigen::Matrix3d k = params.GetIntrinsicMatrix<camera::RAW>();
  EXPECT_VECTOR3D_NEAR(k.diagonal(), Eigen::Vector3d(300, 200, 1), 1e-3);
  EXPECT_VECTOR3D_NEAR(k.col(2), Eigen::Vector3d(710.7, 600.3, 1), 1e-3);

  k = params.GetIntrinsicMatrix<camera::DISTORTED>();
  EXPECT_VECTOR3D_NEAR(k.diagonal(), Eigen::Vector3d(300, 200, 1), 1e-3);
  EXPECT_VECTOR3D_NEAR(k.col(2), Eigen::Vector3d(660.7, 500.3, 1), 1e-3);

  k = params.GetIntrinsicMatrix<camera::DISTORTED_C>();
  EXPECT_VECTOR3D_NEAR(k.diagonal(), Eigen::Vector3d(300, 200, 1), 1e-3);
  EXPECT_VECTOR3D_NEAR(k.col(2), Eigen::Vector3d(60.7, 200.3, 1), 1e-3);

  k = params.GetIntrinsicMatrix<camera::UNDISTORTED>();
  EXPECT_VECTOR3D_NEAR(k.diagonal(), Eigen::Vector3d(300, 200, 1), 1e-3);
  EXPECT_VECTOR3D_NEAR(k.col(2), Eigen::Vector3d(600, 300, 1), 1e-3);

  k = params.GetIntrinsicMatrix<camera::UNDISTORTED_C>();
  EXPECT_VECTOR3D_NEAR(k.diagonal(), Eigen::Vector3d(300, 200, 1), 1e-3);
  EXPECT_VECTOR3D_NEAR(k.col(2), Eigen::Vector3d(0, 0, 1), 1e-3);

  // See what happens when we do a copy
  camera::CameraParameters params2 = params;
  EXPECT_VECTOR2D_NEAR(Eigen::Vector2i(1200, 600), params2.GetDistortedSize(), 1e-3);
  EXPECT_VECTOR2D_NEAR(Eigen::Vector2i(600, 300), params2.GetDistortedHalfSize(), 1e-3);
  EXPECT_VECTOR2D_NEAR(Eigen::Vector2i(1200, 600), params2.GetUndistortedSize(), 1e-3);
  EXPECT_VECTOR2D_NEAR(Eigen::Vector2i(600, 300), params2.GetUndistortedHalfSize(), 1e-3);
  EXPECT_VECTOR2D_NEAR(Eigen::Vector2d(300, 200), params2.GetFocalVector(), 1e-3);
  EXPECT_VECTOR2D_NEAR(Eigen::Vector2d(660.7, 500.3), params2.GetOpticalOffset(), 1e-3);
  EXPECT_VECTOR2D_NEAR(Eigen::Vector2i(50, 100), params2.GetCropOffset(), 1e-3);

  camera::CameraParameters params3(params);
  EXPECT_VECTOR2D_NEAR(Eigen::Vector2i(1200, 600), params3.GetDistortedSize(), 1e-3);
  EXPECT_VECTOR2D_NEAR(Eigen::Vector2i(600, 300), params3.GetDistortedHalfSize(), 1e-3);
  EXPECT_VECTOR2D_NEAR(Eigen::Vector2i(1200, 600), params3.GetUndistortedSize(), 1e-3);
  EXPECT_VECTOR2D_NEAR(Eigen::Vector2i(600, 300), params3.GetUndistortedHalfSize(), 1e-3);
  EXPECT_VECTOR2D_NEAR(Eigen::Vector2d(300, 200), params3.GetFocalVector(), 1e-3);
  EXPECT_VECTOR2D_NEAR(Eigen::Vector2d(660.7, 500.3), params3.GetOpticalOffset(), 1e-3);
  EXPECT_VECTOR2D_NEAR(Eigen::Vector2i(50, 100), params3.GetCropOffset(), 1e-3);

  params3.SetOpticalOffset(Eigen::Vector2d(99, -100));
  EXPECT_VECTOR2D_NEAR(Eigen::Vector2d(99, -100), params3.GetOpticalOffset(), 1e-3);
  EXPECT_VECTOR2D_NEAR(Eigen::Vector2d(660.7, 500.3), params.GetOpticalOffset(), 1e-3);

  // Verify that we don't have memory alignment issues with shared ptrs
  std::shared_ptr<camera::CameraParameters> params4;
  params4.reset(new camera::CameraParameters(params));
  EXPECT_VECTOR2D_NEAR(Eigen::Vector2i(1200, 600), params4->GetDistortedSize(), 1e-3);
  EXPECT_VECTOR2D_NEAR(Eigen::Vector2i(600, 300), params4->GetDistortedHalfSize(), 1e-3);
  EXPECT_VECTOR2D_NEAR(Eigen::Vector2i(1200, 600), params4->GetUndistortedSize(), 1e-3);
  EXPECT_VECTOR2D_NEAR(Eigen::Vector2i(600, 300), params4->GetUndistortedHalfSize(), 1e-3);
  EXPECT_VECTOR2D_NEAR(Eigen::Vector2d(300, 200), params4->GetFocalVector(), 1e-3);
  EXPECT_VECTOR2D_NEAR(Eigen::Vector2d(660.7, 500.3), params4->GetOpticalOffset(), 1e-3);
  EXPECT_VECTOR2D_NEAR(Eigen::Vector2i(50, 100), params4->GetCropOffset(), 1e-3);

  // Maybe alignment problems happen when passing to a function
  Function(params);

  auto f_bind = std::bind(Function, params);
  f_bind();

  std::function<void(camera::CameraParameters const&)> f_bind2 = Function;
  f_bind2(params);

  // Maybe the alignment problem is an STL container?
  std::list<camera::CameraParameters, Eigen::aligned_allocator<camera::CameraParameters> > list;
  list.emplace_back(params);
  list.emplace_back(params);
}


TEST(camera_params, conversion) {
  //       raw d d_c u u_c
  // 1      --->
  // 2      <---
  // 3            <------
  // 4            ------>
  // 5               --->
  // 6               <---
  // 7         ------>
  // 8         <------
  // 9         --------->
  // 10        <---------

  Eigen::VectorXd distortion(1);
  distortion[0] = 0.7;  // FOV model
  camera::CameraParameters params(
      Eigen::Vector2i(1200, 600),
      Eigen::Vector2d(300, 300),
      Eigen::Vector2d(660.5, 500.3), distortion);
  params.SetCropOffset(Eigen::Vector2i(25, 50));

  // RAW to DISTORTED conversions
  Eigen::Vector2d raw = Eigen::Vector2d::Zero(), output = Eigen::Vector2d::Zero();
  params.Convert<camera::RAW, camera::DISTORTED>(raw, &output);
  EXPECT_NEAR(-25, output[0], 1e-6);
  EXPECT_NEAR(-50, output[1], 1e-6);
  params.Convert<camera::DISTORTED, camera::RAW>(output, &output);
  EXPECT_NEAR(0, output[0], 1e-6);
  EXPECT_NEAR(0, output[1], 1e-6);

  // forward backward via 3, 4
  Eigen::Vector2d input(-40, 77.3), output2;
  params.Convert<camera::UNDISTORTED_C, camera::DISTORTED_C>(input, &output);
  params.Convert<camera::DISTORTED_C, camera::UNDISTORTED_C>(output, &output2);
  EXPECT_NEAR(input[0], output2[0], 1e-6);
  EXPECT_NEAR(input[1], output2[1], 1e-6);

  // forward backward via 7, 8
  input << 233.4, 368.9;
  params.Convert<camera::UNDISTORTED, camera::DISTORTED>(input, &output);
  params.Convert<camera::DISTORTED, camera::UNDISTORTED>(output, &output2);
  EXPECT_NEAR(input[0], output2[0], 1e-6);
  EXPECT_NEAR(input[1], output2[1], 1e-6);

  // forward backward via 9, 10
  input << 542.4, 182.5;
  params.Convert<camera::DISTORTED, camera::UNDISTORTED_C>(input, &output);
  params.Convert<camera::UNDISTORTED_C, camera::DISTORTED>(output, &output2);
  EXPECT_NEAR(input[0], output2[0], 1e-6);
  EXPECT_NEAR(input[1], output2[1], 1e-6);

  // forward through 5, 10, 7
  input << 43.6, -213.9;
  params.Convert<camera::UNDISTORTED, camera::UNDISTORTED_C>(input, &output2);
  params.Convert<camera::UNDISTORTED_C, camera::DISTORTED>(output2, &output);
  params.Convert<camera::DISTORTED, camera::UNDISTORTED>(output, &output2);
  EXPECT_NEAR(input[0], output2[0], 1e-6);
  EXPECT_NEAR(input[1], output2[1], 1e-6);

  // forward through 8, 9, 6
  input << 542.4, 182.5;
  params.Convert<camera::UNDISTORTED, camera::DISTORTED>(input, &output2);
  params.Convert<camera::DISTORTED, camera::UNDISTORTED_C>(output2, &output);
  params.Convert<camera::UNDISTORTED_C, camera::UNDISTORTED>(output, &output2);
  EXPECT_NEAR(input[0], output2[0], 1e-6);
  EXPECT_NEAR(input[1], output2[1], 1e-6);
}

// Run all the tests that were declared with TEST()
int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
