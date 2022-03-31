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

#include <camera/camera_model.h>
#include <camera/camera_params.h>

#include <Eigen/Geometry>
#include <ceres/rotation.h>
#include <gtest/gtest.h>

#include <vector>
#include <string>

#define EXPECT_VECTOR3D_NEAR(p1, p2, t) EXPECT_NEAR(p1[0], p2[0], t); EXPECT_NEAR(p1[1], p2[1], t); \
  EXPECT_NEAR(p1[2], p2[2], t);

#define EXPECT_VECTOR2D_NEAR(p1, p2, t) EXPECT_NEAR(p1[0], p2[0], t); EXPECT_NEAR(p1[1], p2[1], t);

TEST(camera_model, angleaxis_transforms) {
  // Make an arbitrary rotation
  Eigen::Matrix3d R1 =
    Eigen::AngleAxisd(-.156 * M_PI,
                      Eigen::Vector3d(.14, -.56, .4).normalized()).matrix();
  Eigen::Matrix3d R2;
  Eigen::Vector3d rodrigues;

  camera::RotationToRodrigues(R1, &rodrigues);
  camera::RodriguesToRotation(rodrigues, &R2);

  for (int j = 0; j < 3; j++) {
    for (int i = 0; i < 3; i++) {
      EXPECT_NEAR(R1(j, i), R2(j, i), 1e-6);
    }
  }

  Eigen::Vector3d input(4, 5, 3);
  Eigen::Vector3d output2 = R1 * input;
  Eigen::Vector3d output1;
  ceres::AngleAxisRotatePoint(&rodrigues[0],
                              &input[0], &output1[0]);
  for (int i = 0; i < 3; i++)
    EXPECT_NEAR(output1[i], output2[i], 1e-6);
}

TEST(camera_model, camera) {
  std::string data_dir = std::string(std::getenv("DATA_DIR"));
  camera::CameraParameters params(data_dir + "iss_tango_camera.xml");  // Ideally we would use a
                                                   // different file which
                                                   // represents an undistorted
                                                   // image.
                                                   // params.undistorted =
                                                   // true;
  params.SetDistortedSize(Eigen::Vector2i(640, 480));
  params.SetUndistortedSize(Eigen::Vector2i(640, 480));

  // Over-ride focal length for the tests below to pass
  params.SetFocalLength(Eigen::Vector2d::Constant(1.0 / (tan(M_PI_2 / 2) / 320)));

  Eigen::Vector3d pos(10, -10, 5);
  Eigen::Matrix3d rotation;
  rotation = Eigen::AngleAxisd(1.0 * M_PI, Eigen::Vector3d::UnitX())
             * Eigen::AngleAxisd(-0.5 * M_PI, Eigen::Vector3d::UnitY())
             * Eigen::AngleAxisd(-0.5 * M_PI, Eigen::Vector3d::UnitZ());
  camera::CameraModel camera(pos, rotation, M_PI_2, 640, 480);
  camera::CameraModel camera2(pos, rotation, params);

  // check that two constructors give same camera
  EXPECT_EQ(camera.GetPosition(), camera2.GetPosition());
  EXPECT_EQ(camera.GetRotation(), camera2.GetRotation());
  EXPECT_EQ(camera.GetTransform().matrix(), camera2.GetTransform().matrix());
  EXPECT_VECTOR2D_NEAR(camera.GetParameters().GetUndistortedSize(),
                       camera2.GetParameters().GetUndistortedSize(), 0);
  EXPECT_DOUBLE_EQ(camera.GetParameters().GetFocalLength(), camera2.GetParameters().GetFocalLength());

  // check that camera is as specified
  EXPECT_VECTOR3D_NEAR(camera.GetPosition(), pos, 1e-5);
  EXPECT_VECTOR2D_NEAR(camera.GetParameters().GetUndistortedSize(), params.GetUndistortedSize(), 0);
  EXPECT_DOUBLE_EQ(camera.GetParameters().GetFocalLength(),   params.GetFocalLength());

  // check that the center of rotation is origin of camera coordinates
  Eigen::Vector3d test_point(10, -10, 5);
  EXPECT_VECTOR3D_NEAR(camera.CameraCoordinates(test_point), Eigen::Vector3d(0, 0, 0), 1e-5);
  EXPECT_VECTOR3D_NEAR(camera.CameraCoordinates(test_point.x(), test_point.y(), test_point.z()),
                       Eigen::Vector3d(0, 0, 0), 1e-5);

  // check that a point at (0, 0, 1) in camera coordinates is in the center of the image
  EXPECT_VECTOR3D_NEAR(camera.CameraCoordinates(10, -11, 5), Eigen::Vector3d(0, 0, 1), 1e-5);
  EXPECT_VECTOR2D_NEAR(camera.ImageCoordinates(Eigen::Vector3d(10, -11, 5)), Eigen::Vector2d(0, 0), 1e-5);
  EXPECT_VECTOR2D_NEAR(camera.ImageCoordinates(10, -11, 5), Eigen::Vector2d(0, 0), 1e-5);
  // and in FOV
  EXPECT_TRUE(camera.IsInFov(10, -11, 5));
  // point behind camera isn't in FOV
  EXPECT_FALSE(camera.IsInFov(Eigen::Vector3d(10, -9, 5)));

  // check points on border and right beyond border of FOV
  Eigen::Vector3d p;
  Eigen::Vector2d t;

  p << 10, -15, 10-1e-5;
  t = camera.ImageCoordinates(p);
  EXPECT_NEAR(t.x(), -320, 1);
  EXPECT_NEAR(t.y(), 0, 1);
  EXPECT_TRUE(camera.IsInFov(p));
  EXPECT_FALSE(camera.IsInFov(p + Eigen::Vector3d(0, 0, 1e-1)));

  p << 10, -15, 0+1e-2;  // due to rounding, 1e-2
  t = camera.ImageCoordinates(p);
  EXPECT_NEAR(t.x(), 320, 1);
  EXPECT_NEAR(t.y(), 0, 1);
  EXPECT_TRUE(camera.IsInFov(p));
  EXPECT_FALSE(camera.IsInFov(p + Eigen::Vector3d(0, 0, -1e-1)));

  p << 6.25+1e-5, -15, 5;
  t = camera.ImageCoordinates(p);
  EXPECT_NEAR(t.x(), 0, 1);
  EXPECT_NEAR(t.y(), -240, 1);
  EXPECT_TRUE(camera.IsInFov(p));
  EXPECT_FALSE(camera.IsInFov(p + Eigen::Vector3d(-1e-1, 0, 0)));

  p << 13.75-1e-2, -15, 5;  // due to rounding, 1e-2
  t = camera.ImageCoordinates(p);
  EXPECT_NEAR(t.x(), 0, 1);
  EXPECT_NEAR(t.y(), 240, 1);
  EXPECT_TRUE(camera.IsInFov(p));
  EXPECT_FALSE(camera.IsInFov(p + Eigen::Vector3d(1e-1, 0, 1e-2)));

  // check SetTransform
  Eigen::Affine3d new_transform;
  new_transform.setIdentity();
  camera.SetTransform(new_transform);
  EXPECT_EQ(camera.CameraCoordinates(1, 2, 3), Eigen::Vector3d(1, 2, 3));
}

TEST(camera_model, DistortionCoeffs) {
  std::string data_dir = std::string(std::getenv("DATA_DIR"));

  // Test with our three camera models
  std::vector<std::string> files {"p2_camera.xml", "iss_tango_camera.xml",
    "earth_tango_camera.xml", "iss_tango_undistorted.xml", "earth_tango_undistorted.xml"};
  for (std::string const& file : files) {
    camera::CameraParameters camera_params(data_dir + file);

    for (float x = -320; x < 320; x += 30) {
      for (float y = -240; y < 240; y += 30) {
        Eigen::Vector2d distort_location, loopback_location;
        camera_params.Convert<camera::UNDISTORTED_C, camera::DISTORTED_C>
          (Eigen::Vector2d(x, y), &distort_location);
        camera_params.Convert<camera::DISTORTED_C, camera::UNDISTORTED_C>
          (distort_location, &loopback_location);
        EXPECT_NEAR(x, loopback_location[0], 1e-4) << file;
        EXPECT_NEAR(y, loopback_location[1], 1e-4) << file;
      }
    }
  }

  // Verify that the optical center is not at the center of the image
  // in undistort location
  camera::CameraParameters camera_params(data_dir + "iss_tango_camera.xml");
  Eigen::Vector2d optical_center;
  camera_params.Convert<camera::UNDISTORTED_C, camera::DISTORTED_C>(Eigen::Vector2d::Zero(), &optical_center);
  EXPECT_NEAR(3.92, optical_center[0], 1e-3);
  EXPECT_NEAR(0.827, optical_center[1], 1e-3);
}

// Run all the tests that were declared with TEST()
int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
