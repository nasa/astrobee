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
#include <sparse_mapping/reprojection.h>

#include <gtest/gtest.h>
#include <Eigen/Geometry>

#include <vector>

TEST(reprojection, pose_estimation) {
  // create camera model
  Eigen::Vector3d true_camera_pos(0, 0, 0);
  Eigen::Matrix3d true_camera_rotation;
  true_camera_rotation = Eigen::AngleAxisd(0 * M_2_PI, Eigen::Vector3d::UnitX()) *
                         Eigen::AngleAxisd(0 * M_2_PI, Eigen::Vector3d::UnitY()) *
                         Eigen::AngleAxisd(0 * M_2_PI, Eigen::Vector3d::UnitZ());
  camera::CameraModel camera(true_camera_pos, true_camera_rotation, 90 * M_PI / 180.0, 640, 480);

  // create correct landmarks and observations
  std::vector<Eigen::Vector3d> landmarks;
  std::vector<Eigen::Vector2d> observations;
  landmarks.push_back(Eigen::Vector3d(8, 0, 8));
  landmarks.push_back(Eigen::Vector3d(2, 1, 5));
  landmarks.push_back(Eigen::Vector3d(-2, 1, 4));
  landmarks.push_back(Eigen::Vector3d(2, 3, 6));
  landmarks.push_back(Eigen::Vector3d(-2, -3, 7));
  landmarks.push_back(Eigen::Vector3d(1, 0, 4));
  landmarks.push_back(Eigen::Vector3d(0.5, -0.5, 6));
  for (size_t i = 0; i < landmarks.size(); i++) {
    // Purposely introducing rounding error here.
    Eigen::Vector2i obs = (camera.ImageCoordinates(landmarks[i]).array() + 0.5).cast<int>();
    observations.push_back(Eigen::Vector2d(obs[0], obs[1]));
  }

  // start with a nearby guess
  Eigen::Vector3d guess_pos(0.2, -0.3, -1.0);
  Eigen::Matrix3d guess_rot;
  guess_rot = Eigen::AngleAxisd(0.4 * M_2_PI, Eigen::Vector3d::UnitX()) *
              Eigen::AngleAxisd(-0.15 * M_2_PI, Eigen::Vector3d::UnitY()) *
              Eigen::AngleAxisd(0.2 * M_2_PI, Eigen::Vector3d::UnitZ());
  camera::CameraModel estimate(guess_pos, guess_rot, camera.GetParameters());

  // check that ceres solver converges
  ceres::Solver::Options options;
  options.linear_solver_type           = ceres::ITERATIVE_SCHUR;
  options.num_threads                  = 1;
  options.max_num_iterations           = 100;
  options.minimizer_progress_to_stdout = false;
  ceres::Solver::Summary summary;
  // run ceres solver
  sparse_mapping::EstimateCamera(&estimate, &landmarks, observations, options, &summary);

  // confirm solution
  Eigen::Vector3d orig_angle     = camera.GetRotation() * Eigen::Vector3d::UnitX();
  Eigen::Vector3d observed_angle = estimate.GetRotation() * Eigen::Vector3d::UnitX();
  EXPECT_NEAR((true_camera_pos - estimate.GetPosition()).norm(), 0, 0.1);
  EXPECT_NEAR(acos(observed_angle.dot(orig_angle)), 0, 0.05);

  // add incorrect observations
  landmarks.push_back(Eigen::Vector3d(-3, 5, -8));
  observations.push_back(Eigen::Vector2d(100, 300));
  landmarks.push_back(Eigen::Vector3d(1, -2, -1));
  observations.push_back(Eigen::Vector2d(400, 100));
  // run ransac
  sparse_mapping::RansacEstimateCamera(landmarks, observations, 50, 4, &estimate);

  // check success
  orig_angle     = camera.GetRotation() * Eigen::Vector3d::UnitX();
  observed_angle = estimate.GetRotation() * Eigen::Vector3d::UnitX();
  EXPECT_NEAR((true_camera_pos - estimate.GetPosition()).norm(), 0, 0.1)
      << "Estimated position: " << estimate.GetPosition().transpose() << " Expected: " << true_camera_pos.transpose();
  EXPECT_NEAR(acos(observed_angle.dot(orig_angle)), 0, 0.05);
}

TEST(reprojection, affine_estimation) {
  // Test solving for affine transform between two datatsets

  // Create datasets with known transform
  Eigen::Matrix3Xd in(3, 100), out(3, 100);
  for (int row = 0; row < in.rows(); row++) {
    for (int col = 0; col < in.cols(); col++) {
      in(row, col) = log(2 * row + 10.0) / sqrt(1.0 * col + 4.0) + sqrt(col * 1.0) / (row + 1.0);
    }
  }

  Eigen::Quaternion<double> Q(1, 3, 5, 2);
  Q.normalize();
  Eigen::Matrix3d R     = Q.toRotationMatrix();
  double          scale = 2.0;
  Eigen::Vector3d S;
  S << -5, 6, -27;
  for (int col = 0; col < in.cols(); col++) out.col(col) = scale * R * in.col(col) + S;

  Eigen::Affine3d A;
  sparse_mapping::Find3DAffineTransform(in, out, &A);

  // See if we got the transform we expected
  EXPECT_LE((scale * R - A.linear()).cwiseAbs().maxCoeff(), 1e-12);
  EXPECT_LE((S - A.translation()).cwiseAbs().maxCoeff(), 1e-12);
  // Translation vector should be the same
  EXPECT_NEAR(A.translation()[0], S[0], 1e-3);
  EXPECT_NEAR(A.translation()[1], S[1], 1e-3);
  EXPECT_NEAR(A.translation()[2], S[2], 1e-3);
}
