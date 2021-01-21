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

#include <ff_common/thread.h>
#include <camera/camera_model.h>
#include <sparse_mapping/tensor.h>
#include <sparse_mapping/sparse_map.h>

#include <opencv2/highgui/highgui.hpp>
#include <Eigen/Geometry>
#include <gtest/gtest.h>

#include <map>
#include <string>
#include <vector>

#define EXPECT_VECTOR3D_NEAR(p1, p2, t) EXPECT_NEAR(p1[0], p2[0], t); EXPECT_NEAR(p1[1], p2[1], t); \
  EXPECT_NEAR(p1[2], p2[2], t);

const std::string DATA_DIR = std::string(std::getenv("DATA_DIR"));

class SparseMapTest : public ::testing::Test {
 protected:
  sparse_mapping::SparseMap* surf_map;
  std::vector<std::string> local_imgs;

  virtual void SetUp() {
    FLAGS_num_threads = 1;

    std::vector<std::string> img_files;
    img_files.reserve(3);
    local_imgs.reserve(3);
    local_imgs.push_back("m0000182.jpg"); img_files.push_back(DATA_DIR + local_imgs.back());
    local_imgs.push_back("m0000189.jpg"); img_files.push_back(DATA_DIR + local_imgs.back());
    local_imgs.push_back("m0000210.jpg"); img_files.push_back(DATA_DIR + local_imgs.back());

    // Write local copies of the images. Why?
    for (size_t cid = 0; cid < img_files.size(); cid++) {
      cv::Mat image = cv::imread(img_files[cid], CV_LOAD_IMAGE_GRAYSCALE);
      cv::imwrite(local_imgs[cid], image);
    }

    // feature detection
    camera::CameraParameters params
      (Eigen::Vector2i(780, 620),
       Eigen::Vector2d::Constant(258.5),
       Eigen::Vector2d(390, 310));
    surf_map = new sparse_mapping::SparseMap(local_imgs, "SURF", params);
  }

  virtual void GenerateFullMap() {
    // Make a data set
    surf_map->DetectFeatures();

    // feature matching
    std::string essential_file = "registration_essential_file.csv";
    std::string matches_file = "registration_matches.txt";
    sparse_mapping::MatchFeatures(essential_file, matches_file, surf_map);

    // build the tracks and incremental BA
    bool rm_invalid_xyz = false;  // no cameras yet hence can't remove invalid xyx
    sparse_mapping::BuildTracks(rm_invalid_xyz, matches_file, surf_map);
    sparse_mapping::IncrementalBA(essential_file, surf_map);

    // Bundle adjustment step
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::ITERATIVE_SCHUR;
    options.num_threads = 1;
    options.max_num_iterations = 10;
    options.minimizer_progress_to_stdout = true;
    ceres::Solver::Summary summary;
    sparse_mapping::BundleAdjustment(surf_map, new ceres::CauchyLoss(1.0), options, &summary);
    EXPECT_LT(summary.final_cost / surf_map->GetNumObservations(), .4 /* px */);

    // Verify we built a good map.
    EXPECT_GE(local_imgs.size(), surf_map->GetNumFrames());
    for (size_t i = 0; i < surf_map->GetNumFrames(); i++) {
      EXPECT_EQ(surf_map->GetFrameFilename(i), local_imgs[i]);
      // at least check the number of keypoints is reasonable, even if
      // we don't do anything rigorous with them
      size_t num_keypoints = surf_map->GetFrameKeypoints(i).outerSize();
      size_t num_features = surf_map->GetFrameFidToPidMap(i).size();
      EXPECT_GT(num_keypoints, 200u);
      EXPECT_GT(num_features, 40u);
      EXPECT_LT(num_features, num_keypoints);
    }

    EXPECT_GT(surf_map->GetNumLandmarks(), 30u);
    // should probably do some sanity check on the landmarks...

    EXPECT_EQ(surf_map->GetRansacIterations(), 1000);
    EXPECT_EQ(surf_map->GetRansacInlierTolerance(), 3);
    EXPECT_NEAR(surf_map->GetCameraParameters().GetFocalLength(), 258.5, 1e-5);
  }

  virtual void TearDown() {
    delete surf_map;
  }
};

TEST(Registration, CheckHuginFileIO) {
  std::vector<std::string> images;
  Eigen::MatrixXd user_ip;  // This shouldn't be full dynamic
  sparse_mapping::ParseHuginControlPoints(DATA_DIR + "m0000182-m0000210.pto",
                                          &images, &user_ip);
  EXPECT_EQ(6, user_ip.rows());
  EXPECT_EQ(15, user_ip.cols());
  EXPECT_STREQ("m0000182.jpg", images[0].c_str());
  EXPECT_STREQ("m0000210.jpg", images[1].c_str());
}

TEST(Registration, CheckXYZFileIO) {
  Eigen::MatrixXd user_xyz;
  sparse_mapping::ParseXYZ(DATA_DIR + "xyz.txt", &user_xyz);
  EXPECT_EQ(3, user_xyz.rows());
  EXPECT_EQ(15, user_xyz.cols());
}

TEST_F(SparseMapTest, Registration) {
  GenerateFullMap();  // This takes a really long time. Maybe we should
                      // be producing artificial maps.

  // Perform Registration
  std::string control_points = DATA_DIR + "m0000182-m0000210.pto";
  std::string xyz_points = DATA_DIR + "xyz.txt";
  std::vector<std::string> files;
  files.push_back(control_points);
  files.push_back(xyz_points);
  bool verification = false;
  sparse_mapping::RegistrationOrVerification(files, verification, surf_map);
  double scale = pow(surf_map->GetWorldTransform().linear().determinant(), 1.0/3.0);
  EXPECT_GT(scale, 0);  // just a sanity check
}
