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

#include <common/thread.h>
#include <camera/camera_model.h>
#include <sparse_mapping/tensor.h>
#include <sparse_mapping/sparse_map.h>

#include <gtest/gtest.h>

#include <memory>
#include <string>
#include <vector>
#include <map>

#define EXPECT_VECTOR3D_NEAR(p1, p2, t) EXPECT_NEAR(p1[0], p2[0], t); EXPECT_NEAR(p1[1], p2[1], t); \
  EXPECT_NEAR(p1[2], p2[2], t);
#define ASSERT_VECTOR2D_NEAR(p1, p2, t) ASSERT_NEAR(p1[0], p2[0], t); ASSERT_NEAR(p1[1], p2[1], t);

struct Parameters {
  std::string detector, descriptor;
  bool close_loop;
};

void CompareFeatures(sparse_mapping::SparseMap const& map1,
                     sparse_mapping::SparseMap const& map_loopback) {
  EXPECT_EQ(map1.GetNumFrames(), map_loopback.GetNumFrames());
  for (size_t frame = 0; frame < map1.GetNumFrames(); frame++) {
    EXPECT_EQ(map1.GetFrameFilename(frame),
              map_loopback.GetFrameFilename(frame));

    // Compare Keypoints
    Eigen::Matrix2Xd const& keys1 = map1.GetFrameKeypoints(frame);
    Eigen::Matrix2Xd const& keys2 = map_loopback.GetFrameKeypoints(frame);
    ASSERT_EQ(keys1.cols(), keys2.cols());
    ASSERT_EQ(keys1.rows(), keys2.rows());
    EXPECT_NEAR(0, (keys1 - keys2).array().abs().sum(),
        1e-6);

    // Compare Descriptors
    for (int fid = 0; fid < keys1.cols(); fid++) {
      cv::Mat row1 = map1.GetDescriptor(frame, fid);
      cv::Mat row2 = map_loopback.GetDescriptor(frame, fid);
      ASSERT_EQ(row1.rows, row2.rows);
      ASSERT_EQ(row1.cols, row2.cols);
      cv::Mat diff = row1 != row2;
      EXPECT_EQ(cv::countNonZero(diff), 0);
    }
  }
}

class SparseMapTest : public ::testing::TestWithParam<Parameters> {
 public:
  virtual ~SparseMapTest() = default;
  virtual void SetUp() {
    FLAGS_num_threads = 1;
    std::string data_dir = std::string(TEST_DIR) + "/data";
    image_filenames.push_back(data_dir + "/m0004000.jpg");
    image_filenames.push_back(data_dir + "/m0004025.jpg");
    image_filenames.push_back(data_dir + "/m0004050.jpg");
    if (GetParam().close_loop) {
      image_filenames.push_back(data_dir + "/m0004000.jpg");
      image_filenames.push_back(data_dir + "/m0004025.jpg");
    }

    params.reset(
        new camera::CameraParameters(Eigen::Vector2i(780, 620),
                                     Eigen::Vector2d::Constant(258.5),
                                     Eigen::Vector2d(390, 310)));
  }

  std::vector<std::string> image_filenames;
  std::shared_ptr<camera::CameraParameters> params;
};

TEST_P(SparseMapTest, MapBuilding) {
  sparse_mapping::SparseMap map(image_filenames, GetParam().detector, *params);

  // Detect features
  map.DetectFeatures();
  map.Save("temp.map");

  // Loop back
  sparse_mapping::SparseMap map_loopback("temp.map");
  CompareFeatures(map, map_loopback);

  // Verify we have the same camera model when we load back up into
  // map_loopback
  ASSERT_VECTOR2D_NEAR(params->GetDistortedSize(),
      map_loopback.GetCameraParameters().GetDistortedSize(), 1e-3);
  ASSERT_VECTOR2D_NEAR(params->GetDistortedHalfSize(),
      map_loopback.GetCameraParameters().GetDistortedHalfSize(), 1e-3);
  ASSERT_VECTOR2D_NEAR(params->GetUndistortedSize(),
      map_loopback.GetCameraParameters().GetUndistortedSize(), 1e-3);
  ASSERT_VECTOR2D_NEAR(params->GetUndistortedHalfSize(),
      map_loopback.GetCameraParameters().GetUndistortedHalfSize(), 1e-3);
  ASSERT_VECTOR2D_NEAR(params->GetCropOffset(),
      map_loopback.GetCameraParameters().GetCropOffset(), 1e-3);
  ASSERT_VECTOR2D_NEAR(params->GetOpticalOffset(),
      map_loopback.GetCameraParameters().GetOpticalOffset(), 1e-3);
  ASSERT_VECTOR2D_NEAR(params->GetFocalVector(),
      map_loopback.GetCameraParameters().GetFocalVector(), 1e-3);
  ASSERT_EQ(params->GetDistortion().size(), 0);
  ASSERT_EQ(map_loopback.GetCameraParameters().GetDistortion().size(), 0);

  // Feature matching
  sparse_mapping::MatchFeatures(sparse_mapping::EssentialFile("temp.map"),
                                sparse_mapping::MatchesFile("temp.map"), &map_loopback);

  // Building tracks
  sparse_mapping::BuildTracks(sparse_mapping::MatchesFile("temp.map"), &map_loopback);

  // Initialize camera positions
  sparse_mapping::IncrementalBA(sparse_mapping::EssentialFile("temp.map"), &map_loopback);
  if (GetParam().close_loop)
    sparse_mapping::CloseLoop(&map_loopback);

  // Bundle Adjustment
  sparse_mapping::BundleAdjust(false, &map_loopback);

  // Check the Map Sanity
  if (GetParam().close_loop) {
    EXPECT_EQ(image_filenames.size() - 2, map_loopback.GetNumFrames());
  } else {
    EXPECT_EQ(image_filenames.size(), map_loopback.GetNumFrames());
  }
  double default_distance =
    (map_loopback.GetFrameGlobalTransform(1).translation() -
     map_loopback.GetFrameGlobalTransform(0).translation()).norm();
  for (size_t cid = 0; cid < map_loopback.GetNumFrames(); cid++) {
    EXPECT_EQ(map_loopback.GetFrameFilename(cid), image_filenames[cid]);
    // Expect that the number of keypoints is reasonable
    EXPECT_GT(map_loopback.GetFrameKeypoints(cid).outerSize(), 200u);
    EXPECT_GT(map_loopback.GetFrameFidToPidMap(cid).size(), 40u);
    // Check that the keyframes aren't too far apart
    if (cid > 0) {
      Eigen::Vector3d pos1 = map_loopback.GetFrameGlobalTransform(cid - 1).translation();
      Eigen::Vector3d pos2 = map_loopback.GetFrameGlobalTransform(cid).translation();
      EXPECT_LT((pos1 - pos2).norm(), default_distance * 1.5);
    }
  }
  EXPECT_GT(map_loopback.GetNumLandmarks(), 30u);
  EXPECT_EQ(map_loopback.GetRansacInlierTolerance(), 3);
  EXPECT_NEAR(map_loopback.GetCameraParameters().GetFocalLength(), 258.5, 1e-5);

  // Test Map Consistency?
  // check that each frame localizes to its own position
  camera::CameraModel guess(map_loopback.GetCameraParameters());
  for (size_t i = 0; i < map_loopback.GetNumFrames(); i++) {
    Eigen::Affine3d transform = map_loopback.GetFrameGlobalTransform(i);
    EXPECT_TRUE(map_loopback.Localize(map_loopback.GetFrameFilename(i), &guess));
    Eigen::Vector3d rot1, rot2;
    camera::RotationToRodrigues(transform.rotation(), &rot1);
    camera::RotationToRodrigues(guess.GetRotation(), &rot2);
    EXPECT_VECTOR3D_NEAR(rot2, rot1, 0.01);
    Eigen::Vector3d translation = transform.inverse().translation();
    EXPECT_VECTOR3D_NEAR(guess.GetPosition(), translation, 0.01);
  }

  // Try localizing an image that wasn't in the map
  EXPECT_TRUE(map_loopback.Localize(std::string(TEST_DIR) + "/data/m0004033.jpg", &guess));
  Eigen::Affine3d
    closest1 = map_loopback.GetFrameGlobalTransform(1),
    closest2 = map_loopback.GetFrameGlobalTransform(2);
  Eigen::Vector3d rot_close, rot_guess;
  camera::RotationToRodrigues(closest1.rotation(), &rot_close);
  camera::RotationToRodrigues(guess.GetRotation(), &rot_guess);
  EXPECT_VECTOR3D_NEAR(rot_guess, rot_close, 0.03);  // THIS MEANS NOTHING SINCE WE DON'T KNOW SCALE
  Eigen::Vector3d
    close_t1 = closest1.inverse().translation(),
    close_t2 = closest2.inverse().translation();
  Eigen::Vector3d estimated = close_t1 + 8.0 / 25 * (close_t2 - close_t1);
  double accuracy = 0.08 * (close_t1 - close_t2).norm();  // unitless number.
  EXPECT_VECTOR3D_NEAR(guess.GetPosition(), estimated, accuracy);

  // Test Saving again with more information
  map_loopback.Save("temp2.map");
  sparse_mapping::SparseMap map_loopback2("temp2.map");
  CompareFeatures(map_loopback, map_loopback2);
  for (size_t frame = 0; frame < map_loopback.GetNumFrames();
      frame++) {
    EXPECT_TRUE(map_loopback.GetFrameGlobalTransform(frame).matrix().isApprox(
          map_loopback2.GetFrameGlobalTransform(frame).matrix()));

    // Check that FidToPidMaps are the same
    std::map<int, int>
      fidpid1 = map_loopback.GetFrameFidToPidMap(frame),
      fidpid2 = map_loopback2.GetFrameFidToPidMap(frame);
    ASSERT_EQ(fidpid1.size(), fidpid2.size());
    for (std::map<int, int>::iterator it = fidpid1.begin();
        it != fidpid1.end(); it++) {
      EXPECT_EQ(fidpid2[it->first], it->second);
    }
  }
  // Check that landmarks are the same
  ASSERT_EQ(map_loopback.GetNumLandmarks(),
      map_loopback2.GetNumLandmarks());
  for (size_t pid = 0; pid < map_loopback.GetNumLandmarks(); pid++) {
    EXPECT_VECTOR3D_NEAR(map_loopback.GetLandmarkPosition(pid),
        map_loopback2.GetLandmarkPosition(pid), 1e-6);

    std::map<int, int> cidpid1 = map_loopback.GetLandmarkCidToFidMap(pid);
    std::map<int, int> cidpid2 = map_loopback.GetLandmarkCidToFidMap(pid);
    ASSERT_EQ(cidpid1.size(), cidpid2.size());
    for (std::map<int, int>::iterator it = cidpid1.begin();
        it != cidpid1.begin(); it++) {
      EXPECT_EQ(cidpid2[it->first], it->second);
    }
  }
}

const Parameters test_parameters[] = {
  {"SURF", "ORGBRISK", false},
  {"SURF", "ORGBRISK", true},
};

INSTANTIATE_TEST_CASE_P(SparseMapTest, SparseMapTest,
    ::testing::ValuesIn(test_parameters));

