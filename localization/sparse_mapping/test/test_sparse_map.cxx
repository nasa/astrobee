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

#include <gtest/gtest.h>

#include <memory>
#include <string>
#include <vector>
#include <map>
#include <iostream>

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

// A little function to be able to use the same map in two tests
void mapFiles(std::string const& detector,
              bool close_loop, std::string * map1, std::string *map2) {
  std::string val = detector;
  if (close_loop) val += "_close_loop1";
  else            val += "_close_loop0";

  *map1 = "map1_" + val + ".map";
  *map2 = "map2_" + val + ".map";
}

TEST_P(SparseMapTest, MapBuilding) {
  sparse_mapping::SparseMap map(image_filenames, GetParam().detector, *params);

  std::string mapfile1, mapfile2;
  mapFiles(GetParam().detector, GetParam().close_loop, &mapfile1, &mapfile2);

  // Detect features
  map.DetectFeatures();
  map.Save(mapfile1);

  // Loop back
  sparse_mapping::SparseMap map_loopback(mapfile1);
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
  sparse_mapping::MatchFeatures(sparse_mapping::EssentialFile(mapfile1),
                                sparse_mapping::MatchesFile(mapfile1), &map_loopback);

  // Building tracks
  bool rm_invalid_xyz = true;
  sparse_mapping::BuildTracks(rm_invalid_xyz,
                              sparse_mapping::MatchesFile(mapfile1), &map_loopback);

  // Initialize camera positions
  sparse_mapping::IncrementalBA(sparse_mapping::EssentialFile(mapfile1), &map_loopback);
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
  map_loopback.Save(mapfile2);
  sparse_mapping::SparseMap map_loopback2(mapfile2);
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

// Test submap extraction and merging on two maps
TEST_P(SparseMapTest, MapExtractMerge) {
  sparse_mapping::SparseMap map(image_filenames, GetParam().detector, *params);

  // This test won't work with maps where we close the loop, as that one has
  // repeated images, which confuses the map extractor
  if (GetParam().close_loop)
    return;

  std::string mapfile1, mapfile2;
  mapFiles(GetParam().detector, GetParam().close_loop, &mapfile1, &mapfile2);

  std::string submap1_file = "map_" + GetParam().detector + "_submap1.map";
  std::string submap2_file = "map_" + GetParam().detector + "_submap2.map";

  // Extract submap1
  LOG(INFO) << "Reading: " << mapfile2 << std::endl;
  sparse_mapping::SparseMap submap1(mapfile2);
  LOG(INFO) << "Extracting submap1." << std::endl;
  std::vector<std::string> images_to_keep1;
  images_to_keep1.push_back(submap1.cid_to_filename_[0]);
  images_to_keep1.push_back(submap1.cid_to_filename_[1]);
  sparse_mapping::ExtractSubmap(&images_to_keep1, &submap1);
  LOG(INFO) << "Writing: " << submap1_file << std::endl;
  submap1.Save(submap1_file);

  // Extract submap2
  LOG(INFO) << "Reading: " << mapfile2 << std::endl;
  sparse_mapping::SparseMap submap2(mapfile2);
  LOG(INFO) << "Extracting submap2." << std::endl;
  std::vector<std::string> images_to_keep2;
  images_to_keep2.push_back(submap2.cid_to_filename_[1]);
  images_to_keep2.push_back(submap2.cid_to_filename_[2]);
  sparse_mapping::ExtractSubmap(&images_to_keep2, &submap2);
  LOG(INFO) << "Writing: " << submap2_file << std::endl;
  submap2.Save(submap2_file);

  // Append submap2 to submap1 and write the merged map
  bool prune_map = true, skip_bundle_adjustment = false;
  int num_image_overlaps_at_endpoints = 10;
  double outlier_factor = 3;
  sparse_mapping::AppendMapFile(submap1_file, submap2_file,
                                num_image_overlaps_at_endpoints, outlier_factor,
                                !skip_bundle_adjustment, prune_map);

  // Read the merged map, and check if we have 3 frames as expected
  LOG(INFO) << "Reading: " << submap1_file << std::endl;
  sparse_mapping::SparseMap merged_map(submap1_file);
  EXPECT_EQ(merged_map.GetNumFrames(), 3);
}

const Parameters test_parameters[] = {
  // Detector,  not used,       closeLoop
  {"SURF",     "ORGBRISK",      false},
  {"SURF",     "ORGBRISK",      true},
};

INSTANTIATE_TEST_CASE_P(SparseMapTest, SparseMapTest,
    ::testing::ValuesIn(test_parameters));

