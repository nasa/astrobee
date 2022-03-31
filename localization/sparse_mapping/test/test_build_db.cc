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
#include <sparse_mapping/sparse_mapping.h>
#include <sparse_mapping/vocab_tree.h>
#include <sparse_mapping/tensor.h>
#include <sparse_mapping/sparse_map.h>

#include <opencv2/highgui/highgui.hpp>
#include <gtest/gtest.h>
#include <glog/logging.h>

#include <map>
#include <string>
#include <vector>

#define EXPECT_VECTOR3D_NEAR(p1, p2, t) EXPECT_NEAR(p1[0], p2[0], t); EXPECT_NEAR(p1[1], p2[1], t); \
  EXPECT_NEAR(p1[2], p2[2], t);

void RunWithDB(std::string const& detector_name) {
  // Create descriptors, save them to disk, and then put them into a vocab tree.
  // Later test the vocab tree with feature matching and localization.

  // Need to be single thread for the safety of Bamboo CI
  FLAGS_num_threads = 1;

  std::vector<std::string> img_files, local_imgs, feat_files;
  std::string data_dir = std::string(std::getenv("DATA_DIR"));
  std::string f;
  f = "m0004000.jpg"; img_files.push_back(data_dir + f); local_imgs.push_back(f);
  f = "m0004025.jpg"; img_files.push_back(data_dir + f); local_imgs.push_back(f);
  f = "m0004050.jpg"; img_files.push_back(data_dir + f); local_imgs.push_back(f);

  // Write local copies of the images.
  for (size_t cid = 0; cid < img_files.size(); cid++) {
    cv::Mat image = cv::imread(img_files[cid], cv::IMREAD_GRAYSCALE);
    cv::imwrite(local_imgs[cid], image);
  }

  std::string features_out = "test_map.map";
  std::string tree_out = "tree.out";
  std::string db_out = "vocab.db";

  // Feature detection
  camera::CameraParameters camera_params(data_dir + "iss_tango_undistorted.xml");
  sparse_mapping::SparseMap features_map(img_files, detector_name, camera_params);
  features_map.DetectFeatures();
  features_map.Save(features_out);

  // Build database and save it to disk
  LOG(INFO) << "\n\n================================================\n";
  LOG(INFO) << "\nBuilding the database\n";
  int depth =  5;
  int branching_factor = 5;
  int restarts = 1;
  sparse_mapping::BuildDB(features_out,
                          detector_name, depth, branching_factor,
                          restarts);


  // Matching features using the database
  LOG(INFO) << "\n\n================================================\n";
  LOG(INFO) << "\nMatching using the database\n";
  int num_similar = 6;
  std::string out_nvm = "output.nvm";
  sparse_mapping::SparseMap map(features_out);
  map.SetNumSimilar(num_similar);
  EXPECT_EQ(static_cast<int>(map.vocab_db_.m_num_nodes), 3);
  map.DetectFeatures();
  std::string essential_file = "essential.csv";
  std::string matches_file = "matches.txt";
  sparse_mapping::MatchFeatures(essential_file, matches_file, &map);
  bool rm_invalid_xyz = true;
  sparse_mapping::BuildTracks(rm_invalid_xyz, matches_file, &map);
  EXPECT_GT(map.GetNumLandmarks(), 30u);  // Limited now by our max pairwise feature.
  sparse_mapping::IncrementalBA(essential_file, &map);
  ceres::Solver::Options options;
  options.linear_solver_type = ceres::ITERATIVE_SCHUR;
  options.num_threads = FLAGS_num_threads;
  options.max_num_iterations = 1000;
  options.minimizer_progress_to_stdout = false;
  ceres::Solver::Summary summary;
  sparse_mapping::BundleAdjustment(&map, new ceres::CauchyLoss(1.0), options, &summary);
  map.Save(out_nvm);

  sparse_mapping::BuildDB(out_nvm,
                          detector_name, depth, branching_factor,
                          restarts);

  // Localize features with database.
  sparse_mapping::SparseMap map2(out_nvm);
  map2.SetNumSimilar(num_similar);
  LOG(INFO) << "\n\n================================================\n";
  LOG(INFO) << "\nLocalizing using the database\n";

  camera::CameraModel camera(Eigen::Vector3d(), Eigen::Matrix3d::Identity(), camera_params);
  std::string img_file = data_dir + "/m0004033.jpg";
  EXPECT_TRUE(map2.Localize(img_file, &camera));

  Eigen::Affine3d closest = map2.GetFrameGlobalTransform(1);
  Eigen::Affine3d closest2 = map2.GetFrameGlobalTransform(2);
  Eigen::Vector3d close_t1 = closest.inverse().translation();
  Eigen::Vector3d close_t2 = closest2.inverse().translation();
  Eigen::Vector3d estimated = close_t1 + 8.0 / 25 * (close_t2 - close_t1);
  double accuracy = 0.5 * (close_t1 - close_t2).norm();  // Unitless number.
  LOG(INFO) << close_t1 << "\n";
  LOG(INFO) << close_t2 << "\n";
  EXPECT_VECTOR3D_NEAR(camera.GetPosition(), estimated, accuracy);

  std::remove(features_out.c_str());
  std::remove(tree_out.c_str());
  std::remove(out_nvm.c_str());
  std::remove(db_out.c_str());
}

TEST(build_db_dbow2_orgbrisk, write_descriptors_build_db) {
  RunWithDB("ORGBRISK");
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
