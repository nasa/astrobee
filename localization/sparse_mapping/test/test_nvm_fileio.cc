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

#include <sparse_mapping/sparse_mapping.h>
#include <sparse_mapping/eigen_vectors.h>

#include <Eigen/Geometry>
#include <gtest/gtest.h>
#include <opencv2/features2d/features2d.hpp>
#include <glog/logging.h>

#include <map>
#include <string>
#include <vector>

TEST(nvm_fileio, read_write_loop) {
  // Generate a control network with very little in it.
  std::vector<Eigen::Matrix2Xd > cid_to_keypoint_map, cid_to_keypoint_map2;
  std::vector<std::string> cid_to_filename, cid_to_filename2;
  std::vector<std::map<int, int> > pid_to_cid_fid, pid_to_cid_fid2;
  std::vector<Eigen::Vector3d> pid_to_xyz, pid_to_xyz2;
  std::vector<Eigen::Affine3d>
    cid_to_camera_transform, cid_to_camera_transform2;
  cid_to_keypoint_map.resize(2, Eigen::Matrix2Xd(2, 2));
  cid_to_keypoint_map[0]
    << 1, 4, 2, 5;
  cid_to_keypoint_map[1]
    << 7, 10, 8, 11;

  cid_to_filename.push_back("monkey");
  cid_to_filename.push_back("dog");
  pid_to_cid_fid.resize(2);
  pid_to_cid_fid[0][0] = 0;
  pid_to_cid_fid[0][1] = 0;
  pid_to_cid_fid[1][0] = 1;
  pid_to_cid_fid[1][1] = 1;
  pid_to_xyz.resize(2);
  pid_to_xyz[0] << -1, -2, -3;
  pid_to_xyz[1] << -4, -5, -6;
  cid_to_camera_transform.resize(2);
  cid_to_camera_transform[0].linear() =
    Eigen::Matrix3d
    (Eigen::AngleAxisd(0.25 * M_PI, Eigen::Vector3d::UnitX()) *
     Eigen::AngleAxisd(-0.25 * M_PI, Eigen::Vector3d::UnitY()));
  cid_to_camera_transform[0].translation() << 100, 101, 102;
  cid_to_camera_transform[1].linear() =
    Eigen::Matrix3d
    (Eigen::AngleAxisd(0.7, Eigen::Vector3d::UnitY()) *
     Eigen::AngleAxisd(1.7, Eigen::Vector3d::UnitZ()));
  cid_to_camera_transform[1].translation() << 200, 201, 202;

  // Write it.
  sparse_mapping::WriteNVM(cid_to_keypoint_map,
                           cid_to_filename,
                           pid_to_cid_fid,
                           pid_to_xyz,
                           cid_to_camera_transform,
                           200, "test.nvm");

  // Read it back in
  sparse_mapping::ReadNVM("test.nvm",
                          &cid_to_keypoint_map2,
                          &cid_to_filename2,
                          &pid_to_cid_fid2,
                          &pid_to_xyz2,
                          &cid_to_camera_transform2);

  // Verify that we get the same solution as what we started with
  ASSERT_EQ(cid_to_keypoint_map.size(), cid_to_keypoint_map2.size());
  ASSERT_EQ(cid_to_filename.size(), cid_to_filename2.size());
  ASSERT_EQ(pid_to_cid_fid.size(), pid_to_cid_fid2.size());
  ASSERT_EQ(pid_to_xyz.size(), pid_to_xyz2.size());
  ASSERT_EQ(cid_to_camera_transform.size(), cid_to_camera_transform2.size());
  for (size_t cid = 0; cid < cid_to_keypoint_map.size(); cid++) {
    ASSERT_EQ(cid_to_keypoint_map[cid].cols(),
              cid_to_keypoint_map2[cid].cols());
    for (ptrdiff_t fid = 0; fid < cid_to_keypoint_map[cid].cols(); fid++) {
      EXPECT_NEAR(cid_to_keypoint_map[cid].col(fid)[0], cid_to_keypoint_map2[cid].col(fid)[0], 1e-6);
      EXPECT_NEAR(cid_to_keypoint_map[cid].col(fid)[1], cid_to_keypoint_map2[cid].col(fid)[1], 1e-6);
    }

    EXPECT_EQ(cid_to_filename[cid], cid_to_filename2[cid]);
    ASSERT_TRUE(cid_to_camera_transform2[cid].data() != NULL);
    for (size_t i = 0; i < 12; i++) {
      EXPECT_NEAR(cid_to_camera_transform[cid].data()[i],
                  cid_to_camera_transform2[cid].data()[i],
                  1e-3);
    }
  }
  for (size_t pid = 0; pid < pid_to_cid_fid.size(); pid++) {
    ASSERT_EQ(pid_to_cid_fid[pid].size(),
              pid_to_cid_fid2[pid].size());
    for (std::map<int, int>::iterator it = pid_to_cid_fid[pid].begin();
         it != pid_to_cid_fid[pid].end(); it++) {
      std::map<int, int>::iterator it2 = pid_to_cid_fid2[pid].find(it->first);
      ASSERT_FALSE(it2 == pid_to_cid_fid2[pid].end());
      EXPECT_EQ(it->second, it2->second);
    }
    ASSERT_TRUE(pid_to_xyz2[pid].data() != NULL);
    for (int i = 0; i < 3; i++) {
      EXPECT_NEAR(pid_to_xyz[pid].data()[i],
                  pid_to_xyz2[pid].data()[i],
                  1e-3);
    }
  }
}

TEST(nvm_fileio, read_write_loop_features) {
  std::vector<cv::KeyPoint> points1, points2;
  cv::Mat descriptors1(2, 8, CV_8UC1, cv::Scalar(0)), descriptors2;

  // Make fake data
  points1.push_back(cv::KeyPoint(1, 2, 3));
  points1.push_back(cv::KeyPoint(4, 5, 6));
  for (int i = 0; i < 8; i++) {
    descriptors1.at<uchar>(0, i) = i;
    descriptors1.at<uchar>(1, i) = 2 * i;
  }

  // Read / Write
  std::string detector = "BRISK", descriptor = "BRISK";
  std::string image_file = "image.jpg";
  std::string feat_file
    = sparse_mapping::ImageToFeatureFile(image_file, descriptor);

  sparse_mapping::WriteFeatures(descriptor,
                                points1, descriptors1,
                                feat_file);
  sparse_mapping::ReadFeatures(feat_file,
                               descriptor,
                               &points2, &descriptors2);

  // Compare
  ASSERT_EQ(points1.size(), points2.size());
  ASSERT_EQ(descriptors1.rows, descriptors2.rows);
  ASSERT_EQ(descriptors1.cols, descriptors2.cols);
  for (int i = 0; i < 8; i++) {
    EXPECT_EQ(descriptors1.at<uchar>(0, i), descriptors2.at<uchar>(0, i));
    EXPECT_EQ(descriptors1.at<uchar>(1, i), descriptors2.at<uchar>(1, i));
  }
}
