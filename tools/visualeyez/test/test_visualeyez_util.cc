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

// Required for the test framework
#include <gtest/gtest.h>

// Required for the test cases
#include <ros/ros.h>

// Visualeyez interface
#include <visualeyez/visualeyez.h>

// Check that targets_o[name] matches targets_i[name] for some given name
void Evaluate(visualeyez::VZTargetMarkers & targets_i,
  visualeyez::VZTargetMarkers & targets_o, std::string const& name) {
  EXPECT_FALSE(targets_i.find(name) == targets_i.end());
  EXPECT_FALSE(targets_o.find(name) == targets_o.end());
  for (visualeyez::VZMarkers::iterator it = targets_i[name].begin();
    it != targets_i[name].end(); it++) {
    EXPECT_EQ(targets_i[name][it->first](0), targets_o[name][it->first](0));
    EXPECT_EQ(targets_i[name][it->first](1), targets_o[name][it->first](1));
    EXPECT_EQ(targets_i[name][it->first](2), targets_o[name][it->first](2));
  }
}

// Write an calibration file, then open and read it. Check that the written content
// matches the original content.
TEST(test_visualeyez_util, IO) {
  // Prepare some input data
  visualeyez::VZTargetMarkers targets_i;
  targets_i["alpha"][visualeyez::VZIndex(1, 1)] = Eigen::Vector3d(1.0, 2.0, 3.0);
  targets_i["alpha"][visualeyez::VZIndex(1, 2)] = Eigen::Vector3d(4.0, 5.0, 6.0);
  targets_i["alpha"][visualeyez::VZIndex(1, 3)] = Eigen::Vector3d(7.0, 8.0, 9.0);
  targets_i["bravo"][visualeyez::VZIndex(2, 1)] = Eigen::Vector3d(9.0, 8.0, 7.0);
  targets_i["bravo"][visualeyez::VZIndex(2, 2)] = Eigen::Vector3d(6.0, 5.0, 4.0);
  targets_i["bravo"][visualeyez::VZIndex(2, 3)] = Eigen::Vector3d(3.0, 2.0, 1.0);
  // Write to the binary file
  EXPECT_TRUE(visualeyez::VisualeyezUtils::WriteConfig("/tmp/test.bin", targets_i));
  // If we have an empty input, then we should get nothing back
  visualeyez::VZTargetMarkers targets_o;
  EXPECT_TRUE(visualeyez::VisualeyezUtils::ReadConfig("/tmp/test.bin", targets_o));
  EXPECT_EQ(targets_o.size(), 0);
  // If we just add one, then we should just get one back
  targets_o["alpha"] = visualeyez::VZMarkers();
  EXPECT_TRUE(visualeyez::VisualeyezUtils::ReadConfig("/tmp/test.bin", targets_o));
  EXPECT_EQ(targets_o.size(), 1);
  Evaluate(targets_i, targets_o, "alpha");
  // If we add another, then we should just get two back (the original should be copied over)
  targets_o["bravo"] = visualeyez::VZMarkers();
  EXPECT_TRUE(visualeyez::VisualeyezUtils::ReadConfig("/tmp/test.bin", targets_o));
  EXPECT_EQ(targets_o.size(), 2);
  Evaluate(targets_i, targets_o, "alpha");
  Evaluate(targets_i, targets_o, "bravo");
  // Now if we add something that doesn't exist we should get a failure
  targets_o["charlie"] = visualeyez::VZMarkers();
  EXPECT_FALSE(visualeyez::VisualeyezUtils::ReadConfig("/tmp/test.bin", targets_o));
  // Fail correctly
  ros::shutdown();
}

// Kabsch algorithm with scaling
TEST(test_visualeyez_util, KabschWithScale) {
  Eigen::Matrix3Xd in(3, 100), out(3, 100);
  Eigen::Quaternion<double> Q(1, 3, 5, 2);
  Q.normalize();
  Eigen::Matrix3d R = Q.toRotationMatrix();
  double scale = 2.0;
  for (int row = 0; row < in.rows(); row++) {
    for (int col = 0; col < in.cols(); col++) {
      in(row, col) = log(2*row + 10.0)/sqrt(1.0*col + 4.0) + sqrt(col*1.0)/(row + 1.0);
    }
  }
  Eigen::Vector3d S;
  S << -5, 6, -27;
  for (int col = 0; col < in.cols(); col++)
    out.col(col) = scale*R*in.col(col) + S;
  Eigen::Affine3d A;
  EXPECT_TRUE(visualeyez::VisualeyezUtils::Kabsch <double> (in, out, A, true));
  EXPECT_LT((scale*R-A.linear()).cwiseAbs().maxCoeff(), 1e-13);
  EXPECT_LT((S-A.translation()).cwiseAbs().maxCoeff(), 1e-13);
}

// Kabsch algorithm without scaling
TEST(test_visualeyez_util, KabschWithoutScale) {
  Eigen::Matrix3Xd in(3, 100), out(3, 100);
  Eigen::Quaternion<double> Q(1, 3, 5, 2);
  Q.normalize();
  Eigen::Matrix3d R = Q.toRotationMatrix();
  for (int row = 0; row < in.rows(); row++) {
    for (int col = 0; col < in.cols(); col++) {
      in(row, col) = log(2*row + 10.0)/sqrt(1.0*col + 4.0) + sqrt(col*1.0)/(row + 1.0);
    }
  }
  Eigen::Vector3d S;
  S << -5, 6, -27;
  for (int col = 0; col < in.cols(); col++)
    out.col(col) = R*in.col(col) + S;
  Eigen::Affine3d A;
  EXPECT_TRUE(visualeyez::VisualeyezUtils::Kabsch < double > (in, out, A, false));
  EXPECT_LT((R-A.linear()).cwiseAbs().maxCoeff(), 1e-13);
  EXPECT_LT((S-A.translation()).cwiseAbs().maxCoeff(), 1e-13);
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv) {
  // Initialize ROS and create a single node handle in order to use ros::Time::now()
  ros::init(argc, argv, "test_visualeyez_util", ros::init_options::AnonymousName);
  ros::NodeHandle nh("~");
  // Initialize the gtest framework
  testing::InitGoogleTest(&argc, argv);
  // Run all test procedures
  return RUN_ALL_TESTS();
}
