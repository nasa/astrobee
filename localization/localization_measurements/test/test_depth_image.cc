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

#include <localization_common/logger.h>
#include <localization_measurements/depth_image.h>

#include <gtest/gtest.h>

namespace lm = localization_measurements;

lm::DepthImage RampedDepthImage() {
  constexpr int num_rows = 10;
  constexpr int num_cols = 10;
  cv::Mat mat(cv::Mat::ones(num_rows, num_cols, CV_8UC1));
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
  for (int row = 0; row < num_rows; ++row) {
    for (int col = 0; col < num_cols; ++col) {
      pcl::PointXYZI point;
      point.x = col;
      point.y = row;
      point.z = row * col;
      point.intensity = row * col;
      cloud->points.emplace_back(point);
    }
  }
  return lm::DepthImage(mat, cloud);
}

TEST(DepthImageTester, InvalidPoints) {
  const auto depth_image = RampedDepthImage();
  {
    const auto p = depth_image.UnfilteredPoint3D(-1, 1);
    ASSERT_TRUE(p == boost::none);
  }
  {
    const auto p = depth_image.UnfilteredPoint3D(1, -1);
    ASSERT_TRUE(p == boost::none);
  }
  {
    const auto p = depth_image.UnfilteredPoint3D(100, 1);
    ASSERT_TRUE(p == boost::none);
  }
  {
    const auto p = depth_image.UnfilteredPoint3D(1, 100);
    ASSERT_TRUE(p == boost::none);
  }
  {
    auto invalid_cloud = depth_image.unfiltered_point_cloud()->makeShared();
    pcl::PointXYZI invalid_point;
    invalid_point.x = 0;
    invalid_point.y = 0;
    invalid_point.z = 0;
    invalid_cloud->points[0] = invalid_point;
    const lm::DepthImage invalid_depth_image(depth_image.image(), invalid_cloud);
    {
      const auto p = invalid_depth_image.InterpolatePoint3D(0.5, 0);
      ASSERT_TRUE(p == boost::none);
    }
    {
      const auto p = invalid_depth_image.InterpolatePoint3D(0.5, 0.5);
      ASSERT_TRUE(p == boost::none);
    }
    {
      const auto p = invalid_depth_image.InterpolatePoint3D(0, 0.5);
      ASSERT_TRUE(p == boost::none);
    }
    {
      const auto p = invalid_depth_image.InterpolatePoint3D(1.5, 1.5);
      ASSERT_FALSE(p == boost::none);
    }
  }
}

TEST(DepthImageTester, AccessSamePointUnfilteredAndInterpolated) {
  const auto depth_image = RampedDepthImage();
  {
    const auto p = depth_image.UnfilteredPoint3D(5, 5);
    ASSERT_TRUE(p != boost::none);
    EXPECT_NEAR(p->x, 5, 1e-6);
    EXPECT_NEAR(p->y, 5, 1e-6);
    EXPECT_NEAR(p->z, 25, 1e-6);
    EXPECT_NEAR(p->intensity, 25, 1e-6);
  }
  {
    const auto p = depth_image.UnfilteredPoint3D(5.1, 5.1);
    ASSERT_TRUE(p != boost::none);
    EXPECT_NEAR(p->x, 5, 1e-6);
    EXPECT_NEAR(p->y, 5, 1e-6);
    EXPECT_NEAR(p->z, 25, 1e-6);
    EXPECT_NEAR(p->intensity, 25, 1e-6);
  }
  {
    const auto p = depth_image.InterpolatePoint3D(5, 5);
    ASSERT_TRUE(p != boost::none);
    EXPECT_NEAR(p->x, 5, 1e-6);
    EXPECT_NEAR(p->y, 5, 1e-6);
    EXPECT_NEAR(p->z, 25, 1e-6);
    EXPECT_NEAR(p->intensity, 25, 1e-6);
  }
  {
    const auto p = depth_image.InterpolatePoint3D(5.1, 5);
    ASSERT_TRUE(p != boost::none);
    EXPECT_NEAR(p->x, 5.1, 1e-6);
    EXPECT_NEAR(p->y, 5, 1e-6);
    EXPECT_NEAR(p->z, 5 * 5.1, 1e-6);
    EXPECT_NEAR(p->intensity, 5 * 5.1, 1e-6);
  }
  {
    const auto p = depth_image.InterpolatePoint3D(5, 5.1);
    ASSERT_TRUE(p != boost::none);
    EXPECT_NEAR(p->x, 5, 1e-6);
    EXPECT_NEAR(p->y, 5.1, 1e-6);
    EXPECT_NEAR(p->z, 5 * 5.1, 1e-6);
    EXPECT_NEAR(p->intensity, 5 * 5.1, 1e-6);
  }
}

// Run all the tests that were declared with TEST()
int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
