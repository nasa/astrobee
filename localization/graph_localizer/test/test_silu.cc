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

#include "test_utilities.h"  // NOLINT
#include <graph_localizer/silu.h>
#include <localization_common/logger.h>

#include <gtsam/base/numericalDerivative.h>

#include <gtest/gtest.h>

namespace gl = graph_localizer;
TEST(SiluTester, Jacobian) {
  for (int i = 0; i < 500; ++i) {
    const double x = gl::RandomDouble();
    gtsam::Matrix H;
    gl::Silu(x, H);
    const auto numerical_H = gtsam::numericalDerivative11<double, double>(
      boost::function<double(const double)>(boost::bind(&gl::Silu, _1, boost::none)), x);
    ASSERT_TRUE(gtsam::equal_with_abs_tol(numerical_H, H.matrix(), 1e-6));
  }
}

TEST(SiluWithOffsetTester, Jacobian) {
  for (int i = 0; i < 500; ++i) {
    const double x = gl::RandomDouble();
    const double offset = gl::RandomPositiveDouble();
    gtsam::Matrix H;
    gl::SiluWithOffset(x, offset, H);
    const auto numerical_H = gtsam::numericalDerivative21<double, double, double>(
      boost::function<double(const double, const double)>(boost::bind(&gl::SiluWithOffset, _1, _2, boost::none)), x,
      offset);
    ASSERT_TRUE(gtsam::equal_with_abs_tol(numerical_H, H.matrix(), 1e-6));
  }
}

TEST(SiluWithOffsetTester, XMuchGreaterThanOffset) {
  const double x = 3001.3;
  const double offset = 5.1;
  const double silu_x = gl::SiluWithOffset(x, offset);
  EXPECT_NEAR(x - offset, silu_x, 1e-6);
}

TEST(SiluWithOffsetTester, XGreaterThanOffset) {
  const double x = 13.2;
  const double offset = 5.1;
  const double silu_x = gl::SiluWithOffset(x, offset);
  EXPECT_NEAR(x - offset, silu_x, 1e-2);
}

TEST(SiluWithOffsetTester, XSlightlyGreaterThanOffset) {
  const double x = 5.2;
  const double offset = 5.1;
  const double silu_x = gl::SiluWithOffset(x, offset);
  EXPECT_NEAR(x - offset, silu_x, 5e-2);
}

TEST(SiluWithOffsetTester, XSlightlyLessThanOffset) {
  const double x = 5.0;
  const double offset = 5.1;
  const double silu_x = gl::SiluWithOffset(x, offset);
  EXPECT_NEAR(0.0, silu_x, 5e-2);
}

TEST(SiluWithOffsetTester, XLessThanOffset) {
  const double x = 1.0;
  const double offset = 5.1;
  const double silu_x = gl::SiluWithOffset(x, offset);
  EXPECT_NEAR(0.0, silu_x, 1e-1);
}

TEST(SiluWithOffsetTester, XMuchLessThanOffset) {
  const double x = 3.0;
  const double offset = 1003.3;
  const double silu_x = gl::SiluWithOffset(x, offset);
  EXPECT_NEAR(0.0, silu_x, 1e-6);
}

TEST(SiluWithOffsetTwoWayTester, Jacobian) {
  for (int i = 0; i < 500; ++i) {
    const double x = gl::RandomDouble();
    const double offset = gl::RandomPositiveDouble();
    gtsam::Matrix H;
    gl::SiluWithOffsetTwoWay(x, offset, H);
    const auto numerical_H = gtsam::numericalDerivative21<double, double, double>(
      boost::function<double(const double, const double)>(boost::bind(&gl::SiluWithOffsetTwoWay, _1, _2, boost::none)),
      x, offset);
    ASSERT_TRUE(gtsam::equal_with_abs_tol(numerical_H, H.matrix(), 1e-6));
  }
}

TEST(SiluWithOffsetTwoWayTester, XMuchGreaterThanOffset) {
  const double x = 3001.3;
  const double offset = 5.1;
  const double silu_x = gl::SiluWithOffsetTwoWay(x, offset);
  EXPECT_NEAR(x - offset, silu_x, 1e-6);
}

TEST(SiluWithOffsetTwoWayTester, XGreaterThanOffset) {
  const double x = 13.2;
  const double offset = 5.1;
  const double silu_x = gl::SiluWithOffsetTwoWay(x, offset);
  EXPECT_NEAR(x - offset, silu_x, 1e-2);
}

TEST(SiluWithOffsetTwoWayTester, XSlightlyGreaterThanOffset) {
  const double x = 5.2;
  const double offset = 5.1;
  const double silu_x = gl::SiluWithOffsetTwoWay(x, offset);
  EXPECT_NEAR(x - offset, silu_x, 5e-2);
}

TEST(SiluWithOffsetTwoWayTester, XSlightlyLessThanOffset) {
  const double x = 5.0;
  const double offset = 5.1;
  const double silu_x = gl::SiluWithOffsetTwoWay(x, offset);
  EXPECT_NEAR(0.0, silu_x, 5e-2);
}

TEST(SiluWithOffsetTwoWayTester, XLessThanOffset) {
  const double x = 1.0;
  const double offset = 5.1;
  const double silu_x = gl::SiluWithOffsetTwoWay(x, offset);
  EXPECT_NEAR(0.0, silu_x, 1e-1);
}

TEST(SiluWithOffsetTwoWayTester, XMuchLessThanOffset) {
  const double x = 3.0;
  const double offset = 1003.3;
  const double silu_x = gl::SiluWithOffsetTwoWay(x, offset);
  EXPECT_NEAR(0.0, silu_x, 1e-6);
}

TEST(SiluWithOffsetTwoWayTester, NegativeXLessThanOffset) {
  const double x = -13.2;
  const double offset = 5.1;
  const double silu_x = gl::SiluWithOffsetTwoWay(x, offset);
  EXPECT_NEAR(x + offset, silu_x, 1e-2);
}

TEST(SiluWithOffsetTwoWayTester, NegativeXSlightlyLessThanOffset) {
  const double x = -5.2;
  const double offset = 5.1;
  const double silu_x = gl::SiluWithOffsetTwoWay(x, offset);
  EXPECT_NEAR(x + offset, silu_x, 5e-2);
}

TEST(SiluWithOffsetTwoWayTester, NegativeXSlightlyGreaterThanOffset) {
  const double x = -5.0;
  const double offset = 5.1;
  const double silu_x = gl::SiluWithOffsetTwoWay(x, offset);
  EXPECT_NEAR(0.0, silu_x, 5e-2);
}

TEST(SiluWithOffsetTwoWayTester, NegativeXGreaterThanOffset) {
  const double x = -1.0;
  const double offset = 5.1;
  const double silu_x = gl::SiluWithOffsetTwoWay(x, offset);
  EXPECT_NEAR(0.0, silu_x, 1e-1);
}

TEST(SiluWithOffsetTwoWayTester, NegativeXMuchGreaterThanOffset) {
  const double x = -3.0;
  const double offset = 1003.3;
  const double silu_x = gl::SiluWithOffsetTwoWay(x, offset);
  EXPECT_NEAR(0.0, silu_x, 1e-6);
}

// Run all the tests that were declared with TEST()
int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
