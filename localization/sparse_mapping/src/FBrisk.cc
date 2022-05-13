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

#include <sparse_mapping/FBrisk.h>

#include <vector>
#include <string>
#include <sstream>

// Adapted from DBoW2 FBrief.cc
namespace DBoW2 {

// --------------------------------------------------------------------------

void FBrisk::meanValue(const std::vector<FBrisk::pDescriptor>& descriptors, FBrisk::TDescriptor& mean) {
  mean.reset();

  if (descriptors.empty()) return;

  const int N2 = descriptors.size() / 2;

  std::vector<int> counters(FBrisk::L, 0);

  std::vector<FBrisk::pDescriptor>::const_iterator it;
  for (it = descriptors.begin(); it != descriptors.end(); ++it) {
    const FBrisk::TDescriptor& desc = **it;
    for (int i = 0; i < FBrisk::L; ++i) {
      if (desc[i]) counters[i]++;
    }
  }

  for (int i = 0; i < FBrisk::L; ++i) {
    if (counters[i] > N2) mean.set(i);
  }
}

// --------------------------------------------------------------------------

double FBrisk::distance(const FBrisk::TDescriptor& a, const FBrisk::TDescriptor& b) {
  return static_cast<double>(a ^ b).count();
}

// --------------------------------------------------------------------------

std::string FBrisk::toString(const FBrisk::TDescriptor& a) {
  return a.to_string();  // reversed
}

// --------------------------------------------------------------------------

void FBrisk::fromString(FBrisk::TDescriptor& a, const std::string& s) {
  std::stringstream ss(s);
  ss >> a;
}

// --------------------------------------------------------------------------

void FBrisk::toMat32F(const std::vector<TDescriptor>& descriptors, cv::Mat& mat) {
  if (descriptors.empty()) {
    mat.release();
    return;
  }

  const int N = descriptors.size();

  mat.create(N, FBrisk::L, CV_32F);

  for (int i = 0; i < N; ++i) {
    const TDescriptor& desc = descriptors[i];
    float* p = mat.ptr<float>(i);
    for (int j = 0; j < FBrisk::L; ++j, ++p) {
      *p = (desc[j] ? 1.f : 0.f);
    }
  }
}
}  // namespace DBoW2
