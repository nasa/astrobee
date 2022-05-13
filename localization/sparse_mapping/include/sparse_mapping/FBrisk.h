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

#ifndef SPARSE_MAPPING_FBRISK_H_
#define SPARSE_MAPPING_FBRISK_H_

#include <DBoW2/FClass.h>

#include <opencv2/core.hpp>
#include <bitset>
#include <vector>
#include <string>

// Adapted from DBoW2 FBrief.h
namespace DBoW2 {

/// Functions to manipulate BRIEF descriptors
class FBrisk : protected FClass {
 public:
  static const int L = 512;  // Descriptor length (in bits)
  typedef std::bitset<L> TDescriptor;
  typedef const TDescriptor* pDescriptor;

  /**
   * Calculates the mean value of a set of descriptors
   * @param descriptors
   * @param mean mean descriptor
   */
  static void meanValue(const std::vector<pDescriptor>& descriptors, TDescriptor& mean);

  /**
   * Calculates the distance between two descriptors
   * @param a
   * @param b
   * @return distance
   */
  static double distance(const TDescriptor& a, const TDescriptor& b);

  /**
   * Returns a string version of the descriptor
   * @param a descriptor
   * @return string version
   */
  static std::string toString(const TDescriptor& a);

  /**
   * Returns a descriptor from a string
   * @param a descriptor
   * @param s string version
   */
  static void fromString(TDescriptor& a, const std::string& s);

  /**
   * Returns a mat with the descriptors in float format
   * @param descriptors
   * @param mat (out) NxL 32F matrix
   */
  static void toMat32F(const std::vector<TDescriptor>& descriptors, cv::Mat& mat);
};
}  // namespace DBoW2

#endif  // SPARSE_MAPPING_FBRISK_H_
