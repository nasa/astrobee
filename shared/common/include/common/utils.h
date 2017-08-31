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

#ifndef COMMON_UTILS_H_
#define COMMON_UTILS_H_

#include <string>
#include <vector>
#include <algorithm>
#include <numeric>

namespace common {

  // List all files in given directory with given extension, e.g.,
  // 'jpg'. The directory name is pre-pended to the file names, that
  // is, the output is in the form dir/file.jpg.  Sort the list
  // alphabetically.
  void ListFiles(std::string const& input_dir, std::string const& ext,
                 std::vector<std::string> * files);
  void PrintProgressBar(FILE* stream, float progress);

  // Replace in given string
  std::string ReplaceInStr(std::string const& in_str,
                           std::string const& before,
                           std::string const& after);

  // A little utility for finding the permutation which sorts
  // a vector in decreasing order by value.
  template<class T>
  struct sorter {
    const std::vector<T> &values;
    explicit sorter(const std::vector<T> &v) : values(v) {}
    bool operator()(int a, int b) { return values[a] > values[b]; }
  };
  template<class T> std::vector<int> rv_order(const std::vector<T> &values) {
    std::vector<int> rv(values.size());
    std::iota(rv.begin(), rv.end(), 0);
    std::sort(rv.begin(), rv.end(), sorter<T>(values));
    return rv;
  }

  std::string dirname(std::string const& file);
  std::string basename(std::string const& file);
  std::string file_extension(std::string const& file);

}  // namespace common

#endif  // COMMON_UTILS_H_
