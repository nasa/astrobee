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


#ifndef JSONLOADER_INSENSITIVE_MAP_H_
#define JSONLOADER_INSENSITIVE_MAP_H_

#include <cctype>
#include <string>
#include <unordered_map>

namespace jsonloader {

struct CaseInsensitiveEquals {
  bool operator()(const std::string &rhs, const std::string &lhs) const {
    if (rhs.size() != lhs.size())
      return false;
    for (std::string::size_type i = 0; i < rhs.size(); i++) {
      if (std::tolower(rhs[i]) != std::tolower(lhs[i]))
        return false;
    }
    return true;
  }
};

struct CaseInsensitiveHash {
  std::size_t operator()(const std::string &s) const {
    std::string s_cpy(s);
    for (std::string::size_type i = 0; i < s_cpy.size(); i++) {
      s_cpy[i] = std::tolower(s_cpy[i]);
    }
    return std::hash<std::string>()(s_cpy);
  }
};

// aww yeah, templated 'using' declaration.
// what is this *madman* going to DO next?
template<class V>
using InsensitiveMap =
  std::unordered_map<std::string, V,
                     CaseInsensitiveHash, CaseInsensitiveEquals>;

}  // namespace jsonloader

#endif  // JSONLOADER_INSENSITIVE_MAP_H_
