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

#ifndef MAPPER_INDEXED_OCTREE_KEY_H_
#define MAPPER_INDEXED_OCTREE_KEY_H_

#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <unordered_set>

namespace octoclass {

typedef uint16_t key_type;

// This class is used in graphs for finding the index for a given key in the octomap
class IndexedOcTreeKey {
 public:
    IndexedOcTreeKey() {}
    IndexedOcTreeKey(key_type a, key_type b, key_type c, uint index_in) {
      k_[0] = a;
      k_[1] = b;
      k_[2] = c;
      index_ = index_in;
    }

    IndexedOcTreeKey(const octomap::OcTreeKey& other, uint index_in) {
      k_[0] = other.k[0];
      k_[1] = other.k[1];
      k_[2] = other.k[2];
      index_ = index_in;
    }

    bool operator==(const IndexedOcTreeKey &other) const {
      return ((k_[0] == other[0]) && (k_[1] == other[1]) && (k_[2] == other[2]));
    }

    bool operator!=(const IndexedOcTreeKey& other) const {
      return( (k_[0] != other[0]) || (k_[1] != other[1]) || (k_[2] != other[2]) );
    }

    IndexedOcTreeKey& operator=(const IndexedOcTreeKey& other) {
      k_[0] = other.k_[0]; k_[1] = other.k_[1]; k_[2] = other.k_[2]; index_ = other.index_;
      return *this;
    }

    const key_type& operator[] (unsigned int i) const {
      return k_[i];
    }

    key_type& operator[] (unsigned int i) {
      return k_[i];
    }

    key_type k_[3];
    uint index_;

    /// Provides a hash function on Keys
    struct KeyHash{
      size_t operator()(const IndexedOcTreeKey& key) const {
        // a simple hashing function
    // explicit casts to size_t to operate on the complete range
    // constanst will be promoted according to C++ standard
        return static_cast<size_t>(key.k_[0])
          + 1447*static_cast<size_t>(key.k_[1])
          + 345637*static_cast<size_t>(key.k_[2]);
      }
    };
};

typedef std::tr1::unordered_set<IndexedOcTreeKey, IndexedOcTreeKey::KeyHash> KeySet;


// Class for saving pairs of keys/indexes
class IndexedKeySet{
 public:
    KeySet set_;

    // Methods
    void Insert(const octomap::OcTreeKey &key, uint &index) {
        set_.insert(IndexedOcTreeKey(key, index));
    }

    bool Key2Index(const octomap::OcTreeKey &key,
                   uint *index_out) {
        std::tr1::unordered_set<IndexedOcTreeKey, IndexedOcTreeKey::KeyHash>::const_iterator key2index;
        key2index = set_.find(IndexedOcTreeKey(key, 0));
        if (key2index == set_.end()) {
            return false;
        } else {
            *index_out = key2index->index_;
            return true;
        }
    }

    size_t Size() { return set_.size(); }
};

}  // namespace octoclass

#endif  // MAPPER_INDEXED_OCTREE_KEY_H_
