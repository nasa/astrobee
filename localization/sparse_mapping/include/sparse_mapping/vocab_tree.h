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

/*
 * Copyright 2011-2012 Noah Snavely, Cornell University
 * (snavely@cs.cornell.edu).  All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:

 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY NOAH SNAVELY ''AS IS'' AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL NOAH SNAVELY OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
 * OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
 * USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
 * DAMAGE.
 *
 * The views and conclusions contained in the software and
 * documentation are those of the authors and should not be
 * interpreted as representing official policies, either expressed or
 * implied, of Cornell University.
 *
 */

#ifndef SPARSE_MAPPING_VOCAB_TREE_H_
#define SPARSE_MAPPING_VOCAB_TREE_H_

#include <google/protobuf/io/zero_copy_stream_impl.h>

#include <vector>
#include <string>
#include <map>

namespace cv {
  class Mat;
}

namespace sparse_mapping {
  class SparseMap;
  class BinaryDB;
  class FloatDB;

  // A class for holding a vocab database of features.
  struct VocabDB {
    // There can be only one type of database now:
    // - DBoW2 binary descriptors (e.g., BRISK, BRIEF)
    // Only one of these is active at one time.
    BinaryDB  * binary_db;

    int m_num_nodes;
    VocabDB();
    ~VocabDB();
    void SaveProtobuf(google::protobuf::io::ZeroCopyOutputStream* output) const;
    void LoadProtobuf(google::protobuf::io::ZeroCopyInputStream* input, int db_type);
  };

  void DBSanityChecks(std::string const& db_type,
                      std::string const& descriptor);

  void BuildDB(std::string const& map_file,
               std::string const& descriptor,
               int depth, int branching_factor, int restarts);

  void ResetDB(VocabDB* db);

  // Query similar images from database
  void QueryDB(std::string const& descriptor,
               VocabDB * vocab_db,
               int num_similar,
               cv::Mat const& descriptors,
               std::vector<int> * indices);

  void BuildDBforDBoW2(sparse_mapping::SparseMap* map,
                       std::string const& descriptor,
                       int depth, int branching_factor, int restarts);
}  // namespace sparse_mapping

#endif  // SPARSE_MAPPING_VOCAB_TREE_H_
