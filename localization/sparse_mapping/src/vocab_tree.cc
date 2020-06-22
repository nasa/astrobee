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

#include <sparse_mapping/vocab_tree.h>
// TODO(bcoltin) remove circular dependency?
#include <sparse_mapping/sparse_map.h>
#include <sparse_mapping/sparse_mapping.h>
#include <sparse_map.pb.h>
#include <glog/logging.h>
#include <opencv2/highgui/highgui.hpp>
#include <ff_common/utils.h>

// DBoW2 utils
#pragma GCC diagnostic ignored "-Wdelete-non-virtual-dtor"
#pragma GCC diagnostic push
#include <DBoW2/DBoW2.h>      // BoW db that works with both float and binary descriptors
#pragma GCC diagnostic pop

#include <vector>
#include <string>

namespace sparse_mapping {

// extend vocabulary and database classes so we can save to protobuf.
// the default saving is in ASCII and extraordinarily large and slow.
template<class TDescriptor, class F>
class ProtobufVocabulary : public DBoW2::TemplatedVocabulary<TDescriptor, F> {
 public:
  ProtobufVocabulary(int k = 10, int L = 5,
          DBoW2::WeightingType weighting = DBoW2::TF_IDF, DBoW2::ScoringType scoring = DBoW2::L1_NORM) :
      DBoW2::TemplatedVocabulary<TDescriptor, F>(k, L, weighting, scoring) {}
  explicit ProtobufVocabulary(google::protobuf::io::ZeroCopyInputStream* input) :
      DBoW2::TemplatedVocabulary<TDescriptor, F>() {LoadProtobuf(input);}
  void SaveProtobuf(google::protobuf::io::ZeroCopyOutputStream* output) const;
  void LoadProtobuf(google::protobuf::io::ZeroCopyInputStream* input);
};

template<class TDescriptor, class F>
class ProtobufDatabase : public DBoW2::TemplatedDatabase<TDescriptor, F> {
 public:
  explicit ProtobufDatabase(google::protobuf::io::ZeroCopyInputStream* input)
     : DBoW2::TemplatedDatabase<TDescriptor, F>() {LoadProtobuf(input);}
  ProtobufDatabase(ProtobufVocabulary<TDescriptor, F> const& voc, bool flag, int val) :
     DBoW2::TemplatedDatabase<TDescriptor, F>(voc, flag, val) {}
  void SaveProtobuf(google::protobuf::io::ZeroCopyOutputStream* output) const;
  void LoadProtobuf(google::protobuf::io::ZeroCopyInputStream* input);
};

typedef ProtobufVocabulary<DBoW2::FBrief::TDescriptor, DBoW2::FBrief> BinaryVocabulary;
typedef ProtobufDatabase<DBoW2::FBrief::TDescriptor, DBoW2::FBrief> BriefDatabase;

// Thin wrappers around DBoW2 databases, to avoid exposing the
// originals in the header file for compilation speed.
class BinaryDB : public BriefDatabase {
 public:
  explicit BinaryDB(google::protobuf::io::ZeroCopyInputStream* input) : BriefDatabase(input) {}
  BinaryDB(BinaryVocabulary const& voc, bool flag, int val):
       BriefDatabase(voc, flag, val){}
};

template<class TDescriptor, class F>
void ProtobufVocabulary<TDescriptor, F>::LoadProtobuf(google::protobuf::io::ZeroCopyInputStream* input) {
  // C++ is a dumb language, we have to put this in front of all member variables inherited from
  // a templated class or it can't find them
  this->m_words.clear();
  this->m_nodes.clear();

  sparse_mapping_protobuf::DBoWVocab vocab;
  if (!ReadProtobufFrom(input, &vocab)) {
    LOG(FATAL) << "Failed to parse vocab file.";
  }

  this->m_k = vocab.k();
  this->m_L = vocab.l();
  this->m_scoring = (DBoW2::ScoringType)vocab.scoring_type();
  this->m_weighting = (DBoW2::WeightingType)vocab.weighting_type();
  int num_nodes = vocab.num_nodes();
  int num_words = vocab.num_words();

  this->createScoringObject();

  this->m_nodes.resize(num_nodes + 1);  // +1 to include root
  this->m_nodes[0].id = 0;

  for (int i = 0; i < num_nodes; ++i) {
    sparse_mapping_protobuf::DBoWNode node;
    if (!ReadProtobufFrom(input, &node)) {
      LOG(FATAL) << "Failed to parse node file.";
    }
    DBoW2::NodeId nid = node.node_id();
    DBoW2::NodeId pid = node.parent_id();
    DBoW2::WordValue weight = (DBoW2::WordValue)node.weight();
    std::string d = node.feature();

    this->m_nodes[nid].id = nid;
    this->m_nodes[nid].parent = pid;
    this->m_nodes[nid].weight = weight;
    this->m_nodes[pid].children.push_back(nid);

    F::fromBytes(this->m_nodes[nid].descriptor, d);
  }

  // words
  this->m_words.resize(num_words);
  for (int i = 0; i < num_words; ++i) {
    sparse_mapping_protobuf::DBoWWord word;
    if (!ReadProtobufFrom(input, &word)) {
      LOG(FATAL) << "Failed to parse word file.";
    }
    DBoW2::NodeId wid = word.word_id();
    DBoW2::NodeId nid = word.node_id();

    this->m_nodes[nid].word_id = wid;
    this->m_words[wid] = &this->m_nodes[nid];
  }
}

template<class TDescriptor, class F>
void ProtobufDatabase<TDescriptor, F>::LoadProtobuf(google::protobuf::io::ZeroCopyInputStream* input) {
  ProtobufVocabulary<TDescriptor, F>* voc = new ProtobufVocabulary<TDescriptor, F>();
  voc->LoadProtobuf(input);
  this->m_voc = voc;

  sparse_mapping_protobuf::DBoWDB db;

  if (!ReadProtobufFrom(input, &db)) {
    LOG(FATAL) << "Failed to parse db file.";
  }

  this->clear();  // resizes inverted file

  this->m_nentries = db.num_entries();
  this->m_use_di = 0;
  this->m_dilevels = 0;

  for (int i = 0; i < db.num_inverted_index(); ++i) {
    sparse_mapping_protobuf::DBoWInvertedIndexEntry entry;
    if (!ReadProtobufFrom(input, &entry)) {
      LOG(FATAL) << "Failed to parse index entry.";
    }
    DBoW2::WordId wid = entry.word_id();
    DBoW2::EntryId eid = entry.entry_id();
    DBoW2::WordValue v = entry.weight();

    this->m_ifile[wid].push_back(typename DBoW2::TemplatedDatabase<TDescriptor, F>::IFPair(eid, v));
  }
}

template<class TDescriptor, class F>
void ProtobufVocabulary<TDescriptor, F>::SaveProtobuf(google::protobuf::io::ZeroCopyOutputStream* output) const {
  sparse_mapping_protobuf::DBoWVocab vocab;

  vocab.set_k(this->m_k);
  vocab.set_l(this->m_L);
  vocab.set_scoring_type(this->m_scoring);
  vocab.set_weighting_type(this->m_weighting);
  vocab.set_num_nodes(this->m_nodes.size() - 1);  // -1 to exclude root node
  vocab.set_num_words(this->m_words.size());
  if (!WriteProtobufTo(vocab, output)) {
    LOG(FATAL) << "Failed to write vocab to file.";
  }

  std::vector<DBoW2::NodeId> parents, children;
  std::vector<DBoW2::NodeId>::const_iterator pit;
  parents.push_back(0);  // root
  while (!parents.empty()) {
    DBoW2::NodeId pid = parents.back();
    parents.pop_back();

    typename DBoW2::TemplatedVocabulary<TDescriptor, F>::Node const& parent = this->m_nodes[pid];
    children = parent.children;
    for (pit = children.begin(); pit != children.end(); pit++) {
      typename DBoW2::TemplatedVocabulary<TDescriptor, F>::Node const& child = this->m_nodes[*pit];

      sparse_mapping_protobuf::DBoWNode node;
      node.set_node_id(child.id);
      node.set_parent_id(pid);
      node.set_weight(child.weight);
      node.set_feature(F::toBytes(child.descriptor));
      if (!WriteProtobufTo(node, output)) {
        LOG(FATAL) << "Failed to write db node to file.";
      }

      // add to parent list
      if (!child.isLeaf()) {
        parents.push_back(*pit);
      }
    }
  }

  typename std::vector<typename DBoW2::TemplatedVocabulary<TDescriptor, F>::Node*>::const_iterator wit;
  for (wit = this->m_words.begin(); wit != this->m_words.end(); wit++) {
    sparse_mapping_protobuf::DBoWWord word;
    typename DBoW2::WordId id = wit - this->m_words.begin();
    word.set_word_id(id);
    word.set_node_id((*wit)->id);
    if (!WriteProtobufTo(word, output)) {
      LOG(FATAL) << "Failed to write word to file.";
    }
  }
}

template<class TDescriptor, class F>
void ProtobufDatabase<TDescriptor, F>::SaveProtobuf(google::protobuf::io::ZeroCopyOutputStream* output) const {
  (dynamic_cast<ProtobufVocabulary<TDescriptor, F>* >(this->m_voc))->SaveProtobuf(output);

  sparse_mapping_protobuf::DBoWDB db;

  db.set_num_entries(this->m_nentries);

  int num_inverted_index = 0;
  typename DBoW2::TemplatedDatabase<TDescriptor, F>::InvertedFile::const_iterator iit;
  for (iit = this->m_ifile.begin(); iit != this->m_ifile.end(); ++iit)
    num_inverted_index += (*iit).size();
  db.set_num_inverted_index(num_inverted_index);
  if (!WriteProtobufTo(db, output)) {
    LOG(FATAL) << "Failed to write db to file.";
  }
  typename DBoW2::TemplatedDatabase<TDescriptor, F>::IFRow::const_iterator irit;
  int word_id = 0;
  for (iit = this->m_ifile.begin(); iit != this->m_ifile.end(); ++iit) {
    for (irit = iit->begin(); irit != iit->end(); ++irit) {
      sparse_mapping_protobuf::DBoWInvertedIndexEntry index;
      index.set_word_id(word_id);
      index.set_entry_id(irit->entry_id);
      index.set_weight(irit->word_weight);
      if (!WriteProtobufTo(index, output)) {
        LOG(FATAL) << "Failed to write db index entry to file.";
      }
    }
    word_id++;
  }
}

// Constructor and destructor for VocabDB
VocabDB::VocabDB():
  binary_db(NULL), m_num_nodes(0) {
}
VocabDB::~VocabDB() {
  ResetDB(this);
}

void VocabDB::SaveProtobuf(google::protobuf::io::ZeroCopyOutputStream* output) const {
  if (binary_db != NULL) {
    binary_db->SaveProtobuf(output);
  } else {
    LOG(ERROR) << "Unsupported database type.";
  }
}

void VocabDB::LoadProtobuf(google::protobuf::io::ZeroCopyInputStream* input, int db_type) {
  ResetDB(this);
  if (db_type == sparse_mapping_protobuf::Map::BINARYDB) {
    binary_db = new BinaryDB(input);
    m_num_nodes = binary_db->size();
  } else {
    LOG(ERROR) << "Using unsupported database type.";
  }
}

void BuildDB(std::string const& map_file,
                             std::string const& descriptor,
                             int depth, int branching_factor, int restarts) {
  SparseMap map(map_file);

  // replace any existing database
  ResetDB(&map.vocab_db_);

  int total_features = 0;
  for (size_t cid = 0; cid < map.GetNumFrames(); cid++)
    total_features += map.GetFrameKeypoints(cid).outerSize();
  while (pow(branching_factor, depth) < total_features) {
    depth++;
    LOG(WARNING) << "Database not large enough, increasing depth.";
  }
  LOG(INFO) << "Total database capacity is " << pow(branching_factor, depth)
            << ", total features to insert are " << total_features << ".";

  BuildDBforDBoW2(&map, descriptor, depth, branching_factor, restarts);

  map.Save(map_file);
}

void ResetDB(VocabDB* db) {
  if (db->binary_db != NULL) {
    delete db->binary_db;
    db->binary_db = NULL;
  }
}

// These are defined here, rather than in the header file,
// since they are very local functions, and to put them
// in the header file would require defining there
// the typedef DVision::BRIEF::bitset, which
// would imply including in the header file all DBoW2
// headers, which will slow compilation.

void MatDescrToVec(cv::Mat const& mat, std::vector<float> * vec) {
  // Go from a row matrix of float descriptors to a vector of
  // descriptors.

  if (mat.rows != 1)
    LOG(FATAL) << "Expecting a single-row matrix.\n";

  (*vec).reserve(mat.cols);
  (*vec).clear();
  for (int c = 0; c < mat.cols; c++) {
    float val = static_cast<float>(mat.at<uchar>(0, c));
    (*vec).push_back(val);
  }
}

void MatDescrToVec(cv::Mat const& mat, DBoW2::BriefDescriptor * brief) {
  // Go from a row matrix of binary descriptors to a vector of
  // descriptors, extracting the bits from each byte along the way.

  if (mat.rows != 1)
    LOG(FATAL) << "Expecting a single-row matrix.\n";

  brief->Initialize(mat.cols);

  for (int c = 0; c < mat.cols; c++)
    brief->desc[c] = mat.at<uchar>(0, c);
}

// Query the database. Return the indices of the images
// which are most similar to the current image. Return
// at most num_similar such indices.
void QueryDB(std::string const& descriptor, VocabDB * vocab_db,
             int num_similar, cv::Mat const& descriptors,
             std::vector<int> * indices) {
  indices->clear();

  if (vocab_db->binary_db != NULL) {
    assert(IsBinaryDescriptor(descriptor));
    BinaryDB & db = *(vocab_db->binary_db);  // shorten

    std::vector<DBoW2::BriefDescriptor> descriptors_vec;
    for (int r = 0; r < descriptors.rows; r++) {
      DBoW2::BriefDescriptor descriptor;
      MatDescrToVec(descriptors.row(r), &descriptor);
      descriptors_vec.push_back(descriptor);
    }

    DBoW2::QueryResults ret;
    db.query(descriptors_vec, ret, num_similar);

    for (size_t j = 0; j < ret.size(); j++) {
      indices->push_back(ret[j].Id);
    }
  } else {
    // no database specified
    return;
  }

  return;
}

void BuildDBforDBoW2(SparseMap* map, std::string const& descriptor,
                     int depth, int branching_factor,
                     int restarts) {
  int num_frames = map->GetNumFrames();

  const DBoW2::WeightingType weight = DBoW2::TF_IDF;
  const DBoW2::ScoringType score = DBoW2::L1_NORM;
  int num_features = 0;

  if (!IsBinaryDescriptor(descriptor)) {
    LOG(ERROR) << "Using unsupported vocabulary database type.";
  } else {
    // Binary descriptors. For each image, copy them from a CV matrix
    // to a vector of vectors. Also extract individual bits from
    // each byte.
    std::vector<std::vector<DBoW2::FBrief::TDescriptor > > features;
    for (int cid = 0; cid < num_frames; cid++) {
      int num_keys = map->GetFrameKeypoints(cid).outerSize();
      num_features += num_keys;
      std::vector<DBoW2::FBrief::TDescriptor> descriptors;
      for (int i = 0; i < num_keys; i++) {
        cv::Mat row = map->GetDescriptor(cid, i);
        DBoW2::FBrief::TDescriptor descriptor;
        MatDescrToVec(row, &descriptor);
        descriptors.push_back(descriptor);
      }
      features.push_back(descriptors);
    }
    BinaryVocabulary voc(branching_factor, depth, weight, score);
    voc.create(features);

    BinaryDB* db = new BinaryDB(voc, false, 0);
    for (size_t i = 0; i < features.size(); i++)
      db->add(features[i]);

    map->vocab_db_.binary_db = db;
    map->vocab_db_.m_num_nodes = db->size();
  }
}

}  // namespace sparse_mapping
