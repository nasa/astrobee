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

#ifndef SPARSE_MAPPING_TEMPLATED_IMAGE_DATABASE_PROTOBUF_H_
#define SPARSE_MAPPING_TEMPLATED_IMAGE_DATABASE_PROTOBUF_H_

#include <sparse_map.pb.h>
#include <google/protobuf/io/zero_copy_stream_impl.h>

// Implementation
template <class TDescriptor, class F>
TemplatedImageDatabase<TDescriptor, F>::TemplatedImageDatabase(google::protobuf::io::ZeroCopyInputStream* input)
    : DBoW2::TemplatedDatabase<TDescriptor, F>() {
  LoadProtobuf(input);
}

template <class TDescriptor, class F>
void TemplatedImageDatabase<TDescriptor, F>::LoadProtobuf(google::protobuf::io::ZeroCopyInputStream* input) {
  TemplatedFeatureVocabulary<TDescriptor, F>* voc = new TemplatedFeatureVocabulary<TDescriptor, F>();
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

template <class TDescriptor, class F>
void TemplatedImageDatabase<TDescriptor, F>::SaveProtobuf(google::protobuf::io::ZeroCopyOutputStream* output) const {
  (dynamic_cast<TemplatedFeatureVocabulary<TDescriptor, F>*>(this->m_voc))->SaveProtobuf(output);

  sparse_mapping_protobuf::DBoWDB db;

  db.set_num_entries(this->m_nentries);

  int num_inverted_index = 0;
  typename DBoW2::TemplatedDatabase<TDescriptor, F>::InvertedFile::const_iterator iit;
  for (iit = this->m_ifile.begin(); iit != this->m_ifile.end(); ++iit) num_inverted_index += (*iit).size();
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
}  // namespace sparse_mapping
#endif  // SPARSE_MAPPING_TEMPLATED_IMAGE_DATABASE_PROTOBUF_H_
