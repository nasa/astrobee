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



#ifndef EXECUTIVE_UTILS_SEQUENCER_PLAN_IO_H_
#define EXECUTIVE_UTILS_SEQUENCER_PLAN_IO_H_

#include <ff_msgs/CompressedFile.h>

#include <boost/iostreams/copy.hpp>
#include <boost/iostreams/device/back_inserter.hpp>
#include <boost/iostreams/device/array.hpp>
#include <boost/iostreams/filter/bzip2.hpp>
#include <boost/iostreams/filter/gzip.hpp>
#include <boost/iostreams/filter/zlib.hpp>
#include <boost/iostreams/filtering_stream.hpp>

#include <glog/logging.h>

#include <string>

namespace sequencer {

bool DecompressData(const char* data, const std::size_t length,
                    int const type, std::string * out);

}  // end namespace sequencer

#endif  // EXECUTIVE_UTILS_SEQUENCER_PLAN_IO_H_
