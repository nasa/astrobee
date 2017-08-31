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

#include <executive/utils/sequencer/plan_io.h>

namespace io = boost::iostreams;

constexpr uintmax_t kMaxSize = 512 * 1024;

bool sequencer::DecompressData(const char* data, const std::size_t length,
                               int const type, std::string *out) {
  if (length == 0) {
    LOG(WARNING) << "received 0 length plan, ignoring";
    return false;
  }

  io::array_source source(data, length);

  io::filtering_ostream ostream;

  switch (type) {
    case ff_msgs::CompressedFile::TYPE_NONE:
    break;
  case ff_msgs::CompressedFile::TYPE_DEFLATE:
    ostream.push(io::zlib_decompressor());
    break;
  case ff_msgs::CompressedFile::TYPE_GZ:
    ostream.push(io::gzip_decompressor());
    break;
  case ff_msgs::CompressedFile::TYPE_BZ2:
    ostream.push(io::bzip2_decompressor());
    break;
  default:
    LOG(ERROR) << "Unsupported or unknown compression type";
    return false;
  }

  out->reserve(kMaxSize);
  ostream.push(io::back_inserter(*out));

  try {
    // TODO(tfmorse): If this is a 'duplicated' deflate stream, this will cause
    // an infinite loop. This should not happen, but there was a bug in the
    // workbench that triggered it. Eventually we should protect against that
    // case, but I'm not sure how yet.
    io::copy(source, ostream);
  } catch (io::zlib_error& e) {
    LOG(ERROR) << "zlib exception uncompressing: " << e.error();
    return false;
  } catch (std::exception& e) {
    LOG(ERROR) << "Exception uncompressing: " << e.what();
    return false;
  }

  return true;
}
