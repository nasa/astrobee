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

#include <common/utils.h>
#include <glog/logging.h>

#include <sys/stat.h>
#include <dirent.h>
#include <string>
#include <vector>

// List all files in given directory with given extension, e.g.,
// 'jpg'. The directory name is pre-pended to the file names, that
// is, the output is in the form dir/file.jpg.  Sort the list
// alphabetically.
void common::ListFiles(std::string const& input_dir, std::string const& ext,
                       std::vector<std::string> * files) {
  // Wipe the output
  files->clear();

  // Load up the directory
  DIR* dirp = opendir(input_dir.c_str());
  if (!dirp) {
    LOG(FATAL) << "Unable to open directory: " << input_dir;
  }

  // Iterate through files in the directory
  dirent* dp;
  while ((dp = readdir(dirp)) != NULL) {
    std::string filename(dp->d_name);
    size_t len = ext.size();
    if (filename.size() > len &&
        filename.substr(filename.size() - len, len) == ext)
      files->push_back(input_dir + "/" + filename);
  }
  closedir(dirp);

  // Sort the files
  std::sort(files->begin(), files->end());
}

void common::PrintProgressBar(FILE* stream, float progress) {
  int width = 70;

  putc('[', stream);
  int pos = static_cast<int>(width * progress);
  for (int i = 0; i < width; i++) {
      if (i < pos)
        putc('=', stream);
      else if (i == pos)
        putc('>', stream);
      else
        putc(' ', stream);
  }
  fprintf(stream, "] %d%%\r", static_cast<int>(progress * 100));
  fflush(stream);
}

std::string common::ReplaceInStr(std::string const& in_str,
                                 std::string const& before,
                                 std::string const& after) {
  std::string out_str = in_str;
  std::size_t found = out_str.find(before);
  if (found != std::string::npos)
    out_str.replace(found, before.size(), after);

  return out_str;
}

std::string common::dirname(std::string const& file) {
  size_t base_path_start = file.find_last_of("\\/");
  if (base_path_start == std::string::npos)
    return ".";
  return file.substr(0, base_path_start);
}

std::string common::basename(std::string const& file) {
  size_t base_path_start = file.find_last_of("\\/");
  if (base_path_start == std::string::npos)
    base_path_start = 0;
  else
    base_path_start++;
  return file.substr(base_path_start, file.size());
}

std::string common::file_extension(std::string const& file) {
  size_t it = file.find_last_of(".");

  if (it == std::string::npos || it + 1 >= file.size())
    return "";

  return file.substr(it + 1);
}

// Extract values from a string to a vector of doubles
void common::parseStr(std::string const& str, std::vector<double> & values) {
  values.clear();
  std::istringstream is(str);
  double val;
  while (is >> val)
    values.push_back(val);
}

