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

// Code taken from www.cs.cmu.edu/~coral/projects/localization/source.html
// Brian Coltin is in the team and said this code could be used for the project

#ifndef CONFIG_READER_CONFIG_READER_H_
#define CONFIG_READER_CONFIG_READER_H_

extern "C" {
#include <lua.h>
#include <lualib.h>
#include <lauxlib.h>
}

#include <glog/logging.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include <cmath>
#include <functional>
#include <string>
#include <vector>

#include "ff_common/init.h"
#include "config_reader/watch_files.h"

namespace config_reader {

class ConfigReader {
 public:
  enum FileFlags {
    Optional = 1 << 0,  // file need not be present for success
  };

  class Table {
    friend class ConfigReader;

   public:
    Table();
    Table(ConfigReader* c, const char* exp);
    Table(Table* t, const char* exp);
    Table(Table* t, int index);
    bool IsInit();
    bool CheckValExists(const char* exp);

    int GetSize();  // Only works for tables that are arrays
    bool GetTable(const char* exp, Table* table);
    bool GetTable(int index, Table* table);
    bool GetStr(const char* exp, std::string* str);
    bool GetStr(int index, std::string* str);
    bool GetBool(const char* exp, bool* val);
    bool GetBool(int index, bool* val);
    bool GetInt(const char* exp, int* val);
    bool GetInt(int index, int* val);
    bool GetLongLong(const char* exp, int64_t* val);
    bool GetLongLong(int index, int64_t* val);
    bool GetUInt(const char* exp, unsigned int* val);
    bool GetUInt(int index, unsigned int* val);
    bool GetReal(const char* exp, float* val);
    bool GetReal(int index, float* val);
    bool GetReal(const char* exp, double* val);
    bool GetReal(int index, double* val);
    bool GetPosReal(const char* exp, float* val);
    bool GetPosReal(int index, float* val);
    bool GetPosReal(const char* exp, double* val);
    bool GetPosReal(int index, double* val);
    bool GetInt(const char* exp, int* val, int min, int max);
    bool GetInt(int index, int* val, int min, int max);
    bool GetReal(const char* exp, float* val, float min, float max);
    bool GetReal(int index, float* val, float min, float max);
    bool GetReal(const char* exp, double* val, double min, double max);
    bool GetReal(int index, double* val, double min, double max);
    bool IsNumber(const char* exp);
    ConfigReader* Config() { return config_; }

   private:
    void Init(ConfigReader* c, int ref, const char* exp, int size);

    ConfigReader* config_;
    int ref_;
    std::string full_exp_;
    int size_;
  };

  ConfigReader();
  explicit ConfigReader(const char* path);
  ~ConfigReader();

  void Close();
  void Reset();
  bool IsOpen();
  void SetPath(const char* path);
  void AddFile(const char* filename, unsigned flags = 0);
  bool ReadFiles();
  bool IsFileModified();
  void CheckFilesUpdated(std::function<void(void)>);
  bool CheckValExists(const char* exp);

  bool GetTable(const char* exp, Table* table);
  bool GetStr(const char* exp, std::string* str);
  bool GetBool(const char* exp, bool* val);
  bool GetInt(const char* exp, int* val);
  bool GetLongLong(const char* exp, int64_t* val);
  bool GetUInt(const char* exp, unsigned int* val);
  bool GetReal(const char* exp, float* val);
  bool GetReal(const char* exp, double* val);
  bool GetPosReal(const char* exp, float* val);
  bool GetPosReal(const char* exp, double* val);
  bool GetInt(const char* exp, int* val, int min, int max);
  bool GetReal(const char* exp, float* val, float min, float max);
  bool GetReal(const char* exp, double* val, double min, double max);

 protected:
  class FileHeader {
   public:
    std::string filename_;     // name of file
    unsigned flags_;           // flags from FileFlags
    WatchFiles::Watch watch_;  // file modification watch
    FileHeader() {}
    FileHeader(const FileHeader& fh);
  };

  bool InitLua();
  void CloseLua();
  void ClearWatches();
  void ShowError(int err_code, const char* filename);
  bool ReadFile(const char* filename, unsigned flags);
  void AddImports();
  void AddStandard();
  bool PutObjectOnLuaStack(const char* exp);
  bool PutObjectOnLuaStack(const char* exp, int ref);
  bool PutObjectOnLuaStack(int index, int ref);
  bool IsTopValid();
  void OutputGetValueError(const char* exp, const char* type);

  bool ReadTable(const char* exp, Table* table);
  bool ReadStr(const char* exp, std::string* str);
  bool ReadBool(const char* exp, bool* val);
  bool ReadInt(const char* exp, int* val);
  bool ReadLongLong(const char* exp, int64_t* val);
  bool ReadUInt(const char* exp, unsigned int* val);
  bool ReadReal(const char* exp, float* val);
  bool ReadReal(const char* exp, double* val);
  bool ReadPosReal(const char* exp, float* val);
  bool ReadPosReal(const char* exp, double* val);
  bool ReadInt(const char* exp, int* val, int min, int max);
  bool ReadReal(const char* exp, float* val, float min, float max);
  bool ReadReal(const char* exp, double* val, double min, double max);
  bool IsNumber();

  std::vector<FileHeader> files_;  // list of config files to load
  lua_State* l_;                   // lua interpreter (valid between readFiles() and clear())
  WatchFiles watch_files_;         // class used to monitor files for changes
  bool modified_;                  // true if config files need to be re-read
  std::string path_;
};

bool FileExists(const char* filename);

}  // namespace config_reader

#endif  // CONFIG_READER_CONFIG_READER_H_
