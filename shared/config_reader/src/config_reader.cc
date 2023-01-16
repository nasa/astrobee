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

#include "config_reader/config_reader.h"
#include <iostream>

/*
  [ References ]

  Lua for configuration files:
    Programming in  Lua
    Chapter 25. Extending your Application
    http://www.lua.org/pil/25.html
  Lua C API function documentation:
    Lua 5.0 Reference Manual
    Chapter 3 - The Application Program Interface
    http://www.lua.org/manual/5.0/manual.html#3
*/

namespace config_reader {

//====================================================================//
bool FileExists(const char *filename) {
  struct stat st;
  return(stat(filename, &st) == 0);
}

//====================================================================//
ConfigReader::Table::Table() :
  config_(NULL),
  ref_(-1),
  full_exp_(""),
  size_(-1) {
}

ConfigReader::Table::Table(ConfigReader *c, const char *exp) :
  config_(NULL),
  ref_(-1),
  full_exp_(exp),
  size_(-1) {
  if (!c->GetTable(exp, this)) {
    LOG(WARNING) << "ConfigReader: Table " << exp << " doesn't exist! Thus " <<
      "this object cannot be used.";
  }
}

ConfigReader::Table::Table(Table *t, const char *exp) :
  config_(NULL),
  ref_(-1),
  full_exp_(t->full_exp_ + '.' + exp),
  size_(-1) {
  if (!t->GetTable(exp, this)) {
    LOG(WARNING) << "ConfigReader: Table " << (t->full_exp_ + '.' + exp).c_str()
      << " doesn't exist! Thus this object cannot be used.";
  }
}

ConfigReader::Table::Table(Table *t, int index) :
  config_(NULL),
  ref_(-1),
  full_exp_(t->full_exp_ + "[" + std::to_string(index) + "]"),
  size_(-1) {
  if (!t->GetTable(index, this)) {
    LOG(WARNING) << "ConfigReader: Table " <<
      (t->full_exp_ + "[" + std::to_string(index) + "]").c_str() <<
      " doesn't exist! Thus this object cannot be used.";
  }
}

void ConfigReader::Table::Init(ConfigReader *c, int ref, const char *exp,
    int size) {
  config_ = c;
  ref_ = ref;
  full_exp_ = exp;
  size_ = size;
}

bool ConfigReader::Table::IsInit() {
  if (config_ == NULL) {
    LOG(WARNING) << "ConfigReader: Table " << full_exp_.c_str() <<
      " not initialize yet! Use GetTable to initialize!";
    return false;
  }
  return true;
}

bool ConfigReader::Table::CheckValExists(const char *exp) {
  if (!IsInit()) {
    return false;
  }

  if (!config_->PutObjectOnLuaStack(exp, ref_)) {
    return false;
  }

  return config_->IsTopValid();
}

// Only works for tables that are arrays or a mix of an array and map. If the
// table is not an array, the size will be 0
int ConfigReader::Table::GetSize() {
  return size_;
}

bool ConfigReader::Table::GetTable(const char *exp, Table *table) {
  if (!IsInit()) {
    return false;
  }

  if (!config_->PutObjectOnLuaStack(exp, ref_)) {
    return false;
  }

  return config_->ReadTable((full_exp_ + '.' + exp).c_str(), table);
}

bool ConfigReader::Table::GetTable(int index, Table *table) {
  if (!IsInit()) {
    return false;
  }

  if (!config_->PutObjectOnLuaStack(index, ref_)) {
    return false;
  }

  return config_->ReadTable(
      (full_exp_ + '[' + std::to_string(index) + ']').c_str(), table);
}

bool ConfigReader::Table::GetStr(const char *exp, std::string *str) {
  if (!IsInit()) {
    return false;
  }

  if (!config_->PutObjectOnLuaStack(exp, ref_)) {
    return false;
  }

  return config_->ReadStr((full_exp_ + '.' + exp).c_str(), str);
}

bool ConfigReader::Table::GetStr(int index, std::string *str) {
  if (!IsInit()) {
    return false;
  }

  if (!config_->PutObjectOnLuaStack(index, ref_)) {
    return false;
  }

  return config_->ReadStr(
      (full_exp_ + '[' + std::to_string(index) + ']').c_str(), str);
}

bool ConfigReader::Table::GetBool(const char *exp, bool *val) {
  if (!IsInit()) {
    return false;
  }

  if (!config_->PutObjectOnLuaStack(exp, ref_)) {
    return false;
  }

  return config_->ReadBool((full_exp_ + '.' + exp).c_str(), val);
}

bool ConfigReader::Table::GetBool(int index, bool *val) {
  if (!IsInit()) {
    return false;
  }

  if (!config_->PutObjectOnLuaStack(index, ref_)) {
    return false;
  }

  return config_->ReadBool(
      (full_exp_ + '[' + std::to_string(index) + ']').c_str(), val);
}

bool ConfigReader::Table::GetInt(const char *exp, int *val) {
  if (!IsInit()) {
    return false;
  }

  if (!config_->PutObjectOnLuaStack(exp, ref_)) {
    return false;
  }

  return config_->ReadInt((full_exp_ + '.' + exp).c_str(), val);
}

bool ConfigReader::Table::GetInt(int index, int *val) {
  if (!IsInit()) {
    return false;
  }

  if (!config_->PutObjectOnLuaStack(index, ref_)) {
    return false;
  }

  return config_->ReadInt(
      (full_exp_ + '[' + std::to_string(index) + ']').c_str(), val);
}

bool ConfigReader::Table::GetLongLong(const char *exp, int64_t *val) {
  if (!IsInit()) {
    return false;
  }

  if (!config_->PutObjectOnLuaStack(exp, ref_)) {
    return false;
  }

  return config_->ReadLongLong((full_exp_ + '.' + exp).c_str(), val);
}

bool ConfigReader::Table::GetLongLong(int index, int64_t *val) {
  if (!IsInit()) {
    return false;
  }

  if (!config_->PutObjectOnLuaStack(index, ref_)) {
    return false;
  }

  return config_->ReadLongLong(
      (full_exp_ + '[' + std::to_string(index) + ']').c_str(), val);
}

bool ConfigReader::Table::GetUInt(const char* exp, unsigned int *val) {
  if (!IsInit()) {
    return false;
  }

  if (!config_->PutObjectOnLuaStack(exp, ref_)) {
    return false;
  }

  return config_->ReadUInt((full_exp_ + '.' + exp).c_str(), val);
}

bool ConfigReader::Table::GetUInt(int index, unsigned int *val) {
  if (!IsInit()) {
    return false;
  }

  if (!config_->PutObjectOnLuaStack(index, ref_)) {
    return false;
  }

  return config_->ReadUInt(
      (full_exp_ + '[' + std::to_string(index) + ']').c_str(), val);
}

bool ConfigReader::Table::GetReal(const char *exp, float *val) {
  if (!IsInit()) {
    return false;
  }

  if (!config_->PutObjectOnLuaStack(exp, ref_)) {
    return false;
  }

  return config_->ReadReal((full_exp_ + '.' + exp).c_str(), val);
}

bool ConfigReader::Table::GetReal(int index, float *val) {
  if (!IsInit()) {
    return false;
  }

  if (!config_->PutObjectOnLuaStack(index, ref_)) {
    return false;
  }

  return config_->ReadReal(
      (full_exp_ + '[' + std::to_string(index) + ']').c_str(), val);
}

bool ConfigReader::Table::GetReal(const char *exp, double *val) {
  if (!IsInit()) {
    return false;
  }

  if (!config_->PutObjectOnLuaStack(exp, ref_)) {
    return false;
  }

  return config_->ReadReal((full_exp_ + '.' + exp).c_str(), val);
}

bool ConfigReader::Table::GetReal(int index, double *val) {
  if (!IsInit()) {
    return false;
  }

  if (!config_->PutObjectOnLuaStack(index, ref_)) {
    return false;
  }

  return config_->ReadReal(
      (full_exp_ + '[' + std::to_string(index) + ']').c_str(), val);
}

bool ConfigReader::Table::GetPosReal(const char *exp, float *val) {
  if (!IsInit()) {
    return false;
  }

  if (!config_->PutObjectOnLuaStack(exp, ref_)) {
    return false;
  }

  return config_->ReadPosReal((full_exp_ + '.' + exp).c_str(), val);
}

bool ConfigReader::Table::GetPosReal(int index, float *val) {
  if (!IsInit()) {
    return false;
  }

  if (!config_->PutObjectOnLuaStack(index, ref_)) {
    return false;
  }

  return config_->ReadPosReal(
      (full_exp_ + '[' + std::to_string(index) + ']').c_str(), val);
}

bool ConfigReader::Table::GetPosReal(const char *exp, double *val) {
  if (!IsInit()) {
    return false;
  }

  if (!config_->PutObjectOnLuaStack(exp, ref_)) {
    return false;
  }

  return config_->ReadPosReal((full_exp_ + '.' + exp).c_str(), val);
}

bool ConfigReader::Table::GetPosReal(int index, double *val) {
  if (!IsInit()) {
    return false;
  }

  if (!config_->PutObjectOnLuaStack(index, ref_)) {
    return false;
  }

  return config_->ReadPosReal(
      (full_exp_ + '[' + std::to_string(index) + ']').c_str(), val);
}

bool ConfigReader::Table::GetInt(const char *exp, int *val, int min, int max) {
  if (!IsInit()) {
    return false;
  }

  if (!config_->PutObjectOnLuaStack(exp, ref_)) {
    return false;
  }

  return config_->ReadInt((full_exp_ + '.' + exp).c_str(), val, min, max);
}

bool ConfigReader::Table::GetInt(int index, int *val, int min, int max) {
  if (!IsInit()) {
    return false;
  }

  if (!config_->PutObjectOnLuaStack(index, ref_)) {
    return false;
  }

  return config_->ReadInt(
      (full_exp_ + '[' + std::to_string(index) + ']').c_str(), val, min, max);
}

bool ConfigReader::Table::GetReal(const char *exp, float *val, float min,
    float max) {
  if (!IsInit()) {
    return false;
  }

  if (!config_->PutObjectOnLuaStack(exp, ref_)) {
    return false;
  }

  return config_->ReadReal((full_exp_ + '.' + exp).c_str(), val, min, max);
}

bool ConfigReader::Table::GetReal(int index, float *val, float min,
    float max) {
  if (!IsInit()) {
    return false;
  }

  if (!config_->PutObjectOnLuaStack(index, ref_)) {
    return false;
  }

  return config_->ReadReal(
      (full_exp_ + '[' + std::to_string(index) + ']').c_str(), val, min, max);
}

bool ConfigReader::Table::GetReal(const char *exp, double *val, double min,
    double max) {
  if (!IsInit()) {
    return false;
  }

  if (!config_->PutObjectOnLuaStack(exp, ref_)) {
    return false;
  }

  return config_->ReadReal((full_exp_ + '.' + exp).c_str(), val, min, max);
}

bool ConfigReader::Table::GetReal(int index, double *val, double min,
    double max) {
  if (!IsInit()) {
    return false;
  }

  if (!config_->PutObjectOnLuaStack(index, ref_)) {
    return false;
  }

  return config_->ReadReal(
    (full_exp_ + '[' + std::to_string(index) + ']').c_str(), val, min, max);
}

bool ConfigReader::Table::IsNumber(const char *exp) {
  if (!IsInit()) {
    return false;
  }

  if (!config_->PutObjectOnLuaStack(exp, ref_)) {
    return false;
  }

  return config_->IsNumber();
}

//====================================================================//
ConfigReader::FileHeader::FileHeader(const FileHeader &fh) {
  filename_ = fh.filename_;
  flags_ = fh.flags_;
  watch_ = fh.watch_;
}

//====================================================================//

ConfigReader::ConfigReader() {
  path_ = "";
  l_ = NULL;
  modified_ = false;
  const char* p = ff_common::GetConfigDir();
  SetPath(p);
  AddStandard();
}

ConfigReader::ConfigReader(const char* path) {
  path_ = "";
  l_ = NULL;
  modified_ = false;
  SetPath(path);
  AddStandard();
}

ConfigReader::~ConfigReader() {
  Reset();
}

void ConfigReader::Close() {
  CloseLua();
}

void ConfigReader::Reset() {
  CloseLua();
  ClearWatches();
  files_.clear();
  watch_files_.reset();
}

bool ConfigReader::IsOpen() {
  return (l_ != NULL);
}

void ConfigReader::SetPath(const char* path) {
  setenv("LUA_PATH", (std::string(path) + "/?.config;"
    + std::string(path) + "/?.lua").c_str(), 1);

  path_ = path;
  path_ += "/";
}

void ConfigReader::AddFile(const char *filename, unsigned flags) {
  FileHeader fh;
  if (filename[0] == '/')
    fh.filename_ = filename;
  else
    fh.filename_ = path_ + filename;
  fh.flags_ = flags;
  fh.watch_.watch(&watch_files_, fh.filename_.c_str());
  files_.push_back(fh);

  modified_ = true;
}

bool ConfigReader::ReadFiles() {
  if (!l_ && !InitLua()) {
    return false;
  }

  modified_ = false;

  bool ok = true;

  for (unsigned i = 0; i < files_.size(); i++) {
    FileHeader &fh = files_[i];
    if (fh.watch_.isFileModified()) {
      fh.watch_.rewatch(fh.filename_.c_str());
    }

    if (!ReadFile(fh.filename_.c_str(), fh.flags_)) {
      ok = false;
    }
  }

  if (ok) {
    AddImports();
  }

  return ok;
}

bool ConfigReader::IsFileModified() {
  unsigned i = 0;
  while (!modified_ && i < files_.size()) {
    modified_ = files_[i].watch_.isFileModified();
    i++;
  }

  return modified_;
}

void ConfigReader::CheckFilesUpdated(std::function<void(void)> callback) {
  if (watch_files_.getEvents() != 0) {
    // If the config file has been modified, reload parameters
    if (IsFileModified()) {
      callback();
    }

    watch_files_.clearEvents();
  }
}

bool ConfigReader::CheckValExists(const char *exp) {
  if (!PutObjectOnLuaStack(exp)) {
    return false;
  }

  return IsTopValid();
}

bool ConfigReader::GetTable(const char *exp, Table *table) {
  if (!PutObjectOnLuaStack(exp)) {
    return false;
  }

  return ReadTable(exp, table);
}

bool ConfigReader::GetStr(const char *exp, std::string *str) {
  if (!PutObjectOnLuaStack(exp)) {
    return false;
  }

  return ReadStr(exp, str);
}

bool ConfigReader::GetBool(const char *exp, bool *val) {
  if (!PutObjectOnLuaStack(exp)) {
    return false;
  }

  return ReadBool(exp, val);
}

bool ConfigReader::GetInt(const char *exp, int *val) {
  if (!PutObjectOnLuaStack(exp)) {
    return false;
  }

  return ReadInt(exp, val);
}

bool ConfigReader::GetLongLong(const char *exp, int64_t *val) {
  if (!PutObjectOnLuaStack(exp)) {
    return false;
  }

  return ReadLongLong(exp, val);
}

bool ConfigReader::GetUInt(const char *exp, unsigned int *val) {
  if (!PutObjectOnLuaStack(exp)) {
    return false;
  }

  return ReadUInt(exp, val);
}

bool ConfigReader::GetReal(const char *exp, float *val) {
  if (!PutObjectOnLuaStack(exp)) {
    return false;
  }

  return ReadReal(exp, val);
}

bool ConfigReader::GetReal(const char *exp, double *val) {
  if (!PutObjectOnLuaStack(exp)) {
    return false;
  }

  return ReadReal(exp, val);
}

bool ConfigReader::GetPosReal(const char *exp, float *val) {
  if (!PutObjectOnLuaStack(exp)) {
    return false;
  }

  return ReadPosReal(exp, val);
}

bool ConfigReader::GetPosReal(const char *exp, double *val) {
  if (!PutObjectOnLuaStack(exp)) {
    return false;
  }

  return ReadPosReal(exp, val);
}

bool ConfigReader::GetInt(const char *exp, int *val, int min, int max) {
  if (!PutObjectOnLuaStack(exp)) {
    return false;
  }

  return ReadInt(exp, val, min, max);
}

bool ConfigReader::GetReal(const char *exp, float *val, float min, float max) {
  if (!PutObjectOnLuaStack(exp)) {
    return false;
  }

  return ReadReal(exp, val, min, max);
}

bool ConfigReader::GetReal(const char *exp, double *val, double min, double max) {
  if (!PutObjectOnLuaStack(exp)) {
    return false;
  }

  return ReadReal(exp, val, min, max);
}

/**********************************Protected***********************************/
bool ConfigReader::InitLua() {
  if (l_) {
    CloseLua();
  }

  l_ = lua_open();
  if (!l_) {
    return false;
  }

  // load libraries
  luaL_openlibs(l_);

  // discard any results from initialization
  lua_settop(l_, 0);

  return true;
}

void ConfigReader::CloseLua() {
  if (l_) {
    lua_close(l_);
  }
  l_ = NULL;
}

void ConfigReader::ClearWatches() {
  for (unsigned i = 0; i < files_.size(); i++) {
    files_[i].watch_.remove();
  }
}

void ConfigReader::ShowError(int err_code, const char *filename) {
  if (err_code == 0) return;

  const char *err_str;
  switch (err_code) {
    case LUA_ERRFILE:   err_str="File not found"; break;
    case LUA_ERRSYNTAX: err_str="Syntax error"; break;
    case LUA_ERRMEM:    err_str="Memory allocation error"; break;
    case LUA_ERRRUN:    err_str="Runtime error"; break;
    case LUA_ERRERR:    err_str="Error running error handler"; break;
    default: err_str="Uknown error";
  }

  int t = lua_gettop(l_);
  if (lua_isstring(l_, t)) {
    const char *str = lua_tolstring(l_, t, NULL);
    LOG(ERROR) << "ConfigReader: " << str;
  }

  LOG(ERROR) << "ConfigReader: " << filename << ": " << err_str;

  Reset();
}

bool ConfigReader::ReadFile(const char *filename, unsigned flags) {
  // it's ok if optional files don't exist
  if ((flags & Optional) != 0 && !FileExists(filename)) {
    return true;
  }

  // try to load the file
  int ret = luaL_loadfile(l_, filename);
  if (ret) {
    ShowError(ret, filename);
  } else {
    // try to execute the file
    ret = lua_pcall(l_, 0, LUA_MULTRET, 0);
    if (ret) {
      ShowError(ret, filename);
    } else {
      lua_settop(l_, 0);  // discard any results
    }
  }

  return (ret == 0);
}

void ConfigReader::AddImports() {
  lua_getglobal(l_, "get_imports");

  if (!lua_isfunction(l_, -1)) {
    LOG(WARNING) << "ConfigReader: Get imports is not a function or lua " <<
      "couldn't find it!";
    lua_settop(l_, 0);
    return;
  }

  if (lua_pcall(l_, 0, 1, 0) != 0) {
    LOG(WARNING) << "ConfigReader: Error running get imports. Lua returned: "
      << lua_tostring(l_, -1);
    lua_settop(l_, 0);
    return;
  }

  if (!lua_istable(l_, -1)) {
    LOG(WARNING) << "ConfigReader: Error get imports didn't return a table!";
    lua_settop(l_, 0);
    return;
  }

  int i, num_files = files_.size();
  std::string temp_filename;
  bool found = false;

  lua_pushnil(l_);  // Needed to iterate through lua array
  while (lua_next(l_, -2)) {
    if (!lua_isstring(l_, -1)) {
      LOG(WARNING) << "Configreader: File name is not a string when adding "
        << "imports.";
      lua_settop(l_, 0);
      return;
    }

    found = false;
    temp_filename = lua_tostring(l_, -1);
    // Lua states that every required package appears only once in the package
    // loaded table. Thus only the files added in c need to be checked.
    for (i = 0; i < num_files && !found; ++i) {
      if (temp_filename.compare(files_[i].filename_) == 0) {
        found = true;
      }
    }

    if (!found) {
      FileHeader fh;
      fh.filename_ = temp_filename;
      fh.flags_ = 0;
      fh.watch_.watch(&watch_files_, fh.filename_.c_str());
      files_.push_back(fh);
    }

    lua_pop(l_, 1);
  }
  lua_settop(l_, 0);
}

void ConfigReader::AddStandard() {
  AddFile("common_lua.config");
}

// This function is used to get objects from the global namespace
bool ConfigReader::PutObjectOnLuaStack(const char *exp) {
  if (!IsOpen()) {
    LOG(WARNING) << "ConfigReader: Lua is not open, open lua before getting "
      << exp;
    return false;
  }

  lua_getglobal(l_, exp);

  return true;
}

// This function is used to get objects from table which has keys (much like a
// c++ map)
bool ConfigReader::PutObjectOnLuaStack(const char *exp, int ref) {
  if (!IsOpen()) {
    LOG(WARNING) << "ConfigReader: Lua is not open, open lua before getting "
      << " value";
    return false;
  }

  lua_rawgeti(l_, LUA_REGISTRYINDEX, ref);

  if (!lua_istable(l_, -1)) {
    LOG(ERROR) << "ConfigReader: Reference doesn't point to table! Make Katie "
      << "fix this!";
    lua_settop(l_, 0);
    return false;
  }

  lua_pushstring(l_, exp);
  lua_gettable(l_, -2);

  return true;
}

// This function is used to get objets from table which has numeric indices
// (much like a c++ array)
bool ConfigReader::PutObjectOnLuaStack(int index, int ref) {
  if (!IsOpen()) {
    LOG(WARNING) << "ConfigReader: Lua is not open, open lua before getting "
      << "value";
    return false;
  }

  lua_rawgeti(l_, LUA_REGISTRYINDEX, ref);

  if (!lua_istable(l_, -1)) {
    LOG(ERROR) << "ConfigReader: Reference doesn't point to table! Make Katie "
      << "fix this!";
    lua_settop(l_, 0);
    return false;
  }

  lua_rawgeti(l_, -1, index);
  return true;
}

bool ConfigReader::IsTopValid() {
  // Lua puts null on the stack if the variable doesn't exist
  if (lua_isnil(l_, -1)) {
    // Remove nil from stack
    lua_settop(l_, 0);
    return false;
  }

  // Only used to check if the value exists not actually reading it so remove
  // it from the stack
  lua_settop(l_, 0);
  return true;
}

void ConfigReader::OutputGetValueError(const char *exp, const char *type) {
  if (lua_isnil(l_, -1)) {
    LOG(WARNING) << "ConfigReader: \"" << exp << "\" doesn't exist!";
  } else {
    int luaType = lua_type(l_, -1);
    LOG(WARNING) << "ConfigReader: \"" << exp << "\" is a " <<
      lua_typename(l_, luaType) << " not a " << type << "!";
  }
}

bool ConfigReader::ReadTable(const char *exp, Table *table) {
  bool ok = lua_istable(l_, -1);
  if (ok) {
    int size = lua_objlen(l_, -1);
    int ref = luaL_ref(l_, LUA_REGISTRYINDEX);
    table->Init(this, ref, exp, size);
  } else {
    OutputGetValueError(exp, "table");
  }

  // Clear stack so it doesn't over fill
  lua_settop(l_, 0);
  return ok;
}

bool ConfigReader::ReadStr(const char *exp, std::string *str) {
  bool ok = lua_isstring(l_, -1);
  if (ok) {
    *str = lua_tostring(l_, -1);
  } else {
    OutputGetValueError(exp, "string");
  }

  lua_settop(l_, 0);
  return ok;
}

bool ConfigReader::ReadBool(const char *exp, bool *val) {
  bool ok = lua_isboolean(l_, -1);
  if (ok) {
    *val = static_cast<bool>(lua_toboolean(l_, -1));
  } else {
    OutputGetValueError(exp, "boolean");
  }

  lua_settop(l_, 0);
  return ok;
}

bool ConfigReader::ReadInt(const char *exp, int *val) {
  bool ok = lua_isnumber(l_, -1);
  if (ok) {
    *val = static_cast<int>(rint(lua_tonumber(l_, -1)));
  } else {
    OutputGetValueError(exp, "integer");
  }

  lua_settop(l_, 0);
  return ok;
}

bool ConfigReader::ReadLongLong(const char *exp, int64_t *val) {
  bool ok = lua_isnumber(l_, -1);
  if (ok) {
    *val = static_cast<int64_t>(llrint(lua_tonumber(l_, -1)));
  } else {
    OutputGetValueError(exp, "long long integer");
  }

  lua_settop(l_, 0);
  return ok;
}

bool ConfigReader::ReadUInt(const char *exp, unsigned int *val) {
  bool ok = lua_isnumber(l_, -1) && rint(lua_tonumber(l_, -1)) >= 0;
  if (ok) {
    *val = static_cast<unsigned int>(rint(lua_tonumber(l_, -1)));
  } else {
    OutputGetValueError(exp, "unsigned integer");
  }

  lua_settop(l_, 0);
  return ok;
}

bool ConfigReader::ReadReal(const char *exp, float *val) {
  bool ok = lua_isnumber(l_, -1);
  if (ok) {
    *val = static_cast<float>(lua_tonumber(l_, -1));
  } else {
    OutputGetValueError(exp, "number");
  }

  lua_settop(l_, 0);
  return ok;
}

bool ConfigReader::ReadReal(const char *exp, double *val) {
  bool ok = lua_isnumber(l_, -1);
  if (ok) {
    *val = static_cast<double>(lua_tonumber(l_, -1));
  } else {
    OutputGetValueError(exp, "number");
  }

  lua_settop(l_, 0);
  return ok;
}

bool ConfigReader::ReadPosReal(const char *exp, float *val) {
  if (!ReadReal(exp, val)) {
    return false;
  }

  if (*val <= 0.0) {
    LOG(WARNING) << "ConfigReader: " << exp << " = " << *val << " is negative.";
    *val = 1E-6;
    return false;
  }

  return true;
}

bool ConfigReader::ReadPosReal(const char *exp, double *val) {
  if (!ReadReal(exp, val)) {
    return false;
  }

  if (*val <= 0.0) {
    LOG(WARNING) << "ConfigReader: " << exp << " = " << *val << " is negative.";
    *val = 1E-6;
    return false;
  }

  return true;
}

bool ConfigReader::ReadInt(const char *exp, int *val, int min, int max) {
  if (!ReadInt(exp, val)) {
    return false;
  }

  if (*val < min || *val > max) {
      LOG(WARNING) << "ConfigReader: " << exp << " = " << *val << " is out of "
        << "range [" << min << ", " << max << "].";

    if (*val < min) {
      *val = min;
    }

    if (*val > max) {
      *val = max;
    }
    return false;
  }

  return true;
}

bool ConfigReader::ReadReal(const char *exp, float *val, float min, float max) {
  if (!ReadReal(exp, val)) {
    return false;
  }

  if (*val < min || *val > max) {
    LOG(WARNING) << "ConfigReader: " << exp << " = " << *val << " is out of "
      << "range [" << min << ", " << max << "].";

    if (*val < min) {
      *val = min;
    }
    if (*val > max) {
      *val = max;
    }
    return false;
  }

  return true;
}

bool ConfigReader::ReadReal(const char *exp, double *val, double min,
    double max) {
  if (!ReadReal(exp, val)) {
    return false;
  }

  if (*val < min || *val > max) {
    LOG(WARNING) << "ConfigReader: " << exp << " = " << *val << " is out of "
      << "range [" << min << ", " << max << "].";
    if (*val < min) {
      *val = min;
    }

    if (*val > max) {
      *val = max;
    }
    return false;
  }
  return true;
}

bool ConfigReader::IsNumber() {
  // The function PutObjectOnLuaStack() must have been called before
  // this, which would populate l_.
  return lua_isnumber(l_, -1);
}

}  // namespace config_reader
