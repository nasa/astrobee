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

#include <jsonloader/validation.h>

#include <glog/logging.h>
#include <json/json.h>

jsonloader::Field::Field(std::string const &name,
  Json::ValueType type, bool required)
  : name_(name), type_(type), required_(required)
{ }

std::string const& jsonloader::Field::name() const noexcept {
  return name_;
}

namespace {

const char* TypeToString(const Json::ValueType type) {
  switch (type) {
  case Json::nullValue: return u8"null";
  case Json::intValue: return u8"int";
  case Json::uintValue: return u8"unsigned int";
  case Json::realValue: return u8"float";
  case Json::stringValue: return u8"string";
  case Json::booleanValue: return u8"boolean";
  case Json::arrayValue: return u8"array";
  case Json::objectValue: return u8"object";
  default:
    LOG(ERROR) << "not all ValueTypes accounted for!";
    return u8"unknown";
  }
}

}  // end namespace

bool jsonloader::Field::Validate(Json::Value const& obj) const {
  if (!obj.isObject()) {
    LOG(ERROR) << "invalid json: not an object.";
    return false;
  }

  if (!obj.isMember(name_)) {
    if (required_) {
      LOG(ERROR) << "invalid json: '" << name_ << "' missing.";
      return false;
    }

    return true;
  }

  if (obj[name_].type() != type_) {
    // special case for needing floats but getting parsed as an int
    if ((obj[name_].type() == Json::uintValue ||
        obj[name_].type() == Json::intValue) &&
        type_ == Json::realValue) {
      return true;
    }

    LOG(ERROR) << "invalid json: '" << name_ << "': wrong type: "
               << "(expected " << TypeToString(type_)
               << " got " << TypeToString(obj[name_].type()) << ")";

    return false;
  }

  return true;
}

bool jsonloader::Validate(Json::Value const& obj, jsonloader::Fields const& fields) {
  for (const jsonloader::Field* f : fields) {
    if (!f->Validate(obj)) {
      LOG(ERROR) << "validation failed.";
      return false;
    }
  }

  return true;
}

namespace {

// Compare two strings, ignoring case
bool streq(std::string const& left, std::string const& right) {
  if (left.size() != right.size()) {
    return false;
  }

  auto length = left.size();
  for (std::string::size_type i = 0; i < length; i++) {
    if (std::tolower(left[i]) != std::tolower(right[i]))
      return false;
  }

  return true;
}

}  // end namespace

jsonloader::StringField::StringField(std::string const& name,
    std::string const& value, bool case_sensitive, bool required)
  : Field(name, Json::stringValue, required),
    value_(value), case_sensitive_(case_sensitive)
{ }

bool jsonloader::StringField::Validate(Json::Value const& obj) const {
  jsonloader::Field::Validate(obj);

  if (!case_sensitive_) {
    if (!streq(value_, obj[name()].asString())) {
      LOG(ERROR) << "invalid json: '" << name() << "': invalid value.";
      return false;
    }
    return true;
  }

  if (value_ != obj[name()].asString()) {
    LOG(ERROR) << "invalid json: '" << name() << "': invalid value.";
    return false;
  }

  return true;
}

jsonloader::ObjectField::ObjectField(std::string const& name,
    jsonloader::Fields const& fields, bool required)
  : Field(name, Json::objectValue, required), fields_(fields)
{ }

bool jsonloader::ObjectField::Validate(Json::Value const& obj) const {
  if (!Field::Validate(obj))
    return false;

  Json::Value const& child = obj[name()];
  for (const Field* f : fields_) {
    if (!f->Validate(child)) {
      return false;
    }
  }

  return true;
}

jsonloader::EnumField::EnumField(std::string const& name,
    jsonloader::EnumField::Values const& values, bool required)
  : Field(name, Json::stringValue, required), values_(values)
{ }

bool jsonloader::EnumField::Validate(Json::Value const& obj) const {
  if (!Field::Validate(obj))
    return false;

  std::string const& actual = obj[name()].asString();
  for (std::string const& v : values_) {
    if (streq(v, actual)) {
      return true;
    }
  }

  LOG(ERROR) << "invalid json: '" << name() << "': invalid value. ";

  return false;
}

