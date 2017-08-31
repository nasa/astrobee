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

#ifndef JSONLOADER_VALIDATION_H_
#define JSONLOADER_VALIDATION_H_

#include <json/value.h>

#include <string>
#include <vector>

namespace jsonloader {

// Make sure a field in an object is the right type, optionally require
// the field to be present.
class Field {
 public:
  Field(std::string const& name, Json::ValueType type, bool required = true);

  virtual bool Validate(Json::Value const& obj) const;

 protected:
  std::string const& name() const noexcept;

 private:
  std::string name_;
  Json::ValueType type_;
  bool required_;
};

using Fields = std::vector<const Field*>;

bool Validate(Json::Value const& obj, Fields const& fields);

// Make sure a field in an object has the right value, optionally
// compare the strings case-sensitively.
class StringField : public Field {
 public:
  StringField(std::string const& name, std::string const& value,
              bool case_sensitive = false, bool required = true);

  bool Validate(Json::Value const& obj) const override;

 private:
  std::string value_;
  bool case_sensitive_;
};

// Compare to a sub-object in an array or object
class ObjectField : public Field {
 public:
  ObjectField(std::string const& name, Fields const& fields,
              bool required = true);

  bool Validate(Json::Value const& obj) const override;

 private:
  Fields fields_;
};

// Make sure a field conforms to a certain set of values
// This doees case-insensitive comparisons.
class EnumField : public Field {
 public:
  using Values = std::vector<std::string>;

  EnumField(std::string const& name, Values const &values,
            bool required = true);

  bool Validate(Json::Value const& obj) const override;

 private:
  Values values_;
};

// Make sure a field matches a specific value
template<typename T>
class ValueField : public Field {
  static_assert(!std::is_same<std::string, T>::value,
                "use StringField to compare strings");

  constexpr Json::ValueType GetType() {
    static_assert(!(std::is_integral<T>::value &&
                  std::is_floating_point<T>::value),
                  "invalid type for ValueField");

    if (std::is_same<bool, T>::value) {
      return Json::booleanValue;
    } else if (std::is_floating_point<T>::value) {
      // TODO(tfmorse): should probably warn the user this is a bad idea...
      return Json::realValue;
    } else if (std::is_integral<T>::value) {
      return Json::intValue;
    }
  }

 public:
  ValueField(std::string const& name, T const& value, bool required = true)
    : Field(name, GetType(), required), value_(value)
  { }

  bool Validate(Json::Value const& obj) const override {
    if (!Field::Validate(obj)) {
      return false;
    }

    Json::Value wanted(value_);
    Json::Value const& actual = obj[name()];

    if (wanted != actual) {
      // Special case: jsoncpp will not decode integers as unsigned. Thus,
      // if we're comparing against an unsigned int template type,
      // we have to coerce them and compare :/
      if (wanted.type() == Json::uintValue &&
          actual.type() == Json::intValue &&
          wanted.isConvertibleTo(Json::uintValue)) {
        return wanted.asUInt() == actual.asUInt();
      }

      return false;
    }

    return true;
  }

 private:
  const T value_;
};

using BoolField = ValueField<bool>;
using IntField = ValueField<int>;

// Make sure a field is within the given range of values.
template<typename T>
class RangeField : public Field {
  static_assert(std::is_arithmetic<T>::value, "Arithmetic type required");

  constexpr Json::ValueType GetType() {
    return (std::is_floating_point<T>::value) ?
            Json::realValue : Json::intValue;
  }

 public:
  RangeField(std::string const& name, T const& min, T const& max,
             bool required = true)
    : Field(name, GetType(), required), min_(min), max_(max)
  { };

  bool Validate(Json::Value const& obj) const override {
    if (!Field::Validate(obj)) {
      return false;
    }

    T val;
    if (std::is_floating_point<T>::value) {
      val = obj[name()].asFloat();
    } else {
      val = obj[name()].asInt();
    }

    if (val < min_ || val > max_) {
      return false;
    }

    return true;
  };

 private:
  T min_, max_;
};

using RangeFieldF = RangeField<float>;  // helper type for floats
using RangeFieldI = RangeField<int>;  // helper type for integers

}  // end namespace jsonloader

#endif  // JSONLOADER_VALIDATION_H_

