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

// Test validation
// Tests the checks on the individual fields, aka maximum and minumum range,
// field type and structure

#include <jsonloader/validation.h>

#include <json/json.h>

#include <glog/logging.h>
#include <gtest/gtest.h>

#include <string>
#include <algorithm>

using jsonloader::Field;
using jsonloader::StringField;
using jsonloader::ValueField;
using jsonloader::RangeField;
using jsonloader::ObjectField;
using jsonloader::EnumField;

TEST(Validation, Field) {
  const std::string data = u8R"(
    { "string": "a string",
      "double":1.0,
      "null":null })";

  Json::Value v;
  Json::Reader().parse(data, v, false);

  Field exists("string", Json::stringValue, true);
  Field upda("404", Json::nullValue, true);
  Field wrong_type("null", Json::booleanValue, true);
  Field not_object("not_important", Json::booleanValue, true);

  Field optional("string", Json::stringValue, false);
  Field optional_404("404", Json::nullValue, false);

  EXPECT_TRUE(exists.Validate(v));
  EXPECT_FALSE(upda.Validate(v));
  EXPECT_FALSE(wrong_type.Validate(v));
  EXPECT_FALSE(not_object.Validate(Json::Value()));
  EXPECT_TRUE(optional.Validate(v));
  EXPECT_TRUE(optional_404.Validate(v));
}

TEST(Validation, FloatAsIntegerField) {
  const std::string data = u8R"({ "value": 10 })";
  Json::Value v;
  Json::Reader().parse(data, v, false);

  Field floatField("value", Json::realValue);
  ASSERT_TRUE(floatField.Validate(v));
  EXPECT_FLOAT_EQ(v["value"].asFloat(), 10.0f);
}

TEST(Validation, StringField) {
  const std::string data = u8R"({ "string": "stringValue" })";
  Json::Value v;
  Json::Reader().parse(data, v, false);

  StringField sensitive("string", "stringValue", true);
  StringField insensitive("string", "stringValue", false);
  StringField bad_sensitive("string", "StringValue", true);
  StringField good_insensitive("string", "StringValue", false);
  StringField bad_insensitive("string", "stringbalue", false);
  StringField bad_insensitive_length("string", "stringValue2", false);
  StringField optional("404", "non-existent", true, false);

  EXPECT_TRUE(sensitive.Validate(v));
  EXPECT_TRUE(insensitive.Validate(v));
  EXPECT_TRUE(good_insensitive.Validate(v));
  EXPECT_FALSE(bad_sensitive.Validate(v));
  EXPECT_FALSE(bad_insensitive.Validate(v));
  EXPECT_FALSE(bad_insensitive_length.Validate(v));
  EXPECT_TRUE(optional.Validate(v));
}

TEST(Validation, ValueField) {
  const std::string data = u8R"(
    { "bool": true,
      "int": 1,
      "uint": 3,
    }
  )";

  Json::Value v;
  Json::Reader().parse(data, v, false);

  ValueField<bool> check_bool("bool", true);
  ValueField<int> check_int("int", 1);
  ValueField<unsigned int> check_uint("uint", 3);
  ValueField<bool> check_optional_bool("404", false, false);

  EXPECT_TRUE(check_bool.Validate(v));
  EXPECT_TRUE(check_int.Validate(v));
  EXPECT_TRUE(check_uint.Validate(v));
  EXPECT_TRUE(check_optional_bool.Validate(v));

  v["bool"] = false;
  v["int"] = 2;
  v["uint"] = 4;

  EXPECT_FALSE(check_bool.Validate(v));
  EXPECT_FALSE(check_int.Validate(v));
  EXPECT_FALSE(check_uint.Validate(v));
}

TEST(Validation, RangeField) {
  // Manually build the object due to type stuff
  Json::Value v(Json::objectValue);
  v["float"] = 1.0f;
  v["double"] = static_cast<double>(2.0);
  v["int"] = 5;

  RangeField<float> goodFloat("float", 0.0f, 2.0f);
  RangeField<float> badFloat("float", 0.0f, 0.5f);
  RangeField<int> goodInt("int", 0, 10);
  RangeField<int> badInt("int", 10, 20);
  RangeField<int> optional("404", 10, 20, false);

  EXPECT_TRUE(goodFloat.Validate(v));
  EXPECT_TRUE(goodInt.Validate(v));
  EXPECT_FALSE(badFloat.Validate(v));
  EXPECT_FALSE(badInt.Validate(v));
  EXPECT_TRUE(optional.Validate(v));
}

TEST(Validation, ObjectField) {
  const std::string data = u8R"({ "parent": { "child": "value"} })";
  Json::Value v;
  Json::Reader().parse(data, v, false);

  ObjectField good("parent",
      { new StringField("child", "value") });
  EXPECT_TRUE(good.Validate(v));

  ObjectField bad("parent",
      { new Field("non_child", Json::nullValue) });
  EXPECT_FALSE(bad.Validate(v));

  ObjectField optional("parent",
      { new Field("non_child", Json::nullValue, false) });
  ObjectField optional_obj("404",
      { new Field("not_found", Json::nullValue) }, false);
  EXPECT_TRUE(optional.Validate(v));
  EXPECT_TRUE(optional_obj.Validate(v));
}

TEST(Validation, EnumField) {
  const std::string data = u8R"({ "item": "blue" })";
  Json::Value v;
  Json::Reader().parse(data, v, false);

  EnumField good("item", { "red", "green", "blue", "grey" });
  EnumField bad("item", { "banana", "apple", "blueberry" });
  EnumField optional("404", { "item", "not", "found" }, false);

  EXPECT_TRUE(good.Validate(v));
  EXPECT_FALSE(bad.Validate(v));
  EXPECT_TRUE(optional.Validate(v));
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
