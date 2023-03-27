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

#include "../../include/light_flow.h"
#include <assert.h>
#include <ff_common/ff_names.h>
#include <jsoncpp/json/allocator.h>
#include <jsoncpp/json/json.h>
#include <jsoncpp/json/value.h>
#include <rclcpp/rclcpp.hpp>
#include <stdio.h>
#include <chrono>
#include <cmath>
#include <ctime>
#include <iostream>
#include <thread>
#include <vector>

namespace light_flow {

#define NUM_LIGHTS 74
#define REAL_NUM_LIGHTS 44

#define TOP_STREAMING 21
#define BOTTOM_STREAMING 22

double getUnixTimestamp() { return static_cast<double>(std::time(nullptr)); }

int hexToInt(std::string hex) { return std::stoul("0x" + hex, nullptr, 16); }

Json::Value getMacroLookup(const Json::Value &proc) {
  Json::Value result(Json::objectValue);
  for (const Json::Value &i : proc["defines"])
    result[i["id"].asString()] = i["value"];
  return result;
}

Json::Value macroExpandRecursive(const Json::Value &macroLookup,
                                 Json::Value obj) {
  if (obj.isArray()) {
    Json::Value result(Json::arrayValue);
    bool anyChanged = false;
    for (const Json::Value &i : obj) {
      Json::Value val = macroExpandRecursive(macroLookup, i);
      bool isChanged = val[0].asBool();
      Json::Value expandedObj = val[1];
      result.append(expandedObj);
      if (isChanged) anyChanged = true;
    }
    Json::Value returnedObject(Json::arrayValue);
    returnedObject.append(anyChanged);
    returnedObject.append(result);
    return returnedObject;
  } else if (obj.isString() && obj.asString()[0] == '$') {
    Json::Value returnedObject(Json::arrayValue);
    returnedObject.append(true);
    returnedObject.append(macroLookup[obj.asString()]);
    return returnedObject;
  } else {
    Json::Value returnedObject(Json::arrayValue);
    returnedObject.append(false);
    returnedObject.append(obj);
    return returnedObject;
  }
}

Json::Value macroExpand(const Json::Value &macroLookup, Json::Value obj) {
  while (true) {
    Json::Value val = macroExpandRecursive(macroLookup, obj);
    bool isChanged = val[0].asBool();
    obj = val[1];
    if (!isChanged) return obj;
  }
  assert(false);
  return Json::nullValue;
}

double floatFromHex(const std::string &hex) {
  if (hex.length() == 1) {
    return static_cast<double>(hexToInt(hex)) / 15.0;
  } else if (hex.length() == 2) {
    return static_cast<double>(hexToInt(hex)) / 256.0;
  } else {
    assert(false);
    return -1;
  }
}

Json::Value compileColor(const Json::Value &rgbHexAsJson) {
  if (rgbHexAsJson.isNull()) return Json::nullValue;
  std::string rgbHex = rgbHexAsJson.asString();
  assert(rgbHex[0] == '#');
  Json::Value returnedObject(Json::arrayValue);
  if (rgbHex.length() == 4) {
    returnedObject.append(floatFromHex(rgbHex.substr(1, 1)));
    returnedObject.append(floatFromHex(rgbHex.substr(2, 1)));
    returnedObject.append(floatFromHex(rgbHex.substr(3, 1)));
  } else if (rgbHex.length() == 7) {
    returnedObject.append(floatFromHex(rgbHex.substr(1, 2)));
    returnedObject.append(floatFromHex(rgbHex.substr(3, 2)));
    returnedObject.append(floatFromHex(rgbHex.substr(5, 2)));
  } else {
    assert(false);
  }
  return returnedObject;
}

Json::Value compileBasicFrame(const Json::Value &obj) {
  assert(obj[0].asString() == "BasicFrame");
  auto startColor = obj[1];
  auto startPos = obj[2];
  auto endColor = obj[4];
  startColor = compileColor(startColor);
  bool isInfinite = (startPos.isNull());
  if (!endColor.isNull()) endColor = compileColor(endColor);
  Json::Value returnedObject(Json::objectValue);
  returnedObject["type"] = "BasicFrame";
  returnedObject["startColor"] = startColor;
  returnedObject["startPos"] = startPos;
  returnedObject["length"] = obj[3];
  returnedObject["endColor"] = endColor;
  returnedObject["isInfinite"] = isInfinite;
  return returnedObject;
}

Json::Value compileFrame(const Json::Value &obj);

Json::Value compileRepeatFrame(const Json::Value &obj) {
  assert(obj[0].asString() == "RepeatFrame");
  auto n = obj[3];
  auto startPos = obj[4];
  bool isInfinite;
  if (n.isNull()) {
    isInfinite = true;
    n = 0;  // ignored
  } else {
    isInfinite = false;
    assert(n.isNumeric() && n.asDouble() > 0);
  }
  if (startPos.isNull()) {
    startPos = 0.0;
  }
  Json::Value returnedObject(Json::objectValue);
  returnedObject["type"] = "RepeatFrame";
  returnedObject["subFrame"] = compileFrame(obj[1]);
  returnedObject["spacing"] = obj[2];
  returnedObject["n"] = n;
  returnedObject["startPos"] = startPos;
  returnedObject["isInfinite"] = isInfinite;
  return returnedObject;
}

Json::Value compileListFrame(const Json::Value &obj) {
  assert(obj[0].asString() == "ListFrame");
  Json::Value subFrames(Json::arrayValue);
  for (u_int i = 1; i < obj.size(); i++) subFrames.append(compileFrame(obj[i]));
  Json::Value returnedObject(Json::objectValue);
  returnedObject["type"] = "ListFrame";
  returnedObject["subFrames"] = subFrames;
  return returnedObject;
}

Json::Value compileFrame(const Json::Value &obj) {
  if (obj.isString()) {
    Json::Value returnedArray(Json::arrayValue);
    returnedArray.append("BasicFrame");
    returnedArray.append(obj);
    return compileBasicFrame(returnedArray);
  }
  const std::string frameType = obj[0].asString();
  if (frameType == "BasicFrame") {
    return compileBasicFrame(obj);
  } else if (frameType == "RepeatFrame") {
    return compileRepeatFrame(obj);
  } else if (frameType == "ListFrame") {
    return compileListFrame(obj);
  }
  assert(false);
  return Json::nullValue;
}

Json::Value compileBasicSequence(const Json::Value &obj) {
  assert(obj[0].asString() == "BasicSequence");
  Json::Value startTime = obj[2];
  Json::Value duration = obj[3];
  Json::Value endFrame = obj[4];
  bool isInfinite;
  if (startTime.isNull()) {
    isInfinite = true;
    startTime = 0.0;  // ignored
    duration = 0.0;   // ignored
  } else {
    isInfinite = false;
  }
  if (!endFrame.isNull()) {
    endFrame = compileFrame(endFrame);
  }
  Json::Value returnedObject(Json::objectValue);
  returnedObject["type"] = "BasicSequence";
  returnedObject["startFrame"] = compileFrame(obj[1]);
  returnedObject["startTime"] = startTime;
  returnedObject["duration"] = duration;
  returnedObject["endFrame"] = endFrame;
  returnedObject["isInfinite"] = isInfinite;
  return returnedObject;
}

Json::Value compileSequence(const Json::Value &obj);

Json::Value compileMoveSequence(const Json::Value &obj) {
  Json::Value returnedObject(Json::objectValue);
  returnedObject["type"] = "MoveSequence";
  returnedObject["subSequence"] = compileSequence(obj[1]);
  returnedObject["rate"] = obj[2];
  return returnedObject;
}

Json::Value compileRepeatSequence(const Json::Value &obj) {
  Json::Value n = obj[3];
  Json::Value startTime = obj[4];
  bool isInfinite;
  if (n.isNull()) {
    isInfinite = true;
    n = 0;  // ignored
  } else {
    isInfinite = false;
  }
  if (startTime.isNull()) startTime = 0.0;
  Json::Value returnedObject(Json::objectValue);
  returnedObject["type"] = "RepeatSequence";
  returnedObject["subSequence"] = compileSequence(obj[1]);
  returnedObject["spacing"] = obj[2];
  returnedObject["n"] = n;
  returnedObject["startTime"] = startTime;
  returnedObject["isInfinite"] = isInfinite;
  return returnedObject;
}

Json::Value compileListSequence(const Json::Value &obj) {
  Json::Value subSequences(Json::arrayValue);
  for (u_int i = 1; i < obj.size(); i++)
    subSequences.append(compileSequence(obj[i]));
  Json::Value returnedObject(Json::objectValue);
  returnedObject["type"] = "ListSequence";
  returnedObject["subSequences"] = subSequences;
  return returnedObject;
}

Json::Value compileSequence(const Json::Value &obj) {
  // syntactic sugar -- interpret a Color as a kind of BasicSequence
  if (obj.isString()) {
    Json::Value parameter(Json::arrayValue);
    parameter.append("BasicSequence");
    parameter.append(obj);
    return compileBasicSequence(parameter);
  }
  const std::string sequenceType = obj[0].asString();
  if (sequenceType == "BasicSequence") {
    return compileBasicSequence(obj);
  } else if (sequenceType == "MoveSequence") {
    return compileMoveSequence(obj);
  } else if (sequenceType == "RepeatSequence") {
    return compileRepeatSequence(obj);
  } else if (sequenceType == "ListSequence") {
    return compileListSequence(obj);
  } else {
    Json::Value parameter(Json::arrayValue);
    parameter.append("BasicSequence");
    parameter.append(obj);
    return compileBasicSequence(parameter);
  }
}

Json::Value compileInterval(const Json::Value &obj) {
  assert(obj[0].asString() == "Interval");
  Json::Value returnedObject(Json::objectValue);
  returnedObject["type"] = "Interval";
  returnedObject["startIndex"] = obj[1];
  returnedObject["length"] = obj[2];
  return returnedObject;
}

Json::Value compileDomain(const Json::Value &obj) {
  assert(obj[0].asString() == "Domain");
  Json::Value intervals(Json::arrayValue);
  for (u_int i = 1; i < obj.size(); i++)
    intervals.append(compileInterval(obj[i]));
  Json::Value returnedObject(Json::objectValue);
  returnedObject["type"] = "Domain";
  returnedObject["intervals"] = intervals;
  return returnedObject;
}

Json::Value compileBasicAnimation(const Json::Value &obj) {
  assert(obj[0].asString() == "BasicAnimation");
  assert((obj.size() - 1) == 2);
  Json::Value returnedObject(Json::objectValue);
  returnedObject["type"] = "BasicAnimation";
  returnedObject["domain"] = compileDomain(obj[1]);
  returnedObject["sequence"] = compileSequence(obj[2]);
  return returnedObject;
}

Json::Value compileListAnimation(const Json::Value &obj) {
  Json::Value basicAnimations(Json::arrayValue);
  for (u_int i = 1; i < obj.size(); i++)
    basicAnimations.append(compileBasicAnimation(obj[i]));
  Json::Value returnedObject;
  returnedObject["type"] = "ListAnimation";
  returnedObject["basicAnimations"] = basicAnimations;
  return returnedObject;
}

Json::Value compileAnimation(const Json::Value &obj) {
  const std::string animationType = obj[0].asString();
  if (animationType == "BasicAnimation")
    return compileBasicAnimation(obj);
  else if (animationType == "ListAnimation")
    return compileListAnimation(obj);
  assert(false);
  return Json::nullValue;
}

Json::Value compileDefine(const Json::Value &obj) {
  Json::Value returnedObject(Json::objectValue);
  returnedObject["type"] = "Define";
  returnedObject["id"] = obj[1];
  returnedObject["value"] = obj[2];
  return returnedObject;
}

Json::Value compileReturn(const Json::Value &obj) {
  assert(obj[0].asString() == "Return");
  assert(obj.size() == 2);
  Json::Value returnedObject(Json::objectValue);
  returnedObject["type"] = "Return";
  returnedObject["value"] = obj[1];
  return returnedObject;
}

Json::Value compileProcedure(const Json::Value &obj) {
  assert(obj[0].asString() == "Procedure");
  assert((obj.size() - 1) >= 3);
  Json::Value defines(Json::arrayValue);
  for (u_int i = 2; i < (obj.size() - 1); i++)
    defines.append(compileDefine(obj[i]));
  assert(obj[1].isString());
  Json::Value returnedObject(Json::objectValue);
  returnedObject["type"] = "Procedure";
  returnedObject["name"] = obj[1];
  returnedObject["defines"] = defines;
  returnedObject["result"] = compileReturn(obj[obj.size() - 1]);
  return returnedObject;
}

Json::Value getProcedureAnimation(const Json::Value &procedure) {
  const Json::Value macroLookup = getMacroLookup(procedure);
  const Json::Value animation =
      compileAnimation(macroExpand(macroLookup, procedure["result"]["value"]));
  Json::Value returnedObject(Json::arrayValue);
  returnedObject.append(procedure["name"]);
  returnedObject.append(animation);
  return returnedObject;
}

Json::Value getProcedureLookup(const Json::Value &procedures) {
  Json::Value result(Json::objectValue);
  for (const Json::Value &i : procedures) {
    const Json::Value ret = getProcedureAnimation(compileProcedure(i));
    result[ret[0].asString()] = ret[1];
  }
  return result;
}

Json::Value compileCall(const Json::Value &obj,
                        const Json::Value &procedureLookup) {
  const std::string name = obj[2].asString();
  Json::Value returnedObject(Json::objectValue);
  returnedObject["type"] = "Call";
  returnedObject["module"] = obj[1];
  returnedObject["name"] = name;
  returnedObject["filter"] = compileColor(obj[3]);
  returnedObject["tempo"] = obj[4];
  returnedObject["animation"] = procedureLookup[name];
  return returnedObject;
}

Json::Value compileCallList(const Json::Value &obj,
                            const Json::Value &procedureLookup) {
  Json::Value result(Json::arrayValue);
  for (const Json::Value &i : obj)
    result.append(compileCall(i, procedureLookup));
  return result;
}

Json::Value compileExecState(const Json::Value &obj,
                             const Json::Value &procedureLookup) {
  assert(obj[0].asString() == "ExecState");
  Json::Value returnedObject(Json::objectValue);
  returnedObject["type"] = "ExecState";
  returnedObject["dimmer"] = obj[1];
  returnedObject["duration"] = obj[2];
  returnedObject["audioActive"] = obj[3];
  returnedObject["calls"] = compileCallList(obj[4], procedureLookup);
  return returnedObject;
}

Json::Value getBasicSequenceFrameInfo(const Json::Value &seq, double t,
                                      const Json::Value &motionOffset) {
  const double offset = t - seq["startTime"].asDouble();
  if (seq["isInfinite"].asBool() ||
      (0 <= offset && offset < seq["duration"].asDouble())) {
    const bool isConstant = seq["endFrame"].isNull();
    if (isConstant) {
      Json::Value returnedObject(Json::objectValue);
      returnedObject["type"] = "FrameInfo";
      returnedObject["isConstant"] = isConstant;
      returnedObject["motionOffset"] = motionOffset;
      returnedObject["startFrame"] = seq["startFrame"];
      return returnedObject;
    } else {
      const double endWeight = offset / seq["duration"].asDouble();
      const double startWeight = 1.0 - endWeight;
      Json::Value returnedObject(Json::objectValue);
      returnedObject["type"] = "FrameInfo";
      returnedObject["isConstant"] = isConstant;
      returnedObject["motionOffset"] = motionOffset;
      returnedObject["startFrame"] = seq["startFrame"];
      returnedObject["startWeight"] = startWeight;
      returnedObject["endFrame"] = seq["endFrame"];
      returnedObject["endWeight"] = endWeight;
      return returnedObject;
    }
  } else {
    return Json::nullValue;
  }
}

Json::Value getSequenceFrameInfo(const Json::Value &seq, double t,
                                 double motionOffset);

Json::Value getMoveSequenceFrameInfo(const Json::Value &seq, double t,
                                     double motionOffset) {
  return getSequenceFrameInfo(seq["subSequence"], t,
                              motionOffset + t * seq["rate"].asDouble());
}

Json::Value getRepeatSequenceFrameInfo(const Json::Value &seq, double t,
                                       double motionOffset) {
  const double offset = t - seq["startTime"].asDouble();
  if (seq["isInfinite"].asBool() ||
      ((0 <= offset) &&
       offset < (seq["n"].asDouble() * seq["spacing"].asDouble()))) {
    const double numPeriods = offset / seq["spacing"].asDouble();
    return getSequenceFrameInfo(
        seq["subSequence"],
        (seq["spacing"].asDouble() * (numPeriods - floor(numPeriods))),
        motionOffset);
  } else {
    return Json::nullValue;
  }
}

Json::Value getListSequenceFrameInfo(const Json::Value &seq, double t,
                                     double motionOffset) {
  for (const Json::Value &i : seq["subSequences"]) {
    const Json::Value f = getSequenceFrameInfo(i, t, motionOffset);
    if (!f.isNull()) return f;
  }
  return Json::nullValue;
}

Json::Value getSequenceFrameInfo(const Json::Value &seq, double t,
                                 double motionOffset) {
  const std::string sequenceType = seq["type"].asString();
  if (sequenceType == "BasicSequence")
    return getBasicSequenceFrameInfo(seq, t, motionOffset);
  else if (sequenceType == "MoveSequence")
    return getMoveSequenceFrameInfo(seq, t, motionOffset);
  else if (sequenceType == "RepeatSequence")
    return getRepeatSequenceFrameInfo(seq, t, motionOffset);
  else if (sequenceType == "ListSequence")
    return getListSequenceFrameInfo(seq, t, motionOffset);
  else
    assert(false);
  return Json::nullValue;
}

void vectorAdd(Json::Value &u, const Json::Value &v) {
  assert(u.size() == v.size());
  for (u_int i = 0; i < u.size(); i++) u[i] = u[i].asDouble() + v[i].asDouble();
}

Json::Value vectorMix(const Json::Value &u, double ku, const Json::Value &v,
                      double kv) {
  Json::Value result(Json::arrayValue);
  for (u_int i = 0; i < u.size(); i++)
    result.append(ku * u[i].asDouble() + kv * v[i].asDouble());
  return result;
}

Json::Value getPixelColorBasicFrame(const Json::Value &frame, double pos) {
  double offset = pos - frame["startPos"].asDouble();
  if (frame["isInfinite"].asBool() ||
      (0 <= offset && offset < frame["length"].asDouble())) {
    if (frame["endColor"].isNull()) {
      return frame["startColor"];
    } else {
      double endWeight = offset / frame["length"].asDouble();
      double startWeight = 1.0 - endWeight;
      return vectorMix(frame["startColor"], startWeight, frame["endColor"],
                       endWeight);
    }
  } else {
    Json::Value returnedObject(Json::arrayValue);
    for (int i = 0; i < 3; i++) returnedObject.append(0);
    return returnedObject;
  }
  assert(false);
  return Json::nullValue;
}

Json::Value getPixelColorFrame(const Json::Value &frame, double pos);

Json::Value getPixelColorRepeatFrame(const Json::Value &frame, double pos) {
  double offset = pos - frame["startPos"].asDouble();
  if (frame["isInfinite"].asBool() ||
      ((0 <= offset) &&
       offset < (frame["n"].asDouble() * frame["spacing"].asDouble()))) {
    double numPeriods = offset / frame["spacing"].asDouble();
    auto subPos =
        frame["spacing"].asDouble() * (numPeriods - floor(numPeriods));
    assert(frame["subFrame"].isObject());
    return getPixelColorFrame(frame["subFrame"], subPos);
  } else {
    return Json::nullValue;
  }
}

Json::Value getPixelColorListFrame(const Json::Value &frame, double pos) {
  Json::Value resultColor(Json::arrayValue);
  for (int j = 0; j < 3; j++) resultColor.append(0);
  assert(resultColor.size() == 3);
  for (u_int i = 0; i < frame["subFrames"].size(); i++) {
    auto subFrame = frame["subFrames"][i];
    assert(subFrame.isObject());
    auto subFrameColor = getPixelColorFrame(subFrame, pos);
    vectorAdd(resultColor, subFrameColor);
  }
  assert(resultColor.size() == 3);
  return resultColor;
}

Json::Value getPixelColorFrame(const Json::Value &frame, double pos) {
  const std::string frameType = frame["type"].asString();
  assert(frame.isObject());
  if (frameType == "BasicFrame")
    return getPixelColorBasicFrame(frame, pos);
  else if (frameType == "RepeatFrame")
    return getPixelColorRepeatFrame(frame, pos);
  else if (frameType == "ListFrame")
    return getPixelColorListFrame(frame, pos);
  assert(false);
  return Json::nullValue;
}

Light::Light(float r, float g, float b)
    : red(r), green(g), blue(b), set(true) {}
Light::Light() : set(false) {}
Light::Light(const std::vector<double> &d) {
  assert(d.size() == 3);
  red = d[0];
  green = d[1];
  blue = d[2];
  set = true;
}

Frame::Frame(int numberOfLights, bool isLeft)
    : lights(numberOfLights), isLeft(isLeft) {}

void Frame::setLight(int index, float red, float green, float blue) {
  lights[index] = Light(red, green, blue);
}

void Frame::setLight(int index, const std::vector<double> &d) {
  lights[index] = Light(d);
}

int Frame::translateIndex(int index) {
  if (index >= 0 && index <= 6) return 43 - index;

  if (index >= 7 && index <= 13) return index - 7;

  // only left
  if (index >= 59 && index <= 73) {
    if (!isLeft) return -1;
    return index - 59 + 7;
  }

  if (index >= 29 && index <= 43) {
    if (!isLeft) return -1;
    return index - 29 + 7 + 15;
  }
  // ----

  // only right
  if (index >= 44 && index <= 58) {
    if (isLeft) return -1;
    return index - 44 + 7;
  }

  if (index >= 14 && index <= 28) {
    if (isLeft) return -1;
    return index - 14 + 7 + 15;
  }
  // ----

  assert(false);
  return -1;
}

void Frame::printLights() {
  for (u_int i = 0; i < lights.size(); i++) {
    if (!lights[i].set) continue;
    int newIndex = translateIndex(i);
    if (newIndex < 0) continue;
    assert(newIndex >= 0 && newIndex <= 43);
    std::cout << "#" << newIndex << ": [" << lights[i].red << ", "
              << lights[i].green << ", " << lights[i].blue << "]" << std::endl;
  }
  std::cout << std::endl;
}

void vectorElementProduct(std::vector<double> &u,
                          const std::vector<double> &v) {
  assert(u.size() == v.size());
  for (u_int i = 0; i < u.size(); i++) u[i] = u[i] * v[i];
}

std::vector<double> jsonArrayToDoubleVector(const Json::Value &array) {
  assert(array.isArray());
  assert(array.size() >= 1);
  std::vector<double> resultantVector;
  resultantVector.reserve(array.size());
  for (const Json::Value &i : array) resultantVector.push_back(i.asDouble());
  return resultantVector;
}

std::vector<double> getPixelColorFrameInfo(const Json::Value &frameInfo,
                                           double pos,
                                           const std::vector<double> &filter) {
  std::vector<double> c0;
  if (frameInfo["isConstant"].asBool()) {
    assert(frameInfo["startFrame"].isObject());
    c0 = jsonArrayToDoubleVector(getPixelColorFrame(
        frameInfo["startFrame"], pos - frameInfo["motionOffset"].asDouble()));
  } else {
    assert(frameInfo["startFrame"].isObject());
    assert(frameInfo["endFrame"].isObject());
    auto startColor = getPixelColorFrame(
        frameInfo["startFrame"], pos - frameInfo["motionOffset"].asDouble());
    auto endColor = getPixelColorFrame(
        frameInfo["endFrame"], pos - frameInfo["motionOffset"].asDouble());
    c0 = jsonArrayToDoubleVector(
        vectorMix(startColor, frameInfo["startWeight"].asDouble(), endColor,
                  frameInfo["endWeight"].asDouble()));
  }
  vectorElementProduct(c0, filter);
  return c0;
}

void applyBasicAnimation(Frame &frame, const Json::Value &animation, double t,
                         const std::vector<double> &filter) {
  Json::Value frameInformation =
      getSequenceFrameInfo(animation["sequence"], t, 0.0);
  if (frameInformation.isNull()) {
    return;
  }

  if (!frameInformation["startFrame"].isNull())
    assert(frameInformation["startFrame"].isObject());

  if (!frameInformation["endFrame"].isNull())
    assert(frameInformation["endFrame"].isObject());

  for (u_int i = 0; i < animation["domain"]["intervals"].size(); i++) {
    auto interval = animation["domain"]["intervals"][i];
    u_int start = interval["startIndex"].asInt();
    u_int length = interval["length"].asInt();
    for (u_int j = start; j < (start + length); j++) {
      frame.setLight(j, getPixelColorFrameInfo(frameInformation, j, filter));
    }
  }
}

void applyListAnimation(Frame &frame, Json::Value animation, double t,
                        const std::vector<double> &filter) {
  for (const Json::Value &i : animation["basicAnimations"])
    applyBasicAnimation(frame, i, t, filter);
}

void applyAnimation(Frame &frame, Json::Value animation, double t,
                    const std::vector<double> &filter) {
  const std::string animationType = animation["type"].asString();
  if (animationType == "BasicAnimation")
    applyBasicAnimation(frame, animation, t, filter);
  else if (animationType == "ListAnimation")
    applyListAnimation(frame, animation, t, filter);
  else
    assert(false);
}

void applyCall(Frame &frame, double t, const Json::Value &call) {
  assert(frame.lights.size() == NUM_LIGHTS);
  applyAnimation(frame, call["animation"], t * call["tempo"].asDouble(),
                 jsonArrayToDoubleVector(call["filter"]));
}

Modules::Modules(int numberOfLights)
    : left(numberOfLights, true), right(numberOfLights, false) {}

void renderFrame(const Json::Value &statesOfExecution, Modules &modules,
                 double timeElapsed) {
  for (int module = 0; module < 2; module++) {
    const bool isLeft = module == 0;
    const bool isRight = !isLeft;

    for (u_int i = 0; i < statesOfExecution["calls"].size(); i++) {
      const Json::Value call = statesOfExecution["calls"][i];
      const std::string callModule = call["module"].asString();
      if (callModule == "both" || ((callModule == "left") && isLeft) ||
          ((callModule == "right") && isRight)) {
        if (isLeft)
          applyCall(modules.left, timeElapsed, call);
        else
          applyCall(modules.right, timeElapsed, call);
      }
    }
  }
}

void compileLightFlow(Json::Value &input) {
  input =
      compileExecState(input["state"], getProcedureLookup(input["procedure"]));
}

void setAllBlack(std::vector<ff_hw_msgs::ConfigureLED> &ledConfigs) {
  for (u_int i = 0; i < ledConfigs.size(); i++) {
    ledConfigs[i].red = 0;
    ledConfigs[i].green = 0;
    ledConfigs[i].blue = 0;
  }
}

ff_hw_msgs::ConfigureLED createLED(uint red, uint green, uint blue,
                                   uint position) {
  ff_hw_msgs::ConfigureLED led;
  led.red = red;
  led.green = green;
  led.blue = blue;
  led.pos = position;
  return led;
}

void publishLightFlow(const Json::Value &statesOfExecution,
                      const rclcpp::Publisher<ff_hw_msgs::ConfigureLEDGroup>::SharedPtr &publishLEDGroup,
                      bool isStreaming) {
  const double period = 33.33 / 1000.0;
  const auto delay_us = std::chrono::duration_cast<std::chrono::microseconds>(
      std::chrono::duration<double>(period));
  Modules frames(NUM_LIGHTS);
  const double initialTime = getUnixTimestamp();
  double newTime = initialTime + 0.0;

  while ((newTime - initialTime) < statesOfExecution["duration"].asDouble()) {
    renderFrame(statesOfExecution, frames, newTime - initialTime);

    ff_hw_msgs::ConfigureLEDGroup leftGroupConfiguration,
        rightGroupConfiguration;
    leftGroupConfiguration.side = "stbd";
    rightGroupConfiguration.side = "port";

    for (int i = 0; i < NUM_LIGHTS; i++) {
      const int newIndex = frames.left.translateIndex(i);
      if (isStreaming &&
          (newIndex == TOP_STREAMING || newIndex == BOTTOM_STREAMING)) {
        // if we are streaming or recording from any camera, light amber at a
        // single LED on this side
        leftGroupConfiguration.leds.push_back(createLED(10, 7, 0, newIndex));
        continue;
      }
      if (!(newIndex < 0 || !frames.left.lights[i].set)) {
        leftGroupConfiguration.leds.push_back(createLED(
            frames.left.lights[i].red * 5.0, frames.left.lights[i].green * 5.0,
            frames.left.lights[i].blue * 5.0, newIndex));
      } else {
        leftGroupConfiguration.leds.push_back(createLED(5, 5, 5, newIndex));
      }
    }
    publishLEDGroup->publish(leftGroupConfiguration);

    for (int i = 0; i < NUM_LIGHTS; i++) {
      const int newIndex = frames.right.translateIndex(i);
      if (isStreaming &&
          (newIndex == TOP_STREAMING || newIndex == BOTTOM_STREAMING)) {
        // if we are streaming or recording from any camera, light amber at a
        // single LED on this side
        // rightGroupConfiguration.leds.push_back(createLED(10, 7, 0,
        // newIndex)); if we are streaming or recording from any camera, light
        // purple at a single LED on this side
        rightGroupConfiguration.leds.push_back(createLED(15, 0, 15, newIndex));
        continue;
      }
      if (!(newIndex < 0 || !frames.right.lights[i].set)) {
        rightGroupConfiguration.leds.push_back(
            createLED(frames.right.lights[i].red * 5.0,
                      frames.right.lights[i].green * 5.0,
                      frames.right.lights[i].blue * 5.0, newIndex));
      } else {
        rightGroupConfiguration.leds.push_back(createLED(5, 5, 5, newIndex));
      }
    }
    publishLEDGroup->publish(rightGroupConfiguration);

    newTime += period;
    std::this_thread::sleep_for(
        std::chrono::duration_cast<std::chrono::microseconds>(delay_us));
  }
}

}  // namespace light_flow
