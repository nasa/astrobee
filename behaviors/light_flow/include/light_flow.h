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

#ifndef LIGHT_FLOW_H_
#define LIGHT_FLOW_H_

#include <ff_hw_msgs/msg/configure_led.hpp>
#include <ff_hw_msgs/msg/configure_led_group.hpp>
namespace ff_hw_msgs {
typedef ff_hw_msgs::msg::ConfigureLED ConfigureLED;
typedef ff_hw_msgs::msg::ConfigureLEDGroup ConfigureLEDGroup;
}  // namespace ff_hw_msgs
#include <ff_common/ff_names.h>
#include <jsoncpp/json/value.h>
#include <rclcpp/rclcpp.hpp>
#include <stdio.h>
#include <string>
#include <vector>

#endif  // LIGHT_FLOW_H_

namespace light_flow {

double getUnixTimestamp();
int hexToInt(std::string hex);
Json::Value getMacroLookup(const Json::Value &proc);
Json::Value macroExpandRecursive(const Json::Value &macroLookup,
                                 Json::Value obj);
Json::Value macroExpand(const Json::Value &macroLookup, Json::Value obj);
double floatFromHex(const std::string &hex);
Json::Value compileColor(const Json::Value &rgbHexAsJson);
Json::Value compileBasicFrame(const Json::Value &obj);
Json::Value compileRepeatFrame(const Json::Value &obj);
Json::Value compileListFrame(const Json::Value &obj);
Json::Value compileFrame(const Json::Value &obj);
Json::Value compileBasicSequence(const Json::Value &obj);
Json::Value compileMoveSequence(const Json::Value &obj);
Json::Value compileRepeatSequence(const Json::Value &obj);
Json::Value compileListSequence(const Json::Value &obj);
Json::Value compileSequence(const Json::Value &obj);
Json::Value compileInterval(const Json::Value &obj);
Json::Value compileDomain(const Json::Value &obj);
Json::Value compileBasicAnimation(const Json::Value &obj);
Json::Value compileListAnimation(const Json::Value &obj);
Json::Value compileAnimation(const Json::Value &obj);
Json::Value compileDefine(const Json::Value &obj);
Json::Value compileReturn(const Json::Value &obj);
Json::Value compileProcedure(const Json::Value &obj);
Json::Value getProcedureAnimation(const Json::Value &procedure);
Json::Value getProcedureLookup(const Json::Value &procedures);
Json::Value compileCall(const Json::Value &obj,
                        const Json::Value &procedureLookup);
Json::Value compileCallList(const Json::Value &obj,
                            const Json::Value &procedureLookup);
Json::Value compileExecState(const Json::Value &obj,
                             const Json::Value &procedureLookup);
Json::Value getBasicSequenceFrameInfo(const Json::Value &seq, double t,
                                      const Json::Value &motionOffset);
Json::Value getSequenceFrameInfo(const Json::Value &seq, double t,
                                 double motionOffset);
Json::Value getMoveSequenceFrameInfo(const Json::Value &seq, double t,
                                     double motionOffset);
Json::Value getRepeatSequenceFrameInfo(const Json::Value &seq, double t,
                                       double motionOffset);
Json::Value getListSequenceFrameInfo(const Json::Value &seq, double t,
                                     double motionOffset);
Json::Value getSequenceFrameInfo(const Json::Value &seq, double t,
                                 double motionOffset);
void vectorAdd(Json::Value &u, const Json::Value &v);
Json::Value vectorMix(const Json::Value &u, double ku, const Json::Value &v,
                      double kv);
Json::Value getPixelColorBasicFrame(const Json::Value &frame, double pos);
Json::Value getPixelColorRepeatFrame(const Json::Value &frame, double pos);
Json::Value getPixelColorListFrame(const Json::Value &frame, double pos);
Json::Value getPixelColorFrame(const Json::Value &frame, double pos);

class Light {
 public:
  float red, green, blue;
  bool set;
  Light(float r, float g, float b);
  Light();
  explicit Light(const std::vector<double> &d);
};

class Frame {
 public:
  std::vector<Light> lights;
  bool isLeft;
  explicit Frame(int numberOfLights, bool isLeft);
  void setLight(int index, float red, float green, float blue);
  void setLight(int index, const std::vector<double> &d);
  int translateIndex(int index);
  void printLights();
};

void vectorElementProduct(std::vector<double> &u, const std::vector<double> &v);
std::vector<double> jsonArrayToDoubleVector(const Json::Value &array);
std::vector<double> getPixelColorFrameInfo(const Json::Value &frameInfo,
                                           double pos,
                                           const std::vector<double> &filter);
void applyBasicAnimation(Frame &frame, const Json::Value &animation, double t,
                         const std::vector<double> &filter);
void applyListAnimation(Frame &frame, Json::Value animation, double t,
                        const std::vector<double> &filter);
void applyAnimation(Frame &frame, Json::Value animation, double t,
                    const std::vector<double> &filter);
void applyCall(Frame &frame, double t, const Json::Value &call);

class Modules {
 public:
  Frame left;
  Frame right;
  explicit Modules(int numberOfLights);
};

void renderFrame(const Json::Value &statesOfExecution, Modules &modules,
                 double timeElapsed);
void compileLightFlow(Json::Value &input);
void setAllBlack(std::vector<ff_hw_msgs::ConfigureLED> &ledConfigs);
ff_hw_msgs::ConfigureLED createLED(uint red, uint green, uint blue,
                                   uint position);
void publishLightFlow(const Json::Value &statesOfExecution,
                      const rclcpp::Publisher<ff_hw_msgs::ConfigureLEDGroup>::SharedPtr &publishLEDGroup,
                      bool isStreaming);

}  // namespace light_flow
