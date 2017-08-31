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

// Visualeyez header
#include <visualeyez/visualeyez.h>

// Config and feedback messages
#include <ff_msgs/VisualeyezConfig.h>
#include <ff_msgs/VisualeyezFeedbackArray.h>

// Default argument values
std::string name_ = "";
std::string file_ = "calibration.bin";
uint32_t samples_ = 3;
ros::ServiceClient srv_;
ros::Subscriber sub_;

// Configure the calibration engine
bool Configure(uint8_t action, std::string const& name = "") {
  ff_msgs::VisualeyezConfig msg;
  msg.request.action = action;
  msg.request.name = name;
  msg.request.pub_tf = true;
  return srv_.call(msg);
}

// Called when new feedback data arrives
void FeedbackCallback(const ff_msgs::VisualeyezFeedbackArray::ConstPtr& msg) {
  std::vector<ff_msgs::VisualeyezFeedback>::const_iterator it;
  for (it = msg->feedback.begin(); it != msg->feedback.end(); it++)
    std::cout << std::setfill('0') << std::setw(4) << it->count << ' ';
  std::cout << '\r' << std::flush;
  for (it = msg->feedback.begin(); it != msg->feedback.end(); it++)
    if (it->count < samples_) return;
  // Calibrate and save
  std::cout << "Calibrating..." << std::endl;
  Configure(ff_msgs::VisualeyezConfig::Request::CALIBRATE, name_);
  std::cout << "Saving..." << std::endl;
  Configure(ff_msgs::VisualeyezConfig::Request::SAVE, file_);
  std::cout << "Done!" << std::endl;
  // Clean up
  srv_ = ros::ServiceClient();
  sub_ = ros::Subscriber();
  // Shutdown
  ros::shutdown();
}

// Main entry point for application
int main(int argc, char *argv[]) {
  // Command line parsing
  char c;
  while ((c = getopt(argc, argv, "hn:s:w:")) != -1) {
    switch (c) {
      case 'n': {
        name_ = std::string(optarg);
        break;
      }
      case 's': {
        int nsamples = atoi(optarg);
        if (nsamples <= 0) {
          std::cout << "Number of samples must be greater than zero" << std::endl;
          return -2;
        }
        samples_ = static_cast<uint32_t>(nsamples);
        break;
      }
      case 'w': {
        file_ = std::string(optarg);
        break;
      }
      case 'h':
      default: {
        std::cout << std::endl;
        std::cout << "Usage: visualeyez_tool [OPTIONS]..." << std::endl;
        std::cout << "Calibrate the visualeyez system" << std::endl;
        std::cout << "  -h         Show usage and help" << std::endl;
        std::cout << "  -n name    Freeflyer name ('" << name_ <<"')" << std::endl;
        std::cout << "  -s number  Minimum number of samples per marker (" << samples_ << ")" << std::endl;
        std::cout << "  -w file    Write to calibration file (" << file_ << ")" << std::endl;
        std::cout << std::endl;
        return -1;
      }
    }
  }
  // Initialize ros and get a callback
  ros::init(argc, argv, "visualeyez_tool");
  ros::NodeHandle nh;
  // Create a service client
  srv_ = nh.serviceClient < ff_msgs::VisualeyezConfig > (
    SERVICE_VISUALEYEZ_CALIBRATE_CONFIG);
  // Subscribe to the visualeyez topic
  sub_ = nh.subscribe(TOPIC_VISUALEYEZ_CALIBRATE_FEEDBACK, 1, FeedbackCallback);
  // Start the calibration
  std::cout << "Recording:" << std::endl;
  Configure(ff_msgs::VisualeyezConfig::Request::RECORD, name_);
  // Keep blocking until
  ros::spin();
  // Success
  return 0;
}
