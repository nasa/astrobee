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

#ifndef MOBILITY_PLANNER_QP_TRAJ_OPT_ROS_SRC_TRAJECTORY_VISUAL_H_
#define MOBILITY_PLANNER_QP_TRAJ_OPT_ROS_SRC_TRAJECTORY_VISUAL_H_

#include <traj_opt_basic/trajectory.h>
#include <traj_opt_basic/types.h>
#include <traj_opt_msgs/Trajectory.h>

#include <string>
#include <vector>

namespace Ogre {
class Vector3;
class Quaternion;
}

namespace rviz {
class Arrow;
class Line;
class Shape;
class Object;
}

namespace traj_opt {

// BEGIN_TUTORIAL
// Declare the visual class for this display.
//
// Each instance of TrajectoryVisual represents the visualization of a single
// traj_opt_msgs::Trajectory message.  Currently it just shows an arrow with
// the direction and magnitude of the acceleration vector, but could
// easily be expanded to include more of the message data.

enum Style { Mike, Sikang, CJ, SE3 };

class TrajectoryVisual {
 public:
  // Constructor.  Creates the visual stuff and puts it into the
  // scene, but in an unconfigured state.
  TrajectoryVisual(Ogre::SceneManager* scene_manager,
                   Ogre::SceneNode* parent_node);

  // Destructor.  Removes the visual stuff from the scene.
  virtual ~TrajectoryVisual();

  void draw();
  // Configure the visual to show the data in the message.
  void setMessage(const traj_opt_msgs::Trajectory::ConstPtr& msg);

  // Set the pose of the coordinate frame the message refers to.
  // These could be done inside setMessage(), but that would require
  // calls to FrameManager and error handling inside setMessage(),
  // which doesn't seem as clean.  This way TrajectoryVisual is only
  // responsible for visualization.
  void setFramePosition(const Ogre::Vector3& position);
  void setFrameOrientation(const Ogre::Quaternion& orientation);

  // Set the color and alpha of the visual, which are user-editable
  // parameters and therefore don't come from the Trajectory message.
  void setColor(float r, float g, float b, float a);
  void setColorV(float r, float g, float b, float a);
  void setColorA(float r, float g, float b, float a);
  void setScale(float thickness);
  void setCurve();
  void setStyle(int style);

  void resetTrajPoints(int traj_points, int tangent_points, bool use_v,
                       bool use_a);

 private:
  traj_opt::Mat3 matFromVecD(const traj_opt::VecD& vec);
  Ogre::Vector3 vecFromVecD(const traj_opt::VecD& vec);
  // Tangent velocity vectors
  std::vector<boost::shared_ptr<rviz::Object> > vel_arrows_;
  std::vector<boost::shared_ptr<rviz::Object> > acc_arrows_;
  // Lines making up the actual trajectory
  std::vector<boost::shared_ptr<rviz::Object> > trajectory_lines_;
  std::vector<boost::shared_ptr<rviz::Shape> > trajectory_balls_;

  double thickness_{0.1};
  int num_traj_points_{50};
  int num_vel_points_{50};
  bool vel_on_{true};
  bool acc_on_{false};

  //    Style style_{Style::Sikang}; // default to working style
  Style style_{Style::Mike};  // default to better style

  static void setShapeFromPosePair(const Ogre::Vector3& p0,
                                   const Ogre::Vector3& p1, double scale,
                                   rviz::Shape* shape);
  static void setShapeFromPosePair(const Ogre::Vector3& p0,
                                   const Ogre::Vector3& p1, double scale,
                                   rviz::Arrow* shape);

  boost::shared_ptr<traj_opt::Trajectory> traj_;

  // A SceneNode whose pose is set to match the coordinate frame of
  // the Trajectory message header.
  Ogre::SceneNode* frame_node_;

  // The SceneManager, kept here only so the destructor can ask it to
  // destroy the ``frame_node_``.
  Ogre::SceneManager* scene_manager_;
};
// END_TUTORIAL

}  // namespace traj_opt

#endif  // MOBILITY_PLANNER_QP_TRAJ_OPT_ROS_SRC_TRAJECTORY_VISUAL_H_
