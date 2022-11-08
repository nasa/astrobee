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

#include <gazebo/common/common.hh>
#include <gazebo/rendering/rendering.hh>
#include <gazebo/gazebo.hh>

#include <functional>

namespace gazebo {

class SystemPluginClient : public SystemPlugin {
 public:
  ~SystemPluginClient() {
  #if GAZEBO_MAJOR_VERSION > 7
    update_.reset();
  #else
    event::Events::DisconnectPreRender(update_);
  #endif
  }

  // Called before initializing
  void Load(int /*_argc*/, char ** /*_argv*/) {}

  // Called when client is finished initializing
  void Init() {
    update_ = event::Events::ConnectPreRender(std::bind(
      &SystemPluginClient::CreateCallback, this));
  }

  // Called when a new entity is created
  void CreateCallback() {
    // Update the lights to disable frustums
    rendering::ScenePtr scene = rendering::get_scene();
    if (!scene || !scene->Initialized())
      return;
    for (uint32_t l = 0; l < scene->LightCount(); l++) {
    #if GAZEBO_MAJOR_VERSION > 9
      rendering::LightPtr light = scene->LightByIndex(l);
    #else
      rendering::LightPtr light = scene->GetLight(l);
    #endif
      if (!light)
        continue;
      rendering::VisualPtr visual = scene->GetVisual(light->Name());
      if (!visual)
        continue;
      Ogre::SceneNode *node = visual->GetSceneNode();
      for (auto i = 0; i < node->numAttachedObjects(); i++) {
        Ogre::MovableObject *obj = node->getAttachedObject(i);
        if (obj->getMovableType() != "Light")
          obj->setVisible(false);
      }
    }
  }

 private:
  event::ConnectionPtr update_;
};

// Register this plugin with the simulator
GZ_REGISTER_SYSTEM_PLUGIN(SystemPluginClient)

}   // namespace gazebo
