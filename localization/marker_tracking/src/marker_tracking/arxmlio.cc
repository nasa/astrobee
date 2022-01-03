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

#include <marker_tracking/arxmlio.h>

#include <glog/logging.h>
#include <tinyxml.h>

namespace marker_tracking {

  static const int BL = 0;
  static const int BR = 1;
  static const int TR = 2;
  static const int TL = 3;

  void AddSubTag(int id,
                 Eigen::Vector3f const& origin,
                 Eigen::Vector3f const& ux,
                 Eigen::Vector3f const& uy,
                 int ix, int iy, int size,
                 ARTagMap* ar_corner_world_location) {
    ar_corner_world_location->insert(std::make_pair(id, Eigen::Matrix<float, 4, 3>()));
    Eigen::Matrix<float, 4, 3>& corners = ar_corner_world_location->operator[](id);
    corners.row(TL) = origin + ux * ix + uy * iy;
    corners.row(TR) = corners.row(TL).transpose() + size * ux;
    corners.row(BL) = corners.row(TL).transpose() + size * uy;
    corners.row(BR) = corners.row(BL).transpose() + size * ux;
  }

  void ParseMultiScaleTag(int id,
                          Eigen::Matrix<float, 4, 3> const& corners,
                          ARTagMap* ar_corner_world_location) {
    // The corners of all the smaller AR tags can be found on a 9 by 11 grid
    Eigen::Vector3f ux = (corners.row(TR) - corners.row(TL)) / 9.0;
    Eigen::Vector3f uy = (corners.row(BL) - corners.row(TL)) / 11.0;

    AddSubTag(id, corners.row(TL), ux, uy, 0, 0, 3, ar_corner_world_location);
    AddSubTag(id + 1, corners.row(TL), ux, uy, 4, 2, 1, ar_corner_world_location);
    AddSubTag(id + 2, corners.row(TL), ux, uy, 6, 0, 3, ar_corner_world_location);
    AddSubTag(id + 3, corners.row(TL), ux, uy, 2, 4, 1, ar_corner_world_location);
    AddSubTag(id + 4, corners.row(TL), ux, uy, 4, 4, 1, ar_corner_world_location);
    AddSubTag(id + 5, corners.row(TL), ux, uy, 6, 4, 1, ar_corner_world_location);
    AddSubTag(id + 6, corners.row(TL), ux, uy, 2, 6, 1, ar_corner_world_location);
    AddSubTag(id + 7, corners.row(TL), ux, uy, 0, 8, 3, ar_corner_world_location);
    AddSubTag(id + 8, corners.row(TL), ux, uy, 4, 6, 5, ar_corner_world_location);
  }

  Eigen::Matrix<float, 3, 1> ParseToVector(std::string const& input) {
    std::stringstream ss(input);
    Eigen::Matrix<float, 3, 1> output;
    for (size_t i = 0; i < 3; i++) {
      ss >> output[i];
    }
    return output;
  }

  void ParseArTag(TiXmlNode* node,
                  ARTagMap* ar_corner_world_location) {
    int id;

    node->ToElement()->QueryIntAttribute("id", &id);
    std::string
      top_left(node->ToElement()->Attribute("topleft")),
      top_right(node->ToElement()->Attribute("topright")),
      bottom_left(node->ToElement()->Attribute("bottomleft"));

    // Parse out the vectors and the forth element
    Eigen::Matrix<float, 4, 3> corners;  // BL, BR, TR, TL
    corners.setZero();
    corners.row(TL) = ParseToVector(top_left);
    corners.row(TR) = ParseToVector(top_right);
    corners.row(BL) = ParseToVector(bottom_left);

    // Check that we have something that is close to 90 degrees
    Eigen::Vector3f ua = corners.row(TR) - corners.row(TL),
      ub = corners.row(BL) - corners.row(TL);
    float angle = acos(ua.dot(ub) / (ua.norm() * ub.norm()));
    static const float SMALL_ANGLE = 1.0 * M_PI / 180.0;
    if (fabs(fabs(angle) - M_PI/2) > SMALL_ANGLE) {
      LOG(INFO) << "Corner measurements:\n" << corners;
      LOG(INFO) << "top edge length:" << (corners.row(TR) - corners.row(TL)).norm();
      LOG(INFO) << "left edge length:" << (corners.row(TL) - corners.row(BL)).norm();
      LOG(ERROR) << "AR " << id << "'s measurements are not square. What should be a right angle, measures "
                 << angle * 180 / M_PI << " deg";
    }

    // Make fourth measurement
    corners.row(BR) = corners.row(TR) + ub.transpose();

    if (node->ValueStr() == "multiscale") {
      // If this was a mutliscale thingy, do more work.
      ParseMultiScaleTag(id, corners,
                         ar_corner_world_location);
    } else {
      // Otherwise we have everything we need
      ar_corner_world_location->insert(std::make_pair(id, corners));
    }
  }

  void LoadARTagLocation(std::string const& filename,
                         ARTagMap* ar_corner_world_location) {
    TiXmlDocument doc(filename);
    if (!doc.LoadFile()) {
      LOG(FATAL) << "Unable to load AR tags filename: " << filename.c_str();
    }

    TiXmlHandle hdoc(&doc);
    TiXmlElement* pelem;
    TiXmlHandle hroot(0);

    // Identify our root
    {
      pelem = hdoc.FirstChildElement().Element();
      if (!pelem)
        LOG(FATAL) << "XML file looks empty";
      if (pelem->ValueStr() != "tags")
        LOG(FATAL) << "XML file doesn't start with tags, instead '" << pelem->Value() << "'";
      hroot = TiXmlHandle(pelem);
    }

    // Iterate throught the following tags and see if they are AR or
    // multiscale something.
    {
      for (TiXmlNode* child = hroot.ToElement()->FirstChild();
           child != NULL; child = child->NextSiblingElement()) {
        if (child->ValueStr() == "ar" || child->ValueStr() == "multiscale")
          ParseArTag(child, ar_corner_world_location);
      }
    }
  }

}  // end namespace marker_tracking
