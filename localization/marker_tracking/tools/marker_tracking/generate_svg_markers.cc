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


#include <config_reader/config_reader.h>

#include <marker_tracking/marker_detector.h>

#include <cstdio>
#include <fstream>
#include <iostream>

const bool kDebugMarker = false;

class SvgGroup {
 private:
  std::ostream &out_;

 public:
  SvgGroup(std::ostream &out, std::string name) : out_(out) {
    out_ << "<g id=\"" << name << "\">" << std::endl;
  }
  ~SvgGroup() { out_ << "</g>" << std::endl; }
};

class DimU {
 public:
  static std::string unit_;
  static int precision_;
  std::string name_;
  float value_;
  DimU(const std::string &name, float value) : name_(name), value_(value) {
    // nothing else
  }
  friend std::ostream &operator<<(std::ostream &output, const DimU &dim) {
    output << dim.name_ << "=\"" << std::fixed << std::setprecision(precision_)
           << dim.value_ << DimU::unit_ << "\"";
    return output;
  }
};

class RectBW {
 public:
  float x_, y_;
  float w_, h_;
  std::string style_;
  RectBW(float x, float y, float width, float height, bool black)
      : x_(x), y_(y), w_(width), h_(height) {
    style_ = std::string("stroke:none; fill:") +
             std::string(black ? "#000000" : "#FFFFFF");
  }
  friend std::ostream &operator<<(std::ostream &output, const RectBW &rect) {
    output << "    <rect " << DimU("x", rect.x_) << " " << DimU("y", rect.y_);
    output << " " << DimU("width", rect.w_) << " " << DimU("height", rect.h_);
    output << " style=\"" << rect.style_ << "\"/>" << std::endl;
    return output;
  }
};

class LineBW {
 public:
  float x1_, y1_, x2_, y2_;
  float width_;
  std::string style_;
  LineBW(float x1, float y1, float x2, float y2, float width, bool black = true)
      : x1_(x1), y1_(y1), x2_(x2), y2_(y2), width_(width) {
    style_ =
        std::string("stroke:") + std::string(black ? "#000000" : "#FFFFFF");
  }
  friend std::ostream &operator<<(std::ostream &output, const LineBW &line) {
    output << "    <line ";
    output << DimU("x1", line.x1_) << " ";
    output << DimU("y1", line.y1_) << " ";
    output << DimU("x2", line.x2_) << " ";
    output << DimU("y2", line.y2_);
    output << " stroke-width=\"" << line.width_ << "px\"";
    output << " style=\"" << line.style_ << "\"/>" << std::endl;
    return output;
  }
};

class CenteredText {
 public:
  float x_, y_, sz_;
  std::string str_;
  std::string font_;
  CenteredText(const std::string &str, float x, float y, float sz,
               const std::string &font)
      : x_(x), y_(y), sz_(sz), str_(str), font_(font) {
    // do nothing for now
  }
  friend std::ostream &operator<<(std::ostream &out, const CenteredText &txt) {
    out << "  <text ";
    out << DimU("x", txt.x_) << " " << DimU("y", txt.y_) << std::endl;
    out << "    style=\"font-family: " << txt.font_ << ";" << std::endl;
    out << "           font-size  : " << txt.sz_ << ";" << std::endl;
    out << "           text-anchor: middle;" << std::endl;
    out << "          \"" << std::endl;
    out << "    >" << txt.str_ << "</text>" << std::endl;
    return out;
  }
};

// default unit is mm
std::string DimU::unit_("mm");
int DimU::precision_ = 2;

int MarkerToSvg(std::ofstream &out, const alvar::MarkerData &marker,
                float pos_x, float pos_y) {
  int margin = static_cast<int>(marker.GetMargin());
  int resolution = marker.GetRes();
  CvMat *mat = marker.GetContent();

  // Create a group for the marker
  // The group is also used to translate the marker to its desired location.
  // However, the transform does not include a scaling factor since other
  // properties like line thickness would also be scaled.
  /*
  out << "  <g transform=\"translate(" << pos_x << "," << pos_y;
  out << ")\">" << std::endl;
  out << RectBW(0, 0, 7, 7, true);
  for (int i = 0; i < resolution; i++) {
    for (int j = 0; j < resolution; j++) {
      std::cout << cvGet2D(mat, i, j).val[0] << " ";
      out << RectBW(i + 2, i + 2, 1, 1, (cvGet2D(mat, i, j).val[0]) == 0);
    }
    std::cout << std::endl;
  }
  */
  // Stupid SVG does not accept units in transforms: the nice translated
  // group does not work. So for now we will do everything manually...
  out << "  <g id=\"marker_" << marker.GetId() << "\">" << std::endl;
  float edge_size = marker.GetMarkerEdgeLength();
  int nmarks = resolution + 2 * margin;
  float mark_size = edge_size / static_cast<float>(nmarks);
  /* Just writing small white squares above the large black overall square
    leaves some faint lines. So we will leave the inside white instead...
  out << RectBW(pos_x, pos_y, edge_size, edge_size, true);
  */
  out << RectBW(pos_x, pos_y, edge_size, static_cast<float>(margin) * mark_size,
                true);
  out << RectBW(pos_x, pos_y + mark_size * (margin + resolution), edge_size,
                static_cast<float>(margin) * mark_size, true);
  out << RectBW(pos_x, pos_y, static_cast<float>(margin) * mark_size, edge_size,
                true);
  out << RectBW(pos_x + mark_size * (margin + resolution), pos_y,
                static_cast<float>(margin) * mark_size, edge_size, true);
  for (int i = 0; i < resolution; i++) {
    for (int j = 0; j < resolution; j++) {
      if (kDebugMarker) {
        std::cout << cvGet2D(mat, i, j).val[0] << " ";
      }
      // it seems that the Alvar library creates the marker column first !
      out << RectBW(pos_x + mark_size * (j + margin),
                    pos_y + mark_size * (i + margin), mark_size, mark_size,
                    (cvGet2D(mat, i, j).val[0]) == 0);
    }
    if (kDebugMarker) {
      std::cout << std::endl;
    }
  }
  out << "  </g>" << std::endl;
  return 0;
}

int DrillHoleToSvg(std::ofstream &out, float dia, float pos_x, float pos_y) {
  out << "  <g>" << std::endl;
  out << "    <circle " << DimU("cx", pos_x) << " " << DimU("cy", pos_y);
  out << " " << DimU("r", dia / 2.0) << " stroke-width=\"1px\"";
  out << " style=\"stroke:#000000; fill:none\"";
  out << "/>" << std::endl;
  out << LineBW(pos_x - dia, pos_y, pos_x + dia, pos_y, 0.5);
  out << LineBW(pos_x, pos_y - dia, pos_x, pos_y + dia, 0.5);
  out << "  </g>" << std::endl;
  return 0;
}

bool WriteSvgHeader(std::ofstream &out, int width, int height) {
  out << "<svg xmlns=\"http://www.w3.org/2000/svg\"" << std::endl;
  out << "    " << DimU("width", width) << " " << DimU("height", height);
  out << ">" << std::endl;
  return true;
}

bool WriteSvgClosing(std::ofstream &out) {
  out << "</svg>" << std::endl;
  return true;
}

int main(int argc, char *argv[]) {
  if (argc != 3) {
    std::cerr << "Usage: generate_svg_makers config_filename output_file"
              << std::endl;
    return 1;
  }

  char *config_filename = argv[1];
  char *output_svg_file = argv[2];

  config_reader::ConfigReader config;
  config.AddFile(config_filename);
  if (!config.ReadFiles()) {
    std::cerr << "Failed to read config files [" << config_filename << "]"
              << std::endl;
    return 1;
  }

  std::ofstream svg_file(output_svg_file, std::ios::out);
  if (!svg_file.is_open()) {
    std::cerr << "Cannot create file: " << output_svg_file << std::endl;
    return 1;
  }

  int drawing_precision;
  float drawing_width;
  float drawing_height;
  float drawing_xoffset;
  float drawing_yoffset;
  std::string drawing_unit;
  bool black_background = false;
  if (!config.GetReal("drawing_width", &drawing_width)) {
    drawing_width = 297;
  }
  if (!config.GetReal("drawing_height", &drawing_height)) {
    drawing_height = 210;
  }
  if (config.GetStr("drawing_unit", &drawing_unit)) {
    DimU::unit_ = drawing_unit;
  }
  if (config.GetInt("drawing_precision", &drawing_precision)) {
    DimU::precision_ = drawing_precision;
  }

  /**
    Get optional drawing offset in X and Y.
    Unfortunately, like the translate command that does not accept
    units, the SVG viewbox command also does not accept units, hence
    is not usable for our application.
    This means that the drawing_[xy]offset are manually applied to
    all drawing elements!
  */
  if (config.CheckValExists("drawing_xoffset")) {
    config.GetReal("drawing_xoffset", &drawing_xoffset);
  } else {
    drawing_xoffset = 0.0;
  }

  if (config.CheckValExists("drawing_yoffset")) {
    config.GetReal("drawing_yoffset", &drawing_yoffset);
  } else {
    drawing_xoffset = 0.0;
  }

  WriteSvgHeader(svg_file, drawing_width, drawing_height);

  //
  // Get the global parameters for the dock target
  //
  int ar_resolution;
  if (!config.GetInt("ar_resolution", &ar_resolution)) {
    std::cerr
        << "missing required parameter [ar_resolution] in the config file "
        << config_filename << std::endl;
    return 1;
  }
  int ar_margin;
  if (!config.GetInt("ar_margin", &ar_margin)) {
    std::cerr << "missing required parameter [ar_margin] in the config file "
              << config_filename << std::endl;
    return 1;
  }

  //
  // Iterate through each marker found in the table
  //
  config_reader::ConfigReader::Table markers_table(&config, "markers");
  SvgGroup *group = new SvgGroup(svg_file, "makers_layer");

  // optionally, create a rectangle for the background
  if (config.CheckValExists("black_background")) {
    if (config.GetBool("black_background", &black_background)) {
      if (black_background) {
        svg_file << RectBW(0.0, 0.0, drawing_width, drawing_height, true);
      }
    }
  }

  for (int i = 0; i < markers_table.GetSize(); i++) {
    int marker_id;
    float marker_size;
    float marker_pos[2];
    float white_margin;
    config_reader::ConfigReader::Table marker_specs(&markers_table, (i + 1));
    if (!marker_specs.GetInt("id", &marker_id)) {
      std::cerr << "missing [id] parameter in the marker table" << std::endl;
      delete (group);
      return 1;
    }
    if (!marker_specs.GetReal("size", &marker_size)) {
      std::cerr << "missing [size] parameter in the marker table" << std::endl;
      delete (group);
      return 1;
    }

    config_reader::ConfigReader::Table marker_location(&marker_specs, "pos");
    marker_location.GetReal(1, &marker_pos[0]);
    marker_location.GetReal(2, &marker_pos[1]);
    std::cout << "id=" << marker_id << " => size=" << marker_size;
    std::cout << "  pos=(" << marker_pos[0] << ", " << marker_pos[1];
    std::cout << ")" << std::endl;

    if (marker_specs.CheckValExists("white_margin")) {
      if (marker_specs.GetReal("white_margin", &white_margin)) {
        svg_file << RectBW(marker_pos[0] - white_margin + drawing_xoffset,
                           marker_pos[1] - white_margin + drawing_yoffset,
                           marker_size + 2.0 * white_margin,
                           marker_size + 2.0 * white_margin, false);
      }
    }
    alvar::MarkerData marker(marker_size,    // Default edge length
                             ar_resolution,  // AR tag resolution
                             ar_margin);     // Margin size

    marker.SetContent(
        alvar::MarkerData::MARKER_CONTENT_TYPE_NUMBER,  // number marker
        marker_id,                                      // ID of the marker
        NULL,                                           // No string
        false,                                          // No Strong Hamming
        false);                                         // Verbose OFF

    if (marker.GetRes() != ar_resolution) {
      std::cerr << "alvar library generated a resolution inconsistent with the "
                   "requested one: "
                << "(" << marker.GetRes() << " / " << ar_resolution << ")"
                << std::endl;
      delete (group);
      return 1;
    }

    // After all parameters extraction, finally do it!
    MarkerToSvg(svg_file, marker,  // we translate to the center of the drawing
                marker_pos[0] + drawing_xoffset,
                marker_pos[1] + drawing_yoffset);
  }
  delete (group);

  if (config.CheckValExists("holes")) {
    config_reader::ConfigReader::Table holes_table(&config, "holes");
    group = new SvgGroup(svg_file, "drillholes_layer");
    for (int i = 0; i < holes_table.GetSize(); i++) {
      float hole_dia;
      float hole_pos[2];
      config_reader::ConfigReader::Table hole_specs(&holes_table, (i + 1));
      if (!hole_specs.GetReal("dia", &hole_dia)) {
        std::cerr << "missing [dia] parameter in the holes table" << std::endl;
        delete (group);
        return 1;
      }
      config_reader::ConfigReader::Table hole_location(&hole_specs, "pos");
      hole_location.GetReal(1, &hole_pos[0]);
      hole_location.GetReal(2, &hole_pos[1]);
      DrillHoleToSvg(svg_file,
                     hole_dia,  // translate to the center of the drawing
                     hole_pos[0] + drawing_xoffset,
                     hole_pos[1] + drawing_yoffset);
    }
    delete (group);
  } else {
    std::cout << "No 'holes' table: skip holes generation" << std::endl;
  }

  if (config.CheckValExists("labels")) {
    config_reader::ConfigReader::Table labels_table(&config, "labels");
    std::string text_font;
    if (!config.GetStr("text_font", &text_font)) {
      std::cerr << "Could not find 'text_font' required for labels: stop!"
                << std::endl;
      return 2;
    }
    group = new SvgGroup(svg_file, "labels_layer");
    for (int i = 0; i < labels_table.GetSize(); i++) {
      config_reader::ConfigReader::Table label_specs(&labels_table, (i + 1));
      std::string text_str;
      float text_size = 24.0;
      float text_pos[2];
      if (!label_specs.GetStr("str", &text_str)) {
        std::cerr << "Missing [str] parameter: skip label #" << i + 1
                  << std::endl;
        continue;
      }
      if (!label_specs.GetReal("size", &text_size)) {
        std::cerr << "missing [size] parameter in the label table #" << i + 1
                  << " : use default=" << text_size << std::endl;
      }
      config_reader::ConfigReader::Table text_location(&label_specs, "pos");
      text_location.GetReal(1, &text_pos[0]);
      text_location.GetReal(2, &text_pos[1]);
      svg_file << CenteredText(text_str, text_pos[0] + drawing_xoffset,
                               text_pos[1] + drawing_yoffset, text_size,
                               text_font);
    }
    delete (group);
  } else {
    std::cout << "No 'labels' table: skip label generation" << std::endl;
  }

  WriteSvgClosing(svg_file);
  svg_file.close();

  return 0;
}
