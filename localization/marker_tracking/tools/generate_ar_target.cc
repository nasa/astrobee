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

#include <alvar/Marker.h>
#include <opencv2/core.hpp>

#include <cstdio>
#include <string>

static const int32_t BOUNDING_X = 0;
static const int32_t BOUNDING_Y = 0;
static const std::string OUTPUT_FORMAT = "marker_%03d.eps";
static const int32_t REFLECTOR_ID_FONT_SIZE = 18;
static const double REFLECTOR_SIZE = 40;  // pts
static const int32_t SQUARE_SIZE = 44;    // pts
static const int32_t TAG_ID_FONT_SIZE = 36;
static const int32_t WARNING_FONT_SIZE = 18;

int main(int argc, char** argv) {
  if (argc != 2) {
    printf("Must specify an id number as the second argument\n");
    return 1;
  }

  int id = atoi(argv[1]);
  char filename[1024];
  snprintf(filename, sizeof(filename), OUTPUT_FORMAT.c_str(), id);

  // Generate AR tag.
  alvar::MarkerData marker;
  marker.SetContent(
                    alvar::MarkerData::MARKER_CONTENT_TYPE_NUMBER,
                    id,      // Integer ID data.
                    NULL,    // String data.
                    false,   // Force strong Hamming encoding.
                    false);  // Verbose debug output.

  int resolution = marker.GetRes();
  int margin = static_cast<int>(marker.GetMargin());
  cv::Mat_<unsigned char> content, content_no_border = cv::cvarrToMat(marker.GetContent());
  cv::copyMakeBorder(content_no_border, content, margin, margin, margin, margin,
                     cv::BORDER_CONSTANT, 0);

  std::cout << "Margin size: " << margin << std::endl;
  std::cout << "resolution: " << resolution << std::endl;
  // Total bounding box is:
  // Width: tag content + black margin + 1-square extra border.
  // Height: tag content + black margin + 4-square extra border for reflectors.
  int bounding_x = BOUNDING_X ? BOUNDING_X :
    (1 + margin + resolution + margin + 1) * SQUARE_SIZE;
  int bounding_y = BOUNDING_Y ? BOUNDING_Y :
    (4 + margin + resolution + margin + 4) * SQUARE_SIZE;

  static const int32_t PAGE_SIZE_X_PTS = 612;  // 72 pts/in * 8.5"
  static const int32_t PAGE_SIZE_Y_PTS = 792;  // 72 pts/in * 11"
  int32_t translate_x = (PAGE_SIZE_X_PTS - bounding_x) / 2;
  int32_t translate_y = -(PAGE_SIZE_Y_PTS - bounding_y) / 2 - bounding_y;  // inverted

  // Write out in EPS format.
  printf("Writing AR target: %s\n", filename);
  FILE* file = fopen(filename, "w");
  fprintf(file,
          "%%!PS-Adobe EPSF-3.0\n"
          "%%%%Creator: generate_ar_target\n"
          "%%%%Title: Ground Truth Marker %d\n"
          "%%%%Origin: 0 0\n"
          "%%%%BoundingBox: 0 0 %d %d\n\n",
          id, bounding_x, bounding_y);
  // Change to x-right, y-down coordinate system to match image bitmap.
  fprintf(file,
          "1 -1 scale\n"
          "%d %d translate\n",
          translate_x, translate_y);
  // Define unit conversion macros from points (1/72in) to mm or grid squares.
  fprintf(file,
          "/mm { 360 mul 127 div } def\n"  // Points (1/72in) to mm
          "/sq { %d mul } def\n\n"         // Grid square size
          "/rf { %.9f mm mul } def\n",     // Reflector size
          SQUARE_SIZE, REFLECTOR_SIZE);
  // Macro to draw a filled square taking one argument, the gray level.
  fprintf(file,
          "/csquare {\n"
          "  newpath\n"
          "  0 0 moveto\n"
          "  0 1 rlineto\n"
          "  1 0 rlineto\n"
          "  0 -1 rlineto\n"
          "  closepath\n"
          "  setgray\n"
          "  fill\n"
          "} def\n\n");
  // Draw squares for AR tag grid.
  fprintf(file,
          "save\n"
          "1 sq 1 sq scale\n"
          "1 4 translate\n");
  for (int r = 0; r < content.rows; ++r) {
    for (int c = 0; c < content.cols; ++c) {
      fprintf(file, "%d csquare\n", content(r, c) / 255);
      fprintf(file, "1 0 translate\n");
    }
    fprintf(file, "%d 1 translate\n", -content.cols);
  }
  fprintf(file, "restore\n\n");
  // Macro to draw a placeholder for a reflective sticker and its ID.
  // Assumes ID text is on the stack.
  fprintf(file,
          "/line { newpath moveto lineto stroke } def\n"
          "/reflector {\n"
          "  newpath\n"  // Draw bounding square.
          "  -0.5 rf -0.5 rf moveto\n"
          "  0 1 rf rlineto\n"
          "  1 rf 0 rlineto\n"
          "  0 -1 rf rlineto\n"
          "  closepath\n"
          "  stroke\n"
          "  newpath\n"  // Draw half-size square.
          "  -0.25 rf -0.25 rf moveto\n"
          "  0 0.5 rf rlineto\n"
          "  0.5 rf 0 rlineto\n"
          "  0 -0.5 rf rlineto\n"
          "  closepath\n"
          "  stroke\n"
          "  newpath\n"  // Draw interior circle.
          "  0 0 0.45 rf 0 360 arc\n"
          "  closepath\n"
          "  stroke\n"
          "  -0.5 rf 0 0.5 rf 0 line\n"  // Draw cross in the center.
          "  0 -0.5 rf 0 0.5 rf line\n"
          "  newpath\n"  // Draw text label (argument to 'show').
          "  -0.5 rf 0.5 rf %d add moveto\n"
          "  1 -1 scale\n"  // Unflip Y so we don't draw text upside down.
          "  show\n"
          "  1 -1 scale\n"
          "} def\n\n",
          REFLECTOR_ID_FONT_SIZE);
  // Macro and font setup for drawing reflector IDs.
  fprintf(file,
          "/initfont {\n"
          "  /Times-Roman findfont\n"
          "  exch\n"
          "  scalefont\n"
          "  setfont\n"
          "} def\n"
          "%d initfont\n\n",
          REFLECTOR_ID_FONT_SIZE);
  // Draw all four reflectors.
  const int reflector_grid_distance_x = resolution + 2;
  const int reflector_grid_distance_y = resolution + 2 * margin + 4;
  fprintf(file,
          "save\n"
          "1 setlinewidth\n"
          "0 setgray\n"
          "2 sq 2 sq translate\n"
          "(%3$05d) reflector\n"
          "0 %2$d sq translate\n"
          "(%4$05d) reflector\n"
          "%1$d sq 0 translate\n"
          "(%5$05d) reflector\n"
          "0 -%2$d sq translate\n"
          "(%6$05d) reflector\n"
          "restore\n\n",
          reflector_grid_distance_x,
          reflector_grid_distance_y,
          id * 100 + 0,
          id * 100 + 1,
          id * 100 + 2,
          id * 100 + 3);
  // Draw AR tag ID and "DO NOT MOVE" text.
  fprintf(file,
          "%1$d initfont\n"
          "%2$d sq %3$d sq %1$d 1.25 mul add translate\n"
          "1 -1 scale\n"
          "newpath\n"
          "0 0 moveto\n"
          "(Tag #%4$03d) show\n",
          TAG_ID_FONT_SIZE,
          1 + margin + 1,
          4 + margin + resolution + margin + 1,
          id);
  fprintf(file,
          "%1$d initfont\n"
          "newpath\n"
          "0 -%1$d 1.5 mul moveto\n"
          "(DO NOT TOUCH) show\n",
          WARNING_FONT_SIZE);

  fclose(file);
}
