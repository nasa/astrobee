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
#include <opencv2/core/core.hpp>

#include <cstdio>
#include <string>

static const int32_t BOUNDING_X = 0;
static const int32_t BOUNDING_Y = 0;
static const std::string OUTPUT_FORMAT = "ms_marker_%03d.eps";
static const int32_t S_SQUARE_SIZE = 6;
static const int32_t TAG_S_SQUARE_SIZE = 9 * S_SQUARE_SIZE;
static const int32_t M_SQUARE_SIZE = 3 * S_SQUARE_SIZE;
static const int32_t L_SQUARE_SIZE = 5 * S_SQUARE_SIZE;
static const int32_t TAG_ID_FONT_SIZE = 36;
static const int32_t WARNING_FONT_SIZE = 18;

// Page layout is

//
// ###   ###
// ###   ###
// ### # ###
//
//   # # #
//
//   # #####
//     #####
// ### #####
// ### #####
// ### #####
//
//   TEXT

void MakeARTarget(int id, cv::Mat_<unsigned char>* output) {
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
  if (resolution != 5 || margin != 2) {
    std::cerr << "Invalid resolution and margin created for ID";
    exit(1);
  }

  cv::Mat_<unsigned char> content_no_border = cv::cvarrToMat(marker.GetContent());
  cv::copyMakeBorder(content_no_border, *output, margin, margin, margin, margin,
                     cv::BORDER_CONSTANT, 0);
}

void DrawARTarget(FILE* file, cv::Mat_<unsigned char>* content, int scale, int left_s_sq, int top_s_sq) {
  // Draw squares for AR tag grid.
  fprintf(file,
          "save\n"
          "1 sq 1 sq scale\n"
          "%1$d %2$d translate\n"
          "%3$d %3$d scale\n",
          left_s_sq, top_s_sq,
          scale);
  for (int r = 0; r < content->rows; ++r) {
    for (int c = 0; c < content->cols; ++c) {
      fprintf(file, "%d csquare\n", (*content)(r, c) / 255);
      fprintf(file, "1 0 translate\n");
    }
    fprintf(file, "%d 1 translate\n", -content->cols);
  }
  fprintf(file, "restore\n\n");
}

int main(int argc, char** argv) {
  if (argc != 2) {
    printf("Must specify an id number as the second argument\n");
    return 1;
  }

  int id = atoi(argv[1]);
  char filename[1024];
  snprintf(filename, sizeof(filename), OUTPUT_FORMAT.c_str(), id);

  // Generate AR tag.
  cv::Mat_<unsigned char> content[9];
  for (int i = 0; i < 9; i++) {
    MakeARTarget(id + i, content + i);
  }

  // Total bounding box is:
  // Width: 11
  // Height: 13 + space for text
  int bounding_x = BOUNDING_X ? BOUNDING_X : 11 * TAG_S_SQUARE_SIZE;
  int bounding_y = BOUNDING_Y ? BOUNDING_Y : 14 * TAG_S_SQUARE_SIZE;

  static const int32_t PAGE_SIZE_X_PTS = 612;  // 72 pts/in * 8.5"
  static const int32_t PAGE_SIZE_Y_PTS = 792;  // 72 pts/in * 11"
  int32_t translate_x = (PAGE_SIZE_X_PTS - bounding_x) / 2;
  int32_t translate_y = -(PAGE_SIZE_Y_PTS - bounding_y) / 2 - bounding_y;  // inverted

  // Write out in EPS format.
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
          "/sq { %d mul } def\n\n",        // Grid square size for small AR
          S_SQUARE_SIZE);
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

  // Draw top left midsize AR
  DrawARTarget(file, content + 0, 3, 9, 9);
  // Draw top small smallsize AR
  DrawARTarget(file, content + 1, 1, 5 * 9, 3 * 9);
  // Draw top right midsize AR
  DrawARTarget(file, content + 2, 3, 7 * 9, 9);
  // Draw small left middle AR
  DrawARTarget(file, content + 3, 1, 3 * 9, 5 * 9);
  // Draw small middle middle AR
  DrawARTarget(file, content + 4, 1, 5 * 9, 5 * 9);
  // Draw small right middle AR
  DrawARTarget(file, content + 5, 1, 7 * 9, 5 * 9);
  // Draw small left bottom AR
  DrawARTarget(file, content + 6, 1, 3 * 9, 7 * 9);
  // Draw midsize bottom left AR
  DrawARTarget(file, content + 7, 3, 9, 9 * 9);
  // Draw large bottom right AR
  DrawARTarget(file, content + 8, 5, 5 * 9, 7 * 9);

  // Draw AR tag ID and "DO NOT MOVE" text.
  fprintf(file,
          "/initfont {\n"
          "  /Times-Roman findfont\n"
          "  exch\n"
          "  scalefont\n"
          "  setfont\n"
          "} def\n"
          "%1$d initfont\n"
          "%2$d sq %3$d sq %1$d 1.25 mul add translate\n"
          "1 -1 scale\n"
          "newpath\n"
          "0 0 moveto\n"
          "(Tag #%4$03d - #%5$03d) show\n",
          TAG_ID_FONT_SIZE,
          4 * 9 - 3,
          13 * 9 - 3,
          id, id + 8);
  fprintf(file,
          "%1$d initfont\n"
          "newpath\n"
          "0 -%1$d 1.5 mul moveto\n"
          "(DO NOT TOUCH) show\n",
          WARNING_FONT_SIZE);

  fclose(file);
}
