// Copyright 2015 Intelligent Robotics Group, NASA ARC

#include <config_reader/config_reader.h>
#include <alvar/Marker.h>
#include <yaml-cpp/yaml.h>

#include <cstdio>
#include <fstream>
#include <iostream>

typedef struct {
  int id;
  int size;
  float pos[2];
  std::string hexCode;
} Tag;

namespace YAML {
  template<>
  struct convert<Tag> {
    static Node encode(const Tag& tag) {
      Node node;
      node.SetStyle(YAML::EmitterStyle::Flow);

      node["pos"].push_back(tag.pos[0]);
      node["pos"].push_back(tag.pos[1]);

      node["size"]=tag.size;

      return node;
    }
  };
}  // namespace YAML

std::string toHex(std::deque<bool> bits) {
  std::stringstream ss;
  ss << "0x";
  ss.unsetf(std::ios_base::dec);
  ss.setf(std::ios_base::hex);
  uint64_t b = 0;
  int bitpos = (0x08 << (bits.size() % 4));
  if (bitpos > 0x08) bitpos >>= 4;
  for (size_t i = 0; i < bits.size(); i++) {
    if (bits[i])
      b = b | bitpos;
    else
      b = b & (0x0f ^ bitpos);
    bitpos >>= 1;
    if (bitpos == 0x00) {
      bitpos = 0x08;
      if (i == 0 && b == 0)
        continue;
      ss << b;
    }
  }
  return ss.str();
}

bool compareById(Tag &a, Tag &b) {
  return a.id < b.id;
}

void writeToYaml(YAML::Node content, std::string filename) {
  std::ofstream fout(filename);
  std::cout <<  content;
  fout << content;
}

void printTagFamily(std::ofstream& fs, std::string familyName, int res, int hamming, std::vector<Tag> markers) {
  fs << "#pragma once\n\nnamespace AprilTags {\n\nconst unsigned long long t" << familyName << "[] =\n{\n   ";
  for (std::vector<Tag>::iterator it = markers.begin(); it != markers.end(); it++) {
    fs << it->hexCode << "l, ";
  }
  fs << "\n};\n\nstatic const TagCodes tagCodes" << familyName << " = TagCodes(" << res*res << "," << hamming <<
        ",t" << familyName << ", ";
  fs << "sizeof(t" << familyName << ")/sizeof(t" << familyName << "[0]));\n\n}";
}

int main(int argc, char* argv[]) {
  if (argc != 3) {
    std::cerr << "Usage: markers2Kalibr config_filename output_file"
              << std::endl;
    return 0;
  }

  char *config_filename = argv[1];
  char *output_file = argv[2];
  char *p = getenv("KALIBR_WS");

  if (p == NULL) {
    std::cerr << "KALIBR_WS was not defined" << std::endl;
    return 0;
  }
  std::string path(p);

  YAML::Node node;
  node["target_type"] = "assymetric_aprilgrid";

  config_reader::ConfigReader config;
  config.AddFile(config_filename);
  if (!config.ReadFiles()) {
    std::cerr << "Failed to read config file [" << config_filename << "]"
              << std::endl;
    return 0;
  }

  config_reader::ConfigReader::Table markers_table;
  if (!config.GetTable("markers", &markers_table)) {
    std::cerr << "Markers can not be found in " << config_filename << std::endl;
    return 0;
  }

  float left_border, bot_border;
  int res, hamming = 4;

  config.GetReal("left_border", &left_border);
  config.GetReal("bot_border", &bot_border);
  config.GetInt("ar_resolution", &res);

  // Name of the Tag family. Can be any name but default is bits + "h" + minimum hamming distance
  std::string family = std::to_string(res*res) + "h" + std::to_string(hamming);

  std::ofstream libraryFile;
  std::fstream tagsHeaderFile;

  libraryFile.open(path + "/src/kalibr/aslam_offline_calibration/ethz_apriltag2/include/apriltags/Tag" + family + ".h");

  alvar::MarkerData marker(1, 5, 2);
  std::vector<Tag> markers;

  for (int i = 0; i < markers_table.GetSize(); i++) {
    config_reader::ConfigReader::Table marker_specs(&markers_table, (i + 1));

    Tag marker_tag;

    marker_specs.GetInt("id", &marker_tag.id);
    marker_specs.GetInt("size", &marker_tag.size);

    config_reader::ConfigReader::Table marker_corner(&marker_specs, "pos");
    marker_corner.GetReal(1, &marker_tag.pos[0]);
    marker_corner.GetReal(2, &marker_tag.pos[1]);

    // Frame of Reference of Positions is the FSW AR Target is X right, Y down with origin in the center
    // Frame of Reference in Kalibr is X right, Y up with orgin in the bottom Left corner.

    // Transforming FSW -> Kalibr
    marker_tag.pos[0] -= left_border;
    marker_tag.pos[1] -= bot_border;
    marker_tag.pos[1] = - marker_tag.pos[1] - marker_tag.size;

    // Alvar uses id's to identify each marker. Kalibr expects hexadecimal codes (AprilTag library)
    // Decode ID of each marker tag into its hexadecimal code.
    std::deque<bool> bs;
    marker.SetContent(alvar::MarkerData::MarkerContentType::MARKER_CONTENT_TYPE_NUMBER, marker_tag.id, 0);
    unsigned char* marker_data = marker.GetContent()->data.ptr;

    for (int d = 0; d < marker.GetContent()->cols*marker.GetContent()->rows; d++) {
      if (marker_data[d])
        bs.push_back(true);
      else
        bs.push_back(false);
    }

    marker_tag.hexCode = toHex(bs);
    markers.push_back(marker_tag);
  }

  // Sort markers by Id number
  std::sort(markers.begin(), markers.end(), compareById);

  node["tags"] = markers;

  writeToYaml(node, output_file);

  printTagFamily(libraryFile, family, res, hamming, markers);

  libraryFile.close();

  // Append TagFamily to AllTags.h if not there already
  tagsHeaderFile.open(path + "/src/kalibr/aslam_offline_calibration/ethz_apriltag2/include/apriltags/AllTags.h",
                      std::ios::out|std::ios::in|std::ios::app);
  std::string line;
  while (std::getline(tagsHeaderFile, line)) {
    if (line.compare("#include \"apriltags/Tag" + family + ".h\"") == 0)
      return 1;
  }
  tagsHeaderFile.clear();
  tagsHeaderFile << "#include \"apriltags/Tag" + family + ".h\"";
  tagsHeaderFile.close();
}
