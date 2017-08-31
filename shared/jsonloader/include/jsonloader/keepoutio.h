// Copyright 20l5 Intelligent Robotics Group, NASA ARC

#ifndef JSONLOADER_KEEPOUTIO_H_
#define JSONLOADER_KEEPOUTIO_H_

#include <string>

#include "jsonloader/keepout.h"

namespace jsonloader {

class FormatError;

// Load a keepout file in JSON format.
// *note:* This will overwrite anything within the passed-in zone.
bool ReadKeepoutFile(std::string const& input_filename, Keepout *zone);

// Load a directory of keepout files in JSON format, merges all zones into
// the two passed in parameters, for whether they are safe or not. Passsing
// a NULL pointer ignores zones of the particular safety.
//
// Skips any keepout files that are invalid according to ReadKeepoutFile
//
// Note: Does not recurse into any subdirectories.
void ReadKeepoutDirectory(std::string const& input_directory,
                          Keepout *safeZone, Keepout *dangerZone);

}  // namespace jsonloader

#endif  // JSONLOADER_KEEPOUTIO_H_
