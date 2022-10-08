\page export_map Exporting a map to the .nvm format

# Overview

The ``export_map`` tool is used to export a map from an Astrobee .map
file in Protobuf format to the plain text .nvm file format. 

This tool does not export the camera intrinsics and image descriptors,
but only the list of images, the position and orientation of each
camera, and the interest point matches (tracks). 

The obtained .nvm file can be imported back with the \ref import_map
tool.

The interest points are offset relative to the optical center, per the 
NVM format convention.

Example:

    $HOME/astrobee/devel/lib/sparse_mapping/export_map  \
      -input_map input.map                              \
      -output_map output.nvm

Command line options:

-input_map <string (default="")>
    Input sparse map file, in Astrobee's protobuf format, with .map
      extension.
-output_map <string (default="")>
    Output sparse map in NVM format.
