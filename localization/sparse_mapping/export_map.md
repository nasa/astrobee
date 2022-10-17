\page export_map Exporting a map to the .nvm format

# Overview

The ``export_map`` tool is used to export a map from an Astrobee .map
file in Protobuf format to the plain text .nvm file format, which is a
format used by various SfM packages, and can be exported by Theia.

This tool does not export the camera intrinsics and image descriptors,
but only the list of images, the position and orientation of each
camera, and the interest point matches (tracks). 

The obtained .nvm file can be imported back with the \ref import_map
tool.

The interest points saved to the nvm file are offset relative to the
optical center, per the NVM format convention. To not do that, use the
``-no_shift`` option.  The ``import_map`` tool then must use the same
option to import back such an .nvm file.

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
-no_shift
    Save the features without shifting them relative to the optical
    center. That makes visualizing them easier.
