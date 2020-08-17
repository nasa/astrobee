\page camera Camera Library

This library handles camera calibration parameters and
transformations, providing functions to handle between undistorted and
distorted coordinate frames, and other helper functions to deal with
reading and applying camera transformations.

A tool named undistort_image is provided that can undistort (and
optionally crop) a given set of images.

The undistorted image dimensions, as well as the focal length and
undistorted optical center will be displayed on the screen.

Usage:

  export ASTROBEE_RESOURCE_DIR=$SOURCE_PATH/astrobee/resources
  export ASTROBEE_CONFIG_DIR=$SOURCE_PATH/astrobee/config
  export ASTROBEE_WORLD=granite
  export ASTROBEE_ROBOT=p4d

  undistort_image input_dir/*[0-9].jpg --output_directory output_dir \
     --robot_camera nav_cam

Other options, useful with the dense mapper, are --save_bgr,
--undistorted_crop_win, and --image_list. See undistort_image.cc for
more information.
