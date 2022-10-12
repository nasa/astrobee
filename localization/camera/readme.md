\page camera Camera library

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

Options:

  --image_list: A file having the list of images to undistort, one per
    line. If not specified it is assumed they are passed in directly on
    the command line.

  --output_directory: Output directory. If not specified, undistorted
    images will saved in the same directory as the inputs.

  --scale: Undistort images at different resolution, with their width
    being a multiple of this scale compared to the camera model. The
    default is 1.

  --undistorted_crop_win: After undistorting, apply a crop window of
      these dimensions centered at the undistorted image center. The
      adjusted dimensions and optical center will be printed on screen.
      Specify as: 'crop_x crop_y'.

  --save_bgr: Save the undistorted images as BGR instead of
      grayscale. (Some tools expect BGR.)

  --histogram_equalization: If true, do histogram equalization.

  --robot_camera: Which of bot's cameras to use. Tested with nav_cam
    and sci_cam. The default is nav_cam.
