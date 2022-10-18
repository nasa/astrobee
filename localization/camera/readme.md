\page camera Camera library

This library handles camera calibration parameters and
transformations, providing functions to handle between undistorted and
distorted coordinate frames, and other helper functions to deal with
reading and applying camera transformations.

A tool named `undistort_image` is provided that can undistort (and
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
    Specify as: 'crop_x crop_y'.  Or specify 'loose' to use the smallest
    crop window that contains all of the undistorted source pixels and
    keeps the optical center centered.

  --save_bgr: Save the undistorted images as BGR instead of
      grayscale. (Some tools expect BGR.)

  --histogram_equalization: If true, do histogram equalization.

  --robot_camera: Which of bot's cameras to use. Tested with nav_cam
    and sci_cam. The default is nav_cam.

  --alpha: Add an alpha channel and make areas outside the remapped
    input transparent. Forces PNG output format. Not compatible with
    save_bgr.

  --cubic: Use more expensive cubic interpolation for best image
    quality.

Notes
-----

For stitching Astrobee SciCam panoramas, the Hugin tools by default
expect input image frames to have a "rectilinear" camera model. We can
satisfy this by preprocessing input frames using `undistort_image`. This
lets us lean on our own quality-controlled SciCam calibration rather
than adding extra lens parameters to complicate Hugin's already huge and
not-always-stable stitching optimization problem.

The following flags are recommended for this use case:

  --undistorted_crop_win loose: The undistorted image is not in general
    rectangular. This argument tells the tool to use the tightest crop
    box that saves all the useful remapped pixels and keeps the optical
    center centered.  The resulting images are much smaller than when
    uncropped and help to avoid Hugin errors that can come from poorly
    conditioned math with an overly-wide field of view.

  --alpha: Since the undistorted image is not rectangular, there will be
    some junk where there is no real data around the borders. By
    default, it is black, but this argument adds an alpha channel so it
    can be transparent. Luckily, Hugin will respect an alpha channel in
    the input images if it is provided, so we won't see random black
    border bits in the stitched output.

  --cubic: We want to avoid losing image quality wherever possible,
    since that is what the panorama users care about! Mild quality
    improvement, takes a bit longer.
