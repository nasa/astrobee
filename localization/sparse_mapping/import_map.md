\page import_map Importing a map in .nvm format

# Overview

The ``import_map`` tool is used to import a map from the NVM format,
which Theia and other SfM packages export to, to Astrobee's .map
format. The \ref export_map program does the reverse operation.

It is assumed that the NVM map was built with nav_cam images, which
were either undistorted or distorted (original ones), and that the
name of the robot which acquired the images is known.

In either case, the NVM file does not store descriptors, so 
a map needs to be rebuilt after it is imported. 

# Import an nvm map made with distorted images

Run:

    export ASTROBEE_ROBOT=bumble
    astrobee/devel/lib/sparse_mapping/import_map \
      -input_map map.nvm -output_map map.map

See further down about how to rebuild the imported map.
    
# Import an nvm map made with undistorted images

There are two cases to consider.

## Keep, after importing, the undistorted images and camera model
 
Run:

    export ASTROBEE_ROBOT=bumble
    astrobee/devel/lib/sparse_mapping/import_map                             \
      -undistorted_camera_params "wid_x wid_y focal_len opt_ctr_x opt_ctr_y" \
      <undistorted images>                                                   \
      -input_map map.nvm -output_map map.map

It is very important that the robot name be specified corretly.

It is assumed that the images were acquired with the nav camera of the
robot given by $ASTROBEE_ROBOT and undistorted with the Astrobee
program ``undistort_image``. The undistorted camera parameters to use
should be as printed on the screen (and saved to disk) by
``undistort_image``.

## Replace with distorted data

If desired to replace on importing the undistorted images with the
original distorted ones, and same for the camera parameters, as it is
usually expected of a sparse map, the above command should be called
instead as:
   
    export ASTROBEE_ROBOT=bumble
    astrobee/devel/lib/sparse_mapping/import_map \
      -undistorted_images_list undist_list.txt   \
      -distorted_images_list dist_list.txt       \
      -input_map map.nvm -output_map map.map

Here, the files ``undist_list.txt`` and ``dist_list.txt`` must have
one image per line and be in one-to-one correspondence. It is
important that both undistorted and distorted images be specified, as
the former are needed to look up camera poses and other data in the
.nvm file before being replaced with the distorted ones.

This use case was tested only with a map exported by Theia, 
which records the images without a directory path, so that is how
they should be specified in the undistorted image list as well. 

It is very important to note that the interest point matches will not
be correct, as they are left undistorted. Only the image names, robot
camera parameters, and camera poses will be accurate. So, this map
should be rebuilt right away.

# Rebuilding an imported map

To rebuild an imported map while keeping the camera poses read from
the nvm file, run:

    export ASTROBEE_ROBOT=bumble
    build_map -rebuild -rebuild_detector SURF \
     -output_map map.map 

To also reoptimize the camera poses, add the option: 

    -rebuild_refloat_cameras

Do not use this option with the BRISK detector, as this detector may
not create enough features for the camera poses to be optimized
correctly.

Note that if the NVM file is created by Theia, it may have the image
names be specified without the path to the directory having them (that
is, instead of image_dir/image1.jpg it may list just image1.jpg). Then
this NVM file needs to be edited manually to add the correct path to
the images before running the import tool. This is taken care of if
both ``-undistorted_images_list`` and ``-distorted_images_list`` are
specified.

