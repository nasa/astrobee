# COLMAP

This directory contains tools for building Astrobee maps using colmap. Downloading colmap: https://colmap.github.io/
and add the colmap executable to your path to use. This was tested with colmap 3.9.1.

incremental.py contains two methods to build maps incrementally: one by merging submaps, and one by adding images over time.
colmap.py allows viewing the colmap database and models in python.
remove_images.py removes a set of images from a colmap map.

Colmap seems extremely promising but we did not have time to fully test it. Generally the standard colmap map creation works well,
but depending on the image sources may sometimes fail and create maps that are not right. This
is seen with ISS walls that do not align. Likely this could be overcome through manual fiddling with the image set as done with our current mapping
procedure.

## Create Colmap map

Follow this procedure to build a map in colmap.

1. Extract images from bags into a single folder (optionally with subfolders) following existing procedure.

2. colmap gui
3. File --> New Project
     Select image folder and create .db file
     Save

4. Processing --> Feature Extraction
     Camera model: RADIAL_FISHEYE
     Check "Shared for all images"
     Extract

5. Processing --> Feature Matching
     Go to "VocabTree" tab
     vocab_tree path --> Select file
         vocab_tree_flickr100K_words32K.bin (from colmap website)
         Run (this step takes a while)

   Exhaustive matching will give better results but is slower. Do exhaustive if you are not in a hurry.
   
6. Reconstruction --> Start Reconstruction
      Check that it looks good

7. Reconstruction --> Bundle Adjustment
      Check "refine_principal_point" to get full camera calibration
      Run
      If it doesn't converge, try increasing max iterations and / or running it again

8. File --> Save Project As (Lets you reload the data in colmap)
9. File --> Export Model  (model_output_folder, Saves in a folder you pick, this lets you reload the model in colmap later, need to load project first)

## Camera Calibration

Colmap generates a camera calibration that is believed to be better than our existing calibration. After building a map, you
can extract the calibration for use in the Astrobee software.

1. (in Colamp gui) File --> Export Model as Text
2. Open cameras.txt in exported text model folder
3. Should have line in format:
    1 RADIAL_FISHEYE 1280 960 f a b k1 k2

4. Copy bumble.config, rename colmap.config, update w/ colmap intrinsics and distortion values as:

   robot_camera_calibrations = {
     nav_cam = {
       distortion_coeff = {k1, k2},
       intrinsic_matrix = {
         f, 0.0, a,
         0.0, f, b,
         0.0, 0.0, 1.0
       },

## Convert to Astrobee Map

Follow these instructions to convert a map generated with colmap into the format used by Astrobee.

1. colmap model_converter --output_type NVM --skip_distortion true --input_path model_output_folder/ --output_path colmap.nvm 

2. export ASTROBEE_ROBOT=colmap
   export ASTROBEE_WORLD=iss
3. rosrun sparse_mapping import_map --input_map colmap.nvm --output_map colmap.map

4. Register map:
   rosrun sparse_mapping build_map -registration file.pto file.txt -num_ba_passes 0 -skip_filtering -output_map colmap.map
5. Rebuild with new features:
   rosrun sparse_mapping build_map -rebuild -rebuild_replace_camera -histogram_equalization -output_map colmap.map -rebuild_detector ORGBRISK
   Reproject error should be ~0.2 (less is better) and have many features
6. Now check map looks good:
   rosrun sparse_mapping nvm_visualize --skip_3d_images colmap.map
7. Build vocab db:
   rosrun sparse_mapping build_map -vocab_db -output_map colmap.map
