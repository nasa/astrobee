\defgroup sparsemapping Sparse Mapping
\ingroup localization

This folder deals with creating and using sparse maps of visual features.

# Package Overview

## Library

The library, in the src and include directories, provides a class
[SparseMap](@ref sparse_mapping::SparseMap) which represents the
map. It has functions to build a map and to localize from a map given
an image.

A map consists of feature descriptors and associated 3D positions of
the features.  A map may also contain a vocabulary database which
enables fast lookup of similar images.

### Map Files

Maps are stored as protobuf files, which are defined in the `protobuf` folder.

## ROS Node

The ROS node takes images and a map as input, and outputs visual features
detected in the image and their 3D coordinates.

### Inputs

* `/hw/cam_nav`: Camera images
* Map file (specified on command line, via `--map_file`)

### Outputs

* `/localization/mapped_landmarks/features`
* `/localization/mapped_landmarks/registration`

## Tools and procedures

### Record a bag

Record on the robot in order to not drop too many frames. Launch the
camera node. Connect to the robot. Create a subdirectory in /data
where the data will be recorded, as the home directory on the robot is
too small.  Move the robot slowly to make sure neighboring images have
enough overlap, and to reduce motion blur. Run:

  rosbag record /hw/cam_nav

The name of the topic containing the images may differ from /hw/cam_nav.
To see what topics a bag file contains one can use the command

  rosbag info bagfile.bag

### Filter the bag

Usually the bags are acquired at a very high frame rate, and they are
huge. A preliminary filtering of the bag images while still on the
robot can be done with the command:

  rosbag filter input.bag output.bag "(topic == '/hw/cam_nav') and (float(t.nsecs)/1e+9 <= 0.1)"

Here, for every second of recorded data, we keep only the first tenth
of a second. This number may need to be adjusted. Later, a further
selection of the images can be done.

### Copy the bag from the robot:

From the local machine, fetch the bag:

  rsync -avzP astrobee@10.42.0.32:/data/bagfile.bag .

Here, the IP address of P4D was used, which may differ from your robot's IP address.

### Merging Bags

The bags created on the ISS are likely split into many smaller bags,
for easy and reliability of transfer. Those can be merged into one bag
as follows:

  export BUILD_PATH=$HOME/freeflyer_build/native
  source $BUILD_PATH/devel/setup.bash
  python freeflyer/localization/sparse_mapping/tools/merge_bags.py \
    <output bag> <input bags>

###Extracting Images

To extract images from a bag file:

    freeflyer_build/native/devel/lib/localization_node/extract_image_bag <bagfile.bag> \
      -image_topic /hw/cam_nav -output_directory <output dir>

(the above assumes that the software was built with ROS on).

### Building a Map

The `build_map` tools aids in constructing a map. See [here](@ref buildmap) for
further details.

###Visualization

To visualize a map, use the command:

    nvm_visualize <output.map> [ <image1.jpg> <image2.jpg> ... ]

In the viewer, press `a` and `d` (or the left and right arrow keys, or
the Ins and Del keys on the num pad) to navigate through the
images. Press `c` to show the cameras and triangulated points in
3D. Press `q` to exit the viewer.

In `c` mode, press left and right to visualize the localization
results for the input image `imageN.jpg`. If no images are
passed in, the arrow keys display each frame of the map.

Also, in the `c` mode, when points and cameras are shown in 3D,
pressing `a` will save the current 3D pose to disk, while pressing `b`
will read a 3D pose from disk and apply it. This way, when two viewers
are open side by side, they can be made to show the results from the
same perspective.

The viewer can display just a subset of the cameras, using the `-first`
and `-last` options, and the size of cameras can be set with `-scale`.

Only the camera positions can be displayed, without the 3D points, if
the viewer is invoked with `-skip_3d_points`. The cameras can be made
to rotate around the center of mass of the cameras, rather than the
origin, using the `d` key. When in 3D view, rendering of thumbnails of
the images can be skipped, as that makes the viewer slow, with
'-skip_3d_images'.

Clicking on an interest point with the middle mouse will display all
images in which that interest point was detected, with the interest
point in each of them shown as a red dot. Clicking back on the
original window with the middle mouse will make these images go away.

### Localize a Single Frame

To test localization of a single frame, use the command:

    localize <map.map> <image.jpg>

If invoked with the option -verbose_localization, it will list the
images most similar to the one being localized. To increase the 
number of similar images, use the -num_similar option. Another
useful flag is --v 2 when it will print more verbose information.

### Testing Localization 

To test localization of many images, one can acquire two sets of
images of the same indoor environment, and create two maps ready for
localization. That is, maps are built, registered to the world
coordinate system, rebuilt with BRISK, and then a vocabulary database
is created. Name those maps reference and source.

For each image in the source map, one can localize it against the
reference map, and compare its camera position and orientation after
localization with the "known" position and orientation from the source
map.

This is not a fool-proof test, since neither of the two maps contains
measured ground truth, rather a simulated version of it, yet it can be
useful, assuming that maps are individually accurate enough and
well-registered.

This functionality is implemented in the localize_cams tool. Usage:

  localize_cams -num_similar 20 -ransac_inlier_tolerance 5      \
    -num_ransac_iterations 200 -min_brisk_features 400          \
    -max_brisk_features 800 -min_brisk_threshold 10             \
    -default_brisk_threshold 75 -max_brisk_threshold 75         \
    -detection_retries 5 -num_threads 2                         \
    -early_break_landmarks 200                                  \
    -reference_map ref.map -source_map source.map

Here we use values that are different from 

  astrobee/config/localization.config 

which are used for localization on the robot, since those are optimized
for speed and here we want more accuracy.

### Extract sub-maps

The tool `extract_submap` can be used to extract a submap from a map,
containing only a specified list of images, or a given range of
image indices, or images with camera center in a given box. Usage:

    extract_submap -input_map <input map> -output_map <output map> \
      <images to keep>

or
    extract_submap -input_map <input map> -output_map <output map> \
      -image_list <file>
or
    extract_submap -input_map <input map> -output_map <output map> \
     -exclude <images to exclude>

or

    extract_submap -input_map <input map> -output_map <output map> \
      -cid_range "min_cid max_cid"

(here first image has cid = 0, and the range is inclusive at both
ends), or

    extract_submap -input_map <input map> -output_map <output map> \
      -xyz_box "xmin xmax ymin ymax zmin zmax"

If it is desired to not re-adjust the cameras after the submap is
extracted (for example, if the map is already registered), use the
`-skip_bundle_adjustment` option. 

If the input map has a vocabulary database of features, it will
need to be rebuilt for the extracted submap using

  build_map -vocab_db


#### Merge Maps

Given a set of maps, they can be merged using the command:

    merge_maps <input maps> -output_map merged.map -num_image_overlaps_at_endpoints 10

It is very important to note that only maps that have not been pruned
can be merged, hence the maps should have been built with
-skip_pruning. If a map is already pruned, it needs to be rebuilt, as
follows:

    build_map -rebuild -skip_pruning -rebuild_detector <detector> -output_map <output map>

and then these regenerated maps can be merged. Note that the merged
map will be pruned as well, unless merging is invoked also with
-skip_pruning. Also note that the above won't rebuild the vocabulary
database (if desired, for brisk features). For that one should use
additionally the '-vocab_db' option.

Merging is more likely to succeed if the images at the endpoints of
one map are similar to images at the endpoints of the second map, and
in particular, if some of the same images show up at the endpoints of
both maps. A larger value of `-num_image_overlaps_at_endpoints` may
result in higher success but will take more time.

Registration to the real-world coordinate system must be (re-)done
after the maps are merged, as the bundle adjustment done during merging
may move things somewhat.

The input maps to be merged need not be registered, but that may help
improve the success of merging. Also, it may be preferable that
the images at the beginning and end of the maps to merge be close
to points used in registration. The implication here is that the
more geometrically correct the input maps are, and the more similar
to each other, the more likely merging will succeed.

After a merged map is created and registered, it can be rebuilt with
the BRISK detector to be used on the robot. 

When manipulating many submaps, it is suggested that bundle adjustment
be skipped during merging, using -skip_bundle_adjustment, until the
final map is computed, as this step can be time-consuming.

#### How To Build a Map Efficiently

Often times map-building can take a long time, or it can fail. A
cautious way of building a map is to build it in portions (perhaps on
different machines), examine them, and merge them with 'merge_maps'.

If map-building failed, parts of it could still be salvageable (one
can use nvm_visualize for inspection). Valid submaps can be extracted
with 'extract_submap'. Then those can be merged with 'merge_maps'.

When two maps to be merged overlap only in the middle, and they are
both large, the number -num_image_overlaps_at_endpoints will need to
be large which would make merging very slow. A very useful option can
then be the flag '-fast_merge' for this tool. It won't create matches
among the two maps, but will instead identify the shared images among
the two maps thus merging the maps, if shared images exist.

If no such images are available, but the two maps do see the same
physical location in some portions (if from different views), each of
the two maps can be first merged with the same small map of that
shared location, and then the newly merged map which now will have
shared images can be merged with the '-fast_merge' flags.

To summarize, with careful map surgery (extract submaps and merge
submaps) large maps can be made from smaller or damaged ones within
reasonable time.

All these operations should be performed on maps with SURF features.
Hence the general approach for building maps is to create small SURF
maps using the command:

  build_map -feature_detection -feature_matching -track_building \
   -incremental_ba -tensor_initialization -bundle_adjustment     \
   -skip_pruning -num_subsequent_images 100 images/*jpg          \
   -output_map <map file>

examine them individually, merging them as appropriate, then
performing bundle adjustment (while skipping pruning) and registration
as per build_map.md. Only when a good enough map is obtained, a
renamed copy of it should be rebuilt with BRISK features and a
vocabulary database to be used on the robot.

# How to Add to a Map Images for Which Localization Fails

The current approach has several steps (in the future this proces may
be streamlined). 

First a new SURF map is built from the new images (using
-skip_pruning). The program merge_maps is invoked on the old and new
maps (in this order), using the -skip_pruning flag and with bundle
adjustment (the latter is the default in merge_maps). The combined map
is re-registered, and the submap corresponding to the new images is
extracted (without redoing bundle adjustment). The obtained SURF map
of new images is now in the same coordinate system as the old one.

The new map is used as the source map in localize_cams, with the old
map (the BRISK version of it) being the reference map. The images for
which localization error is good are saved to a file. Then
extract_submap is invoked on the merged map to exclude the images with
good localization, leaving in the combined SURF map the old images
together with the new images with bad localization. The combined map
is rebuilt with BRISK and a vocabulary database.

#### Reducing the Number of Images in a Map

Sometimes a map has too many similar images. The tool reduce_map.py
attempts to reduce their number without sacrificing the map quality.

It is very important that the input map is not pruned, so when it is
created (or rebuilt) the -skip_pruning flag must be used.  It should
be made of of BRISK features, registered, and with a vocab db.

Usage:

  python reduce_map.py -input_map <input map> -min_brisk_threshold <val> \
         -default_brisk_threshold <val> -max_brisk_threshold <val>       \
         -localization_error <val> -work_dir <work dir>                  \
         -sample_rate <val>

A sequence of ever-smaller output maps are saved in the work
directory. They are pruned maps, unlike the input unpruned map. 

The algorithm is as follows. Randomly remove a fraction (stored in
-sample_rate, typically 1/4 th) of images form a map. Localize the
images from the full map against the images of the reduced map. Images
for which localization fails with more than a given error (typically 2
cm) are added back to the reduced map. This is repeated until no more
images need adding (this is called the inner iteration).

The reduced map is written to 

  <work_dir>/submap_iter0.map

Then more images are taken out of the map and all the previous process
is repeated (this is called the outer iteration), each time obtaining
a smaller map named

  <work_dir>/submap_iter<outer iter>.map

One should carefully evaluate these output maps. Likely after a couple
of outer iterations the quality of the map may start degrading.
