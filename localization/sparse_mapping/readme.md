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

* `/hw/cam_nav`: Camera Images
* Map File (specified on command line, via `--map_file`)

### Outputs

* `/localization/mapped_landmarks/features`
* `/localization/mapped_landmarks/registration`

## Tools and procedures

### Record a bag.

Record on the robot in order to not drop too many frames. Launch the
camera node. Connect to the robot. Create a subdirectory in /data
where the data will be recorded, as the home directory on the robot is
too small.  Move the robot slowly to make sure neighboring images have
enough overlap, and to reduce motion blur. Run:

  rosbag record /hw/cam_nav

### Filter the bag

Usually the bags are acquired at a very high frame rate, and they are
huge. A preliminary filtering of the bag images while still on the
robot can be done with the command:

  rosbag filter input.bag output.bag "(topic == '/hw/cam_nav') and (float(t.nsecs)/1e+9 <= 0.1)"

Here, for every second of recorded data, we keep only the first tenth
of a second. This number may need to be adjusted. Later, a further
selection of the images can be done.

### Copy the bag from the robot:

From the host machine, fetch the bag:

  rsync -avzP astrobee@10.42.0.32:/data/bagfile.bag .

Here, the IP address of P4D was used, which may differ from your robot's IP address.

###Extracting Images

To extract images from a bag file:

    freeflyer_build/native/devel/lib/localization_node/extract_image_bag <bagfile.bag>

(the above assumes that the software was built with ROS on).

### Building a Map

The `build_map` tools aids in constructing a map. See [here](@ref buildmap) for
further details.

###Visualization

To visualize a map, use the command:

    nvm_visualize <output.map> [ <image1.jpg> <image2.jpg> ... ]

In the viewer, press `a` and `d` (or the left and right arrow
keys) to navigate through the images. Press `c` to show the cameras
and triangulated points in 3D. Press `q` to exit the viewer.

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

Only the camera positions can be displayed if the viewer is invoked with
`-skip_3d_points` and `-jump_to_3d_view`. The cameras can be made to rotate
around the center of mass of the cameras, rather than the origin, using
the `d` key.

### Localize a Single Frame

To test localization of a single frame, use the command:

    localize <map.map> <image.jpg>

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

    localize_cams -num_similar 20 -ransac_inlier_tolerance 20     \
      -ransac_iterations 100 -min_features 200 -max_features 800  \
      -brisk_threshold 100 -detection_retries 2 -num_threads 2    \
      -reference_map ref.map -source_map source.map

Here we have used the settings from 
astrobee/config/localization.config as when localization happens on
the robot.

### Extract sub-maps

The tool `extract_submap` can be used to extract a submap from a map,
containing only a given subset of the images or images with camera
center in a given box. This can be useful if the map failed to build
properly, but parts of it are still salvageable. Those can be
extracted, new small maps can be created of the region that failed,
then all maps can be merged together with `merge_maps`.

    extract_submap -input_map <input map> -output_map <output map> <images to keep>

or

    extract_submap -input_map <input map> -output_map <output map> -xyz_box "xmin xmax ymin ymax zmin zmax"

If it is desired to not re-adjust the cameras after the submap is
extracted (for example, if the map is already registered), use the
`-skip_bundle_adjustment` option.

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
-skip_pruning.

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

###Working with Robot Data

[This section is outdated.]

To pull information from ROS bag files:

    rostopic echo -b <bag.bag>  -p /cam_nav/image  --noarr > <bag.images.csv>
    rostopic echo -b <bag.bag>  -p /ground_truth           > <bag.ground_truth.csv>

These CSV files obtained from the ROS bag contain the timestamp for
every image captured, as well as ground truth measurements for the bot
position and orientation (not necessarily at the same times the images
were captured).

Next, interpolate the ground truth measurements at image timestamps,
and save the resulting collection of images and their poses as a
measured map file. Along the way, convert the measurements from the
bot coordinate frame to the camera coordinate frame using info in
`communications/ff_frame_store/launch/ff_frame_store.launch`

    parse_cam -ground_truth_file <ground_truth.csv> -image_file <images.csv> -image_set_dir <image dir> -camera_calibration <calib.xml> -output_map <images.map>

This command creates a map based on the ground truth data.

Sometimes it is desirable to build a map only from a subset of the
images in the bag (see the documentation of the `select_images`
executable above). In that case, `parse_cam` can be invoked with the
desired subset with the `-image_subset_dir` option.

###Trajectory Generation

[This section is outdated.]

A tool to generate a trajectory that P3 can follow:

    gen_trajectory -num_loops 3 -num_samples 50 -trajectory_file trajectory.csv
