\defgroup buildmap Map Building
\ingroup sparsemapping

Here we describe how to build a map.

# Summary {#buildmap}

1. Record a bag file. Record offboard from the robot to limit the frame rate.
   Move the robot slowly to make sure neighboring images are connected and
   reduce motion blur.
       rosbag record /nav_cam/image
2. Extract images from the bag file.
       rosrun sparse_mapping extract_image_bag bag.bag
3. Reduce the number of images, deleting images that overlap highly.
       select_images bag_images/*
4. Build the map.
       build_map bag_images/*
7. Find control points in hugin, and create a list of their coordinates.
8. Register the map.
       build_map -registration hugin.pto xyz.txt -output_map output_brisk.map

# Map Building

We go through how a map is made.

## Building a Map

Execute this command to construct a complete map:

    build_map <image files> [ -undistorted_images ] [ -num_subsequent_images <val> ]

The `-undistorted_images` flag should be specified only if `parse_tango`
was called to create the images with the same flag.
Otherwise, `build_map` internally undistorts the interest points before matching them.
The map distortion parameters are taken from the `nav\_cam` in `cameras.config`.

During map building, every image will be matched against every
subsequent image in the sequence. To use only a limited number of
subsequent images, set the value passed to the `-num_subsequent_images` flag.
Later, we will also see how to match only similar images using a vocabulary tree.

The runtime of the algoirthm is directly proportional to the number of input images
times the number input to `-num_subsequent_images`. Making the latter small will result
in more drift. If you know that a region will be revisited after say 100 images, use this
number for this parameter. Making this too big will result in very slow map building. 

### Map Building Pipeline

The `build_map` command runs a number of steps, which can also be invoked
individually for further control.

1. **Detect Interest Points**

    build_map <image files> -feature_detection [ -sample_rate <N> ]
                            [ -detector <detector> ] [ -descriptor <descriptor> ]

  Detects features in all of the input images and save them to a map file. The
  `-sample_rate <N>` flag, if specified, builds the map from only one out of N input images.
  If desired, the feature detector and feature descriptor can be specified. The default
  is ORGBRISK.

2. **Match Images**

    build_map -feature_matching [ -num_subsequent_images <val> ]

  Match the detected features between images, detecting similar
  features that appear in multiple images. The number of
  subsequent images to match against can be specified, otherwise all pairwise
  matches are evaluated.

3. **Build Tracks**

    build_map -track_building

  Take the feature matchings and form "tracks" of features seen
  consistently across multiple frames.

4. **Incremental Bundle Adjustment**

    build_map -tensor_initialization

  Incremental bundle adjustment for transform initialization.

5. **Bundle Adjustment**

    build_map -bundle_adjustment

  Adjust the initial transformations to minimize error with bundle adjustment.

6. **Map Rebuilding**

    build_map -rebuild

  Rebuilds the map with a different feature set (by default, BRISK features). The initial
  map can be built with high quality features, such as SURF, and then rebuilt with
  faster features for localization, such as BRISK.

7. **Vocabulary Database**

    build_map -vocab_db

  Builds a vocabulary database for fast lookup of matching image pairs.
  Without this, we have to compare to every image in the map for localization.
  The vocabulary database makes the runtime logarithmic instead of linear.

The above options can also be chained. For example, to
run the pipeline without tensor initialization, you could do:

    build_map <image files> -feature_detection -feature_matching -track_building -bundle_adjustment

### Additional Options

There are a few steps that can be used which are not included in the default map
building process. These include:

* `-tensor_initialization`: Initialize transformation matrices between nearby images.
* `-loop_closure`: Take a map where images start repeating, and close the loop.
* `-covariance_computation`: Compute the covariance of the triangulated points
   (after bundle adjustment only).
* `-registration`: Register to a real-world coordinate system, discussed later.
* `-verification`: Verify how an already registered map performs on an independently 
   acquired set of control points and corresponding 3D measurements.
* `-info`: Print some information about the map, including list of images.
* `-assume_nonsequential`: If true, assume during incremental SfM that an 
   image need not be similar to the one before it. Slows down the process a lot.
`
The `build_map` command uses the file `output.map` as both input and output
unless the flag `-output_map` is specified.

### Reducing the Map Size

We can reduce the number of images in the map by removing highly
similar images. It is usually both faster and more accurate to build a
map from such a subset.

    select_images image1 image2 ...

The similar images that are highly repetitive will be deleted.

## Map Registration

Maps are built in arbitrary coordinate systems. They need to be
aligned into a real-world coordinate system using manually defined
control points.

To transform a map to real-world coordinates using control points,
first open a subset of the images used to build the map in Hugin.  Go
to the "Expert" interface, then select matching control points across
a pair of images (make sure the left and right image are not the
same). Then repeat this process for several more pairs.

Save the Hugin project to disk. Create a separate text file which
contains the world coordinates of the control points picked earlier,
with each line in the "x y z" format, and in the same order as the
Hugin project file.  That is to say, if a control point was picked in
several image pairs in Hugin, it must show up also several times in
the text file. In the xyz text file all lines starting with the pound
sign (#) are ignored, as well as all entries on any line beyond three
numerical values.

The locations of the control points can be found in 
`localization/marker_tracking/ros/launch/granite_lab_tags.xml`

Then register the map with the command:

    build_map -registration <hugin files> <xyz files> -num_ba_passes 1 -skip_filtering -output_map <mapfile.map>

There can be multiple such files passed as input. Control point files
are expected to end in .pto, while xyz files in .txt.

At the end of this, bundle adjustment is redone using the control
points and xyz measurements as additional information. The xyz
measurements are kept fixed during this optimization (unlike the xyz
points obtained purely through interest point matching and
triangulation) because the measurements are known fixed quantities.

## Map Verification

A registered and bundle-adjusted map can be used to study how well it
predicts the computed 3D locations for an independently acquired set
of control points and 3D measurements. These are in the same format as
for registration. The map is not modified in any way during this step.

    build_map -verification <hugin files> <xyz files>

