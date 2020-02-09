\defgroup buildmap Map Building
\ingroup sparsemapping

Here we describe how to build a map.

# Summary {#buildmap}

1. Reduce the number of images.

2. Set up the environment.

3. Build the map.

4. Find control points in Hugin, and create a list of their coordinates.

5. Register the map.

# Map Building

We go through how a map is made.

## Reduce the number of images.

Here, we delete the images that overlap highly.

  select_images -density_factor 1.4 bag_images/*.jpg

This is a non-reversible operation, so it should be invoked on a copy
of the images.

The higher the value of the density factor, the more images will be
kept. Some experimentation with this number is necessary. A value of 1.4
seems to work well. Ideally the images should have perhaps on the order of 
2/3 to 3/4 of overlap.

Alternatively, one can simply first pick every 10th or 20th image,
such as:

  ls bag_images/*0.jpg

then copy these to a new directory.

In either case, one should inspect the images in the 'eog' viewer, 
and delete redundant ones from it. 

## Setup the Environment

In the first step, one needs to set some environmental variables, as
follows:

export ASTROBEE_RESOURCE_DIR=/path/to/freeflyer/astrobee/resources
export ASTROBEE_CONFIG_DIR=/path/to/freeflyer/astrobee/config
export ASTROBEE_ROBOT=p4d
export ASTROBEE_WORLD=granite

Here, p4d is the robot being used to take pictures, and the world is
the granite table. These may need to change, depending on your
goals. Under the hood, the following configuration files will be read:

  $ASTROBEE_CONFIG_DIR/cameras.config

which contains the image width and height (the camera we use is
the nav cam) and

  $ASTROBEE_CONFIG_DIR/robots/$ASTROBEE_ROBOT.config

having nav cam's intrinsics. If your camera is not the nav cam on p4d,
and none of the other available config files apply, you can just
temporarily modify the above files to reflect your camera's parameters
(without checking in your changes).

More details on these and other environmental variables can be found in

  freeflyer/astrobee/readme.md

## Building a Map

Execute this command to construct a complete map:

  build_map <image files> [ -num_subsequent_images <val> ] -output_map <output.map>

During map building, every image will be matched against every
subsequent image in the sequence. To use only a limited number of
subsequent images, set the value passed to the
`-num_subsequent_images` flag. Later, we will also see how to match
only similar images using a vocabulary tree.

The runtime of the algorithm is directly proportional to the number of
input images times the number input to
`-num_subsequent_images`. Making the latter small will result in more
drift. If you know that a region will be revisited after say 100
images, use this number for this parameter. Making this too big will
result in very slow map building.

### Map Building Pipeline

The `build_map` command runs a number of steps, which can also be invoked
individually for further control.

1. **Detect Interest Points**

    build_map <image files> -feature_detection [ -sample_rate <N> ]
                            [ -detector <detector> ] [ -descriptor <descriptor> ]

Detects features in all of the input images and save them to a map
file. The `-sample_rate <N>` flag, if specified, builds the map from
only one out of N input images. If desired, the feature detector and
feature descriptor can be specified. The default is ORGBRISK.

2. **Match Images**

    build_map -feature_matching [ -num_subsequent_images <val> ]

Match the detected features between images, detecting similar features
that appear in multiple images. The number of subsequent images to
match against can be specified, otherwise all pairwise matches are
evaluated.

3. **Build Tracks**

    build_map -track_building

Take the feature matches and form "tracks" of features seen
consistently across multiple frames.

4. **Incremental Bundle Adjustment**

    build_map -tensor_initialization

  Incremental bundle adjustment for transform initialization.

5. **Bundle Adjustment**

    build_map -bundle_adjustment

  Adjust the initial transformations to minimize error with bundle adjustment.

6. **Map Rebuilding**

    build_map -rebuild

Rebuilds the map with a different feature set (by default, BRISK
features). The initial map can be built with high quality features,
such as SURF, and then rebuilt with faster features for localization,
such as BRISK. During rebuilding the cameras are kept fixed by
default, since BRISK features, while faster, may be fewer and less
accurate.

Rebuilding is much faster than building from scratch, since it borrows
from the original map the information about which images can be
matched to which, and also reuses the camera positions.

To replace the camera intrinsics during rebuilding, one can use
-rebuild_replace_camera, when the camera is set via ASTROBEE_ROBOT.
Camera positions and orientations can be re-optimized with
-rebuild_refloat_cameras. To rebuild with a desired feature detector,
use the option -rebuild_detector.

Note that rebuilding the map does not rebuild the vocabulary database
which should be done as below.

7. **Vocabulary Database**

    build_map -vocab_db

Builds a vocabulary database for fast lookup of matching image pairs.
Without this, we have to compare to every image in the map for
localization.  The vocabulary database makes the runtime logarithmic
instead of linear.

The above options can also be chained. For example, to
run the pipeline without tensor initialization, you could do:

    build_map <image files> -feature_detection -feature_matching -track_building -bundle_adjustment

It is important to note that normally build_map prunes a map from
features that show up in just one image after the vocabulary database
is built (unless it is invoked with -skip_pruning). Hence, invoking
the -vocab_db on a map already pruned will create a much smaller
database against which localization won't work so well. The only way
to recover the pruned features is to invoke map-building with the
-rebuild flag, followed again by the -vocab_db flag which will rebuild
the database of vocabulary features and then prune the map.

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

The following options can be used to create more interest point features:

  -min_surf_features, -max_surf_features, -min_surf_threshold,
  -default_surf_threshold, -max_surf_threshold, -min_brisk_features,
  -max_brisk_features, -min_brisk_threshold, -default_brisk_threshold,
  -max_brisk_threshold

The `build_map` command uses the file `output.map` as both input and output
unless the flag `-output_map` is specified.

## Map Registration

Maps are built in an arbitrary coordinate system. They need to be
transformed to a real-world coordinate system using manually defined
control points.

To accomplish this, first open a subset of the images used to build
the map in Hugin, such as:

  hugin <image files>

It will ask to enter a value for the FoV (field of view). That value
is not important since we won't use it. One can input 10 degrees,
for example. 

Go to the "Expert" interface, then select matching control points
across a pair of images (make sure the left and right image are not
the same). Then repeat this process for several more pairs.

Save the Hugin project to disk. Create a separate text file which
contains the world coordinates of the control points picked earlier,
with each line in the "x y z" format, and in the same order as the
Hugin project file.  That is to say, if a control point was picked in
several image pairs in Hugin, it must show up also the same number of
times in the text file. In the xyz text file all lines starting with
the pound sign (#) are ignored, as well as all entries on any line
beyond three numerical values.

The xyz locations of the control points for the granite lab can be
found in 

  localization/marker_tracking/ros/launch/granite_lab_tags.xml

If a new set of world coordinates needs to be acquired, one can use
the Total Station, as described in total_station.md, which is in the
same directory as this file.

How to create xyz coordinates for the JPM module of ISS is described
later on in this section.

Register the map with the command:
    
    /bin/cp -fv mapfile.map mapfile.registered.map
    build_map -registration <hugin files> <xyz files> -num_ba_passes 0 \
     -registration_skip_bundle_adjustment -skip_filtering              \
     -output_map mapfile.registered.map

There can be multiple such files passed as input. Control point files
are expected to end in .pto, while xyz files in .txt.

In practice, to not make mistakes, it is far easier to have both Hugin
and the text file with the xyz points opened at the same time. Each
time a point is added in Hugin and the project is saved, its xyz
coordinates can be saved to the text file, and the above command can
be run.

After registration is done, it will print each transformed coordinate
point from the map and its corresponding measured point, as well as the 
error among the two. That will look as follows:

transformed computed xyz -- measured xyz -- error norm (meters)
-0.0149 -0.0539  0.0120 --  0.0000  0.0000  0.0000 --  0.0472 img1.jpg img2.jpg
 1.8587  0.9533  0.1531 --  1.8710  0.9330  0.1620 --  0.0254 img3.jpg img4.jpg

The error norm should be no more than 3-5 cm. If for a point the error
is too large, perhaps something went wrong in picking the points. That
point can be deleted and reacquired, perhaps with a different image
pair.

If all errors are large, that may mean the camera calibration is wrong
and needs to be redone, and the map rebuilt, using

  build_map -rebuild -rebuild_refloat_cameras -rebuild_replace_camera

or one should create images that are closer to the points used in
registration.

Note that we did registration without bundle adjustment, which would
further refine the cameras using the xyz world coordinates as a
constraint. The latter should not be necessary if the map is 
geometrically correct. 

If such bundle adjustment is desired, it will keep the xyz
measurements fixed during this optimization (unlike the xyz points
obtained purely through interest point matching and triangulation)
because the measurements are assumed already accurate.

When a registered SURF map is available with features already picked
in Hugin as described earlier, and it is desired to register a map of
the same region but with different images, instead of picking
registration points in the new images it is simpler to merge that map
to the registered map while doing bundle adjustment, re-register
the merged map, and extract from it the submap corresponding to the
new image set. 

### Registration on the ISS

No xyz coordinate measurements exist for the ISS. Instead, 3D points
were picked in simulation in the JPM module of ISS. They are available
in the file iss_registration.txt in this directory. These points can
be visualized in the ISS as follows:

Open two terminals, and in each one type:

  export BUILD_PATH=$HOME/freeflyer_build/native
  source $BUILD_PATH/devel/setup.bash

In the first terminal start the simulator:

  roslaunch astrobee sim.launch speed:=0.75 rviz:=true  
 
In the second, run:

  python $HOME/freeflyer/localization/sparse_mapping/tools/view_control_points.py \
    $HOME/freeflyer/localization/sparse_mapping/iss_registration.txt

(here it is assumed that the freeflyer directory is in your home directory).

Go back to the simulated ISS and examine the registration points.
If the Rviz display looks too cluttered, most topics can be turned off.
The registration points will be shown in Rviz under 

Debug/Sensors/Localization/Registration

If this topic is unchecked, it should be checked and one should
run the Python script above again.

Each point will be displayed as a red dot and a white text label,
according to the fourth column in iss_registration.txt. Sometimes the
ISS obscures the text labels, in that case it can be temporarily
turned off in Rviz. It is suggested to use the points starting with
letter "V", as those were carefully validated. Points starting with
letter "P" were not validated. Points starting with letter "B" were
shown to be not accurate and should not be used.

To create new points in this file, one runs in a terminal (after
setting up the environment as above):

  rostopic echo /clicked_point

then goes to RViz, clicks on the toolbar on "Publish Point", and
clicks on a point on the ISS body. Its coordinates will be echoed in
the terminal. Note that these points will be in the "rviz" frame,
while we need them in the "world" frame. To perform this conversion,
flip the sign of the y and z coordinates.

After the file with the datapoints is saved, re-running the earlier
Python command will refresh them.

## Map Verification

A registered and bundle-adjusted map can be used to study how well it
predicts the computed 3D locations for an independently acquired set
of control points and 3D measurements. These are in the same format as
for registration. The map is not modified in any way during this step.

    build_map -verification <hugin files> <xyz files>

## Map Performance

To test how the map may perform on the target platform, do the following:

### Stage the new map

1. Copy the new map on the robot MLP (preference in /data)

    scp <map2test.map> mlp:/data
    
2. On the MLP, move the current map aside

    ssh mlp
    cd /res/maps
    mv granite.map _granite.map

3. On the MLP, create a symlink to the new map

    ln -s /data/<map2test.map /res/maps/granite.map

### Stage the bag with images

    rsync --archive --partial --progress directory_of_bags mlp:/data/bags


### Stage the feature counter utility (should be added to the install at one point)

    scp <freeflyer_src>/marker_tracking/ros/tools/features_counter.py mlp:

### Launch the localization node from the LLP

    ssh llp
    roslaunch astrobee astrobee.launch llp:=disabled mlp:=mlp nodes:=framestore,dds_ros_bridge,localization_node

### Enable the mapped landmark production (from MLP)

   export ROS_MASTER_URI=http://llp:11311
   rosservice call /loc/ml/enable true

### Play the bags

    cd /data/bags/directory_of_bags
    rosbag play --loop *.bag

### Evaluate performance

1. Look at the load with htop

2. Watch the frequency of feature production

    rostopic hz -w 5 /loc/ml/features

3. Watch the number of features being produced

    ~/features_counter.py ml


