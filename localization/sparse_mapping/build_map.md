\page buildmap Map building

Here we describe how to build a map.

# Summary {#buildmap}

1. Reduce the number of images.

2. Set up the environment.

3. Build the map.

4. Find control points in Hugin, and create a list of their coordinates.

5. Register the map.

# Map building

We go through how a map is made.

## Reduce the number of images

Here, we delete the images that overlap highly. (This tool is, like all
others, in astrobee_build/native.)

  select_images -density_factor 1.4 <image dir>/*.jpg

This is a non-reversible operation, so it should be invoked on a copy
of the images.

The higher the value of the density factor, the more images will be
kept. Some experimentation with this number is necessary. A value of
1.4 seems to work well. It may be needed to decrease this to 1.0 if
images appear to be too dense. Ideally the images should have perhaps
on the order of 2/3 to 3/4 of overlap. This tool is not perfect. One
should inspect the images in the 'eog' viewer, and delete redundant
ones from it manually, using the Delete key.

The images can also be inspected and deleted with nvm_visualize, a
tool included with this software. See readme.md for details.  This
tool, unlike eog, echoes each image name as it is displayed, which can
be useful with image manipulation tasks.

If the images do not have enough overlap, the selection tool needs to
be run again with a different value of this factor, or otherwise
images must be added back manually.

Alternatively, one can simply first pick every 10th or 20th image,
such as:

  ls <image dir>/*.jpg

then copy these to a new directory.

It is important to avoid rotating the bot in place when acquiring
images, as then the map could be of poor quality. Hence, the robot
should have some translation motion (in addition to any rotation) when
the data is acquired.

## Setup the environment

In the first step, one needs to set some environmental variables, as
follows:

export ASTROBEE_RESOURCE_DIR=$SOURCE_PATH/astrobee/resources
export ASTROBEE_CONFIG_DIR=$SOURCE_PATH/astrobee/config
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

  astrobee/astrobee/readme.md

## Building a map

Execute this command to construct a complete map:

  build_map <image dir>/*.jpg [ -num_subsequent_images <val> ] \
    -histogram_equalization -output_map <output.map>

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

The flag -histogram_equalization equalizes the histogram of the images
before doing feature detection. It was shown to create maps that are
more robust to illumination changes.

In practice, the map is build in pieces, and then merged. Then the
above process needs to be modified. See readme.md in this directory
for how this approach should go.

### Map building pipeline

The `build_map` command runs a number of steps, which can also be invoked
individually for further control.

1. **Detect interest points**

    build_map <image dir>/*.jpg -feature_detection [ -sample_rate <N> ]
      -histogram_equalization [ -detector <detector> ] [ -descriptor <descriptor> ]

Detects features in all of the input images and save them to a map
file. The `-sample_rate <N>` flag, if specified, builds the map from
only one out of N input images. If desired, the feature detector and
feature descriptor can be specified. The default is ORGBRISK.

Here and below we omitted for brevity the -output_map option that is
needed for the tool to run. The images need to be specified only at
step 1 above, and not below, as by then they are rememberd by the map.

2. **Match images**

    build_map -feature_matching -histogram_equalization [ -num_subsequent_images <val> ]

Match the detected features between images, detecting similar features
that appear in multiple images. The number of subsequent images to
match against can be specified, otherwise all pairwise matches are
evaluated.

3. **Build tracks**

    build_map -track_building -histogram_equalization

Take the feature matches and form "tracks" of features seen
consistently across multiple frames.

4. **Incremental bundle adjustment**

    build_map -incremental_ba -histogram_equalization

  Incremental bundle adjustment for transform initialization.

5. **Bundle adjustment**

    build_map -bundle_adjustment -histogram_equalization

  Adjust the initial transformations to minimize error with bundle adjustment.

If the options 

  -first_ba_index and -last_ba_index 

are specified, only cameras with indices between these (including both
endpoints) will be optimized during bundle adjustment.

6. **Map rebuilding**

    build_map -rebuild -histogram_equalization

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

Rebuilding a map while floating the cameras is not recommended if the
map had images taken out of it as is typically done to reduce its size
in order to be deployed on the robot. Refloating the cameras may then
result in the map breaking up into several connected components that
would drift from each other.

If it is desired to take out images from the map, it should happen at
this stage, before the vocabulary database and pruning happens at the
next step. See readme.md when it comes to such operations, where the
script grow_map.py is used.

7. **Vocabulary database**

    build_map -vocab_db

Builds a vocabulary database for fast lookup of matching image pairs.
Without this, we have to compare to every image in the map for
localization.  The vocabulary database makes parts of the runtime
logarithmic instead of linear.

It is very important to note that when the vocabulary database is
created the map is pruned from features that show up in just one
image. This is an irreversible operation and after it no other
operations can be performed on this map, such as extracting as submap,
or even recreating the vocabulary database again without a large loss
of quality. Hence this operation must be the very last to be applied
to a map.

At this stage the map is ready to be used on the robot.

### Building a SURF map only

The above options can also be chained. For example, to run the
pipeline to just create a SURF map one can do:

    build_map <image dir>/*.jpg -feature_detection -feature_matching \
      -track_building -incremental_ba -bundle_adjustment             \
      -histogram_equalization -num_subsequent_images 100

### Additional options

There are a few steps that can be used which are not included in the default map
building process. These include:

* `-loop_closure`: Take a map where images start repeating, and close the loop.
   This is not used much, as loop closure is handled automatically for loops
   smaller than what is given in -num_subsequent_images. For very large loops
   it is better to build the map in two overlapping pieces, and use merge_maps
   to merge them which will close loops as well.
* `-covariance_computation`: Compute the covariance of the triangulated points
   (after bundle adjustment only).
* `-registration`: Register to a real-world coordinate system, discussed later.
* `-verification`: Verify how an already registered map performs on an independently 
   acquired set of control points and corresponding 3D measurements.
* `-info`: Print some information about the map, including list of images,
   and if histogram equalization was used, and the latter can have the values:
   0 (not used), 1 (was used), 2 (unknown).

The following options can be used to create more interest point features:

  -min_surf_features, -max_surf_features, -min_surf_threshold,
  -default_surf_threshold, -max_surf_threshold, -min_brisk_features,
  -max_brisk_features, -min_brisk_threshold, -default_brisk_threshold,
  -max_brisk_threshold, -histogram_equalization

The `build_map` command uses the file `output.map` as both input and output
unless the flag `-output_map` is specified.

## Map registration

Maps are built in an arbitrary coordinate system. They need to be
transformed to a real-world coordinate system using manually defined
control points.

To accomplish this, first open a subset of the images used to build
the map in Hugin, such as:

  hugin <image dir>/*.jpg

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

  astrobee/localization/sparse_mapping/granite_xyz_controlPoints.txt

and the control points for the AR tags on the dock station can be
found in

  localization/marker_tracking/ros/launch/granite_lab_tags.xml

If a new set of world coordinates needs to be acquired, one can use
the Total Station, as described in total_station.md, which is in the
same directory as this file.

How to create xyz coordinates for the JPM module of ISS and information
about control points in the MGTF is described later in this document.

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

  build_map -rebuild -rebuild_refloat_cameras -rebuild_replace_camera \
    -histogram_equalization

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

  export BUILD_PATH=$HOME/astrobee_build/native
  source $BUILD_PATH/devel/setup.bash

In the first terminal start the simulator:

  roslaunch astrobee sim.launch speed:=0.75 rviz:=true  
 
In the second, run:

  python $SOURCE_PATH/localization/sparse_mapping/tools/view_control_points.py \
    $SOURCE_PATH/localization/sparse_mapping/iss_registration.txt

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

### Registration in the MGTF

A set of 10 registration points were measured in the MGTF with the
Total Station. They are in the file

  $SOURCE_PATH/localization/sparse_mapping/mgtf_registration.txt

Two of these are on the back wall, and the rest are on the metal
columns on the side walls, with four on each wall. Half of the points
are at eye level, and half at about knee-level.

Each such point is a corner of a portion of a checkerboard pattern, 
and it has a number written on the paper it is printed on, which is
the id from the above file. 

## Map verification

A registered and bundle-adjusted map can be used to study how well it
predicts the computed 3D locations for an independently acquired set
of control points and 3D measurements. These are in the same format as
for registration. The map is not modified in any way during this step.

    build_map -verification <hugin files> <xyz files>

## Sparse map performance and quality evaluation on the robot

(See below about how it can be done on a local machine.)

To test how the map may perform on the robot, do the following:

### Stage the new map

1. Copy the new map on the robot MLP (preference in /data)
```
    scp <map2test.map> mlp:/data
 ```   
2. On the MLP, move the current map aside
```
    ssh mlp
    cd /res/maps
    mv granite.map _granite.map
```
3. On the MLP, create a symlink to the new map
```
    ln -s /data/<map2test.map /res/maps/granite.map
```

### Stage the bag with images

    rsync --archive --partial --progress directory_of_bags mlp:/data/bags

### Stage the feature counter utility (should be added to the install at one point)

    scp $SOURCE_PATH/localization/marker_tracking/ros/tools/features_counter.py mlp:

### Launch the localization node on LLP

    ssh llp
    roslaunch astrobee astrobee.launch llp:=disabled mlp:=mlp nodes:=framestore,dds_ros_bridge,localization_node

(TODO(oalexan1): Not sure if to set the robot name above. We are using
one robot to localize with the data and map for another one.)
 
### Enable localization and the mapped landmark production (on MLP)

    export ROS_MASTER_URI=http://llp:11311
    rosservice call /loc/ml/enable true

If this command returns an error saying that the service is not
available, wait a little and try again.

### Play the bags (on MLP)

    cd /data/bags/directory_of_bags
    export ROS_MASTER_URI=http://llp:11311
    rosbag play --loop *.bag                               \
      /mgt/img_sampler/nav_cam/image_record:=/hw/cam_nav   \
      /loc/ml/features:=/loc/ml/old_features               \
      /loc/ml/registration:=/loc/ml/old_registration

It is important to check the topics that were recorded to the bag. If
the nav camera was recorded on /mgt/img_sampler/nav_cam/image_record
instead of /hw/cam_nav, as it happens when recording data on the ISS,
it must be redirected to the proper topic, as we do above. If
localization was running when the bag was recorded and hence the
topics /loc/ml/features and /loc/ml/registration were recorded, they
must be redirected to something else (above /tmp1 and /tmp2 was used)
to not conflict with actual localization results that would be now
created based on the images in the bag.

### Examine the performance and features on MLP

1. Look at the load with htop

2. Watch the frequency of feature production
```
    rostopic hz -w 5 /loc/ml/features
```
and echo the pose being output with the features:
```
    rostopic echo /loc/ml/features | grep -A 17 header:
```
3. Watch the number of features being produced
```
    ~/features_counter.py ml
```

## Verify localization against a sparse map on a local machine

To test localization of data from a bag against a map, one need not
run things on the robot, but use instead a local machine. This should
result on similar results as on the robot, but the speed of
computations may differ.

Set up the environment in every terminal that is used. Ensure that you
use the correct robot name below.

    source $BUILD_PATH/devel/setup.bash
    export ASTROBEE_RESOURCE_DIR=$SOURCE_PATH/astrobee/resources
    export ASTROBEE_CONFIG_DIR=$SOURCE_PATH/astrobee/config
    export ASTROBEE_WORLD=iss
    export ASTROBEE_ROBOT=bumble # your robot's name may be different
    export ROS_MASTER_URI=http://127.0.0.1:11311/

Examine the localization configuration file:

    astrobee/config/localization.config

Sym link the map to test:

    mkdir -p $SOURCE_PATH/astrobee/resources/maps
    rm -fv $SOURCE_PATH/astrobee/resources/maps/iss.map
    ln -s $(pwd)/mymap.map $SOURCE_PATH/astrobee/resources/maps/iss.map

Start the localization node:

    roslaunch astrobee astrobee.launch mlp:=local llp:=disabled \
      nodes:=framestore,localization_node

Enable localization:

    rosservice call /loc/ml/enable true

Then, as above, one must play a bag while redirecting the existing
/loc topics, and ensure that the images are published on /hw/cam_nav.

The poses of the newly localized camera images can be displayed as:

  rostopic echo /loc/ml/features | grep -A 17 header:

and compared to the old ones via:

  rostopic echo /loc/ml/old_features | grep -A 17 header:

## Evaluating the map without running the localization node

See astrobee/tools/ekf_bag/readme.md for how to run the
sparse_map_eval tool that takes as inputs a bag and a BRISK map and
prints the number of detected features.

Note that this approach may give slightly different results than using
the localization node, and even with using this node, things can
differ somewhat if running on a local machine vs running on the robot.
Hence, the most faithful test is the one in which such experiments are
performed on a robot, and ensuring that the software version on that
robot is the same as for the real robot on the space station, if the
goal is to prepare a map for an actual flight. The software version on
the robot can be found using:

cat /opt/astrobee/version.txt 

