\page sparsemapping Sparse Mapping

# Creation of sparse maps for robot localization

## What is a map

A map consists of feature descriptors and associated 3D positions of
the features.  A map may also contain a vocabulary database which
enables fast lookup of similar images.

## Map files

Maps are stored as protobuf files.

## ROS node

The ROS node takes images and a map as input, and outputs visual features
detected in the image and their 3D coordinates.

### Inputs

* `/hw/cam_nav`: Camera images
* The map file. See [build_map](build_map.md) at the bottom for its assumed location.

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

### Merging bags

The bags created on the ISS are likely split into many smaller bags,
for easy and reliability of transfer. Those can be merged into one bag
as follows:

    astrobee_build/devel/lib/localization_node/merge_bags \
      -output_bag <output bag> <input bags>

### Extracting images

To extract images from a bag file:

     extract_image_bag <bagfile.bag> -use_timestamp_as_image_name \
       -image_topic /hw/cam_nav -output_directory <output dir>

The above assumes that the software was built with ROS on. This tool should
exist in astrobee_build/native.

Please check using 'rosbag info' the nav cam topic in the bag, as its
name can change.

### Building a map

The `build_map` tools aids in constructing a map. See
[build_map](build_map.md) for further details.

### Visualization

To visualize a map, or just a list of images, use the command:

    nvm_visualize [ <output.map> ] [ <image1.jpg> <image2.jpg> ... ]

In the viewer, press `a` and `d` (or the left and right arrow keys, or
the Ins and Del keys on the num pad) to navigate through the
images. The current image being displayed will be echoed on the
command line. Press `c` to show the cameras and triangulated points in
3D. Press `q` to exit the viewer. Press 'Home' and 'End' to go 
to first and last image.

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

Clicking with the left mouse button on an image will print its name
and the pixel coordinates where the mouse hit. This can be useful in
collecting a subset of the images. (After clicking, a bug in OpenCV
disables the arrow keys, then one can navigate with the "Ins" and
"Del" keys on the numpad.)

This tool can be invoked to just look at images, without any map being
built. It can also delete images in this mode, with the 'Delete' and
'x' keys, if invoked as:

    nvm_visualize -enable_image_deletion <image dir>/*jpg

### Localize a single frame

All the commands below assume that the environment was set up, 
as specified in build_map.md.

To test localization of a single frame, use the command:

    localize <map.map> <image.jpg> -histogram_equalization

If invoked with the option -verbose_localization, it will list the
images most similar to the one being localized. To increase the 
number of similar images, use the -num_similar option. Another
useful flag is --v 2 when it will print more verbose information.
Most of the options of the localize_cams tool (see below)
are also accepted. 

### Testing localization using two maps

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
      -max_brisk_features 800 -min_brisk_threshold 20             \
      -default_brisk_threshold 90 -max_brisk_threshold 110        \
      -detection_retries 5 -num_threads 2                         \
      -early_break_landmarks 100 -histogram_equalization          \
      -reference_map ref.map -source_map source.map

Here we use values that are different from 

    astrobee/config/localization.config 

which are used for localization on the robot, since those are optimized
for speed and here we want more accuracy.

### Testing localization using a bag 

See: 

    astrobee/tools/ekf_bag/readme.md

for how to see how well a BRISK map with a vocabulary database does
when localizing images from a bag.

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


#### Merge maps

Given a set of SURF maps, they can be merged using the command:

    merge_maps <input maps> -output_map merged.map \
      -num_image_overlaps_at_endpoints 50

It is very important to note that only maps with SURF features (see
build_map.md) can be merged. If a map has BRISK features, it needs to
be rebuilt with SURF features, as follows:

      build_map -rebuild -histogram_equalization       \
        -rebuild_detector SURF -output_map <output map>

and then these regenerated maps can be merged.

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
be skipped during merging, using the 

    -skip_bundle_adjustment

option until the final map is computed, as this step can be
time-consuming.

If the first of the two maps to merge is already registered, it may be
desirable to keep that portion fixed during merging. To achieve that,
the merging can be done without bundle adjustment, and then build_map
can be invoked only to do bundle adjustment, while specifying the
range of cameras to optimize (the ones from the second map). See
build_map.md for details.
  
#### How to build a map efficiently

Often times map-building can take a long time, or it can fail. A
cautious way of building a map is to build it in portions (perhaps on
different machines), examine them, and merge them with `merge_maps`.

If map-building failed, parts of it could still be salvageable (one
can use nvm_visualize for inspection). Valid submaps can be extracted
with `extract_submap`. Then those can be merged with `merge_maps`.

When two maps to be merged overlap only in the middle, and they are
both large, the number -num_image_overlaps_at_endpoints will need to
be large which would make merging very slow. A very useful option can
then be the flag `-fast_merge` for this tool. It won't create matches
among the two maps, but will instead identify the shared images among
the two maps thus merging the maps, if shared images exist.

If no such images are available, but the two maps do see the same
physical location in some portions (if from different views), each of
the two maps can be first merged with the same small map of that
shared location, and then the newly merged map which now will have
shared images can be merged with the `-fast_merge` flags.

To summarize, with careful map surgery (extract submaps and merge
submaps) large maps can be made from smaller or damaged ones within
reasonable time.

All these operations should be performed on maps with SURF features.
Hence the general approach for building maps is to create small SURF
maps using the command:

    build_map -feature_detection -feature_matching -track_building    \
     -incremental_ba -bundle_adjustment                               \
     -histogram_equalization -num_subsequent_images 100               \
     images/*jpg -output_map <map file>

examine them individually, merging them as appropriate, then
performing bundle adjustment and registration as per build_map.md. 
Only when a good enough map is obtained, a renamed copy of it 
should be rebuilt with BRISK features and a vocabulary database
to be used on the robot.

#### Map strategy for the space station

For the space station, there exists one large SURF map with many
images, and a small BRISK map with fewer images. If new images are
acquired, it is suggested several small maps be assembled from them,
and those be merged to the large SURF map.

The key idea here is to add some well-chosen subsequences of those new
images to the SURF map, but only a few, for which localization failed,
to the BRISK map. This is best illustrated by an example.

Say the large original SURF map had 2000 images, and the smaller BRISK
map had only 1400 images. Say 300 new images have been acquired, and
for 80 of them localization failed. So, things will change as follows:

  - SURF map goes from  2000 to 2000 + 300 = 2300 images
  - BRISK map goes from 1400 to 1400 +  80 = 1480 images

The precise details are described in the next section.

#### Growing a map when more images are acquired

Sometimes things in the desired environment change enough, or the
lighting changes, and an existing map may no longer do as well in some
areas. Then, new images should be added to the map. The tool
grow_map.py can help with that.

First, out of the new images a set must be selected from which to
build a new map. We assume that the images in that set are further
broken up into several subsets, with each image in a subset
overlapping with the next one in that subset. For example, there can
exist a subset for the ceiling, one for the walls, etc.

A SURF map can be built for each subset, and those can be merged. (A
very useful option here can be -fast_merge.) Then, the new merged SURF
map can be merged with the earlier SURF map, and the combined map can
be bundle-adjusted and registered. While such a SURF map will be very
reliable, it will contain redundant information, so we will outline
below how to create a BRISK map that starts the same as the BRISK map
that existed before that, with only a minimum of images added for
which localization now fails. The new large SURF map will be used as
the source of the camera poses for the updated BRISK map, but the
latter will have fewer images.

We need some notation. Let prev_brisk_vocab_hist.map be the previous
BRISK map, before new images are added, prev_surf_registered.map be
the previous registered SURF map (that can have more images than
prev_brisk_vocab_hist.map). Let curr_surf_registered.map be the large
SURF map that is obtained from prev_surf_registered.map merged with
the images acquired this time, and curr_brisk_no_prune_hist.map be
obtained from curr_surf_registered.map using the -rebuild and
-histogram_equalization options. The prev_brisk_vocab_hist.map will
already be pruned and have a vocabulary database, but
curr_brisk_no_prune_hist.map is assumed to have none of these. Not
being pruned is very important. And again, all these maps must be
registered.

Let also list1.txt, ..., listN.txt contain the images for those
subsets of new images mentioned earlier, such as list_wall.txt,
list_floor.txt, etc., with the order of images in this list be the
same as in the submap for the subset made from that list (which is the
same order as output by build_map -info). We will create a new BRISK
map, named curr_brisk_vocab_hist.map, that will be pruned and with a
vocabulary database (so ready to be used, just like
prev_brisk_vocab_hist.map) that will have only some of the new images
as follows:

 - Start with prev_brisk_vocab_hist.map
 - Add all new images in list1.txt for which localization failed
   against prev_brisk_vocab_hist.map, thus getting a map named curr1.map.
 - Add all new images in list2.txt for which localization against
   curr1.map failed. 
 - Etc.

The logic here is very simple. Don't add all new images at once. Add
them in batches, and for each batch remember the fact that the
previous batch already made the map better, so don't add images from a
batch if they are not strictly necessary.

The following Python code implements this:

    python ~/astrobee/localization/sparse_mapping/tools/grow_map.py   \
      -histogram_equalization -small_map prev_brisk_vocab_hist.map    \
      -big_map curr_brisk_no_prune_hist.map -work_dir work            \
      -output_map curr_brisk_vocab_hist.map                           \
      list1.txt list2.txt ... listN.txt

After this is finished, the work directory can be wiped.

It is very important to never remove like this images from the SURF
map. This can result in this map breaking into disconnected sets when
being bundle adjusted. If desired to make the SURF map smaller, one
should examine the submaps it is made of (typically each submap has
its images in a subdirectory), and then one should carefully study
which submap (or portions of it) are not necessary for the whole map's
cohesiveness.

Also note that the grow_map.py script takes a lot of other parameters
on input that must be the same as in localization.config.

#### Reducing the number of images in a map

Sometimes a map has too many similar images. The tool reduce_map.py
attempts to reduce their number without sacrificing the map quality.

To use this feature, it is very important that the input map is not
pruned. This map should be made of of BRISK features and registered. It
need not have a vocab db.

Usage:

    python reduce_map.py -input_map <input map> -min_brisk_threshold <val> \
           -default_brisk_threshold <val> -max_brisk_threshold <val>       \
           -localization_error <val> -work_dir <work dir>                  \
           -sample_rate <val> -histogram_equalization

The BRISK thresholds here must be as when the map was built (ideally
like in localization.config). The -histogram_equalization flag is
necessary if your map was built with it.

A sequence of ever-smaller output maps are saved in the work
directory. They are pruned maps, with a vocabulary database, unlike
the input unpruned map.

The algorithm is as follows. Randomly remove a fraction (stored in
-sample_rate, typically 1/4th) of images form a map. Localize the
images from the full map against the images of the reduced map. Images
for which localization fails with more than a given error (typically 2
cm) are added back to the reduced map. This is repeated until no more
images need adding.

The reduced map is written to 

    <work_dir>/submap_iter0.map

Then more images are taken out of the map and all the previous process
is repeated (this is called the outer iteration), each time obtaining
a smaller map named

    <work_dir>/submap_iter<outer iter>.map

One should carefully evaluate these output maps. Likely after a couple
of attempts the quality of the map may start degrading. To use
more attempts, set the value of the -attempts variable.

Instead of taking images out of the map randomly, one can start with a
reduced map with a small list of desired images which can be set with
-image_list, and then all images for which localization fails will be
added back to it.


\subpage map_building
\subpage total_station
\subpage granite_lab_registration
\subpage using_faro