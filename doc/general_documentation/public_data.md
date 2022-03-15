\page using_telemetry Using Astrobee Robot Telemetry Logs

The ISS Astrobee Facility has established an [Astrobee ISS telemetry
log public release
folder](https://nasagov.box.com/s/4ign43svk39guhy9ev8t5xkzui6tqjm1).

Astrobee telemetry logs are also called "bag files" and use the
[rosbag](http://wiki.ros.org/rosbag) format.

## Understanding telemetry in context

This document focuses primarily on how to use Astrobee robot telemetry
logs. However, for best understanding of an Astrobee ISS activity, you
may benefit from context clues found in other information resources,
including:

- Debrief notes. Immediately following an ISS activity, the Astrobee
  ground team assembles debrief notes that review the activity
  objectives, which objectives succeeded or failed, what anomalies were
  observed and follow-up actions were required, etc. These notes are
  usually archived on the NASA-internal [Free Flyer Ops
  wiki](https://babelfish.arc.nasa.gov/confluence/display/FFOPS/).

- Action logs. During the activity, certain notes may be logged live,
  for example, as timestamped comments on a NASA-internal wiki
  page. Typical events noted in the log might include when the robot
  software was reset, when a crew procedure step started/finished, when
  an anomaly was observed, etc.

- ISS crew procedures. Astronauts often perform setup/teardown tasks
  around an Astrobee activity (such as setting "nominal" light levels in
  the module, clearing clutter out of the module, installing Astrobee
  payloads, setting up ISS camcorders to live stream the activity,
  etc.). For some activities, astronauts play a much larger role, such
  as manually flying an Astrobee around to collect imagery. The crew
  procedure will tell you what instructions the astronaut was following.

- Ground procedures. The concept of operations for a typical Astrobee
  activity includes a team of operators on the ground playing multiple
  roles. These operators follow a ground procedure that often includes
  much more detail about the planned robot actions than the crew
  procedure. Astrobee ground procedures follow a table format which
  includes a column for the operator to record notes. If the operator
  filled out this column, it can provide valuable context.

- ISS imagery. Many Astrobee activities are recorded live by ISS
  camcorders pre-positioned by an astronaut. In other cases, astronauts
  may opportunistically grab a camera to capture some video or a few
  photos. After privacy review, this imagery can typically be found by
  searching for "astrobee" on the NASA-internal site [Imagery
  Online](https://io.jsc.nasa.gov/). A third-party camera view can be
  invaluable for understanding what actually happened during an
  activity, especially if there were significant anomalies.

- Post-activity data processing.
  - Certain simple processing steps are almost always applied to the
    telemetry logs before you receive them. For example:
    - Telemetry bag files that were split on the robot may have been
      merged into a single bag file.
    - Bag file problems such as missing message definition metadata may
      have been repaired.
  - However, other types of post-activity processing are performed on a
    case-by-case basis, and the results may not be included in the ISS
    log file you receive. For example:
    - It is common to log Astrobee NavCam imagery during an ISS activity
      so that post-activity processing can generate higher-quality
      position estimates using a costly bundle-adjustment algorithm that
      can't run in real-time onboard the robot. When this type of
      processing has been performed, you may prefer to request and use
      the higher-quality post-activity position estimate vs. the
      real-time position estimate logged on the ISS.

- Research publications. If the activity results were published, the
  publications are often the best resource for detailed context.

In some cases, you may have access to the publicly released Astrobee
telemetry log but not to these other resources documenting the ISS
activity. Our data release processes are still evolving. For now, if you
can't find a resource you need, feel free to ask for more information.

## Basics of the bag format

Astrobee telemetry logs are also called "bag files" and use the
[rosbag](http://wiki.ros.org/rosbag) format.

The [ROS](https://www.ros.org/) middleware framework allows a complex
robot software architecture to be implemented in distributed
fashion. Individual software components are implemented as nodes that
communicate using anonymous publish/subscribe message passing.

Each message type has a format defined in a `*.msg` file that describes
what fields are present in the message. When a node starts publishing a
message, it first specifies both the message type and the "topic" name
it will use for publishing. This same topic can be used by other nodes
to subscribe and start receiving the message.

A ROS bag file includes two main types of information:
- Metadata about what topics are included and the message type for each
  topic
- The actual contents of the timestamped messages

## Finding the right bag

In the [Astrobee ISS telemetry log public release
folder](https://nasagov.box.com/s/4ign43svk39guhy9ev8t5xkzui6tqjm1), the
telemetry is packaged as ZIP archives, ordered by ISS activity date.

The ZIP archive file name includes the Astrobee robot serial number and
robot name. For example, in "2021-03-26_SN003_bumble.zip",
"SN003_bumble" means Astrobee serial number 3, also known as "Bumble".
If multiple robots were involved in an activity, there will be multiple
ZIP files.

When a ZIP archive is unpacked, it may contain multiple bag files. To
explain why, we should first note that Astrobee flight software is able
to log two concurrent telemetry streams:

1. The `immediate` stream contains relatively low-bandwidth housekeeping
   information useful for robot debugging. The list of topics logged in
   this stream stays fairly consistent over all Astrobee activities,
   only occasionally changing due to Astrobee baseline flight software
   upgrades. Bags that record the `immediate` stream typically have a
   filename ending in `ars_default.bag`. (Where "`ars`" stands for
   "Astrobee Robot Software".) Recording start/stop for this stream is
   usually related to starting or restarting the Astrobee baseline
   flight software.

2. The `delayed` stream is configured much more dynamically. Typically,
   the Astrobee user defining the objectives of the activity will
   develop one or more logging profiles (i.e., lists of topics to log)
   that suit their objectives. During the course of a single ISS
   activity, ground operators can start and stop bag recording as well
   as switching logging profiles, as directed by the ground
   procedure. The tail end of the filename for each `delayed` stream bag
   file is also set by the operator. Recording start/stop is frequently
   used to separate the bag files relating to different procedure steps
   and give them meaningful names. Because high-bandwidth imagery
   logging can quickly exhaust Astrobee's onboard storage, `delayed`
   stream bag recording is often disabled for most of the activity.

To reiterate, these two streams are logged concurrently, so it is common
to have an `immediate` bag file and a `delayed` bag file that cover
overlapping time intervals, recording different topics. The fact that
the two streams are stored in separate bag files is not a barrier to
analyzing them together: if needed, we have tools that enable you to
merge bag files in message timestamp order.

The first part of each bag filename gives the date and time when
recording of the bag file started, specified in the UTC (also known as
GMT) time zone used onboard the ISS. For long activities that generate
many bag files, you will probably want to use context information such
as the ground procedure to determine which bag files are relevant for
your needs.

## Install before working with bags

You can get both the ROS bag file tools and the message definitions
necessary for working with Astrobee bag files by following the
installation instructions for the Astrobee robot software: \ref
install-nonNASA

Depending on the activity, if there was guest science software installed
and publishing additional message types, you may need to get the
necessary message definitions by installing that software as well. For
example, here are the [ISAAC software installation
instructions](https://nasa.github.io/isaac/html/md_INSTALL.html).

We also note that, if `rosbag` processing is all you need, it should
suffice to do a minimal install of just the `ff_msgs` and `ff_hw_msgs`
ROS packages from the [`astrobee`](https://github.com/nasa/astrobee)
repo without building the entire Astrobee software system and without
installing all of its many third-party dependencies. And likewise, just
the `isaac_msgs` and `isaac_hw_msgs` ROS packages from the
[`isaac_msgs`](https://github.com/nasa/isaac_msgs) repo should suffice
for ISAAC messages. (But that slimmed-down install process has not yet
been tested and documented.)

## Preparing the bag for analysis

As Astrobee flight software continues to evolve, its telemetry message
definitions occasionally change in ways that break backward
compatibility for analysis of archived telemetry bags.

We recommend "fixing" a bag (rewriting it for compatibility with the
latest message definitions) before using it, as follows:

```console
ASTROBEE_DIR=$HOME/astrobee
$ASTROBEE_DIR/src/scripts/postprocessing/rosbag_fix_all.py in.bag
```

However, fixing the bag may not be needed if you plan to analyze it
only with `bagpy` and not with the core ROS tools.

## Figuring out what's in the bag

There are many broader [tutorials on
rosbag](http://wiki.ros.org/rosbag/Tutorials), and a full discussion is
beyond the scope of this document.

However, here are a few shortcuts for finding what's in a bag:

- `rosbag info some.bag`: Lists topics contained in the bag,
  their message types, and other information.
- `rosmsg info package_name/MsgType`: Lists the fields contained in a
  message type.

You can also browse an overview of Astrobee message types in these
folders:

- [`ff_msgs`](https://github.com/nasa/astrobee/tree/master/communications/ff_msgs/msg)
- [`ff_hw_msgs`](https://github.com/nasa/astrobee/tree/master/communications/ff_hw_msgs/msg)
- [`isaac_msgs`](https://github.com/nasa/isaac_msgs/tree/master/isaac_msgs/msg)
- [`isaac_hw_msgs`](https://github.com/nasa/isaac_msgs/tree/master/isaac_hw_msgs/msg)

## Common bag processing tasks

### Converting a bag to CSV format

The following example CSV export uses a bag from the publicly released ZIP archive
`2021-03-26_SN003_bumble.zip`:

```console
ASTROBEE_DIR=$HOME/astrobee
BAG=20210326_1855_phase1Loc_survey_bay_5_attempt_2.bag
# generate a smaller bag containing only the topic of interest (filtering out any bad message types)
$ASTROBEE_DIR/src/scripts/postprocessing/rosbag_topic_filter.py $BAG -a /loc/pose loc_pose.bag
# export to CSV
rostopic echo -b loc_pose.bag -p /loc/pose > loc_pose.csv
```

### Combining different message topics based on their timestamps

Often we want to analyze two message topics together, combining the
fields of messages on both topics that were received around the same
time. For example, we might want to join sensor measurement messages
with pose messages to generate combined sensor+pose records that can be
used to plot a map.

We provide the `csv_join.py` script to perform this task on exported CSV
files. The following example uses a bag from the publicly released ZIP
archive `2021-03-26_SN003_bumble.zip`:

```console
ASTROBEE_DIR=$HOME/astrobee
BAG=20210326_1855_phase1Loc_survey_bay_5_attempt_2.bag

$ASTROBEE_DIR/src/scripts/postprocessing/rosbag_topic_filter.py $BAG -a /hw/wifi hw_wifi.bag
rostopic echo -b hw_wifi.bag -p /hw/wifi > hw_wifi.csv

$ASTROBEE_DIR/src/scripts/postprocessing/rosbag_topic_filter.py $BAG -a /loc/pose loc_pose.bag
rostopic echo -b loc_pose.bag -p /loc/pose > loc_pose.csv

$ASTROBEE_DIR/src/scripts/postprocessing/csv_join.py hw_wifi.csv loc_pose.csv wifi_plus_pose.csv
```

### Displaying imagery found within a bag file

Try using the [`rqt_image_view`](http://wiki.ros.org/rqt_image_view) or
[`rqt_bag`](http://wiki.ros.org/rqt_bag) tools.

TODO: Test and provide a detailed example with an Astrobee bag.

### Exporting imagery found within a bag file

There are multiple tutorials about how to do this with ROS bags. Not
clear what is the best method. Or an internal example we could build
from: the Astrobee Facility has a script that extracts imagery in order
to generate a movie for NASA imagery release review.

TODO: Test and provide a detailed example with an Astrobee bag.

## Potential issues

### Incomplete bag metadata

When processing a bag file, you may see an error message like this:

    genmsg.msg_loader.MsgNotFound: Cannot locate message [Header]: unknown package [std_msgs] on search path [{}]

It typically indicates that the bag file metadata is missing some
required message definition information, most commonly due to a
[known](https://github.com/rosjava/rosjava_bootstrap/issues/16)
[bug](https://github.com/nasa/astrobee/issues/402) with messages
published by nodes using `rosjava`, which we use on the Astrobee HLP).

If you encounter this problem, first make sure you [fixed the bag as
described above](#preparing-the-bag-for-analysis).

If that doesn't work, you can try filtering out the bad topics instead of
trying to fix their message definitions:

```console
ASTROBEE_DIR=$HOME/astrobee
$ASTROBEE_DIR/src/scripts/postprocessing/rosbag_topic_filter.py in.bag -r "/gs/*" -r /hw/cam_sci/compressed fixed.bag
```

And this command may help determine which topics are problematic
in case more topics need to be fixed with the filtering approach:

```console
$ASTROBEE_DIR/src/scripts/postprocessing/rosbag_detect_bad_topics.py in.bag
```

As our processes improve, we hope to ensure future bag files have this
metadata issue fixed before public data release, so you will not have to
deal with it.

### Bags containing messages with outdated message definitions

When processing a bag file, you may see an error message indicating
that some messages need to be migrated.

This is because, as Astrobee flight software continues to evolve, its
telemetry message definitions occasionally change in ways that break
backward compatibility for analysis of archived telemetry bags.

If you encounter this problem, [fix the bag as described
above](#preparing-the-bag-for-analysis).

### Timestamp clock skew

Astrobee has multiple processors (the LLP, MLP, and HLP) that have
independent clocks. We describe the difference between their times
as clock skew.

In some cases, clock skew can be large enough to impact your telemetry
analysis when trying to combine messages logged on different processors.
For example, you might want to determine the robot's position when it
captured a SciCam image (logged on the HLP) using pose messages (logged
on the MLP). In that case, clock skew between HLP and MLP would
introduce a position error proportional to the robot's velocity times
the clock skew time delta.

Astrobee's processor clocks are nominally kept in sync using the Network
Time Protocol (NTP), but that synchronization has not always been
reliable. NTP is designed for hosts with long uptimes and tries to bring
them into sync gradually. It tends not to work as well in the Astrobee
context where each processor's clock starts completely uninitialized
(the clocks are not battery backed), and robot operations don't allow
much time for sync before starting important telemetry logging.

Astrobee processes for mitigating clock skew have gradually improved,
including:
- Improved startup procedures do a better job of forcing the clocks to
  sync precisely at startup. Older bags are likely to suffer more
  clock skew.
- Astrobee `immediate` stream bag files now include clock skew
  measurements. You can analyze clock skew with the [`clock_skew.py`
  script](https://github.com/nasa/astrobee/blob/master/management/sys_monitor/tools/clock_skew.py).
  In theory, you could also use these measurements to correct for clock
  skew, but that has not been implemented.

Note that the same clock skew issues can also arise between Astrobee's
processors and:
- The processor of a payload installed in the Astrobee
- Another Astrobee
- Other timestamped data, such as ISS camcorder video

In these cases, you can check procedures and notes for the activity to
determine if any special measures were taken to prevent or to measure
clock skew.
