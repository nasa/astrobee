\page merge_bags Merge bags

Often times an Astrobee recording is in multiple bags, so those need
to be merged before further processing. There exist two tools for
that: this one, ``merge_bags``, written in C++, and ``merge_bags.py``,
written in Python. The C++ tool, which is documented here, is more
robust, as apparently the Python one may fail if certain topics are
present in the the input bag files.

The ``merge_bags`` program is also more versatile. It can filter
images by acquisition time range, topic, and can keep only one image
for a given time interval (useful if the images are too frequent).

The acquisition time for an image or point cloud message is determined
from the timestamp field in the header of that message, which is more
reliable than extracting it from the message time itself.

Example:

    merge_bags bag1.bag bag2.bag -output_bag merged_bag.bag

Command-line options:

-output_bag <string (default="")>
  The output bag.
-start_time <double (default=-1.0)>
  If non-negative, start at this time, measured in seconds since epoch.
-stop_time <double (default=-1.0)>
  If non-negative, stop before this time, measured in seconds since epoch.
-save_topics <string (default="")>
  If non-empty, the topics to write. Use quotes and a space as separator. 
  Example: '/hw/cam_nav /hw/cam_sci'.
-min_time_spacing <double (default=-1.0)>
  If non-negative, for each input bag, keep only messages for a given topic 
  which differ by no less than this time amount, in seconds. Assumes that 
  messages in a bag are stored in ascending order of time.
