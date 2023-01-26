# Sample archived Astrobee ISS telemetry bags

These bags are used to test backward compatibility of the latest
Astrobee software with archived Astrobee ISS telemetry bags. See the
[telemetry backward compatibility
guide](https://nasa.github.io/astrobee/html/md_doc_general_documentation_maintaining_telemetry.html)
for context.

Our testing checks whether the `rosbag_fix_all.py` script can fix the
archived test bags so that they can be processed by the standard ROS
tools using the latest Astrobee software message definitions.

Each bag in this folder:

- Contains sample messages recorded during a single ISS activity by one
  Astrobee robot.

- Is sampled from the bags contained in one of the ZIP archives in the
  [Astrobee ISS public data release
  folder](https://nasagov.app.box.com/s/4ign43svk39guhy9ev8t5xkzui6tqjm1).
  The bag filename is derived from the ZIP archive filename.

- Specifically, contains one sample message on each ROS telemetry topic
  recorded within the ZIP archive, as collected by scanning the bags in
  the archive using the `rosbag_sample.py` script. (The full bags
  are multiple gigabytes in size and are not needed for basic backward
  compatibility testing.)
