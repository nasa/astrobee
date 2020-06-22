\page msg_conversions Message Conversions

This library assissts in converting messages between various formats,
mainly in converting geometric transforms between Eigen, ROS, C arrays,
and Lua files.

The package also includes two helper nodes:

* `pose_stamped_msg_cnv`: This node takes a pose ros topic and outputs a tf2 frame.
It takes as input the command line arguments `--input_topic`, `--input_frame`, and `--output_frame`.
* `landmark_msg_cnv`: This node reads VisualLandmarks messages, and outputs point clouds that
can be shown in rviz. The input and output topics are specified via `--input_topic` and `--output_topic`
command line arguments.

