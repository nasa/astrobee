\page dds_ros_bridge DDS ROS Bridge

The DDS ROS Bridge acts like a translator between the ground data system (GDS) and flight software (FSW). It receives rapid messages from GDS and converts them to ros messages and vise versa. It also sends compressed file acknowledgements upon receiving compressed files. Furthermore, it is responsible for reading in the command configuration file and sending it in a rapid command configuration message to GDS.
